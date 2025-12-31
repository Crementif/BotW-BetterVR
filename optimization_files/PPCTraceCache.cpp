#include "PPCTraceCache.h"
#include "config/ActiveSettings.h"
#include "Cafe/GameProfile/GameProfile.h"
#include "Common/FileStream.h"
#include "util/helpers/helpers.h"

#include <algorithm>
#include <chrono>

PPCTraceCache& PPCTraceCache::GetInstance()
{
	static PPCTraceCache instance;
	return instance;
}

void PPCTraceCache::Initialize(uint64_t titleId)
{
	m_titleId = titleId;
	m_enabled = g_current_game_profile->IsTraceCacheEnabled();
	m_shutdownSignal = false;
	m_dirty = false;

	if (!m_enabled)
	{
		cemuLog_log(LogType::Force, "[TraceCache] Disabled for this title");
		return;
	}

	// Allocate sparse hash table (~1.5MB instead of 512MB)
	m_hashTable = std::make_unique<HashEntry[]>(HASH_TABLE_SIZE);

	// Initialize all slots to empty
	for (size_t i = 0; i < HASH_TABLE_SIZE; i++)
	{
		m_hashTable[i].address.store(EMPTY_SLOT, std::memory_order_relaxed);
		m_hashTable[i].hitCount.store(0, std::memory_order_relaxed);
		m_hashTable[i].lastSeen.store(0, std::memory_order_relaxed);
	}

	// Load existing trace data
	LoadFromDisk();

	// Start background save thread
	m_saveThread = std::thread(&PPCTraceCache::SaveThreadFunc, this);

	cemuLog_log(LogType::Force, "[TraceCache] Initialized for title {:016x} (using {}KB sparse table)",
		titleId, (HASH_TABLE_SIZE * sizeof(HashEntry)) / 1024);
}

void PPCTraceCache::Shutdown()
{
	if (!m_enabled)
		return;

	// Signal save thread to stop
	m_shutdownSignal = true;
	m_saveCondVar.notify_one();

	if (m_saveThread.joinable())
		m_saveThread.join();

	// Final save
	SaveToDisk();

	// Clear data
	m_hashTable.reset();

	cemuLog_log(LogType::Force, "[TraceCache] Shutdown complete");
}

PPCTraceCache::HashEntry* PPCTraceCache::FindOrInsert(uint32_t address)
{
	uint32_t startIndex = HashAddress(address);
	uint32_t index = startIndex;

	// Linear probing with wrap-around
	for (size_t i = 0; i < HASH_TABLE_SIZE; i++)
	{
		HashEntry& entry = m_hashTable[index];
		uint32_t currentAddr = entry.address.load(std::memory_order_relaxed);

		if (currentAddr == address)
		{
			// Found existing entry
			return &entry;
		}

		if (currentAddr == EMPTY_SLOT)
		{
			// Try to claim this slot
			uint32_t expected = EMPTY_SLOT;
			if (entry.address.compare_exchange_strong(expected, address, std::memory_order_relaxed))
			{
				// Successfully claimed
				return &entry;
			}
			// Another thread claimed it, check if it's ours
			if (entry.address.load(std::memory_order_relaxed) == address)
			{
				return &entry;
			}
			// It's a different address, continue probing
		}

		index = (index + 1) & (HASH_TABLE_SIZE - 1);
	}

	// Table is full (shouldn't happen with 128K slots)
	return nullptr;
}

void PPCTraceCache::RecordHit(uint32_t ppcAddress)
{
	if (!m_enabled)
		return;

	HashEntry* entry = FindOrInsert(ppcAddress);
	if (entry)
	{
		uint32_t timestamp = m_timestampCounter.fetch_add(1, std::memory_order_relaxed);
		entry->hitCount.fetch_add(1, std::memory_order_relaxed);
		entry->lastSeen.store(timestamp, std::memory_order_relaxed);
		m_dirty = true;
	}
}

std::vector<uint32_t> PPCTraceCache::GetPrewarmAddresses()
{
	if (!m_enabled)
		return {};

	std::vector<TraceEntry> entries;
	entries.reserve(MAX_CACHED_ENTRIES);

	// Collect from hash table
	for (size_t i = 0; i < HASH_TABLE_SIZE; i++)
	{
		uint32_t addr = m_hashTable[i].address.load(std::memory_order_relaxed);
		if (addr != EMPTY_SLOT)
		{
			uint32_t hitCount = m_hashTable[i].hitCount.load(std::memory_order_relaxed);
			if (hitCount > 0)
			{
				entries.push_back({
					addr,
					hitCount,
					m_hashTable[i].lastSeen.load(std::memory_order_relaxed)
				});
			}
		}
	}

	// Sort by hit count descending
	std::sort(entries.begin(), entries.end(), [](const TraceEntry& a, const TraceEntry& b) {
		return a.hitCount > b.hitCount;
	});

	// Return top N addresses
	std::vector<uint32_t> result;
	size_t count = std::min(entries.size(), PREWARM_TOP_N);
	result.reserve(count);
	for (size_t i = 0; i < count; i++)
	{
		result.push_back(entries[i].ppcAddress);
	}

	cemuLog_log(LogType::Force, "[TraceCache] Returning {} addresses for prewarm (from {} total)", result.size(), entries.size());
	return result;
}

void PPCTraceCache::LoadFromDisk()
{
	auto cachePath = ActiveSettings::GetCachePath("shaderCache/transferable/{:016x}_ppctraces.bin", m_titleId);

	auto fileData = FileStream::LoadIntoMemory(cachePath);
	if (!fileData || fileData->size() < sizeof(TraceCacheHeader))
	{
		cemuLog_log(LogType::Force, "[TraceCache] No existing cache found, starting fresh");
		return;
	}

	const uint8_t* data = fileData->data();
	const TraceCacheHeader* header = reinterpret_cast<const TraceCacheHeader*>(data);

	// Validate header
	if (header->magic != 0x54435050 || // 'PPCT' in little-endian
		header->titleId != m_titleId)
	{
		cemuLog_log(LogType::Force, "[TraceCache] Cache header invalid, starting fresh");
		return;
	}

	// Version check - we can load older versions
	if (header->version > TRACE_CACHE_VERSION)
	{
		cemuLog_log(LogType::Force, "[TraceCache] Cache version too new, starting fresh");
		return;
	}

	size_t expectedSize = sizeof(TraceCacheHeader) + header->entryCount * sizeof(TraceEntry);
	if (fileData->size() < expectedSize)
	{
		cemuLog_log(LogType::Force, "[TraceCache] Cache file truncated, starting fresh");
		return;
	}

	// Load entries into hash table
	const TraceEntry* entries = reinterpret_cast<const TraceEntry*>(data + sizeof(TraceCacheHeader));
	uint32_t loadedCount = 0;

	for (uint32_t i = 0; i < header->entryCount; i++)
	{
		const TraceEntry& entry = entries[i];
		HashEntry* hashEntry = FindOrInsert(entry.ppcAddress);
		if (hashEntry)
		{
			hashEntry->hitCount.store(entry.hitCount, std::memory_order_relaxed);
			hashEntry->lastSeen.store(entry.lastSeenTimestamp, std::memory_order_relaxed);
			loadedCount++;
		}
	}

	cemuLog_log(LogType::Force, "[TraceCache] Loaded {} entries from cache", loadedCount);
}

void PPCTraceCache::SaveToDisk()
{
	if (!m_dirty)
		return;

	std::vector<TraceEntry> entries;
	entries.reserve(MAX_CACHED_ENTRIES);

	// Collect entries from hash table
	for (size_t i = 0; i < HASH_TABLE_SIZE; i++)
	{
		uint32_t addr = m_hashTable[i].address.load(std::memory_order_relaxed);
		if (addr != EMPTY_SLOT)
		{
			uint32_t hitCount = m_hashTable[i].hitCount.load(std::memory_order_relaxed);
			if (hitCount > 0)
			{
				entries.push_back({
					addr,
					hitCount,
					m_hashTable[i].lastSeen.load(std::memory_order_relaxed)
				});
			}
		}
	}

	if (entries.empty())
		return;

	// Sort by hit count and keep top entries
	std::sort(entries.begin(), entries.end(), [](const TraceEntry& a, const TraceEntry& b) {
		return a.hitCount > b.hitCount;
	});

	if (entries.size() > MAX_CACHED_ENTRIES)
		entries.resize(MAX_CACHED_ENTRIES);

	// Write to disk
	auto cachePath = ActiveSettings::GetCachePath("shaderCache/transferable/{:016x}_ppctraces.bin", m_titleId);

	FileStream* fs = FileStream::createFile2(cachePath);
	if (!fs)
	{
		cemuLog_log(LogType::Force, "[TraceCache] Failed to create cache file");
		return;
	}

	TraceCacheHeader header;
	header.magic = 0x54435050; // 'PPCT'
	header.version = TRACE_CACHE_VERSION;
	header.titleId = m_titleId;
	header.entryCount = static_cast<uint32_t>(entries.size());
	header.reserved = 0;

	fs->writeData(&header, sizeof(header));
	fs->writeData(entries.data(), entries.size() * sizeof(TraceEntry));
	delete fs;

	m_dirty = false;
	cemuLog_log(LogType::Force, "[TraceCache] Saved {} entries to cache", entries.size());
}

void PPCTraceCache::SaveNow()
{
	std::lock_guard<std::mutex> lock(m_saveMutex);
	SaveToDisk();
}

void PPCTraceCache::SaveThreadFunc()
{
	SetThreadName("PPCTraceCache");

	while (!m_shutdownSignal)
	{
		std::unique_lock<std::mutex> lock(m_saveMutex);
		m_saveCondVar.wait_for(lock, std::chrono::milliseconds(SAVE_INTERVAL_MS));

		if (m_shutdownSignal)
			break;

		if (m_dirty)
		{
			SaveToDisk();
		}
	}
}
