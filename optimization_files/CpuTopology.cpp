#include "CpuTopology.h"
#include "Common/precompiled.h"

#if BOOST_OS_WINDOWS
#include <Windows.h>
#endif

#if BOOST_OS_LINUX
#include <sched.h>
#include <fstream>
#include <sstream>
#endif

CpuTopology& CpuTopology::GetInstance()
{
	static CpuTopology instance;
	return instance;
}

void CpuTopology::Initialize()
{
	if (m_initialized)
		return;
	m_initialized = true;
	DetectTopology();
}

#if BOOST_OS_WINDOWS
void CpuTopology::DetectTopology()
{
	// Get logical processor count
	SYSTEM_INFO sysInfo;
	GetSystemInfo(&sysInfo);
	m_logicalProcessorCount = sysInfo.dwNumberOfProcessors;

	// Use GetLogicalProcessorInformationEx to detect hybrid CPU topology
	DWORD bufferSize = 0;
	GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &bufferSize);

	if (bufferSize == 0)
	{
		// Fallback: no hybrid detection, treat all cores as equal
		m_physicalCoreCount = m_logicalProcessorCount;
		for (uint32_t i = 0; i < m_logicalProcessorCount; i++)
		{
			m_coresByPerformance.push_back(i);
		}
		return;
	}

	std::vector<uint8_t> buffer(bufferSize);
	if (!GetLogicalProcessorInformationEx(RelationProcessorCore,
		reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(buffer.data()), &bufferSize))
	{
		// Fallback
		m_physicalCoreCount = m_logicalProcessorCount;
		for (uint32_t i = 0; i < m_logicalProcessorCount; i++)
		{
			m_coresByPerformance.push_back(i);
		}
		return;
	}

	// Parse the processor information
	uint8_t* ptr = buffer.data();
	uint8_t* end = ptr + bufferSize;
	uint32_t physicalCoreIndex = 0;

	while (ptr < end)
	{
		auto* info = reinterpret_cast<PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX>(ptr);

		if (info->Relationship == RelationProcessorCore)
		{
			// Extract efficiency class (0 = E-core, 1 = P-core on Intel 12th gen+)
			uint8_t efficiencyClass = info->Processor.EfficiencyClass;

			// Get the processor mask for this physical core
			for (WORD groupIndex = 0; groupIndex < info->Processor.GroupCount; groupIndex++)
			{
				KAFFINITY mask = info->Processor.GroupMask[groupIndex].Mask;

				// Find all logical processors in this mask
				for (uint32_t bit = 0; bit < 64; bit++)
				{
					if (mask & (1ULL << bit))
					{
						uint32_t logicalIndex = bit + (groupIndex * 64);

						CpuCoreInfo coreInfo;
						coreInfo.logicalProcessorIndex = logicalIndex;
						coreInfo.physicalCoreIndex = physicalCoreIndex;
						coreInfo.efficiencyClass = efficiencyClass;
						coreInfo.isPerformanceCore = (efficiencyClass > 0);
						m_coreInfo.push_back(coreInfo);

						if (efficiencyClass > 0)
							m_performanceCores.push_back(logicalIndex);
						else
							m_efficiencyCores.push_back(logicalIndex);
					}
				}
			}
			physicalCoreIndex++;
		}

		ptr += info->Size;
	}

	m_physicalCoreCount = physicalCoreIndex;

	// Check if this is a hybrid CPU (has both P and E cores)
	m_isHybrid = !m_performanceCores.empty() && !m_efficiencyCores.empty();

	// Build cores by performance list (P-cores first)
	m_coresByPerformance.clear();
	for (uint32_t core : m_performanceCores)
		m_coresByPerformance.push_back(core);
	for (uint32_t core : m_efficiencyCores)
		m_coresByPerformance.push_back(core);

	// If not hybrid, just list all cores
	if (m_coresByPerformance.empty())
	{
		for (uint32_t i = 0; i < m_logicalProcessorCount; i++)
			m_coresByPerformance.push_back(i);
	}
}

bool CpuTopology::SetThreadAffinity(std::thread& thread, uint64_t affinityMask)
{
	return SetThreadAffinity(thread.native_handle(), affinityMask);
}

bool CpuTopology::SetThreadAffinity(std::thread::native_handle_type handle, uint64_t affinityMask)
{
	DWORD_PTR result = SetThreadAffinityMask(static_cast<HANDLE>(handle), static_cast<DWORD_PTR>(affinityMask));
	return result != 0;
}

bool CpuTopology::SetCurrentThreadAffinity(uint64_t affinityMask)
{
	DWORD_PTR result = SetThreadAffinityMask(GetCurrentThread(), static_cast<DWORD_PTR>(affinityMask));
	return result != 0;
}

bool CpuTopology::SetThreadAffinityToCore(std::thread& thread, uint32_t logicalProcessorIndex)
{
	return SetThreadAffinity(thread, 1ULL << logicalProcessorIndex);
}

bool CpuTopology::SetThreadAffinityToCore(std::thread::native_handle_type handle, uint32_t logicalProcessorIndex)
{
	return SetThreadAffinity(handle, 1ULL << logicalProcessorIndex);
}

#elif BOOST_OS_LINUX

void CpuTopology::DetectTopology()
{
	// Get logical processor count
	m_logicalProcessorCount = std::thread::hardware_concurrency();

	// Try to read CPU topology from sysfs
	// Check for hybrid CPU by looking at cpu_capacity or core_cpus_list

	std::set<uint32_t> physicalCores;

	for (uint32_t cpu = 0; cpu < m_logicalProcessorCount; cpu++)
	{
		CpuCoreInfo info;
		info.logicalProcessorIndex = cpu;
		info.efficiencyClass = 1; // Default to P-core
		info.isPerformanceCore = true;

		// Try to read physical core ID
		std::string corePath = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/core_id";
		std::ifstream coreFile(corePath);
		if (coreFile.is_open())
		{
			coreFile >> info.physicalCoreIndex;
			physicalCores.insert(info.physicalCoreIndex);
		}

		// Try to read CPU capacity (for big.LITTLE detection)
		std::string capacityPath = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/cpu_capacity";
		std::ifstream capacityFile(capacityPath);
		if (capacityFile.is_open())
		{
			uint32_t capacity;
			capacityFile >> capacity;
			// Lower capacity = efficiency core
			if (capacity < 900) // Threshold for E-cores
			{
				info.efficiencyClass = 0;
				info.isPerformanceCore = false;
			}
		}

		m_coreInfo.push_back(info);

		if (info.isPerformanceCore)
			m_performanceCores.push_back(cpu);
		else
			m_efficiencyCores.push_back(cpu);
	}

	m_physicalCoreCount = physicalCores.size();
	if (m_physicalCoreCount == 0)
		m_physicalCoreCount = m_logicalProcessorCount;

	m_isHybrid = !m_performanceCores.empty() && !m_efficiencyCores.empty();

	// Build cores by performance
	for (uint32_t core : m_performanceCores)
		m_coresByPerformance.push_back(core);
	for (uint32_t core : m_efficiencyCores)
		m_coresByPerformance.push_back(core);

	if (m_coresByPerformance.empty())
	{
		for (uint32_t i = 0; i < m_logicalProcessorCount; i++)
			m_coresByPerformance.push_back(i);
	}
}

bool CpuTopology::SetThreadAffinity(std::thread& thread, uint64_t affinityMask)
{
	return SetThreadAffinity(thread.native_handle(), affinityMask);
}

bool CpuTopology::SetThreadAffinity(std::thread::native_handle_type handle, uint64_t affinityMask)
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	for (int i = 0; i < 64; i++)
	{
		if (affinityMask & (1ULL << i))
			CPU_SET(i, &cpuset);
	}
	return pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset) == 0;
}

bool CpuTopology::SetCurrentThreadAffinity(uint64_t affinityMask)
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	for (int i = 0; i < 64; i++)
	{
		if (affinityMask & (1ULL << i))
			CPU_SET(i, &cpuset);
	}
	return sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == 0;
}

bool CpuTopology::SetThreadAffinityToCore(std::thread& thread, uint32_t logicalProcessorIndex)
{
	return SetThreadAffinity(thread, 1ULL << logicalProcessorIndex);
}

bool CpuTopology::SetThreadAffinityToCore(std::thread::native_handle_type handle, uint32_t logicalProcessorIndex)
{
	return SetThreadAffinity(handle, 1ULL << logicalProcessorIndex);
}

#else
// macOS and other platforms - basic fallback

void CpuTopology::DetectTopology()
{
	m_logicalProcessorCount = std::thread::hardware_concurrency();
	m_physicalCoreCount = m_logicalProcessorCount;
	m_isHybrid = false;

	for (uint32_t i = 0; i < m_logicalProcessorCount; i++)
		m_coresByPerformance.push_back(i);
}

bool CpuTopology::SetThreadAffinity(std::thread& thread, uint64_t affinityMask) { return false; }
bool CpuTopology::SetThreadAffinity(std::thread::native_handle_type handle, uint64_t affinityMask) { return false; }
bool CpuTopology::SetCurrentThreadAffinity(uint64_t affinityMask) { return false; }
bool CpuTopology::SetThreadAffinityToCore(std::thread& thread, uint32_t logicalProcessorIndex) { return false; }
bool CpuTopology::SetThreadAffinityToCore(std::thread::native_handle_type handle, uint32_t logicalProcessorIndex) { return false; }

#endif

uint64_t CpuTopology::BuildAffinityMask(const std::vector<uint32_t>& cores)
{
	uint64_t mask = 0;
	for (uint32_t core : cores)
	{
		if (core < 64)
			mask |= (1ULL << core);
	}
	return mask;
}

std::string CpuTopology::GetTopologyDescription() const
{
	std::string desc;
	if (m_isHybrid)
	{
		desc = fmt::format("Hybrid CPU: {} P-cores, {} E-cores ({} logical processors)",
			m_performanceCores.size(), m_efficiencyCores.size(), m_logicalProcessorCount);
	}
	else
	{
		desc = fmt::format("CPU: {} physical cores, {} logical processors",
			m_physicalCoreCount, m_logicalProcessorCount);
	}
	return desc;
}
