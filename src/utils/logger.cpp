#include "pch.h"

#include "logger.h"
#include "bettervr_version.h"

#if !defined(_WIN32)
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#endif

static constexpr size_t kMaxQueuedRecords = 4096;
static constexpr uint32_t kUncleanShutdownTimeoutMs = 2000;
static constexpr uint32_t kFlushTimeoutMs = 500;
static constexpr uint32_t kJoinForever = 0xFFFFFFFFu;

struct LogRecord {
    std::string text;
    LogType type = INFO;
    uint32_t threadId = 0;
    uint64_t time = 0;
};

struct WallClock {
    uint32_t hour = 0;
    uint32_t minute = 0;
    uint32_t second = 0;
    uint32_t millisecond = 0;
};

// appends have to stay atomic since test harnesses share one log file between processes
static void OpenLogFile();
static void CloseLogFile();
static bool IsLogFileOpen();
static void WriteToLogFile(const char* data, size_t size);
static void OpenConsole();
static void CloseConsole();
static void WriteToConsole(const char* data, size_t size);
static void WriteToDebugger(const char* text);
static uint64_t GetWallClockTicks();
static bool DecodeWallClock(uint64_t ticks, WallClock& out);
static uint32_t CurrentThreadId();
static uint32_t CurrentProcessId();
static bool JoinWriterThread(std::thread& thread, uint32_t timeoutMs);
static void LogSystemHardwareInfo();

static constinit std::atomic_bool s_writerExited = true;
// relative to the working directory until the host names Cemu's user data folder
static std::filesystem::path s_filePath = "BetterVR_log.txt";

#if defined(_WIN32)

// FILE_APPEND_DATA without FILE_WRITE_DATA makes each write an atomic append
static constexpr DWORD kLogFileAccess = FILE_APPEND_DATA;
static constexpr DWORD kLogFileShare = FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;

static HANDLE s_fileHandle = INVALID_HANDLE_VALUE;
static HANDLE s_consoleHandle = NULL;

static void OpenLogFile() {
    s_fileHandle = CreateFileW(s_filePath.c_str(), kLogFileAccess, kLogFileShare, nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
}

static void CloseLogFile() {
    if (s_fileHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(s_fileHandle);
        s_fileHandle = INVALID_HANDLE_VALUE;
    }
}

static bool IsLogFileOpen() {
    return s_fileHandle != INVALID_HANDLE_VALUE;
}

static void WriteToLogFile(const char* data, size_t size) {
    DWORD bytesWritten = 0;
    WriteFile(s_fileHandle, data, (DWORD)size, &bytesWritten, nullptr);
}

static void OpenConsole() {
    AllocConsole();
    SetConsoleTitleA("BetterVR Debugging Console");
    s_consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
}

static void CloseConsole() {
    s_consoleHandle = NULL;
    FreeConsole();
}

static void WriteToConsole(const char* data, size_t size) {
    if (s_consoleHandle == NULL || s_consoleHandle == INVALID_HANDLE_VALUE) {
        return;
    }
    DWORD charsWritten = 0;
    WriteConsoleA(s_consoleHandle, data, (DWORD)size, &charsWritten, NULL);
}

static void WriteToDebugger(const char* text) {
    OutputDebugStringA(text);
}

static uint64_t GetWallClockTicks() {
    FILETIME systemTime;
    GetSystemTimeAsFileTime(&systemTime);
    return ((uint64_t)systemTime.dwHighDateTime << 32) | systemTime.dwLowDateTime;
}

static bool DecodeWallClock(uint64_t ticks, WallClock& out) {
    FILETIME utcTime;
    utcTime.dwLowDateTime = (DWORD)(ticks & 0xFFFFFFFFull);
    utcTime.dwHighDateTime = (DWORD)(ticks >> 32);

    FILETIME localFileTime;
    SYSTEMTIME localTime;
    if (!FileTimeToLocalFileTime(&utcTime, &localFileTime) || !FileTimeToSystemTime(&localFileTime, &localTime)) {
        return false;
    }

    out = { localTime.wHour, localTime.wMinute, localTime.wSecond, localTime.wMilliseconds };
    return true;
}

static uint32_t CurrentThreadId() {
    return GetCurrentThreadId();
}

static uint32_t CurrentProcessId() {
    return GetCurrentProcessId();
}

static bool JoinWriterThread(std::thread& thread, uint32_t timeoutMs) {
    // a timeout means we're holding the loader lock, so detach instead of blocking forever
    if (WaitForSingleObject(thread.native_handle(), timeoutMs) == WAIT_OBJECT_0) {
        thread.join();
        return true;
    }
    thread.detach();
    return false;
}

static void LogSystemHardwareInfo() {
    int cpuInfo[4] = {0, 0, 0, 0};
    __cpuid(cpuInfo, 0x80000000);
    const unsigned int maxExId = static_cast<unsigned int>(cpuInfo[0]);

    std::string cpuBrand;
    if (maxExId >= 0x80000004) {
        char brand[49] = {};
        __cpuid(reinterpret_cast<int*>(brand + 0), 0x80000002);
        __cpuid(reinterpret_cast<int*>(brand + 16), 0x80000003);
        __cpuid(reinterpret_cast<int*>(brand + 32), 0x80000004);
        cpuBrand = brand;
        while (!cpuBrand.empty() && cpuBrand.front() == ' ') cpuBrand.erase(cpuBrand.begin());
        while (!cpuBrand.empty() && cpuBrand.back() == ' ') cpuBrand.pop_back();
        if (!cpuBrand.empty()) {
            Log::print<INFO>("CPU: {}", cpuBrand.c_str());
        }
    }

    MEMORYSTATUSEX statex{};
    statex.dwLength = sizeof(statex);
    if (GlobalMemoryStatusEx(&statex)) {
        const double totalGiB = double(statex.ullTotalPhys) / (1024.0 * 1024.0 * 1024.0);
        Log::print<INFO>("RAM: {:.2f} GiB", totalGiB);
    }
}

#else

static int s_fileDescriptor = -1;
static int s_consoleDescriptor = -1;

static void OpenLogFile() {
    s_fileDescriptor = open(s_filePath.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
}

static void CloseLogFile() {
    if (s_fileDescriptor >= 0) {
        close(s_fileDescriptor);
        s_fileDescriptor = -1;
    }
}

static bool IsLogFileOpen() {
    return s_fileDescriptor >= 0;
}

static void WriteToLogFile(const char* data, size_t size) {
    while (size > 0) {
        const ssize_t written = write(s_fileDescriptor, data, size);
        if (written <= 0) {
            break;
        }
        data += written;
        size -= (size_t)written;
    }
}

static void OpenConsole() {
    s_consoleDescriptor = STDERR_FILENO;
}

static void CloseConsole() {
    s_consoleDescriptor = -1;
}

static void WriteToConsole(const char* data, size_t size) {
    if (s_consoleDescriptor < 0) {
        return;
    }
    while (size > 0) {
        const ssize_t written = write(s_consoleDescriptor, data, size);
        if (written <= 0) {
            break;
        }
        data += written;
        size -= (size_t)written;
    }
}

static void WriteToDebugger(const char*) {
}

static uint64_t GetWallClockTicks() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static bool DecodeWallClock(uint64_t ticks, WallClock& out) {
    const time_t seconds = (time_t)(ticks / 1000);
    tm localTime = {};
    if (localtime_r(&seconds, &localTime) == nullptr) {
        return false;
    }
    out = { (uint32_t)localTime.tm_hour, (uint32_t)localTime.tm_min, (uint32_t)localTime.tm_sec, (uint32_t)(ticks % 1000) };
    return true;
}

static uint32_t CurrentThreadId() {
    return (uint32_t)gettid();
}

static uint32_t CurrentProcessId() {
    return (uint32_t)getpid();
}

static bool JoinWriterThread(std::thread& thread, uint32_t timeoutMs) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (!s_writerExited.load(std::memory_order_acquire)) {
        if (std::chrono::steady_clock::now() >= deadline) {
            thread.detach();
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    thread.join();
    return true;
}

static void LogSystemHardwareInfo() {
    std::ifstream cpuInfo("/proc/cpuinfo");
    for (std::string line; std::getline(cpuInfo, line);) {
        if (!line.starts_with("model name")) {
            continue;
        }
        const size_t separator = line.find(':');
        if (separator == std::string::npos) {
            continue;
        }
        std::string cpuBrand = line.substr(separator + 1);
        while (!cpuBrand.empty() && cpuBrand.front() == ' ') cpuBrand.erase(cpuBrand.begin());
        while (!cpuBrand.empty() && cpuBrand.back() == ' ') cpuBrand.pop_back();
        if (!cpuBrand.empty()) {
            Log::print<INFO>("CPU: {}", cpuBrand.c_str());
        }
        break;
    }

    const long pageCount = sysconf(_SC_PHYS_PAGES);
    const long pageSize = sysconf(_SC_PAGE_SIZE);
    if (pageCount > 0 && pageSize > 0) {
        Log::print<INFO>("RAM: {:.2f} GiB", double(pageCount) * double(pageSize) / (1024.0 * 1024.0 * 1024.0));
    }
}

#endif

// Never destructed because a detached writer thread (see StopWriterThread) may still touch these
static std::mutex& s_queueMutex = *new std::mutex();
static std::condition_variable& s_wakeWriter = *new std::condition_variable();
static std::condition_variable& s_drained = *new std::condition_variable();
static std::thread s_writerThread;
static std::atomic_bool s_running = false;
static bool s_writerBusy = false;
static constinit std::atomic_bool s_consoleWanted = false;
static bool s_consoleOpen = false;

// only the writer thread opens or closes the console, so a write never races a closing console
static void SyncConsole() {
    const bool wanted = s_consoleWanted.load(std::memory_order_relaxed);
    if (wanted == s_consoleOpen) {
        return;
    }
    s_consoleOpen = wanted;
    if (wanted) {
        OpenConsole();
    }
    else {
        CloseConsole();
    }
}

// Reusing s_queueMutex here would self-deadlock with Start/StopWriterThread
static std::mutex s_transitionMutex;
static size_t s_instanceRefCount = 0;

// Sticky: once a writer is detached, never start another one
static constinit std::atomic_bool s_writerDetached = false;

static std::vector<LogRecord>& s_produce = *new std::vector<LogRecord>();
static std::vector<LogRecord>& s_consume = *new std::vector<LogRecord>();
static size_t s_produceCount = 0;
static uint32_t s_droppedCount = 0;

static std::string& s_batchBuffer = *new std::string();
static std::string& s_directBuffer = *new std::string();
static uint32_t s_processId = CurrentProcessId();

static constinit std::atomic_bool s_showTimestamps = true;
static constinit std::atomic_bool s_showThreadIds = false;

static const char* GetLogTypeName(LogType type) {
    switch (type) {
        case RENDERING: return "RENDER";
        case INTEROP: return "INTEROP";
        case CONTROLS: return "CONTROL";
        case PPC: return "PPC";
        case XR_DEBUGUTILS: return "XR";
        case ARROW_SHOT_CAPTURE: return "ARROW";
        case ROOMSCALE: return "ROOMSCALE";
        case INFO: return "INFO";
        case WARNING: return "WARN";
        case ERROR: return "ERROR";
        case VERBOSE: return "VERBOSE";
    }
    return "?";
}

static void FillRecord(LogRecord& record, LogType type, std::string_view message) {
    record.text.assign(message);
    record.type = type;
    record.threadId = CurrentThreadId();
    record.time = GetWallClockTicks();
}

static void AppendRecord(std::string& batch, const LogRecord& record) {
    if (s_showTimestamps.load(std::memory_order_relaxed)) {
        if (WallClock localTime; DecodeWallClock(record.time, localTime)) {
            fmt::format_to(std::back_inserter(batch), "[{:02}:{:02}:{:02}.{:03}] ", localTime.hour, localTime.minute, localTime.second, localTime.millisecond);
        }
    }

    if (s_showThreadIds.load(std::memory_order_relaxed)) {
        fmt::format_to(std::back_inserter(batch), "[{}:{}] ", s_processId, record.threadId);
    }

    fmt::format_to(std::back_inserter(batch), "[{:<7}] ", GetLogTypeName(record.type));
    batch.append(record.text);
    batch.append("\n");
}

static void WriteBatch(const std::string& batch) {
    if (batch.empty()) {
        return;
    }

    if (IsLogFileOpen()) {
        WriteToLogFile(batch.data(), batch.size());
    }

    WriteToConsole(batch.data(), batch.size());

#ifdef _DEBUG
    WriteToDebugger(batch.c_str());
#endif
}

static void DrainQueue() {
    SyncConsole();
    while (true) {
        size_t count = 0;
        uint32_t dropped = 0;
        {
            std::lock_guard<std::mutex> lock(s_queueMutex);
            if (s_produceCount == 0) {
                s_writerBusy = false;
                break;
            }

            s_produce.swap(s_consume);
            count = s_produceCount;
            s_produceCount = 0;
            dropped = s_droppedCount;
            s_droppedCount = 0;
            s_writerBusy = true;
        }

        s_batchBuffer.clear();
        for (size_t index = 0; index < count; ++index) {
            AppendRecord(s_batchBuffer, s_consume[index]);
        }

        if (dropped > 0) {
            LogRecord droppedRecord;
            FillRecord(droppedRecord, WARNING, fmt::format("... {} log messages dropped, queue was full", dropped));
            AppendRecord(s_batchBuffer, droppedRecord);
        }

        WriteBatch(s_batchBuffer);
    }

    s_drained.notify_all();
}

static void WriterThreadMain() {
    while (s_running.load(std::memory_order_relaxed)) {
        {
            std::unique_lock<std::mutex> lock(s_queueMutex);
            s_wakeWriter.wait(lock, [] { return s_produceCount > 0 || !s_running.load(std::memory_order_relaxed) || s_consoleWanted.load(std::memory_order_relaxed) != s_consoleOpen; });
        }
        DrainQueue();
    }

    DrainQueue();
    s_writerExited.store(true, std::memory_order_release);
}

static void FlushBlocking(uint32_t timeoutMs) {
    std::unique_lock<std::mutex> lock(s_queueMutex);
    if (!s_running.load(std::memory_order_relaxed)) {
        return;
    }
    s_drained.wait_for(lock, std::chrono::milliseconds(timeoutMs), [] { return s_produceCount == 0 && !s_writerBusy; });
}

static void StartWriterThread() {
    std::lock_guard<std::mutex> lock(s_queueMutex);
    if (s_running.load(std::memory_order_relaxed)) {
        return;
    }

    s_produce.reserve(512);
    s_consume.reserve(512);
    s_running.store(true, std::memory_order_relaxed);
    s_writerExited.store(false, std::memory_order_release);
    s_writerThread = std::thread(WriterThreadMain);
}

static bool StopWriterThread(uint32_t timeoutMs) {
    std::thread writerThread;
    {
        std::scoped_lock lock(s_queueMutex);
        if (!s_running.load(std::memory_order_relaxed)) {
            return true;
        }
        s_running.store(false, std::memory_order_relaxed);
        writerThread = std::move(s_writerThread);
    }

    s_wakeWriter.notify_all();

    if (!writerThread.joinable()) {
        return true;
    }

    if (timeoutMs == kJoinForever) {
        writerThread.join();
        return true;
    }

    return JoinWriterThread(writerThread, timeoutMs);
}


static bool ShutdownLogging(uint32_t timeoutMs) {
    if (!s_running.load(std::memory_order_relaxed)) {
        return true;
    }

    Log::print<INFO>("Pausing BetterVR log writer (no active Vulkan instances)...");

    if (!StopWriterThread(timeoutMs)) {
        s_writerDetached.store(true, std::memory_order_relaxed);
        return false;
    }

    DrainQueue();

    {
        std::scoped_lock lock(s_queueMutex);
        CloseLogFile();
    }

    return true;
}

void Log::submit(LogType type, std::string_view message) {
    {
        std::scoped_lock lock(s_queueMutex);

        if (!s_running.load(std::memory_order_relaxed)) {
            // opening for append is idempotent, so reopening is safe to redo on every call
            if (!IsLogFileOpen()) {
                OpenLogFile();
            }

            LogRecord record;
            FillRecord(record, type, message);

            s_directBuffer.clear();
            AppendRecord(s_directBuffer, record);
            WriteBatch(s_directBuffer);
            return;
        }

        if (s_produceCount >= kMaxQueuedRecords) {
            s_droppedCount++;
            return;
        }

        if (s_produceCount >= s_produce.size()) {
            s_produce.emplace_back();
        }

        FillRecord(s_produce[s_produceCount++], type, message);
    }

    s_wakeWriter.notify_one();
}

void Log::SetFilePath(const std::filesystem::path& path) {
    std::scoped_lock lock(s_queueMutex);
    s_filePath = path;
}

Log::Log() {
    OpenLogFile();
    StartWriterThread();

    Log::print<INFO>("Successfully started BetterVR {}!", BETTERVR_VERSION_STRING);
    LogSystemHardwareInfo();
}

Log::~Log() {
    const bool cleanShutdown = ShutdownLogging(kUncleanShutdownTimeoutMs);
    if (!cleanShutdown || s_writerDetached.load(std::memory_order_relaxed)) {
        return;
    }

    {
        std::scoped_lock lock(s_queueMutex);
        CloseLogFile();
    }

    if (s_consoleOpen) {
        s_consoleOpen = false;
        CloseConsole();
    }
}

void Log::OnInstanceCreated() {
    std::scoped_lock lock(s_transitionMutex);
    if (s_instanceRefCount++ != 0) {
        return;
    }

    if (s_writerDetached.load(std::memory_order_relaxed)) {
        return;
    }

    {
        std::scoped_lock lock(s_queueMutex);
        if (!IsLogFileOpen()) {
            OpenLogFile();
        }
    }
    StartWriterThread();
}

void Log::OnInstanceDestroyed() {
    std::scoped_lock lock(s_transitionMutex);
    if (s_instanceRefCount == 0) {
        return;
    }
    if (--s_instanceRefCount != 0) {
        return;
    }

    ShutdownLogging(kUncleanShutdownTimeoutMs);
}

void Log::SetShowTimestamps(bool enabled) {
    s_showTimestamps.store(enabled, std::memory_order_relaxed);
}

void Log::SetShowThreadIds(bool enabled) {
    s_showThreadIds.store(enabled, std::memory_order_relaxed);
}

void Log::SetConsoleVisible(bool visible) {
    s_consoleWanted.store(visible, std::memory_order_relaxed);
    if (s_running.load(std::memory_order_relaxed)) {
        std::lock_guard<std::mutex> lock(s_queueMutex);
        s_wakeWriter.notify_one();
    }
}

void Log::Flush() {
    FlushBlocking(kFlushTimeoutMs);
}

void Log::printTimeElapsed(const char* message_prefix, uint64_t startTimeStamp) {
    Log::print<INFO>("{}: {} ms", message_prefix, (double)(GetTimeStamp() - startTimeStamp) / 1000000.0);
}
