#include "pch.h"

#include "utils/platform.h"

#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

// Win32 section names carry a namespace prefix and backslashes, POSIX wants a single leading slash
static std::string ToShmName(const char* name) {
    std::string_view view(name);
    if (const size_t separator = view.find_last_of("\\/"); separator != std::string_view::npos) {
        view.remove_prefix(separator + 1);
    }
    return "/" + std::string(view);
}

void Platform::FailFast() {
    std::abort();
}

std::filesystem::path Platform::GetExecutableDirectory() {
    std::error_code error;
    return std::filesystem::read_symlink("/proc/self/exe", error).parent_path();
}

bool Platform::OpenUrl(const char* url) {
    pid_t pid = 0;
    char* const argv[] = { (char*)"xdg-open", (char*)url, nullptr };
    if (posix_spawnp(&pid, "xdg-open", nullptr, nullptr, argv, environ) != 0) {
        return false;
    }
    int status = 0;
    return waitpid(pid, &status, 0) == pid && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

Platform::SharedMemoryBlock::SharedMemoryBlock(const char* name, size_t size) {
    const std::string shmName = ToShmName(name);

    const int fd = shm_open(shmName.c_str(), O_CREAT | O_RDWR, 0600);
    if (fd < 0) {
        Log::print<WARNING>("Failed to create the shared memory section '{}' (errno {})", shmName.c_str(), errno);
        return;
    }

    struct stat info = {};
    if (fstat(fd, &info) == 0 && (size_t)info.st_size < size && ftruncate(fd, (off_t)size) != 0) {
        Log::print<WARNING>("Failed to size the shared memory section '{}' (errno {})", shmName.c_str(), errno);
        close(fd);
        return;
    }

    void* view = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (view == MAP_FAILED) {
        Log::print<WARNING>("Failed to map the shared memory section '{}' (errno {})", shmName.c_str(), errno);
        return;
    }

    m_view = view;
    m_size = size;
}

void Platform::SharedMemoryBlock::Reset() {
    if (m_view != nullptr) {
        munmap(m_view, m_size);
        m_view = nullptr;
    }
    m_size = 0;
}
