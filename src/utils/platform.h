#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <utility>

#if defined(_WIN32)
#define BETTERVR_DEBUG_BREAK() __debugbreak()
#else
#define BETTERVR_DEBUG_BREAK() __builtin_trap()
#endif

namespace Platform {
    // implemented by the host (host_module.cpp)
    void ShowFatalError(const char* title, const char* message);
    [[noreturn]] void FailFast();

    std::filesystem::path GetExecutableDirectory();
    // in the default browser, blocking until the system has taken the link
    bool OpenUrl(const char* url);

    class SharedMemoryBlock {
    public:
        SharedMemoryBlock() = default;
        SharedMemoryBlock(const char* name, size_t size);
        ~SharedMemoryBlock() { Reset(); }

        SharedMemoryBlock(SharedMemoryBlock&& other) noexcept { Adopt(other); }
        SharedMemoryBlock& operator=(SharedMemoryBlock&& other) noexcept {
            if (this != &other) {
                Reset();
                Adopt(other);
            }
            return *this;
        }

        SharedMemoryBlock(const SharedMemoryBlock&) = delete;
        SharedMemoryBlock& operator=(const SharedMemoryBlock&) = delete;

        explicit operator bool() const { return m_view != nullptr; }
        void* View() const { return m_view; }
        void Reset();

    private:
        void Adopt(SharedMemoryBlock& other) {
            m_view = std::exchange(other.m_view, nullptr);
            m_handle = std::exchange(other.m_handle, uintptr_t{ 0 });
            m_size = std::exchange(other.m_size, size_t{ 0 });
        }

        void* m_view = nullptr;
        uintptr_t m_handle = 0;
        size_t m_size = 0;
    };
}
