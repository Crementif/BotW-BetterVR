#include "pch.h"

#include "utils/platform.h"

void Platform::FailFast() {
    RaiseFailFastException(nullptr, nullptr, 0);
    std::abort();
}

std::filesystem::path Platform::GetExecutableDirectory() {
    wchar_t path[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameW(nullptr, path, (DWORD)std::size(path));
    return std::filesystem::path(std::wstring_view(path, length)).parent_path();
}

bool Platform::OpenUrl(const char* url) {
    const HRESULT comResult = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    const INT_PTR shellResult = (INT_PTR)ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
    if (SUCCEEDED(comResult)) {
        CoUninitialize();
    }
    return shellResult > 32;
}

Platform::SharedMemoryBlock::SharedMemoryBlock(const char* name, size_t size) {
    HANDLE mapping = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, (DWORD)size, name);
    if (mapping == nullptr) {
        Log::print<WARNING>("Failed to create the shared memory section '{}' (Win32 error {})", name, GetLastError());
        return;
    }

    void* view = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, size);
    if (view == nullptr) {
        Log::print<WARNING>("Failed to map the shared memory section '{}' (Win32 error {})", name, GetLastError());
        CloseHandle(mapping);
        return;
    }

    m_view = view;
    m_handle = (uintptr_t)mapping;
    m_size = size;
}

void Platform::SharedMemoryBlock::Reset() {
    if (m_view != nullptr) {
        UnmapViewOfFile(m_view);
        m_view = nullptr;
    }
    if (m_handle != 0) {
        CloseHandle((HANDLE)m_handle);
        m_handle = 0;
    }
    m_size = 0;
}
