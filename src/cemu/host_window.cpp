#include "pch.h"

#include "cemu/host_window.h"

#if defined(_WIN32)

static HWND s_topWindow = NULL;
static HWND s_renderWindow = NULL;

void HostWindow::Discover() {
    struct EnumWindowsData {
        DWORD cemuPid;
        HWND outHwnd;
    } enumData = { .cemuPid = GetCurrentProcessId(), .outHwnd = NULL };

    EnumWindows([](HWND iteratedHwnd, LPARAM data) -> BOOL {
        EnumWindowsData* enumData = (EnumWindowsData*)data;
        DWORD currPid;
        GetWindowThreadProcessId(iteratedHwnd, &currPid);
        if (currPid == enumData->cemuPid) {
            constexpr size_t bufSize = 256;
            wchar_t buf[bufSize];
            GetWindowTextW(iteratedHwnd, buf, bufSize);
            if (wcsstr(buf, kTitleMatch) != nullptr) {
                enumData->outHwnd = iteratedHwnd;
                return FALSE;
            }
        }
        return TRUE;
    },
    (LPARAM)&enumData);
    s_topWindow = enumData.outHwnd;

    // the most nested child window is the rendering window
    HWND iteratedHwnd = s_topWindow;
    while (true) {
        HWND nextIteratedHwnd = FindWindowExW(iteratedHwnd, NULL, NULL, NULL);
        if (nextIteratedHwnd == NULL) {
            break;
        }
        iteratedHwnd = nextIteratedHwnd;
    }
    s_renderWindow = iteratedHwnd;
}

HostWindow::Pointer HostWindow::QueryPointer() {
    Pointer state;
    if (s_renderWindow == NULL) {
        return state;
    }
    state.available = true;

    POINT cursor = {};
    GetCursorPos(&cursor);
    ScreenToClient(s_renderWindow, &cursor);
    state.cursorX = cursor.x;
    state.cursorY = cursor.y;

    RECT rect = {};
    GetClientRect(s_renderWindow, &rect);
    state.clientWidth = (uint32_t)(rect.right - rect.left);
    state.clientHeight = (uint32_t)(rect.bottom - rect.top);

    state.focused = s_topWindow == GetForegroundWindow();
    if (state.focused) {
        state.mouseDown[0] = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
        state.mouseDown[1] = (GetAsyncKeyState(VK_RBUTTON) & 0x8000) != 0;
        state.mouseDown[2] = (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0;
    }
    return state;
}

void HostWindow::RequestShutdown() {
    PostMessageW(s_topWindow, WM_CLOSE, 0, 0);
}

#else

void HostWindow::Discover() {
}

HostWindow::Pointer HostWindow::QueryPointer() {
    return {};
}

void HostWindow::RequestShutdown() {
    Log::print<WARNING>("The OpenXR runtime asked the session to exit, but closing Cemu's window is not implemented on this platform.");
}

#endif
