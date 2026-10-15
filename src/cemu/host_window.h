#pragma once

#include <cstdint>

namespace HostWindow {
    inline constexpr wchar_t kTitleMatch[] = L"Cemu";

    struct Pointer {
        bool available = false;
        bool focused = false;
        int32_t cursorX = 0;
        int32_t cursorY = 0;
        uint32_t clientWidth = 0;
        uint32_t clientHeight = 0;
        bool mouseDown[3] = {};
    };

    void Discover();
    Pointer QueryPointer();
    void RequestShutdown();
}
