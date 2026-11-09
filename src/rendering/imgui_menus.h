#pragma once

#include "rendering/openxr.h"

struct ImFont;

namespace ImGuiMenus {
    // Pages of the mod menu, in the order they appear in the sidebar and cycle with the triggers/shoulder buttons
    inline constexpr uint8_t PLAYSTYLE_PAGE = 0;
    inline constexpr uint8_t COMFORT_PAGE = 1;
    inline constexpr uint8_t CONTROLS_PAGE = 2;
    inline constexpr uint8_t DISPLAY_PAGE = 3;
    inline constexpr uint8_t SYSTEM_PAGE = 4;
    inline constexpr uint8_t GUIDE_CONTROLS_PAGE = 5;
    inline constexpr uint8_t GUIDE_EQUIPMENT_PAGE = 6;
    inline constexpr uint8_t GUIDE_SWING_PAGE = 7;
    inline constexpr uint8_t GUIDE_WHISTLE_PAGE = 8;
    inline constexpr uint8_t CREDITS_PAGE = 9;
    inline constexpr uint8_t DEBUG_PAGE = 10;
    inline constexpr uint8_t PAGE_COUNT = 11;

    inline std::atomic_bool g_isOpen = false;
    inline std::atomic_uint8_t g_currentPage = PLAYSTYLE_PAGE;

    bool IsPageAvailable(uint8_t page);

    // false until the overlay exists
    bool ProcessInputs(OpenXR::InputState& inputs, const VPADStatus& vpadStatus);
    // a debug window with a text field has the keyboard
    bool BlocksGameInput();

    void OpenLinkInBrowser(const char* url);

    extern ImFont* g_titleFont;

    void DrawFPSOverlay(class RND_Renderer* renderer);
}
