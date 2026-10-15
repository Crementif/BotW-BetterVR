#pragma once

namespace Game {
    struct Title {
        std::string_view name;
        // the EU, US and JP releases
        std::array<uint64_t, 3> titleIds;
        std::string_view vrPack;
        std::string_view graphicsPack;
        // a community pack the game needs in VR, applied after the VR pack
        std::string_view requiredPack;
        // the 3D render targets before the graphics pack scales them to the eye resolution
        uint32_t renderWidth;
        uint32_t renderHeight;
    };
    std::span<const Title> GetTitles();
    const Title* FindTitle(uint64_t titleId);
    void RegisterHooks();

    // taken on the game's thread so presenting doesn't read guest memory while it changes
    struct Presentation {
        bool inGame;
        bool pauseMenuOpen;
        bool quickMenuOpen;
        bool fadeVisible;
        bool flatCamera;
        bool blackBars;
        bool loadingScreenVisible;
        bool aimingItemHeld;
        uint32_t recordingOutputMode;
    };
    Presentation GetPresentation();

    glm::mat4 GetTrackingOrigin();
    // blacks out the view while a snap turn happens
    float GetComfortFadeAmount();
    void QueueWorldOverlays(long presentedFrameIdx);
    void DrawDebugWindows();

    bool IsCapturingMono();
    bool UsesBlackBars();
    uint32_t GetSharedMemoryAddress();
}
