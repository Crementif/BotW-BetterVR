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
        // the update the patches are made for
        uint16_t requiredVersion;
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
        // the loading screens with a full-view white backdrop, FadeStatus_00 is the one shown while a save loads
        bool loadingBackdropVisible;
        bool aimingItemHeld;
        bool cutsceneActive;
        bool cutsceneCameraSwitchable;
        bool cutsceneFirstPerson;
        bool cutsceneShowsCameraPrompt;
        bool dialogVisible;
        uint32_t cutsceneCameraToggles;
        uint32_t recordingOutputMode;
    };
    Presentation GetPresentation();

    glm::mat4 GetTrackingOrigin();
    // blacks out the view while a snap turn happens
    float GetComfortFadeAmount();
    void QueueWorldOverlays(long presentedFrameIdx);
    void DrawDebugWindows();

    struct GuideCard {
        const char* icon;
        const char* title;
        const char* hint;
        glm::fvec3 cardOffset;
        glm::fvec3 zoneOffset;
        float zoneRadius;
    };
    inline constexpr size_t MaxGuideCards = 6;
    struct Guide {
        std::array<GuideCard, MaxGuideCards> cards;
        size_t count;
        uint64_t generation;
    };
    Guide GetGuide();

    bool IsCapturingMono();
    bool UsesBlackBars();
    uint32_t GetSharedMemoryAddress();
}
