#pragma once
#include "utils/mod_settings.h"

namespace Screens {
    uint32_t GetAddress(ScreenId screen);
    bool HasInstance(ScreenId screen);
    bool IsVisible(ScreenId screen);
    bool IsAnyFadeVisible();
    bool IsLoadingVisible();
    bool IsTitleVisible();
}

namespace Events {
    struct Settings {
        bool firstPerson;                  // use Link's perspective, ignore the animated event camera
        bool disablePlayerDrivenLinkHands; // let event control the hands instead of the VR controllers
        bool ignoreCameraRotation;         // some events will pan the camera, but in first-person it should usually be ignored to avoid nausea. Doors opening is okay, but panning down to a chest is not.
        bool demoEnableCameraInput;        // there's already events that allow user camera control. This isn't used or overwritten atm.
    };

    bool IsActive();
    void ToggleCameraMode();
    std::optional<Settings> GetFirstPersonSettings();
    bool UsesAnimatedHands();
}

namespace GameState {
    void RegisterHooks();

    bool IsInGame();
    bool IsShowingMenu();
    bool IsFirstPerson();
    bool IsThirdPerson();
    bool UseBlackBarsDuringEvents();

    bool IsScopeModeActive();
    bool IsCameraFinderModeActive();
    bool IsFlatCameraModeActive();
    bool UseFlatCameraPresentation();
    bool UseMonoFrameBufferTemporarilyDuringMenusOrPictures();

    uint32_t GetRecordingOutputMode();
}
