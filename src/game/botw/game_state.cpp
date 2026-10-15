#include "pch.h"
#include "game_state.h"
#include "game/game.h"
#include "cemu/cemu_host.h"
#include "game/botw/camera/camera.h"
#include "game/botw/player/bow.h"
#include "game/botw/player/player.h"
#include "game/botw/player/weapon.h"
#include "instance.h"

static std::atomic_bool s_eventActive = false;
static std::atomic<Events::Settings> s_currentEventSettings = Events::Settings{};
static std::unordered_map<std::string, Events::Settings> s_eventSettings;
static std::atomic_uint32_t s_recordingOutputMode = 0;
static std::atomic<Game::Presentation> s_presentation = Game::Presentation{};

constexpr Events::Settings defaultFirstPersonSettings = {
    .firstPerson = true,
    .disablePlayerDrivenLinkHands = false,
    .ignoreCameraRotation = true
};

uint32_t Screens::GetAddress(ScreenId screen) {
    uint32_t screenManagerInstance = Guest::Get<uint32_t>(0x1047E650).getLE();
    if (screenManagerInstance == 0) {
        return 0;
    }

    uint32_t screenPtrs = Guest::Get<uint32_t>(screenManagerInstance + 0x18).getLE();
    return Guest::Get<uint32_t>(screenPtrs + (std::to_underlying(screen) * 4)).getLE();
}

bool Screens::HasInstance(ScreenId screen) {
    return GetAddress(screen) != 0;
}

bool Screens::IsVisible(ScreenId screen) {
    uint32_t screenPtr = GetAddress(screen);
    if (screenPtr == 0) {
        return false;
    }

    // true when the screen is open or transitioning to be closed/opened, the inverse of Screen::isClosed()
    const int8_t openPriority = (int8_t)Guest::Get<uint8_t>(screenPtr + 0x95).getLE();
    const int8_t screenState = (int8_t)Guest::Get<uint8_t>(screenPtr + 0x96).getLE();
    return screenState != 0 || openPriority > 0;
}

bool Screens::IsAnyFadeVisible() {
    return IsVisible(ScreenId::FadeDemo_00) || IsVisible(ScreenId::Fade) || IsVisible(ScreenId::FadeStatus_00);
}

bool Screens::IsLoadingVisible() {
    // not the fade screens, plenty of those sit open during normal gameplay
    return IsVisible(ScreenId::LoadingWeapon_00);
}

bool Screens::IsTitleVisible() {
    return IsVisible(ScreenId::Title_00);
}

bool Events::IsActive() {
    return s_eventActive.load(std::memory_order_relaxed);
}

std::optional<Events::Settings> Events::GetFirstPersonSettings() {
    if (!IsActive() || GetSettings().GetCutsceneCameraMode() == EventMode::ALWAYS_THIRD_PERSON) {
        return std::nullopt;
    }

    Settings settings = s_currentEventSettings.load(std::memory_order_relaxed);
    if (GetSettings().AlwaysPreventFirstPersonCutsceneCameraMovement()) {
        settings.ignoreCameraRotation = true;
    }
    return settings;
}

static void ApplyEventSetting(Events::Settings& entry, std::string_view setting) {
    if (setting == "FP_ON") entry.firstPerson = true;
    else if (setting == "FP_OFF") entry.firstPerson = false;
    else if (setting == "HND_ON") entry.disablePlayerDrivenLinkHands = false;
    else if (setting == "HND_OFF") entry.disablePlayerDrivenLinkHands = true;
    else if (setting == "PAN_ON") entry.ignoreCameraRotation = false;
    else if (setting == "PAN_OFF") entry.ignoreCameraRotation = true;
    else if (setting == "CTRL_ON") entry.demoEnableCameraInput = false;
    else if (setting == "CTRL_OFF") entry.demoEnableCameraInput = true;
    else Log::print<WARNING>("Unknown cutscene default setting: {}", setting);
}

// the table is a list of "EventName,SETTING,SETTING" strings in patch_Settings_Cutscenes.asm
static void LoadEventSettingsTable(uint32_t tableAddress) {
    if (!s_eventSettings.empty()) {
        return;
    }

    const char* currPtr = Guest::CString(tableAddress);
    while (*currPtr != '\0') {
        const std::string_view line(currPtr);
        currPtr += line.length() + 1;

        size_t commaPos = line.find(',');
        if (commaPos == std::string_view::npos) {
            continue;
        }

        Events::Settings entry = {};
        for (auto setting : std::views::split(line.substr(commaPos + 1), ',')) {
            ApplyEventSetting(entry, std::string_view(setting.begin(), setting.end()));
        }
        s_eventSettings.insert_or_assign(std::string(line.substr(0, commaPos)), entry);
    }

    Log::print<VERBOSE>("Initialized cutscene default settings for {} events.", s_eventSettings.size());
}

static void PublishPresentation();

// todo: Fix sped-up cutscenes. You can go in-game into a save file that didn't have DLC yet, and it'll give you a sped up Demo200_0.
static void hook_GetEventName(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    static std::string s_currentEvent;
    static std::string s_currentEntryPoint;

    const bool isEventActive = hCPU->gpr[3] != 0;
    if (!isEventActive) {
        if (!s_currentEvent.empty()) {
            Log::print<INFO>("Event '{}' has now ended", s_currentEvent);
            s_currentEvent.clear();
            s_currentEntryPoint.clear();
            s_eventActive = false;
            PublishPresentation();
        }
        return;
    }

    const std::string_view eventName = Guest::CString(hCPU->gpr[4]);
    const std::string_view entryPointName = Guest::CString(hCPU->gpr[5]);
    if (s_currentEvent == eventName && s_currentEntryPoint == entryPointName) {
        return;
    }
    s_currentEvent = eventName;
    s_currentEntryPoint = entryPointName;
    Log::print<INFO>("Event '{}' is now active (using entry point '{}').", eventName, entryPointName);

    Events::Settings settings = defaultFirstPersonSettings;
    if (auto it = s_eventSettings.find(s_currentEvent); it != s_eventSettings.end()) {
        settings = it->second;
        Log::print<INFO>(" - First Person: {}", settings.firstPerson ? "ON" : "OFF");
        Log::print<INFO>(" - Ignore Camera Rotation: {}", settings.ignoreCameraRotation ? "ON" : "OFF");
        Log::print<INFO>(" - Disable Player-Driven Link Hands: {}", settings.disablePlayerDrivenLinkHands ? "ON" : "OFF");
    }
    else {
        Log::print<INFO>(" - No specific settings found for this event, using defaults.");
    }
    s_currentEventSettings = settings;
    s_eventActive = true;
    PublishPresentation();
}

bool GameState::IsFirstPerson() {
    if (IsFlatCameraModeActive()) {
        return false;
    }

    if (!Events::IsActive()) {
        return GetSettings().GetCameraMode() == CameraMode::FIRST_PERSON;
    }

    const EventMode mode = GetSettings().GetCutsceneCameraMode();
    if (mode == EventMode::ALWAYS_THIRD_PERSON) {
        return false;
    }
    return mode != EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS || s_currentEventSettings.load(std::memory_order_relaxed).firstPerson;
}

bool GameState::IsThirdPerson() {
    return !IsFirstPerson();
}

bool GameState::UseBlackBarsDuringEvents() {
    if (IsFlatCameraModeActive() || !Events::IsActive() || IsFirstPerson()) {
        return false;
    }

    return GetSettings().UseBlackBarsForCutscenes();
}

bool GameState::IsInGame() {
    return Camera::GetFramesSinceLastUpdate() <= 4 && !Screens::HasInstance(ScreenId::PauseMenuInfo_00);
}

bool GameState::IsShowingMenu() {
    return IsFlatCameraModeActive() || !IsInGame() || Screens::HasInstance(ScreenId::ShopBG_00) || Screens::HasInstance(ScreenId::MessageDialog);
}

constexpr uint32_t RuneMgrInstanceAddress = 0x10463798;
constexpr uint32_t RuneMgrFlagsOffset = 0x4C;
constexpr uint32_t RuneMgrSelectedRuneOffset = 0x1D4;

bool GameState::IsScopeModeActive() {
    constexpr uint32_t TelescopeActiveMask = 1 << 5;

    const uint32_t runeMgr = Guest::Get<uint32_t>(RuneMgrInstanceAddress).getLE();
    return runeMgr != 0 && (Guest::Get<uint32_t>(runeMgr + RuneMgrFlagsOffset).getLE() & TelescopeActiveMask) != 0;
}

bool GameState::IsCameraFinderModeActive() {
    // Bit 4 is shared by every equipped rune, so also require RuneType_Camera.
    constexpr uint32_t RuneActiveMask = 1 << 4;
    constexpr uint32_t CameraRune = 5;

    const uint32_t runeMgr = Guest::Get<uint32_t>(RuneMgrInstanceAddress).getLE();
    return runeMgr != 0 && (Guest::Get<uint32_t>(runeMgr + RuneMgrFlagsOffset).getLE() & RuneActiveMask) != 0 && Guest::Get<uint32_t>(runeMgr + RuneMgrSelectedRuneOffset).getLE() == CameraRune;
}

bool GameState::IsFlatCameraModeActive() {
    return IsScopeModeActive() || IsCameraFinderModeActive();
}

bool GameState::UseFlatCameraPresentation() {
    return IsFlatCameraModeActive() && !Screens::IsAnyFadeVisible();
}

bool GameState::UseMonoFrameBufferTemporarilyDuringMenusOrPictures() {
    const RND_Renderer* renderer = VRManager::instance().XR->GetRenderer();
    return !Screens::IsAnyFadeVisible() && (IsFlatCameraModeActive() || Screens::HasInstance(ScreenId::PauseMenuInfo_00) || (renderer != nullptr && renderer->IsGameCapturing3DFrameBuffer()));
}

uint32_t GameState::GetRecordingOutputMode() {
    return s_recordingOutputMode.load(std::memory_order_relaxed);
}

Game::Presentation Game::GetPresentation() {
    return s_presentation.load(std::memory_order_relaxed);
}

static void PublishPresentation() {
    Game::Presentation presentation = {};
    presentation.inGame = GameState::IsInGame();
    presentation.pauseMenuOpen = Screens::HasInstance(ScreenId::PauseMenuInfo_00);
    presentation.quickMenuOpen = Screens::HasInstance(ScreenId::MainShortCut_00);
    presentation.fadeVisible = Screens::IsAnyFadeVisible();
    presentation.flatCamera = GameState::UseFlatCameraPresentation();
    presentation.blackBars = GameState::UseBlackBarsDuringEvents();
    presentation.loadingScreenVisible = Screens::IsLoadingVisible();
    presentation.aimingItemHeld = Weapons::IsAimingItemHeld();
    presentation.recordingOutputMode = GameState::GetRecordingOutputMode();
    s_presentation.store(presentation, std::memory_order_relaxed);
}

static void LogScreenChanges() {
#ifdef _DEBUG
    static std::unordered_set<ScreenId> s_prevEnabledScreens;
    std::unordered_set<ScreenId> currentEnabledScreens;
    for (uint32_t i = 0; i < std::to_underlying(ScreenId::ScreenId_END); i++) {
        ScreenId id = (ScreenId)i;
        if (Screens::HasInstance(id)) {
            if (!s_prevEnabledScreens.contains(id)) {
                if (currentEnabledScreens.empty()) {
                    Log::print<INFO>("---------");
                }
                Log::print<INFO>("Screen {} is ON", ScreenIdToString(id));
            }
            currentEnabledScreens.emplace(id);
        }
        else if (s_prevEnabledScreens.contains(id)) {
            Log::print<INFO>("Screen {} is OFF", ScreenIdToString(id));
        }
    }
    s_prevEnabledScreens = std::move(currentEnabledScreens);
#endif
}

// sead::GameFrameworkCafe::presentAndDrawDone, once per presented frame
static void hook_UpdateSettings(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t recordingOutputMode = 0;
    Guest::ReadBE(0x10416BF4, &recordingOutputMode);
    s_recordingOutputMode.store(recordingOutputMode, std::memory_order_relaxed);

    if (GetSettings().IsDebuggingToolsEnabled()) {
        VRManager::instance().Debugger->UpdateEntityMemory();
    }

    Camera::OnGameFrame();
    Player::OnGameFrame();
    Weapons::OnGameFrame();
    Bow::OnGameFrame();
    LogScreenChanges();
    PublishPresentation();

    LoadEventSettingsTable(hCPU->gpr[6]);
}

static void hook_IsLoadingScreenVisible(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    hCPU->gpr[3] = Screens::IsLoadingVisible() ? 1 : 0;
}

// the Wii U timebase that OSSleepTicks counts in
constexpr double WiiUTimerTicksPerSecond = 62156250.0;
constexpr std::chrono::nanoseconds ThrottledFramePeriod = std::chrono::nanoseconds(1'000'000'000 / 60);

static void hook_GetFrameThrottleTicks(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    static std::chrono::steady_clock::time_point s_nextFrameDeadline = {};

    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if ((!Screens::IsLoadingVisible() && !Screens::IsTitleVisible()) || now >= s_nextFrameDeadline) {
        s_nextFrameDeadline = now + ThrottledFramePeriod;
        hCPU->gpr[3] = 0;
        return;
    }

    const std::chrono::steady_clock::duration remaining = s_nextFrameDeadline - now;
    s_nextFrameDeadline += ThrottledFramePeriod;
    hCPU->gpr[3] = (uint32_t)(std::chrono::duration<double>(remaining).count() * WiiUTimerTicksPerSecond);
}

void GameState::RegisterHooks() {
    CemuHost::RegisterHook("hook_UpdateSettings", &hook_UpdateSettings);
    CemuHost::RegisterHook("hook_GetEventName", &hook_GetEventName);
    CemuHost::RegisterHook("hook_IsLoadingScreenVisible", &hook_IsLoadingScreenVisible);
    CemuHost::RegisterHook("hook_GetFrameThrottleTicks", &hook_GetFrameThrottleTicks);
}
