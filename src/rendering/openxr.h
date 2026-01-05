#pragma once

#include "hooking/rumble.h"

class OpenXR {
    friend class RND_Renderer;

public:
    OpenXR();
    ~OpenXR();

    enum EyeSide : uint8_t {
        LEFT = 0,
        RIGHT = 1
    };

    struct Capabilities {
        LUID adapter;
        D3D_FEATURE_LEVEL minFeatureLevel;
        bool supportsOrientational;
        bool supportsPositional;
        bool supportsMutatableFOV;
        bool isOculusLinkRuntime;
        bool isMetaSimulator;
    } m_capabilities = {};

    union InputState {
        struct InGame {
            bool in_game = true;
            XrTime inputTime;
            std::optional<EyeSide> lastPickupSide = std::nullopt;

            // shared
            XrActionStateBoolean mapAndInventory;

            XrActionStateBoolean leftTrigger;
            XrActionStateBoolean rightTrigger;

            // unique
            XrActionStateVector2f camera;
            XrActionStateVector2f move;

            XrActionStateBoolean jump;
            XrActionStateBoolean crouchAndScope;
            XrActionStateBoolean run;
            XrActionStateBoolean attack;
            XrActionStateBoolean useRune;
            XrActionStateBoolean throwWeapon;
            XrActionStateBoolean cancel;
            XrActionStateBoolean interact;
            std::array<XrActionStateFloat, 2> grab;
            std::array<bool, 2> drop_weapon; // LEFT/RIGHT

            struct ButtonState {
                enum class Event {
                    None,
                    ShortPress,
                    LongPress,
                    DoublePress
                };

                bool wasDownLastFrame = false;
                bool longFired = false;
                bool waitingForSecond = false;
                std::chrono::steady_clock::time_point pressStartTime;
                std::chrono::steady_clock::time_point lastReleaseTime;

                Event lastEvent = Event::None;

                void resetFrameFlags() { lastEvent = Event::None; }
                void resetButtonState() {
                    wasDownLastFrame = false;
                    longFired = false;
                    waitingForSecond = false;
                }
            };
            std::array<ButtonState, 2> grabState; // LEFT/RIGHT
            ButtonState runState;
            ButtonState crouchAndScopeState;
            ButtonState mapAndInventoryState;
            std::array<XrActionStatePose, 2> pose;
            std::array<XrSpaceLocation, 2> poseLocation;
            std::array<XrSpaceVelocity, 2> poseVelocity;
            // todo: remove relative controller positions if it turns out to be unnecessary
            std::array<XrSpaceLocation, 2> hmdRelativePoseLocation;
        } inGame;
        struct InMenu {
            bool in_game = false;
            XrTime inputTime;
            std::optional<EyeSide> lastPickupSide = std::nullopt;

            // shared
            XrActionStateBoolean mapAndInventory;

            XrActionStateBoolean leftTrigger;
            XrActionStateBoolean rightTrigger;

            // unique
            XrActionStateVector2f scroll;
            XrActionStateVector2f navigate;

            XrActionStateBoolean select;
            XrActionStateBoolean back;
            XrActionStateBoolean sort;
            XrActionStateBoolean hold;

            XrActionStateBoolean leftGrip;
            XrActionStateBoolean rightGrip;
        } inMenu;
    };
    std::atomic<InputState> m_input = InputState{};
    std::atomic<glm::fquat> m_inputCameraRotation = glm::identity<glm::fquat>();

    struct GameState {
        bool in_game = false;
        bool was_in_game = false;
        bool map_open = false;
        bool prevent_menu_inputs = false;
        std::chrono::steady_clock::time_point prevent_menu_time;
        bool prevent_grab_inputs = false;
        std::chrono::steady_clock::time_point prevent_grab_time;
        WeaponType right_weapon_type = WeaponType::SmallSword;
        WeaponType left_weapon_type = WeaponType::SmallSword;
        bool last_weapon_held_hand = 0; // 0 = left, 1 = right
        bool is_weapon_or_object_held = false; // true if either a weapon or a throwable object is held
        bool is_throwable_object_held = false; // true if a throwable object is held
    } gameState ;

    std::atomic<GameState> m_gameState{};

    void CreateSession(const XrGraphicsBindingD3D12KHR& d3d12Binding);
    void CreateActions();
    std::array<XrViewConfigurationView, 2> GetViewConfigurations();
    std::optional<XrSpaceLocation> UpdateSpaces(XrTime predictedDisplayTime);
    std::optional<InputState> UpdateActions(XrTime predictedFrameTime, glm::fquat controllerRotation, bool inMenu);
   
    void ProcessEvents();

    XrSession GetSession() const { return m_session; }
    RND_Renderer* GetRenderer() const { return m_renderer.get(); }
    RumbleManager* GetRumbleManager() const { return m_rumbleManager.get(); }

private:
    XrPath GetXRPath(const char* str) const {
        XrPath path;
        checkXRResult(xrStringToPath(m_instance, str, &path), std::format("Failed to get path for {}", str).c_str());
        return path;
    };

    XrInstance m_instance = XR_NULL_HANDLE;
    XrSystemId m_systemId = XR_NULL_SYSTEM_ID;
    XrSession m_session = XR_NULL_HANDLE;
    XrSpace m_stageSpace = XR_NULL_HANDLE;
    XrSpace m_headSpace = XR_NULL_HANDLE;
    std::array<XrSpace, 2> m_handSpaces = { XR_NULL_HANDLE, XR_NULL_HANDLE };
    std::array<XrPath, 2> m_handPaths = { XR_NULL_PATH, XR_NULL_PATH };

    // gameplay actions
    XrActionSet m_gameplayActionSet = XR_NULL_HANDLE;
    XrAction m_gripPoseAction = XR_NULL_HANDLE;
    XrAction m_aimPoseAction = XR_NULL_HANDLE;
    XrAction m_moveAction = XR_NULL_HANDLE;
    XrAction m_cameraAction = XR_NULL_HANDLE;
    XrAction m_grabAction = XR_NULL_HANDLE;
    
    XrAction m_jumpAction = XR_NULL_HANDLE;
    XrAction m_crouchAndScopeAction = XR_NULL_HANDLE;
    XrAction m_runAction = XR_NULL_HANDLE;
    XrAction m_attackAction = XR_NULL_HANDLE;
    XrAction m_useRuneAction = XR_NULL_HANDLE;
    XrAction m_throwWeaponAction = XR_NULL_HANDLE;

    XrAction m_cancelAction = XR_NULL_HANDLE;
    XrAction m_interactAction = XR_NULL_HANDLE;

    XrAction m_inGame_leftTriggerAction = XR_NULL_HANDLE;
    XrAction m_inGame_rightTriggerAction = XR_NULL_HANDLE;

    XrAction m_inGame_mapAndInventoryAction = XR_NULL_HANDLE;
    XrAction m_rumbleAction = XR_NULL_HANDLE;

    // menu actions
    XrActionSet m_menuActionSet = XR_NULL_HANDLE;
    XrAction m_scrollAction = XR_NULL_HANDLE;
    XrAction m_navigateAction = XR_NULL_HANDLE;
    XrAction m_selectAction = XR_NULL_HANDLE; // A button
    XrAction m_backAction = XR_NULL_HANDLE; // B button
    XrAction m_sortAction = XR_NULL_HANDLE; // Y button
    XrAction m_holdAction = XR_NULL_HANDLE; // X button
    XrAction m_leftGripAction = XR_NULL_HANDLE; // left bumper
    XrAction m_rightGripAction = XR_NULL_HANDLE; // right bumper

    XrAction m_inMenu_leftTriggerAction= XR_NULL_HANDLE;
    XrAction m_inMenu_rightTriggerAction = XR_NULL_HANDLE;

    XrAction m_inMenu_mapAndInventoryAction = XR_NULL_HANDLE;

    std::unique_ptr<RND_Renderer> m_renderer;
    std::unique_ptr<RumbleManager> m_rumbleManager;

    constexpr static XrPosef s_xrIdentityPose = { .orientation = { .x = 0, .y = 0, .z = 0, .w = 1 }, .position = { .x = 0, .y = 0, .z = 0 } };

    XrDebugUtilsMessengerEXT m_debugMessengerHandle = XR_NULL_HANDLE;

    PFN_xrGetD3D12GraphicsRequirementsKHR func_xrGetD3D12GraphicsRequirementsKHR = nullptr;
    PFN_xrConvertTimeToWin32PerformanceCounterKHR func_xrConvertTimeToWin32PerformanceCounterKHR = nullptr;
    PFN_xrConvertWin32PerformanceCounterToTimeKHR func_xrConvertWin32PerformanceCounterToTimeKHR = nullptr;
    PFN_xrCreateDebugUtilsMessengerEXT func_xrCreateDebugUtilsMessengerEXT = nullptr;
    PFN_xrDestroyDebugUtilsMessengerEXT func_xrDestroyDebugUtilsMessengerEXT = nullptr;
};
using ButtonState = OpenXR::InputState::InGame::ButtonState;
using EyeSide = OpenXR::EyeSide;

template <>
struct std::formatter<EyeSide> : std::formatter<string> {
    auto format(const EyeSide side, std::format_context& ctx) const {
        return std::format_to(ctx.out(), "{}", side == EyeSide::LEFT ? "LEFT" : "RIGHT");
    }
};