#pragma once

#include "rendering/rumble.h"
#include "utils/mod_settings.h"
#include "utils/controller_bindings.h"

// the friend declaration alone isn't enough for GetRenderer
class RND_Renderer;

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
        XrVersion minVulkanApiVersion;
        XrVersion maxVulkanApiVersion;
        bool usesVulkanEnable2;
        bool supportsDepthComposition;
        VkInstance vulkanInstance;
        VkPhysicalDevice vulkanPhysicalDevice;
        bool supportsOrientational;
        bool supportsPositional;
        bool supportsMutatableFOV;
        bool isOculusLinkRuntime;
        bool isMetaSimulator;
        bool supportsPicoController;
        bool supportsPicoUltraController;
        bool supportsCosmosController;
        bool supportsHPMixedRealityController;
        bool supportsFrameController;
        bool supportsDisplayRefreshRate;
        bool supportsEyeGaze;
        ControllerType activeControllerType = ControllerType::Unknown;
    } m_capabilities = {};

    struct InputState {
        struct Shared {
            bool in_game = true;
            XrTime inputTime;

            std::array<XrActionStatePose, 2> pose;
            std::array<XrSpaceLocation, 2> poseLocation;
            std::array<XrSpaceVelocity, 2> poseVelocity;
            std::array<XrActionStatePose, 2> aimPose;
            std::array<XrSpaceLocation, 2> aimPoseLocation;
            std::array<XrSpaceLocation, 2> hmdRelativePoseLocation;

            XrActionStateBoolean inventory_map;
            XrActionStateBoolean modMenu;
            // up, right, down, left
            std::array<XrActionStateBoolean, 4> dpad;
        } shared;

        struct InGame {
            XrActionStateBoolean crouch_scope;

            XrActionStateVector2f move;
            XrActionStateVector2f camera;

            std::array<XrActionStateFloat, 2> grab;
            XrActionStateBoolean jump_cancel;
            XrActionStateBoolean run_interact;
            XrActionStateBoolean useRune_dpadMenu;

            XrActionStateBoolean useLeftItem;
            XrActionStateBoolean useRightItem;

            XrActionStateBoolean cancel;
            XrActionStateBoolean sprint;
            XrActionStateBoolean scope;
        } inGame;
        struct InMenu {
            XrActionStateVector2f scroll;
            XrActionStateVector2f navigate;

            XrActionStateBoolean select;
            XrActionStateBoolean back;
            XrActionStateBoolean sort;
            XrActionStateBoolean hold;

            XrActionStateBoolean leftGrip;
            XrActionStateBoolean rightGrip;

            XrActionStateBoolean leftTrigger;
            XrActionStateBoolean rightTrigger;
        } inMenu;
    };
    std::atomic<InputState> m_input = InputState{};

    void CreateSession(const XrGraphicsBindingVulkan2KHR& vulkanBinding);
    void CreateActions();

    // with enable2 the runtime creates the Vulkan instance and device itself, the enable fallback only gives back extension names
    bool UsesVulkanEnable2() const { return m_capabilities.usesVulkanEnable2; }
    std::vector<std::string> GetRequiredVulkanInstanceExtensions() const;
    std::vector<std::string> GetRequiredVulkanDeviceExtensions() const;
    uint32_t ClampVulkanApiVersion(uint32_t requestedApiVersion) const;
    VkResult CreateVulkanInstance(const VkInstanceCreateInfo& createInfo, const VkAllocationCallbacks* allocator, VkInstance* pInstance);
    VkResult CreateVulkanDevice(VkPhysicalDevice physicalDevice, const VkDeviceCreateInfo& createInfo, const VkAllocationCallbacks* allocator, VkDevice* pDevice);
    VkPhysicalDevice GetVulkanGraphicsDevice(VkInstance vulkanInstance);
    std::array<XrViewConfigurationView, 2> GetViewConfigurations();
    std::optional<XrSpaceLocation> UpdateSpaces(XrTime predictedDisplayTime);
    std::optional<InputState> UpdateActions(XrTime predictedFrameTime, bool inMenu);
    // where the eyes look in stage space, on headsets that track them and only while they do
    std::optional<glm::fvec3> LocateEyeGaze(XrTime predictedDisplayTime) const;
    void UpdateSeatedHeightCalibration(XrTime predictedDisplayTime, const std::optional<XrSpaceLocation>& headLocation);
    void RequestSeatedHeightCalibration(XrTime notBeforeTime = 0);
    float GetDisplayRefreshRate() const { return m_displayRefreshRate; }
    std::optional<float> GetCalibratedSeatedEyeHeight() const;

    void ProcessEvents();

    XrSession GetSession() const { return m_session; }
    RND_Renderer* GetRenderer() const { return m_renderer.get(); }
    RumbleManager* GetRumbleManager() const { return m_rumbleManager.get(); }

private:
    XrPath GetXRPath(const char* str) const {
        XrPath path;
        checkXRResult(xrStringToPath(m_instance, str, &path), fmt::format("Failed to get path for {}", str).c_str());
        return path;
    };
    void ReplaceStageSpace(float floorOffset);

    XrInstance m_instance = XR_NULL_HANDLE;
    XrSystemId m_systemId = XR_NULL_SYSTEM_ID;
    XrSession m_session = XR_NULL_HANDLE;
    XrSpace m_stageSpace = XR_NULL_HANDLE;
    XrSpace m_headSpace = XR_NULL_HANDLE;
    std::vector<XrSpace> m_retiredStageSpaces;
    float m_stageFloorOffset = 0.0f;
    std::atomic_bool m_seatedHeightCalibrationRequested = false;
    std::atomic<XrTime> m_seatedHeightCalibrationNotBeforeTime = 0;
    bool m_hasSeatedHeightCalibration = false;
    bool m_seenFirstLoadingScreen = false;
    std::optional<PlayMode> m_lastSeenPlayMode;
    std::array<XrSpace, 2> m_inGameHandSpaces = { XR_NULL_HANDLE, XR_NULL_HANDLE };
    std::array<XrSpace, 2> m_inGameAimSpaces = { XR_NULL_HANDLE, XR_NULL_HANDLE };
    std::array<XrSpace, 2> m_inMenuHandSpaces = { XR_NULL_HANDLE, XR_NULL_HANDLE };
    std::array<XrSpace, 2> m_inMenuAimSpaces = { XR_NULL_HANDLE, XR_NULL_HANDLE };
    std::array<XrPath, 2> m_handPaths = { XR_NULL_PATH, XR_NULL_PATH };

    XrAction m_inGameGripPoseAction = XR_NULL_HANDLE;
    XrAction m_inGameAimPoseAction = XR_NULL_HANDLE;
    XrAction m_inMenuGripPoseAction = XR_NULL_HANDLE;
    XrAction m_inMenuAimPoseAction = XR_NULL_HANDLE;

    XrActionSet m_gameplayActionSet = XR_NULL_HANDLE;
    XrAction m_moveAction = XR_NULL_HANDLE;
    XrAction m_cameraAction = XR_NULL_HANDLE;
    
    XrAction m_grab_interactAction = XR_NULL_HANDLE;
    XrAction m_jumpAction = XR_NULL_HANDLE;
    XrAction m_run_interactAction = XR_NULL_HANDLE;
    XrAction m_useRune_dpadMenu_Action = XR_NULL_HANDLE;
    XrAction m_inGame_modMenuAction = XR_NULL_HANDLE; //imgui mod menu

    XrAction m_useLeftItemAction = XR_NULL_HANDLE;
    XrAction m_useRightItemAction = XR_NULL_HANDLE;

    XrAction m_crouch_scopeAction = XR_NULL_HANDLE;
    XrAction m_inGame_inventory_mapAction = XR_NULL_HANDLE;

    XrAction m_cancelAction = XR_NULL_HANDLE;
    XrAction m_sprintAction = XR_NULL_HANDLE;
    XrAction m_scopeAction = XR_NULL_HANDLE;
    std::array<XrAction, 4> m_inGameDpadActions = {};

    XrAction m_rumbleAction = XR_NULL_HANDLE;

    // synced with both the gameplay and menu sets since the eyes are tracked in menus too
    XrActionSet m_eyeGazeActionSet = XR_NULL_HANDLE;
    XrAction m_eyeGazeAction = XR_NULL_HANDLE;
    XrSpace m_eyeGazeSpace = XR_NULL_HANDLE;

    XrActionSet m_menuActionSet = XR_NULL_HANDLE;
    XrAction m_scrollAction = XR_NULL_HANDLE;
    XrAction m_navigateAction = XR_NULL_HANDLE;
    XrAction m_selectAction = XR_NULL_HANDLE; // A button
    XrAction m_backAction = XR_NULL_HANDLE; // B button
    XrAction m_sortAction = XR_NULL_HANDLE; // Y button
    XrAction m_holdAction = XR_NULL_HANDLE; // X button
    XrAction m_leftGripAction = XR_NULL_HANDLE; // left bumper
    XrAction m_rightGripAction = XR_NULL_HANDLE; // right bumper

    XrAction m_leftTriggerAction= XR_NULL_HANDLE;
    XrAction m_rightTriggerAction = XR_NULL_HANDLE;

    XrAction m_inMenu_modMenuAction = XR_NULL_HANDLE; //imgui mod menu
    XrAction m_inMenu_inventory_mapAction = XR_NULL_HANDLE;
    std::array<XrAction, 4> m_inMenuDpadActions = {};

    std::unique_ptr<RND_Renderer> m_renderer;
    std::unique_ptr<RumbleManager> m_rumbleManager;

    constexpr static XrPosef s_xrIdentityPose = { .orientation = { .x = 0, .y = 0, .z = 0, .w = 1 }, .position = { .x = 0, .y = 0, .z = 0 } };

    XrDebugUtilsMessengerEXT m_debugMessengerHandle = XR_NULL_HANDLE;

    PFN_xrGetVulkanGraphicsRequirements2KHR func_xrGetVulkanGraphicsRequirements2KHR = nullptr;
    PFN_xrCreateVulkanInstanceKHR func_xrCreateVulkanInstanceKHR = nullptr;
    PFN_xrCreateVulkanDeviceKHR func_xrCreateVulkanDeviceKHR = nullptr;
    PFN_xrGetVulkanGraphicsDevice2KHR func_xrGetVulkanGraphicsDevice2KHR = nullptr;

    PFN_xrGetVulkanGraphicsRequirementsKHR func_xrGetVulkanGraphicsRequirementsKHR = nullptr;
    PFN_xrGetVulkanInstanceExtensionsKHR func_xrGetVulkanInstanceExtensionsKHR = nullptr;
    PFN_xrGetVulkanDeviceExtensionsKHR func_xrGetVulkanDeviceExtensionsKHR = nullptr;
    PFN_xrGetVulkanGraphicsDeviceKHR func_xrGetVulkanGraphicsDeviceKHR = nullptr;

#if defined(_WIN32)
    PFN_xrConvertTimeToWin32PerformanceCounterKHR func_xrConvertTimeToPlatformTime = nullptr;
    PFN_xrConvertWin32PerformanceCounterToTimeKHR func_xrConvertPlatformTimeToTime = nullptr;
#else
    PFN_xrConvertTimeToTimespecTimeKHR func_xrConvertTimeToPlatformTime = nullptr;
    PFN_xrConvertTimespecTimeToTimeKHR func_xrConvertPlatformTimeToTime = nullptr;
#endif
    PFN_xrGetDisplayRefreshRateFB func_xrGetDisplayRefreshRateFB = nullptr;
    std::atomic<float> m_displayRefreshRate = 0.0f;
    PFN_xrCreateDebugUtilsMessengerEXT func_xrCreateDebugUtilsMessengerEXT = nullptr;
    PFN_xrDestroyDebugUtilsMessengerEXT func_xrDestroyDebugUtilsMessengerEXT = nullptr;
};
using EyeSide = OpenXR::EyeSide;

template <>
struct fmt::formatter<EyeSide> : fmt::formatter<std::string_view> {
    auto format(const EyeSide side, fmt::format_context& ctx) const {
        return fmt::format_to(ctx.out(), "{}", side == EyeSide::LEFT ? "LEFT" : "RIGHT");
    }
};
