#include "pch.h"

#include "openxr.h"
#include "instance.h"

#ifndef XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME
#define XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME "XR_BD_ultra_controller_interaction"
#endif

static XrBool32 XR_DebugUtilsMessengerCallback(XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageType, const XrDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData) {
    Log::print<XR_DEBUGUTILS>("Function {}: {}", callbackData->functionName, callbackData->message);
    return XR_FALSE;
}

OpenXR::OpenXR() {
    uint32_t xrExtensionCount = 0;
    xrEnumerateInstanceExtensionProperties(NULL, 0, &xrExtensionCount, NULL);
    std::vector<XrExtensionProperties> instanceExtensions;
    instanceExtensions.resize(xrExtensionCount, { XR_TYPE_EXTENSION_PROPERTIES, NULL });
    {
        XrResult result = xrEnumerateInstanceExtensionProperties(NULL, xrExtensionCount, &xrExtensionCount, instanceExtensions.data());
        if (result == XR_ERROR_RUNTIME_FAILURE) {
            Log::print<ERROR>("Couldn't enumerate OpenXR extensions! Is the OpenXR runtime installed and set to the correct runtime? Restarting might help, or going to SteamVR/Oculus Link's Settings and making sure OpenXR is enabled.");
        }
        checkXRResult(result, "Couldn't enumerate OpenXR extensions!");
    }

    // Create instance with required extensions
    bool d3d12Supported = false;
    bool depthSupported = false;
    bool timeConvSupported = false;
    bool debugUtilsSupported = false;
    for (XrExtensionProperties& extensionProperties : instanceExtensions) {
        Log::print<VERBOSE>("Found available OpenXR extension: {}", extensionProperties.extensionName);
        if (strcmp(extensionProperties.extensionName, XR_KHR_D3D12_ENABLE_EXTENSION_NAME) == 0) {
            d3d12Supported = true;
        }
        if (strcmp(extensionProperties.extensionName, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME) == 0) {
            depthSupported = true;
        }
        else if (strcmp(extensionProperties.extensionName, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME) == 0) {
            timeConvSupported = true;
        }
        else if (strcmp(extensionProperties.extensionName, XR_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
#ifdef _DEBUG
            debugUtilsSupported = Log::isLogTypeEnabled<XR_DEBUGUTILS>();
#endif
        }
        else if (strcmp(extensionProperties.extensionName, XR_BD_CONTROLLER_INTERACTION_EXTENSION_NAME) == 0) {
            m_capabilities.supportsPicoController = true;
        }
        else if (strcmp(extensionProperties.extensionName, XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME) == 0) {
            m_capabilities.supportsPicoUltraController = true;
        }
        else if (strcmp(extensionProperties.extensionName, XR_HTC_VIVE_COSMOS_CONTROLLER_INTERACTION_EXTENSION_NAME) == 0) {
            m_capabilities.supportsCosmosController = true;
        }
        else if (strcmp(extensionProperties.extensionName, XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME) == 0) {
            m_capabilities.supportsHPMixedRealityController = true;
        }
    }

    if (!d3d12Supported) {
        Log::print<ERROR>("OpenXR runtime doesn't support D3D12 (XR_KHR_D3D12_ENABLE)!");
        throw std::runtime_error("Current OpenXR runtime doesn't support Direct3D 12 (XR_KHR_D3D12_ENABLE). See the Github page's troubleshooting section for a solution!");
    }
    if (!depthSupported) {
        Log::print<ERROR>("OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH)!");
        throw std::runtime_error("Current OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH). See the Github page's troubleshooting section for a solution!");
    }
    if (!timeConvSupported) {
        Log::print<WARNING>("OpenXR runtime doesn't support converting time from/to XrTime (XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME). Not required, as of this version.");
    }
    if (!debugUtilsSupported && Log::isLogTypeEnabled<XR_DEBUGUTILS>()) {
        Log::print<INFO>("OpenXR runtime doesn't support debug utils (XR_EXT_DEBUG_UTILS)! Errors/debug information will no longer be able to be shown!");
    }

    std::vector<const char*> enabledExtensions = { XR_KHR_D3D12_ENABLE_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME };
    if (timeConvSupported) enabledExtensions.emplace_back(XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME);
    if (debugUtilsSupported) enabledExtensions.emplace_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (m_capabilities.supportsPicoController) enabledExtensions.emplace_back(XR_BD_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsPicoUltraController) enabledExtensions.emplace_back(XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsCosmosController) enabledExtensions.emplace_back(XR_HTC_VIVE_COSMOS_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsHPMixedRealityController) enabledExtensions.emplace_back(XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME);

    XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
    xrInstanceCreateInfo.createFlags = 0;
    xrInstanceCreateInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
    xrInstanceCreateInfo.enabledExtensionNames = enabledExtensions.data();
    xrInstanceCreateInfo.enabledApiLayerCount = 0;
    xrInstanceCreateInfo.enabledApiLayerNames = NULL;
    xrInstanceCreateInfo.applicationInfo = { "BetterVR", 1, "Cemu", 1, XR_API_VERSION_1_0 };
    {
        XrResult result = XR_ERROR_RUNTIME_FAILURE;
        for (int i = 0; i < 3; i++) {
             result = xrCreateInstance(&xrInstanceCreateInfo, &m_instance);
             if (XR_SUCCEEDED(result)) {
                 break;
             }
             std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (result == XR_ERROR_RUNTIME_FAILURE) {
            Log::print<ERROR>("Failed to create OpenXR instance! Is the OpenXR runtime installed and set to the correct runtime? Restarting might help, or going to SteamVR/Oculus Link's Settings and making sure OpenXR is enabled.");
        }
        checkXRResult(result, "Failed to initialize the OpenXR instance!");
    }

    // Load extension pointers for this XrInstance
    xrGetInstanceProcAddr(m_instance, "xrGetD3D12GraphicsRequirementsKHR", (PFN_xrVoidFunction*)&func_xrGetD3D12GraphicsRequirementsKHR);
    if (timeConvSupported) {
        xrGetInstanceProcAddr(m_instance, "xrConvertTimeToWin32PerformanceCounterKHR", (PFN_xrVoidFunction*)&func_xrConvertTimeToWin32PerformanceCounterKHR);
        xrGetInstanceProcAddr(m_instance, "xrConvertWin32PerformanceCounterToTimeKHR", (PFN_xrVoidFunction*)&func_xrConvertWin32PerformanceCounterToTimeKHR);
    }
    if (debugUtilsSupported) {
        xrGetInstanceProcAddr(m_instance, "xrCreateDebugUtilsMessengerEXT", (PFN_xrVoidFunction*)&func_xrCreateDebugUtilsMessengerEXT);
        xrGetInstanceProcAddr(m_instance, "xrDestroyDebugUtilsMessengerEXT", (PFN_xrVoidFunction*)&func_xrDestroyDebugUtilsMessengerEXT);
    }

    // Create debug utils messenger
    if (debugUtilsSupported) {
        XrDebugUtilsMessengerCreateInfoEXT utilsMessengerCreateInfo = { XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        utilsMessengerCreateInfo.messageSeverities = XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        utilsMessengerCreateInfo.messageTypes = XR_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_CONFORMANCE_BIT_EXT;
        utilsMessengerCreateInfo.userCallback = &XR_DebugUtilsMessengerCallback;
        func_xrCreateDebugUtilsMessengerEXT(m_instance, &utilsMessengerCreateInfo, &m_debugMessengerHandle);
    }

    // Get system information
    XrSystemGetInfo xrSystemGetInfo = { XR_TYPE_SYSTEM_GET_INFO };
    xrSystemGetInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    checkXRResult(xrGetSystem(m_instance, &xrSystemGetInfo, &m_systemId), "No (available) head mounted display found!");

    XrSystemProperties xrSystemProperties = { XR_TYPE_SYSTEM_PROPERTIES };
    checkXRResult(xrGetSystemProperties(m_instance, m_systemId, &xrSystemProperties), "Couldn't get system properties of the given VR headset!");
    m_capabilities.supportsOrientational = xrSystemProperties.trackingProperties.orientationTracking;
    m_capabilities.supportsPositional = xrSystemProperties.trackingProperties.positionTracking;

    XrInstanceProperties properties = { XR_TYPE_INSTANCE_PROPERTIES };
    checkXRResult(xrGetInstanceProperties(m_instance, &properties), "Failed to get runtime details using xrGetInstanceProperties!");

    XrViewConfigurationProperties stereoViewConfiguration = { XR_TYPE_VIEW_CONFIGURATION_PROPERTIES };
    checkXRResult(xrGetViewConfigurationProperties(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, &stereoViewConfiguration), "There's no VR headset available that allows stereo rendering!");
    m_capabilities.supportsMutatableFOV = stereoViewConfiguration.fovMutable;

    XrGraphicsRequirementsD3D12KHR graphicsRequirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_D3D12_KHR };
    checkXRResult(func_xrGetD3D12GraphicsRequirementsKHR(m_instance, m_systemId, &graphicsRequirements), "Couldn't get D3D12 requirements for the given VR headset!");
    m_capabilities.adapter = graphicsRequirements.adapterLuid;
    m_capabilities.minFeatureLevel = graphicsRequirements.minFeatureLevel;

    // Print configuration used, mostly for debugging purposes
    Log::print<INFO>("Acquired system to be used:");
    Log::print<INFO>(" - System Name: {}", xrSystemProperties.systemName); // Oculus Quest2
    Log::print<INFO>(" - Runtime Name: {}", properties.runtimeName); // Oculus
    Log::print<INFO>(" - Runtime Version: {}.{}.{}", XR_VERSION_MAJOR(properties.runtimeVersion), XR_VERSION_MINOR(properties.runtimeVersion), XR_VERSION_PATCH(properties.runtimeVersion));
    Log::print<INFO>(" - Supports Mutable FOV: {}", m_capabilities.supportsMutatableFOV ? "Yes" : "No");
    Log::print<INFO>(" - Supports Orientation Tracking: {}", xrSystemProperties.trackingProperties.orientationTracking ? "Yes" : "No");
    Log::print<INFO>(" - Supports Positional Tracking: {}", xrSystemProperties.trackingProperties.positionTracking ? "Yes" : "No");
    Log::print<INFO>(" - Supports D3D12 feature level {} or higher", graphicsRequirements.minFeatureLevel);

    m_capabilities.isOculusLinkRuntime = std::string(properties.runtimeName) == "Oculus";
    Log::print<INFO>(" - Using Meta Quest Link OpenXR runtime: {}", m_capabilities.isOculusLinkRuntime ? "Yes" : "No");
    
    m_capabilities.isMetaSimulator = std::string(properties.runtimeName).find("Meta XR Simulator") != std::string::npos;
}

OpenXR::~OpenXR() {
    this->m_renderer.reset();

    for (XrSpace& handSpace : m_inGameHandSpaces) {
        if (handSpace != XR_NULL_HANDLE) {
            xrDestroySpace(handSpace);
            handSpace = XR_NULL_HANDLE;
        }
    }
    for (XrSpace& aimSpace : m_inGameAimSpaces) {
        if (aimSpace != XR_NULL_HANDLE) {
            xrDestroySpace(aimSpace);
            aimSpace = XR_NULL_HANDLE;
        }
    }
    for (XrSpace& handSpace : m_inMenuHandSpaces) {
        if (handSpace != XR_NULL_HANDLE) {
            xrDestroySpace(handSpace);
            handSpace = XR_NULL_HANDLE;
        }
    }
    for (XrSpace& aimSpace : m_inMenuAimSpaces) {
        if (aimSpace != XR_NULL_HANDLE) {
            xrDestroySpace(aimSpace);
            aimSpace = XR_NULL_HANDLE;
        }
    }

    if (m_headSpace != XR_NULL_HANDLE) {
        xrDestroySpace(m_headSpace);
    }

    if (m_stageSpace != XR_NULL_HANDLE) {
        xrDestroySpace(m_stageSpace);
    }

    for (XrSpace retiredSpace : m_retiredStageSpaces) {
        xrDestroySpace(retiredSpace);
    }
    m_retiredStageSpaces.clear();

    if (m_session != XR_NULL_HANDLE) {
        xrDestroySession(m_session);
    }

    if (m_debugMessengerHandle != XR_NULL_HANDLE) {
        func_xrDestroyDebugUtilsMessengerEXT(m_debugMessengerHandle);
    }

    if (m_instance != XR_NULL_HANDLE) {
        xrDestroyInstance(m_instance);
    }
}

std::array<XrViewConfigurationView, 2> OpenXR::GetViewConfigurations() {
    uint32_t eyeViewsConfigurationCount = 0;
    checkXRResult(xrEnumerateViewConfigurationViews(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &eyeViewsConfigurationCount, nullptr), "Can't get number of individual views for stereo view available");
    checkAssert(eyeViewsConfigurationCount == 2, std::format("Expected 2 views for the stereo configuration but got {} which is unsupported!", eyeViewsConfigurationCount).c_str());

    std::array<XrViewConfigurationView, 2> xrViewConf = { XrViewConfigurationView{ XR_TYPE_VIEW_CONFIGURATION_VIEW }, XrViewConfigurationView{ XR_TYPE_VIEW_CONFIGURATION_VIEW } };
    checkXRResult(xrEnumerateViewConfigurationViews(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, eyeViewsConfigurationCount, &eyeViewsConfigurationCount, xrViewConf.data()), "Can't get individual views for stereo view available!");

    Log::print<INFO>("Swapchain configuration to be used:");
    Log::print<INFO>(" - [Left] Max View Resolution: w={}, h={} with {} samples", xrViewConf[0].maxImageRectWidth, xrViewConf[0].maxImageRectHeight, xrViewConf[0].maxSwapchainSampleCount);
    Log::print<INFO>(" - [Right] Max View Resolution: w={}, h={}  with {} samples", xrViewConf[1].maxImageRectWidth, xrViewConf[1].maxImageRectHeight, xrViewConf[1].maxSwapchainSampleCount);
    Log::print<INFO>(" - [Left] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[0].recommendedImageRectWidth, xrViewConf[0].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    Log::print<INFO>(" - [Right] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[1].recommendedImageRectWidth, xrViewConf[1].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    return xrViewConf;
}

bool OpenXR::CreateSession(const XrGraphicsBindingD3D12KHR& d3d12Binding, bool fatal) {
    Log::print<INFO>("Creating the OpenXR session...");

    XrSessionCreateInfo sessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    sessionCreateInfo.systemId = m_systemId;
    sessionCreateInfo.next = &d3d12Binding;
    sessionCreateInfo.createFlags = 0;
    XrResult result = xrCreateSession(m_instance, &sessionCreateInfo, &m_session);
    if (XR_FAILED(result)) {
        if (fatal) {
            checkXRResult(result, "Failed to create Vulkan-based OpenXR session!");
        }
        else {
            Log::print<ERROR>("Failed to create the OpenXR session (result {})! The runtime is probably gone.", (int)result);
            m_session = XR_NULL_HANDLE;
            return false;
        }
    }

    Log::print<INFO>("Creating the OpenXR spaces...");
    m_hasSeatedHeightCalibration = false;
    m_seenFirstLoadingScreen = false;
    m_lastSeenPlayMode.reset();
    ReplaceStageSpace(0.0f);

    XrReferenceSpaceCreateInfo headSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    headSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    headSpaceCreateInfo.poseInReferenceSpace = s_xrIdentityPose;
    result = xrCreateReferenceSpace(m_session, &headSpaceCreateInfo, &m_headSpace);
    if (XR_FAILED(result)) {
        if (fatal) {
            checkXRResult(result, "Failed to create reference space for head!");
        }
        else {
            Log::print<ERROR>("Failed to create the reference space for the head (result {})!", (int)result);
            return false;
        }
    }

    return true;
}

// Every call on a dead instance answers with an error, which makes this a cheap way to tell a
// runtime that retired one session from a runtime that is gone entirely.
bool OpenXR::InstanceIsAlive() const {
    if (m_instance == XR_NULL_HANDLE) {
        return false;
    }
    XrInstanceProperties properties = { XR_TYPE_INSTANCE_PROPERTIES };
    return XR_SUCCEEDED(xrGetInstanceProperties(m_instance, &properties));
}

// floorOffset is where the space's origin sits above the runtime's floor, so every located pose shifts together
void OpenXR::ReplaceStageSpace(float floorOffset) {
    XrReferenceSpaceCreateInfo stageSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    stageSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    stageSpaceCreateInfo.poseInReferenceSpace = s_xrIdentityPose;
    stageSpaceCreateInfo.poseInReferenceSpace.position.y = floorOffset;

    XrSpace newStageSpace = XR_NULL_HANDLE;
    checkXRResult(xrCreateReferenceSpace(m_session, &stageSpaceCreateInfo, &newStageSpace), "Failed to create reference space for stage!");

    // other threads may still be locating against the old handle, so it is only released with the session
    if (m_stageSpace != XR_NULL_HANDLE) {
        m_retiredStageSpaces.push_back(m_stageSpace);
    }
    m_stageSpace = newStageSpace;
    m_stageFloorOffset = floorOffset;
}

// Link's eye level above his feet while standing, tuned in-game (his bind pose puts them at 1.59m)
static constexpr float kLinkEyeHeight = 1.65f;

void OpenXR::RequestSeatedHeightCalibration(XrTime notBeforeTime) {
    m_seatedHeightCalibrationNotBeforeTime.store(notBeforeTime, std::memory_order_relaxed);
    m_seatedHeightCalibrationRequested.store(true, std::memory_order_release);
}

std::optional<float> OpenXR::GetCalibratedSeatedEyeHeight() const {
    if (!m_hasSeatedHeightCalibration) {
        return std::nullopt;
    }
    return kLinkEyeHeight + m_stageFloorOffset;
}

void OpenXR::UpdateSeatedHeightCalibration(XrTime predictedDisplayTime, const std::optional<XrSpaceLocation>& headLocation) {
    const PlayMode playMode = GetSettings().GetPlayMode();
    if (m_lastSeenPlayMode.has_value() && playMode != m_lastSeenPlayMode.value() && playMode == PlayMode::SEATED) {
        RequestSeatedHeightCalibration();
    }
    m_lastSeenPlayMode = playMode;

    if (!m_seenFirstLoadingScreen && CemuHooks::IsScreenVisible(ScreenId::LoadingWeapon_00)) {
        m_seenFirstLoadingScreen = true;
        if (playMode == PlayMode::SEATED) {
            RequestSeatedHeightCalibration();
        }
    }

    if (playMode != PlayMode::SEATED) {
        m_seatedHeightCalibrationRequested.store(false, std::memory_order_relaxed);
        if (m_stageFloorOffset != 0.0f) {
            ReplaceStageSpace(0.0f);
            Log::print<INFO>("Seated height calibration cleared, using the real height above the floor again");
        }
        m_hasSeatedHeightCalibration = false;
        return;
    }

    if (!m_seatedHeightCalibrationRequested.load(std::memory_order_acquire)) {
        return;
    }
    if (predictedDisplayTime < m_seatedHeightCalibrationNotBeforeTime.load(std::memory_order_relaxed)) {
        return;
    }
    if (!headLocation.has_value() || !std::isfinite(headLocation->pose.position.y)) {
        return;
    }

    const float realEyeHeight = headLocation->pose.position.y + m_stageFloorOffset;
    m_seatedHeightCalibrationRequested.store(false, std::memory_order_relaxed);
    ReplaceStageSpace(realEyeHeight - kLinkEyeHeight);
    m_hasSeatedHeightCalibration = true;
    Log::print<INFO>("Seated height calibrated: eyes are {:.2f}m above the floor, stage origin moved by {:.2f}m", realEyeHeight, m_stageFloorOffset);
}

void OpenXR::CreateActions() {
    Log::print<INFO>("Creating the OpenXR actions...");

    m_handPaths = { GetXRPath("/user/hand/left"), GetXRPath("/user/hand/right") };

    auto createAction = [this](const XrActionSet& actionSet, const char* id, const char* name, XrActionType actionType, XrAction& action) {
        XrActionCreateInfo actionInfo = { XR_TYPE_ACTION_CREATE_INFO };
        actionInfo.actionType = actionType;
        strncpy_s(actionInfo.actionName, id, XR_MAX_ACTION_NAME_SIZE-1);
        strncpy_s(actionInfo.localizedActionName, name, XR_MAX_LOCALIZED_ACTION_NAME_SIZE-1);
        actionInfo.countSubactionPaths = (uint32_t)m_handPaths.size();
        actionInfo.subactionPaths = m_handPaths.data();
        checkXRResult(xrCreateAction(actionSet, &actionInfo, &action), std::format("Failed to create action for {}", id).c_str());
    };

    {
        XrActionSetCreateInfo actionSetInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
        strcpy_s(actionSetInfo.actionSetName, "gameplay_fps");
        strcpy_s(actionSetInfo.localizedActionSetName, "Gameplay");
        actionSetInfo.priority = 0;
        checkXRResult(xrCreateActionSet(m_instance, &actionSetInfo, &m_gameplayActionSet), "Failed to create controller actions for gameplay_fps!");

        createAction(m_gameplayActionSet, "pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, m_inGameGripPoseAction);
        createAction(m_gameplayActionSet, "aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT, m_inGameAimPoseAction);
        createAction(m_gameplayActionSet, "move", "Move", XR_ACTION_TYPE_VECTOR2F_INPUT, m_moveAction);
        createAction(m_gameplayActionSet, "camera", "Camera Rotation", XR_ACTION_TYPE_VECTOR2F_INPUT, m_cameraAction);
        createAction(m_gameplayActionSet, "ingame_modmenu", "Mod Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inGame_modMenuAction);

        createAction(m_gameplayActionSet, "grab_interact", "Interact / Pick up objects from floor or weapon from body slots", XR_ACTION_TYPE_FLOAT_INPUT, m_grab_interactAction);
        createAction(m_gameplayActionSet, "jump", "Jump", XR_ACTION_TYPE_BOOLEAN_INPUT, m_jumpAction);
        createAction(m_gameplayActionSet, "run_interact_cancel", "Interact (Quick press) - Run/Cancel Interaction (Long press)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_run_interactAction);
        createAction(m_gameplayActionSet, "userune_dpadmenu", "Use Rune (Quick press) - Dpad menu (Long press)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_useRune_dpadMenu_Action);

        createAction(m_gameplayActionSet, "userighthanditem", "Use/Attack/Throw item held in right hand (Melee attacks/Draw bow/Throw object)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_useRightItemAction);
        createAction(m_gameplayActionSet, "uselefthanditem", "Use item held in left hand (Rune/Shield Parry)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_useLeftItemAction);

        createAction(m_gameplayActionSet, "crouch_scope", "Crouch (Quick press) - Open Scope (Long press)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_crouch_scopeAction);
        createAction(m_gameplayActionSet, "ingame_inventory_map", "Open Inventory (Quick press) - Open Map (Long press)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inGame_inventory_mapAction);

        createAction(m_gameplayActionSet, "rumble", "Rumble", XR_ACTION_TYPE_VIBRATION_OUTPUT, m_rumbleAction);
    }

    {
        XrActionSetCreateInfo actionSetInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
        strcpy_s(actionSetInfo.actionSetName, "menu");
        strcpy_s(actionSetInfo.localizedActionSetName, "Menu Navigation");
        actionSetInfo.priority = 0;
        checkXRResult(xrCreateActionSet(m_instance, &actionSetInfo, &m_menuActionSet), "Failed to create controller bindings for the menu!");

        createAction(m_menuActionSet, "pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, m_inMenuGripPoseAction);
        createAction(m_menuActionSet, "aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT, m_inMenuAimPoseAction);

        createAction(m_menuActionSet, "scroll", "Scroll (Right Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_scrollAction);
        createAction(m_menuActionSet, "navigate", "Navigate (Left Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_navigateAction);
        createAction(m_menuActionSet, "select", "Select (A Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_selectAction);
        createAction(m_menuActionSet, "cancel", "Back/Cancel (B Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_backAction);
        createAction(m_menuActionSet, "sort", "Sort (Y Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_sortAction);
        createAction(m_menuActionSet, "hold", "Hold (X Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_holdAction);
        createAction(m_menuActionSet, "left_grip", "Switch To Left Tab (L Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_leftGripAction);
        createAction(m_menuActionSet, "right_grip", "Switch To Right Tab (R Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_rightGripAction);
        createAction(m_menuActionSet, "lefttrigger", "Left Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_leftTriggerAction);
        createAction(m_menuActionSet, "righttrigger", "Right Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_rightTriggerAction);

        createAction(m_menuActionSet, "inmenu_inventory_map", "Close Inventory (Wii U - Start Button) - Close Map (Wii U - Select Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inMenu_inventory_mapAction);
        createAction(m_menuActionSet, "inmenu_modmenu", "Mod Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inMenu_modMenuAction);
    }

    // Suggested bindings are instance wide and the runtime refuses them once any of the action
    // sets has been attached to a session, so they are given once for the process, not per session.
    {
        ControllerActionBindings bindings;
        bindings.inGameGripPoseAction = m_inGameGripPoseAction;
        bindings.inGameAimPoseAction = m_inGameAimPoseAction;
        bindings.inMenuGripPoseAction = m_inMenuGripPoseAction;
        bindings.inMenuAimPoseAction = m_inMenuAimPoseAction;
        bindings.moveAction = m_moveAction;
        bindings.cameraAction = m_cameraAction;
        bindings.grab_interactAction = m_grab_interactAction;
        bindings.jumpAction = m_jumpAction;
        bindings.run_interactAction = m_run_interactAction;
        bindings.useRune_dpadMenu_Action = m_useRune_dpadMenu_Action;
        bindings.inGame_modMenuAction = m_inGame_modMenuAction;
        bindings.useLeftItemAction = m_useLeftItemAction;
        bindings.useRightItemAction = m_useRightItemAction;
        bindings.crouch_scopeAction = m_crouch_scopeAction;
        bindings.inGame_inventory_mapAction = m_inGame_inventory_mapAction;
        bindings.rumbleAction = m_rumbleAction;
        bindings.scrollAction = m_scrollAction;
        bindings.navigateAction = m_navigateAction;
        bindings.selectAction = m_selectAction;
        bindings.backAction = m_backAction;
        bindings.sortAction = m_sortAction;
        bindings.holdAction = m_holdAction;
        bindings.leftGripAction = m_leftGripAction;
        bindings.rightGripAction = m_rightGripAction;
        bindings.leftTriggerAction = m_leftTriggerAction;
        bindings.rightTriggerAction = m_rightTriggerAction;
        bindings.inMenu_modMenuAction = m_inMenu_modMenuAction;
        bindings.inMenu_inventory_mapAction = m_inMenu_inventory_mapAction;

        SuggestControllerBindings(m_instance, bindings, m_capabilities.supportsPicoController, m_capabilities.supportsPicoUltraController, m_capabilities.supportsCosmosController, m_capabilities.supportsHPMixedRealityController);
    }

    AttachActionsToSession();
}

// Attaching the action sets, creating their action spaces and the rumble manager all belongs to a
// session, so a recreated session needs this run again. The action sets and the actions themselves
// live on the instance and are only ever created once.
void OpenXR::AttachActionsToSession() {
    XrSessionActionSetsAttachInfo attachInfo = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    std::array actionSets = { m_gameplayActionSet, m_menuActionSet };
    attachInfo.countActionSets = (uint32_t)actionSets.size();
    attachInfo.actionSets = actionSets.data();
    // A recreated session gets attached to the very same action sets, which is allowed, but a
    // runtime that refuses it must not take Cemu down: input stays off until it works again.
    const XrResult attachResult = xrAttachSessionActionSets(m_session, &attachInfo);
    m_actionsAttached = XR_SUCCEEDED(attachResult);
    if (!m_actionsAttached) {
        Log::print<ERROR>("Failed to attach the action sets to the OpenXR session (result {})! Controller input stays off until the session is rebuilt again.", (int)attachResult);
        return;
    }

    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        XrActionSpaceCreateInfo createInfo = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        createInfo.action = m_inGameGripPoseAction;
        createInfo.subactionPath = m_handPaths[side];
        createInfo.poseInActionSpace = s_xrIdentityPose;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_inGameHandSpaces[side]), "Failed to create action space for hand pose!");

        createInfo.action = m_inGameAimPoseAction;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_inGameAimSpaces[side]), "Failed to create action space for aim pose!");
    }

    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        XrActionSpaceCreateInfo createInfo = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        createInfo.action = m_inMenuGripPoseAction;
        createInfo.subactionPath = m_handPaths[side];
        createInfo.poseInActionSpace = s_xrIdentityPose;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_inMenuHandSpaces[side]), "Failed to create action space for hand pose!");

        createInfo.action = m_inMenuAimPoseAction;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_inMenuAimSpaces[side]), "Failed to create action space for menu aim pose!");
    }

    // initialize rumble manager
    m_rumbleManager = std::make_unique<RumbleManager>(m_session, m_rumbleAction);
    m_rumbleManager.get()->initializeXrPathsAndStartTime(m_instance);

    m_capabilities.activeControllerType = DetectActiveControllerType(m_instance, m_session);
}

void CheckButtonState(bool buttonPressed, ButtonState& buttonState) {
    buttonState.resetFrameFlags();
    
    constexpr std::chrono::milliseconds longPressThreshold{ 250 };
    const bool down = buttonPressed;
    const auto now = std::chrono::steady_clock::now();
    
    // Rising edge - button just pressed
    if (down && !buttonState.wasDownLastFrame) {
        buttonState.pressStartTime = now;
    }
    
    // Pressed state - check for long press threshold
    if (down) {
        auto pressDuration = now - buttonState.pressStartTime;
        
        if (pressDuration >= longPressThreshold) {
            buttonState.lastEvent = ButtonState::Event::LongPress;
            buttonState.longFired = true;
            if (!buttonState.longFired_stillPressed) {
                buttonState.longFired_stillPressed = true;
                buttonState.longFired_actedUpon = true;
            }
        }
    }
    else {
        buttonState.longFired_stillPressed = false;
        buttonState.longFired_actedUpon = false;
    }
    
    // Falling edge - button just released
    if (!down && buttonState.wasDownLastFrame) {
        // Only register short press if long press didn't fire
        if (!buttonState.longFired) {
            buttonState.lastEvent = ButtonState::Event::ShortPress;
        }
        else
            buttonState.longFired = false; // reset long press fired flag
    }
    
    // Store current state for next frame
    buttonState.wasDownLastFrame = down;
}

std::optional<OpenXR::InputState> OpenXR::UpdateActions(XrTime predictedFrameTime, glm::fquat controllerRotation, bool inMenu) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::XRUpdateActions);

    // reading action state while the session is stopped is illegal and would fail every call
    if (!IsSessionRunning()) {
        return std::nullopt;
    }

    // syncing actions that were never attached to the session fails every single frame
    if (!m_actionsAttached) {
        return std::nullopt;
    }

    XrActiveActionSet activeActionSet = { (inMenu ? m_menuActionSet : m_gameplayActionSet), XR_NULL_PATH };

    XrActionsSyncInfo syncInfo = { XR_TYPE_ACTIONS_SYNC_INFO };
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &activeActionSet;
    checkXRResult(xrSyncActions(m_session, &syncInfo), "Failed to sync actions!");

    InputState newState = m_input.load();
    newState.shared.in_game = !inMenu;
    newState.shared.inputTime = predictedFrameTime;

    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        auto locatePose = [&](XrAction action, XrSpace handSpace, XrActionStatePose& poseState, XrSpaceLocation& outLocation, XrSpaceVelocity* outVelocity, const char* errorContext) {
            XrActionStateGetInfo getPoseInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getPoseInfo.action = action;
            getPoseInfo.subactionPath = m_handPaths[side];
            poseState = { XR_TYPE_ACTION_STATE_POSE };
            checkXRResult(xrGetActionStatePose(m_session, &getPoseInfo, &poseState), errorContext);

            outLocation = { XR_TYPE_SPACE_LOCATION };
            if (!poseState.isActive) {
                if (outVelocity != nullptr) {
                    *outVelocity = { XR_TYPE_SPACE_VELOCITY };
                }
                return;
            }

            XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
            XrSpaceVelocity spaceVelocity = { XR_TYPE_SPACE_VELOCITY };
            if (outVelocity != nullptr) {
                spaceLocation.next = &spaceVelocity;
                outVelocity->linearVelocity = { 0.0f, 0.0f, 0.0f };
                outVelocity->angularVelocity = { 0.0f, 0.0f, 0.0f };
            }

            checkXRResult(xrLocateSpace(handSpace, m_stageSpace, predictedFrameTime, &spaceLocation), "Failed to get location from controllers!");
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                outLocation = spaceLocation;

                if (outVelocity != nullptr && (spaceLocation.locationFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) != 0) {
                    // rotate angular velocity to world space when it's using a buggy runtime
                    auto mode = GetSettings().AngularVelocityFixer_GetMode();
                    bool isUsingQuestRuntime = m_capabilities.isOculusLinkRuntime;
                    if ((mode == AngularVelocityFixerMode::AUTO && isUsingQuestRuntime) || mode == AngularVelocityFixerMode::FORCED_ON) {
                        glm::vec3 angularVelocity = ToGLM(spaceVelocity.angularVelocity);
                        glm::fquat fix_angle = glm::fquat(0.924, -0.383, 0, 0);
                        angularVelocity = (ToGLM(spaceLocation.pose.orientation) * (fix_angle * angularVelocity)); // TODO: Contact other modders for similar issues with angular velocity being not on the grip rotation (quest 2) + Tune the angular velocity based on manually calculated on rotation positions
                        spaceVelocity.angularVelocity = { angularVelocity.x, angularVelocity.y, angularVelocity.z };
                    }

                    *outVelocity = spaceVelocity;
                }
            }
        };

        locatePose(
            newState.shared.in_game ? m_inGameGripPoseAction : m_inMenuGripPoseAction,
            newState.shared.in_game ? m_inGameHandSpaces[side] : m_inMenuHandSpaces[side],
            newState.shared.pose[side],
            newState.shared.poseLocation[side],
            &newState.shared.poseVelocity[side],
            "Failed to get pose of controller!"
        );

        locatePose(
            newState.shared.in_game ? m_inGameAimPoseAction : m_inMenuAimPoseAction,
            newState.shared.in_game ? m_inGameAimSpaces[side] : m_inMenuAimSpaces[side],
            newState.shared.aimPose[side],
            newState.shared.aimPoseLocation[side],
            nullptr,
            "Failed to get aim pose of controller!"
        );
    }
    // update shared actions
    XrActionStateGetInfo getInventoryMapInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
    getInventoryMapInfo.action = newState.shared.in_game ? m_inGame_inventory_mapAction : m_inMenu_inventory_mapAction;
    getInventoryMapInfo.subactionPath = XR_NULL_PATH;
    auto& inventory_mapAction = newState.shared.inventory_map;
    inventory_mapAction = { XR_TYPE_ACTION_STATE_BOOLEAN };
    checkXRResult(xrGetActionStateBoolean(m_session, &getInventoryMapInfo, &inventory_mapAction), "Failed to get inventory_help action value!");

    auto& inventory_mapButtonState = newState.shared.inventory_mapState;
    if (inventory_mapAction.isActive == XR_TRUE) {
        auto buttonPressed = inventory_mapAction.currentState == XR_TRUE;
        CheckButtonState(buttonPressed, inventory_mapButtonState); 
    }

    XrActionStateGetInfo getModMenuInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
    getModMenuInfo.action = newState.shared.in_game ? m_inGame_modMenuAction : m_inMenu_modMenuAction;
    getModMenuInfo.subactionPath = XR_NULL_PATH;
    auto& modMenuAction = newState.shared.modMenu;
    modMenuAction = { XR_TYPE_ACTION_STATE_BOOLEAN };
    checkXRResult(xrGetActionStateBoolean(m_session, &getModMenuInfo, &modMenuAction), "Failed to get mod menu action value!");

    auto& modMenuButtonState = newState.shared.modMenuState;
    if (modMenuAction.isActive == XR_TRUE) {
        auto buttonPressed = modMenuAction.currentState == XR_TRUE;
        CheckButtonState(buttonPressed, modMenuButtonState);
    }

    // update in-menu or in-game actions
    if (inMenu) {
        XrActionStateGetInfo getScrollInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getScrollInfo.action = m_scrollAction;
        newState.inMenu.scroll = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getScrollInfo, &newState.inMenu.scroll), "Failed to get navigate action value!");

        XrActionStateGetInfo getNavigationInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getNavigationInfo.action = m_navigateAction;
        newState.inMenu.navigate = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getNavigationInfo, &newState.inMenu.navigate), "Failed to get select action value!");

        XrActionStateGetInfo getSelectInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getSelectInfo.action = m_selectAction;
        newState.inMenu.select = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getSelectInfo, &newState.inMenu.select), "Failed to get select action value!");

        XrActionStateGetInfo getBackInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getBackInfo.action = m_backAction;
        newState.inMenu.back = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getBackInfo, &newState.inMenu.back), "Failed to get back action value!");

        XrActionStateGetInfo getSortInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getSortInfo.action = m_sortAction;
        newState.inMenu.sort = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getSortInfo, &newState.inMenu.sort), "Failed to get sort action value!");

        XrActionStateGetInfo getHoldInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getHoldInfo.action = m_holdAction;
        newState.inMenu.hold = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getHoldInfo, &newState.inMenu.hold), "Failed to get hold action value!");

        auto& holdButtonState = newState.inMenu.holdState;
        if (newState.inMenu.hold.isActive == XR_TRUE) {
            auto buttonPressed = newState.inMenu.hold.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, holdButtonState);
        }

        XrActionStateGetInfo getLeftGripInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getLeftGripInfo.action = m_leftGripAction;
        newState.inMenu.leftGrip = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getLeftGripInfo, &newState.inMenu.leftGrip), "Failed to get left grip action value!");

        if (newState.inMenu.leftGrip.currentState == XR_TRUE) {
            newState.shared.lastPickupSide = OpenXR::EyeSide::LEFT;
        }

        XrActionStateGetInfo getRightGripInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRightGripInfo.action = m_rightGripAction;
        newState.inMenu.rightGrip = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRightGripInfo, &newState.inMenu.rightGrip), "Failed to get right grip action value!");

        if (newState.inMenu.rightGrip.currentState == XR_TRUE) {
            newState.shared.lastPickupSide = OpenXR::EyeSide::RIGHT;
        }

        XrActionStateGetInfo getLeftTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getLeftTriggerInfo.action = m_leftTriggerAction;
        getLeftTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inMenu.leftTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getLeftTriggerInfo, &newState.inMenu.leftTrigger), "Failed to get left trigger action value!");

        XrActionStateGetInfo getRightTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRightTriggerInfo.action = m_rightTriggerAction;
        getRightTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inMenu.rightTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRightTriggerInfo, &newState.inMenu.rightTrigger), "Failed to get right trigger action value!");
    }
    else {
        for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
            XrActionStateGetInfo getGrabInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getGrabInfo.action = m_grab_interactAction;
            getGrabInfo.subactionPath = m_handPaths[side];
            newState.inGame.grab[side] = { XR_TYPE_ACTION_STATE_FLOAT };
            checkXRResult(xrGetActionStateFloat(m_session, &getGrabInfo, &newState.inGame.grab[side]), "Failed to get grab action value!");

            auto& buttonState = newState.inGame.grabState[side];
            if (newState.inGame.grab[side].isActive == XR_TRUE) {
                auto buttonPressed = newState.inGame.grab[side].currentState > 0.51f;
                CheckButtonState(buttonPressed, buttonState);
            }
        }

        XrActionStateGetInfo getCrouchScopeInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getCrouchScopeInfo.action = m_crouch_scopeAction;
        getCrouchScopeInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.crouch_scope = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getCrouchScopeInfo, &newState.inGame.crouch_scope), "Failed to get crouch and map action value!");

        auto& crouch_scopeButtonState = newState.inGame.crouch_scopeState;
        if (newState.inGame.crouch_scope.isActive == XR_TRUE) {
            auto buttonPressed = newState.inGame.crouch_scope.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, crouch_scopeButtonState);
        }

        XrActionStateGetInfo getMoveInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getMoveInfo.action = m_moveAction;
        newState.inGame.move = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getMoveInfo, &newState.inGame.move), "Failed to get move action value!");

        XrActionStateGetInfo getCameraInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getCameraInfo.action = m_cameraAction;
        newState.inGame.camera = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getCameraInfo, &newState.inGame.camera), "Failed to get camera action value!");

        XrActionStateGetInfo getJumpInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getJumpInfo.action = m_jumpAction;
        getJumpInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.jump_cancel = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getJumpInfo, &newState.inGame.jump_cancel), "Failed to get jump action value!");

        XrActionStateGetInfo getRunInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRunInfo.action = m_run_interactAction;
        getRunInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.run_interact = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRunInfo, &newState.inGame.run_interact), "Failed to get run action value!");

        auto& runButtonState = newState.inGame.runState;
        if (newState.inGame.run_interact.isActive == XR_TRUE) {
            auto buttonPressed = newState.inGame.run_interact.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, runButtonState);
        }

        XrActionStateGetInfo getUseRuneInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getUseRuneInfo.action = m_useRune_dpadMenu_Action;
        getUseRuneInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.useRune_dpadMenu = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getUseRuneInfo, &newState.inGame.useRune_dpadMenu), "Failed to get use rune action value!");

        auto& useRuneButtonState = newState.inGame.useRune_runeMenuState;
        if (newState.inGame.useRune_dpadMenu.isActive == XR_TRUE) {
            auto buttonPressed = newState.inGame.useRune_dpadMenu.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, useRuneButtonState);
        }

        XrActionStateGetInfo getUseRightItemInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getUseRightItemInfo.action = m_useRightItemAction;
        getUseRightItemInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.useRightItem = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getUseRightItemInfo, &newState.inGame.useRightItem), "Failed to get useRightItem action value!");

        XrActionStateGetInfo getUseLeftItemInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getUseLeftItemInfo.action = m_useLeftItemAction;
        getUseLeftItemInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.useLeftItem = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getUseLeftItemInfo, &newState.inGame.useLeftItem), "Failed to get useLeftItem action value!");
    }
    this->m_input.store(newState);
    return newState;
}


std::optional<XrSpaceLocation> OpenXR::UpdateSpaces(XrTime predictedDisplayTime) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::XRUpdateSpaces);

    if (!IsSessionRunning()) {
        return std::nullopt;
    }

    XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
    if (XrResult result = xrLocateSpace(m_headSpace, m_stageSpace, predictedDisplayTime, &spaceLocation); XR_SUCCEEDED(result)) {
        if (result != XR_ERROR_TIME_INVALID) {
            checkXRResult(result, "Failed to get space location!");
        }
        checkXRResult(result, "Failed to get space location!");
    }
    if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0)
        return std::nullopt;

    return spaceLocation;
}

bool OpenXR::IsSessionRunning() const {
    return m_renderer && m_renderer->IsSessionRunning();
}

std::optional<std::array<XrView, 2>> OpenXR::GetPoses(long frameIdx) const {
    return m_renderer ? m_renderer->GetPoses(frameIdx) : std::nullopt;
}

std::optional<glm::fmat4> OpenXR::GetMiddlePose(long frameIdx) const {
    return m_renderer ? m_renderer->GetMiddlePose(frameIdx) : std::nullopt;
}

void OpenXR::ProcessEvents() {
    auto processSessionStateChangedEvent = [this](XrEventDataSessionStateChanged* stateChangedEvent) {
        // a session that was already replaced can still have events in the queue and acting on those
        // would tear down the session that is current now
        if (stateChangedEvent->session != m_session) {
            Log::print<INFO>("OpenXR reported a session state change for a session that was already replaced, ignoring it!");
            return;
        }

        switch (stateChangedEvent->state) {
            case XR_SESSION_STATE_IDLE:
                Log::print<INFO>("OpenXR has indicated that the session is idle!");
                m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
                break;
            case XR_SESSION_STATE_READY: {
                Log::print<INFO>("OpenXR has indicated that the session is ready!");
                m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
                if (m_sessionExitRequested) {
                    // the runtime asked this application to exit, so a session is not ours to take
                    Log::print<WARNING>("The runtime retired this application, the session is not begun!");
                }
                else if (m_renderer) {
                    // The runtime stopped the session before (headset asleep, focus lost, ...) and now
                    // hands it back: without beginning it again no frame is ever waited on, so neither
                    // the headset image nor the controller input comes back.
                    Log::print<INFO>("OpenXR has handed the session back, re-beginning it!");
                    if (!m_renderer->BeginSession()) {
                        Log::print<WARNING>("Failed to re-begin the OpenXR session! BetterVR stays paused until the runtime offers the session again.");
                    }
                }
                else {
                    // The constructor begins the session and throws if that fails, and this runs on
                    // the thread presenting Cemu's frames, so the throw has to stop here.
                    try {
                        m_renderer = std::make_unique<RND_Renderer>(m_session);
                        // The renderer of the replaced session is only kept around because its frame
                        // pump may still be blocked inside xrWaitFrame; its ImGui overlay belongs to
                        // Cemu's framebuffer rather than to the session, so it moves over to the new
                        // renderer. Two overlays would mean two ImGui contexts in one process, and
                        // the moment the retired renderer dies the context every hook uses is gone.
                        if (m_retiredRenderer) {
                            if (auto overlay = m_retiredRenderer->ReleaseImGuiOverlay(); overlay != nullptr) {
                                m_renderer->SetImGuiOverlay(std::move(overlay));
                                m_renderer->TakeImGuiFrameStateFrom(*m_retiredRenderer);
                                Log::print<INFO>("The ImGui overlay of the replaced session was carried over to the new one!");
                            }
                        }
                    }
                    catch (const std::exception& e) {
                        Log::print<ERROR>("Failed to create the renderer for the OpenXR session: {}", e.what());
                        m_sessionGraceDeadlineMs.store(SteadyNowMs() + kLostSessionGraceMs, std::memory_order_relaxed);
                    }
                }
                break;
            }
            case XR_SESSION_STATE_SYNCHRONIZED:
                Log::print<INFO>("OpenXR has indicated that the session is synchronized!");
                break;
            case XR_SESSION_STATE_FOCUSED:
                Log::print<INFO>("OpenXR has indicated that the session is focused!");
                break;
            case XR_SESSION_STATE_VISIBLE:
                Log::print<INFO>("OpenXR has indicated that the session should be visible!");
                break;
            case XR_SESSION_STATE_STOPPING:
                Log::print<INFO>("OpenXR has indicated that the session should be ended!");
                if (m_renderer) {
                    m_renderer->EndSession();
                }
                if (m_rumbleManager) {
                    m_rumbleManager->SetSessionActive(false);
                }
                break;
            case XR_SESSION_STATE_EXITING:
                Log::print<INFO>("OpenXR has indicated that the session should be destroyed!");
                // This is the state that tells the application to exit: a session that reached it can
                // never be begun again. The runtime walks the app through it when the app is closed in
                // the headset, and a freshly created session is retired again after a few seconds,
                // which keeps the app alive in the headset instead of closing it. Sleeping never sends
                // this state, a sleeping headset only sends STOPPING and hands the session back later,
                // so there is nothing to recover here and Cemu has to follow the runtime.
                m_sessionExitRequested.store(true, std::memory_order_relaxed);
                m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
                if (m_renderer) {
                    m_renderer->EndSession();
                }
                if (m_rumbleManager) {
                    m_rumbleManager->SetSessionActive(false);
                }
                Log::print<WARNING>("The runtime retired the OpenXR session, closing Cemu!");
                // an exception is thrown here instead of using exit() to allow Cemu to ideally gracefully shutdown
                PostMessage(CemuHooks::m_cemuTopWindow, WM_CLOSE, 0, 0);
                break;
            // note: the OpenXR headers this project builds against stop at XR_SESSION_STATE_EXITING,
            // the later ENDING / DESTROYED states land in the default branch below and are only ever
            // reached after one of the paths above already let go of the session
            case XR_SESSION_STATE_LOSS_PENDING:
                Log::print<INFO>("OpenXR has indicated that the session is going to be lost!");
                // stop using the session that is about to disappear; the runtime is expected to walk
                // the session through IDLE -> READY again, at which point it gets begun once more
                if (m_renderer) {
                    m_renderer->EndSession();
                }
                if (m_rumbleManager) {
                    m_rumbleManager->SetSessionActive(false);
                }
                // the runtime is not obliged to hand a usable session back after this, so start the
                // clock on rebuilding the session ourselves
                m_sessionGraceDeadlineMs.store(SteadyNowMs() + kLostSessionGraceMs, std::memory_order_relaxed);
                break;
            default:
                Log::print<INFO>("OpenXR has indicated that an unknown session state has occurred!");
                break;
        }
    };

    XrEventDataBuffer eventData = { XR_TYPE_EVENT_DATA_BUFFER };
    XrResult result = xrPollEvent(m_instance, &eventData);

    while (result == XR_SUCCESS) {
        switch (eventData.type) {
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
                processSessionStateChangedEvent((XrEventDataSessionStateChanged*)&eventData);
                break;
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                Log::print<WARNING>("OpenXR has indicated that the instance is going to be lost!");
                break;
            case XR_TYPE_EVENT_DATA_EVENTS_LOST:
                Log::print<WARNING>("OpenXR has indicated that events are being lost!");
                break;
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
                Log::print<INFO>("OpenXR has indicated that the interaction profile has changed, re-detecting controller type...");
                m_capabilities.activeControllerType = DetectActiveControllerType(m_instance, m_session);
                break;
            }
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
                auto* spaceChangeEvent = (XrEventDataReferenceSpaceChangePending*)&eventData;
                Log::print<WARNING>("OpenXR has indicated that reference space {} has changed (recenter), seated height will be recalibrated!", std::to_underlying(spaceChangeEvent->referenceSpaceType));
                RequestSeatedHeightCalibration(spaceChangeEvent->changeTime);
                break;
            }
            default:
                Log::print<WARNING>("OpenXR has indicated that an unknown event with type {} has occurred!", std::to_underlying(eventData.type));
                break;
        }

        eventData = { XR_TYPE_EVENT_DATA_BUFFER };
        result = xrPollEvent(m_instance, &eventData);
    }

    // the pumps of a replaced session outlive it by the length of their blocked wait
    if (m_retiredRenderer && SteadyNowMs() > m_retiredRendererDestroyAfterMs) {
        m_retiredRenderer.reset();
    }

    // A session the runtime refuses to produce frames for is beyond repair once the headset has
    // been asleep for a while, and waiting for the runtime to hand a working one back has not
    // worked out either, so rebuild the session from scratch in that case.
    const bool framesStopped = m_renderer && m_renderer->NeedsSessionRestart();
    const long long graceDeadline = m_sessionGraceDeadlineMs.load(std::memory_order_relaxed);
    const bool lostAndNotComingBack = graceDeadline != 0 && SteadyNowMs() > graceDeadline;
    if (m_sessionExitRequested) {
        // the runtime closed this application, rebuilding a session would only put it back into the
        // headset; Cemu is already being closed by the state handler above
        m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
    }
    else if (framesStopped || lostAndNotComingBack) {
        if (m_sessionRestarts >= kMaxSessionRestarts) {
            static long long s_lastGiveUpLogMs = 0;
            if (SteadyNowMs() - s_lastGiveUpLogMs > 30000) {
                s_lastGiveUpLogMs = SteadyNowMs();
                Log::print<ERROR>("Giving up on recreating the OpenXR session after {} attempts, the headset display stays off until Cemu is restarted!", m_sessionRestarts);
            }
            m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
            // an instance that is gone cannot ever hand a display back, so the game is not worth
            // keeping open in that case
            if (!InstanceIsAlive()) {
                Log::print<WARNING>("The OpenXR instance is gone, closing Cemu!");
                PostMessage(CemuHooks::m_cemuTopWindow, WM_CLOSE, 0, 0);
            }
        }
        else {
            Log::print<WARNING>("Rebuilding the OpenXR session (attempt {}) because {}!", m_sessionRestarts + 1,
                framesStopped ? "the runtime stopped producing frames for it" : "the runtime lost it and did not hand it back");
            ++m_sessionRestarts;
            m_lastSessionRestartMs = SteadyNowMs();
            m_sessionGraceDeadlineMs.store(0, std::memory_order_relaxed);
            RecreateSession();
        }
    }

    // a session that has been running happily for a while earns another set of attempts
    if (m_sessionRestarts > 0 && m_lastSessionRestartMs != 0 && SteadyNowMs() - m_lastSessionRestartMs > 60000 &&
        m_renderer && m_renderer->ProducedFrameRecently()) {
        m_sessionRestarts = 0;
    }
}

void OpenXR::RecreateSession() {
    if (m_sessionExitRequested) {
        Log::print<WARNING>("The runtime retired this application, the OpenXR session is not recreated!");
        return;
    }

    Log::print<WARNING>("Recreating the OpenXR session...");

    // This runs on the thread that presents Cemu's frames and the creation calls below throw on
    // failure, so a runtime that refuses a new session must not take the emulator down with it.
    try {
        if (m_rumbleManager) {
            m_rumbleManager->SetSessionActive(false);
            m_rumbleManager.reset();
        }

        // A replaced renderer cannot be destroyed right away: one of its frame pumps may still be
        // blocked inside xrWaitFrame on the dying session and would wake up on freed memory. It
        // gets retired here and destroyed once that wait has certainly returned.
        if (m_renderer) {
            m_renderer->AbandonSession();
            // The layers of the renderer hold the swapchains this session owns, and destroying the
            // session destroys those handles: the layers have to let go of them before that happens.
            m_renderer->AbandonSessionSwapchains();
            m_retiredRenderer = std::move(m_renderer);
            m_retiredRendererDestroyAfterMs = SteadyNowMs() + kRetiredRendererGraceMs;
        }

        if (m_session != XR_NULL_HANDLE) {
            xrDestroySession(m_session);
            m_session = XR_NULL_HANDLE;
        }

        // every space below belongs to the session that just went away, so it went with it
        m_headSpace = XR_NULL_HANDLE;
        m_stageSpace = XR_NULL_HANDLE;
        m_retiredStageSpaces.clear();
        for (XrSpace& space : m_inGameHandSpaces) space = XR_NULL_HANDLE;
        for (XrSpace& space : m_inGameAimSpaces) space = XR_NULL_HANDLE;
        for (XrSpace& space : m_inMenuHandSpaces) space = XR_NULL_HANDLE;
        for (XrSpace& space : m_inMenuAimSpaces) space = XR_NULL_HANDLE;

        XrGraphicsBindingD3D12KHR d3d12Binding = { XR_TYPE_GRAPHICS_BINDING_D3D12_KHR };
        d3d12Binding.device = VRManager::instance().D3D12->GetDevice();
        d3d12Binding.queue = VRManager::instance().D3D12->GetCommandQueue();

        CreateSession(d3d12Binding, false);
        if (m_session == XR_NULL_HANDLE) {
            // no session to attach anything to; the clock below fires another attempt
            m_sessionGraceDeadlineMs.store(SteadyNowMs() + kNewSessionGraceMs, std::memory_order_relaxed);
            return;
        }
        AttachActionsToSession();

        // The runtime walks the new session through READY on its own, which is where the renderer
        // and the layers get picked back up; if it never does, the clock above fires another rebuild.
        m_sessionGraceDeadlineMs.store(SteadyNowMs() + kNewSessionGraceMs, std::memory_order_relaxed);
    }
    catch (const std::exception& e) {
        Log::print<ERROR>("Rebuilding the OpenXR session failed: {}", e.what());
    }
}
