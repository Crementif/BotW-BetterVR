#include "openxr.h"
#include "instance.h"

static XrBool32 XR_DebugUtilsMessengerCallback(XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageType, const XrDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData) {
    Log::print<XR_DEBUGUTILS>("[XR Debug Utils] Function {}: {}", callbackData->functionName, callbackData->message);
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
#if defined(_DEBUG)
            debugUtilsSupported = Log::isLogTypeEnabled<XR_DEBUGUTILS>();
#endif
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

    std::vector<const char*> enabledExtensions = { XR_KHR_D3D12_ENABLE_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME };
    if (debugUtilsSupported) enabledExtensions.emplace_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);

    XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
    xrInstanceCreateInfo.createFlags = 0;
    xrInstanceCreateInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
    xrInstanceCreateInfo.enabledExtensionNames = enabledExtensions.data();
    xrInstanceCreateInfo.enabledApiLayerCount = 0;
    xrInstanceCreateInfo.enabledApiLayerNames = NULL;
    xrInstanceCreateInfo.applicationInfo = { "BetterVR", 1, "Cemu", 1, XR_API_VERSION_1_0 };
    {
        XrResult result = xrCreateInstance(&xrInstanceCreateInfo, &m_instance);
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

    if (m_headSpace != XR_NULL_HANDLE) {
        xrDestroySpace(m_headSpace);
    }

    if (m_stageSpace != XR_NULL_HANDLE) {
        xrDestroySpace(m_stageSpace);
    }

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

void OpenXR::CreateSession(const XrGraphicsBindingD3D12KHR& d3d12Binding) {
    Log::print<INFO>("Creating the OpenXR session...");

    XrSessionCreateInfo sessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    sessionCreateInfo.systemId = m_systemId;
    sessionCreateInfo.next = &d3d12Binding;
    sessionCreateInfo.createFlags = 0;
    checkXRResult(xrCreateSession(m_instance, &sessionCreateInfo, &m_session), "Failed to create Vulkan-based OpenXR session!");

    Log::print<INFO>("Creating the OpenXR spaces...");
    XrReferenceSpaceCreateInfo stageSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    stageSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    stageSpaceCreateInfo.poseInReferenceSpace = s_xrIdentityPose;
    checkXRResult(xrCreateReferenceSpace(m_session, &stageSpaceCreateInfo, &m_stageSpace), "Failed to create reference space for stage!");

    XrReferenceSpaceCreateInfo headSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    headSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    headSpaceCreateInfo.poseInReferenceSpace = s_xrIdentityPose;
    checkXRResult(xrCreateReferenceSpace(m_session, &headSpaceCreateInfo, &m_headSpace), "Failed to create reference space for head!");
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

        createAction(m_gameplayActionSet, "pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT, m_gripPoseAction);
        createAction(m_gameplayActionSet, "aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT, m_aimPoseAction);
        createAction(m_gameplayActionSet, "move", "Move (Left Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_moveAction);
        createAction(m_gameplayActionSet, "camera", "Camera (Right Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_cameraAction);

        createAction(m_gameplayActionSet, "grab", "Grab", XR_ACTION_TYPE_FLOAT_INPUT, m_grabAction);
        createAction(m_gameplayActionSet, "interact", "Interact/Action (A Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_interactAction);
        createAction(m_gameplayActionSet, "jump", "Jump (X Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_jumpAction);
        createAction(m_gameplayActionSet, "crouchandscope", "Crouch or Scope (Left Thumbstick Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_crouchAndScopeAction);
        createAction(m_gameplayActionSet, "run", "Run (B Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_runAction);
        createAction(m_gameplayActionSet, "use_rune", "Use Rune (Left Bumper)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_useRuneAction);
        createAction(m_gameplayActionSet, "throw_weapon", "Throw Weapon (Right Bumper)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_throwWeaponAction);

        createAction(m_gameplayActionSet, "attack", "Attack (Y Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_attackAction);
        createAction(m_gameplayActionSet, "cancel", "Dash/Close (B Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_cancelAction);
        
        createAction(m_gameplayActionSet, "ingame_lefttrigger", "Left Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inGame_leftTriggerAction);
        createAction(m_gameplayActionSet, "ingame_righttrigger", "Right Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inGame_rightTriggerAction);

        createAction(m_gameplayActionSet, "ingame_mapandinventory", "Open/Close Map or Inventory (Select or Start Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inGame_mapAndInventoryAction);

        createAction(m_gameplayActionSet, "rumble", "Rumble", XR_ACTION_TYPE_VIBRATION_OUTPUT, m_rumbleAction);
    }

    {
        XrActionSetCreateInfo actionSetInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
        strcpy_s(actionSetInfo.actionSetName, "menu");
        strcpy_s(actionSetInfo.localizedActionSetName, "Menu Navigation");
        actionSetInfo.priority = 0;
        checkXRResult(xrCreateActionSet(m_instance, &actionSetInfo, &m_menuActionSet), "Failed to create controller bindings for the menu!");

        createAction(m_menuActionSet, "scroll", "Scroll (Left Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_scrollAction);
        createAction(m_menuActionSet, "navigate", "Navigate (Right Thumbstick)", XR_ACTION_TYPE_VECTOR2F_INPUT, m_navigateAction);
        createAction(m_menuActionSet, "select", "Select (A Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_selectAction);
        createAction(m_menuActionSet, "cancel", "Back/Cancel (B Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_backAction);
        createAction(m_menuActionSet, "sort", "Sort (Y Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_sortAction);
        createAction(m_menuActionSet, "hold", "Hold (X Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_holdAction);
        createAction(m_menuActionSet, "left_grip", "Switch To Left Tab (L Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_leftGripAction);
        createAction(m_menuActionSet, "right_grip", "Switch To Right Tab (R Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_rightGripAction);
        createAction(m_menuActionSet, "inmenu_lefttrigger", "Left Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inMenu_leftTriggerAction);
        createAction(m_menuActionSet, "inmenu_righttrigger", "Right Trigger", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inMenu_rightTriggerAction);

        createAction(m_menuActionSet, "inmenu_mapandinventory", "Open/Close Map (Select Button)", XR_ACTION_TYPE_BOOLEAN_INPUT, m_inMenu_mapAndInventoryAction);
    }

    {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_inGame_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/menu/click") },

            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/left/input/select/click") },
            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/right/input/select/click") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = m_inMenu_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/menu/click") },
            XrActionSuggestedBinding{ .action = m_selectAction, .binding = GetXRPath("/user/hand/right/input/select/click") },
            XrActionSuggestedBinding{ .action = m_backAction, .binding = GetXRPath("/user/hand/right/input/menu/click") },
            XrActionSuggestedBinding{ .action = m_sortAction, .binding = GetXRPath("/user/hand/left/input/select/click") },
            XrActionSuggestedBinding{ .action = m_holdAction, .binding = GetXRPath("/user/hand/left/input/menu/click") }
        };
        XrInteractionProfileSuggestedBinding suggestedBindingsInfo = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
        suggestedBindingsInfo.interactionProfile = GetXRPath("/interaction_profiles/khr/simple_controller");
        suggestedBindingsInfo.countSuggestedBindings = (uint32_t)suggestedBindings.size();
        suggestedBindingsInfo.suggestedBindings = suggestedBindings.data();
        checkXRResult(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindingsInfo), "Failed to suggest simple controller profile bindings!");
    }

    {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_inGame_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/thumbstick/click") },
            
            XrActionSuggestedBinding{ .action = m_moveAction, .binding = GetXRPath("/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_cameraAction, .binding = GetXRPath("/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_interactAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_cancelAction, .binding = GetXRPath("/user/hand/right/input/a/click") },

            XrActionSuggestedBinding{ .action = m_jumpAction, .binding = GetXRPath("/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = m_crouchAndScopeAction, .binding = GetXRPath("/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = m_runAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_attackAction, .binding = GetXRPath("/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = m_useRuneAction, .binding = GetXRPath("/user/hand/left/input/y/click") },

            XrActionSuggestedBinding{ .action = m_inGame_leftTriggerAction, .binding = GetXRPath("/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = m_inGame_rightTriggerAction, .binding = GetXRPath("/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = m_rumbleAction, .binding = GetXRPath("/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = m_rumbleAction, .binding = GetXRPath("/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = m_scrollAction, .binding = GetXRPath("/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_navigateAction, .binding = GetXRPath("/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_inMenu_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = m_selectAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_backAction, .binding = GetXRPath("/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = m_sortAction, .binding = GetXRPath("/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = m_holdAction, .binding = GetXRPath("/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = m_leftGripAction, .binding = GetXRPath("/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_rightGripAction, .binding = GetXRPath("/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_inMenu_leftTriggerAction, .binding = GetXRPath("/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = m_inMenu_rightTriggerAction, .binding = GetXRPath("/user/hand/right/input/trigger/value") },

        };
        XrInteractionProfileSuggestedBinding suggestedBindingsInfo = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
        suggestedBindingsInfo.interactionProfile = GetXRPath("/interaction_profiles/oculus/touch_controller");
        suggestedBindingsInfo.countSuggestedBindings = (uint32_t)suggestedBindings.size();
        suggestedBindingsInfo.suggestedBindings = suggestedBindings.data();
        checkXRResult(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindingsInfo), "Failed to suggest Oculus Touch Controller Profile bindings!");
    }

    {
        // todo: add bindings for the in-game map and inventory
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_gripPoseAction, .binding = GetXRPath("/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_aimPoseAction, .binding = GetXRPath("/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = m_inGame_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = m_moveAction, .binding = GetXRPath("/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_cameraAction, .binding = GetXRPath("/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/left/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = m_grabAction, .binding = GetXRPath("/user/hand/right/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = m_interactAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_cancelAction, .binding = GetXRPath("/user/hand/right/input/a/click") },

            XrActionSuggestedBinding{ .action = m_jumpAction, .binding = GetXRPath("/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = m_crouchAndScopeAction, .binding = GetXRPath("/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = m_runAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_attackAction, .binding = GetXRPath("/user/hand/left/input/a/click") },
            XrActionSuggestedBinding{ .action = m_useRuneAction, .binding = GetXRPath("/user/hand/left/input/b/click") },

            XrActionSuggestedBinding{ .action = m_inGame_leftTriggerAction, .binding = GetXRPath("/user/hand/left/input/trigger/click") },
            XrActionSuggestedBinding{ .action = m_inGame_rightTriggerAction, .binding = GetXRPath("/user/hand/right/input/trigger/click") },

            XrActionSuggestedBinding{ .action = m_rumbleAction, .binding = GetXRPath("/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = m_rumbleAction, .binding = GetXRPath("/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = m_scrollAction, .binding = GetXRPath("/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_navigateAction, .binding = GetXRPath("/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = m_inMenu_mapAndInventoryAction, .binding = GetXRPath("/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = m_selectAction, .binding = GetXRPath("/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = m_backAction, .binding = GetXRPath("/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = m_sortAction, .binding = GetXRPath("/user/hand/left/input/b/click") },
            XrActionSuggestedBinding{ .action = m_holdAction, .binding = GetXRPath("/user/hand/left/input/a/click") },
            XrActionSuggestedBinding{ .action = m_leftGripAction, .binding = GetXRPath("/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_rightGripAction, .binding = GetXRPath("/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = m_inMenu_leftTriggerAction, .binding = GetXRPath("/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = m_inMenu_rightTriggerAction, .binding = GetXRPath("/user/hand/right/input/trigger/value") },
        };
        XrInteractionProfileSuggestedBinding suggestedBindingsInfo = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
        suggestedBindingsInfo.interactionProfile = GetXRPath("/interaction_profiles/valve/index_controller");
        suggestedBindingsInfo.countSuggestedBindings = (uint32_t)suggestedBindings.size();
        suggestedBindingsInfo.suggestedBindings = suggestedBindings.data();
        checkXRResult(xrSuggestInteractionProfileBindings(m_instance, &suggestedBindingsInfo), "Failed to suggest Valve Index Controller Profile bindings!");
    }

    XrSessionActionSetsAttachInfo attachInfo = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    std::array actionSets = { m_gameplayActionSet, m_menuActionSet };
    attachInfo.countActionSets = (uint32_t)actionSets.size();
    attachInfo.actionSets = actionSets.data();
    checkXRResult(xrAttachSessionActionSets(m_session, &attachInfo), "Failed to attach action sets to session!");

    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        XrActionSpaceCreateInfo createInfo = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        createInfo.action = m_gripPoseAction;
        createInfo.subactionPath = m_handPaths[side];
        createInfo.poseInActionSpace = s_xrIdentityPose;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_handSpaces[side]), "Failed to create action space for hand pose!");
    }

    // initialize rumble manager
    m_rumbleManager = std::make_unique<RumbleManager>(m_session, m_rumbleAction);
    m_rumbleManager.get()->initializeXrPathsAndStartTime(m_instance);
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
        }
    }
    
    // Falling edge - button just released
    if (!down && buttonState.wasDownLastFrame) {
        // Only register short press if long press didn't fire
        if (!buttonState.longFired) {
            buttonState.lastEvent = ButtonState::Event::ShortPress;
        }
    }
    
    // Store current state for next frame
    buttonState.wasDownLastFrame = down;
}

std::optional<OpenXR::InputState> OpenXR::UpdateActions(XrTime predictedFrameTime, glm::fquat controllerRotation, bool inMenu) {
    XrActiveActionSet activeActionSet = { (inMenu ? m_menuActionSet : m_gameplayActionSet), XR_NULL_PATH };

    XrActionsSyncInfo syncInfo = { XR_TYPE_ACTIONS_SYNC_INFO };
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &activeActionSet;
    checkXRResult(xrSyncActions(m_session, &syncInfo), "Failed to sync actions!");

    const float playerHeightOffsetMeters = CemuHooks::GetSettings().playerHeightSetting.getLE();

    InputState newState = m_input.load();
    newState.inGame.in_game = !inMenu;
    newState.inGame.inputTime = predictedFrameTime;
    //newState.inGame.lastPickupSide = m_input.load().inGame.lastPickupSide;
    //newState.inGame.grabState = m_input.load().inGame.grabState;
    //newState.inGame.mapAndInventoryState = m_input.load().inGame.mapAndInventoryState;

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

        XrActionStateGetInfo getLeftGripInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getLeftGripInfo.action = m_leftGripAction;
        newState.inMenu.leftGrip = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getLeftGripInfo, &newState.inMenu.leftGrip), "Failed to get left grip action value!");

        if (newState.inMenu.leftGrip.currentState == XR_TRUE) {
            newState.inMenu.lastPickupSide = OpenXR::EyeSide::LEFT;
        }

        XrActionStateGetInfo getRightGripInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRightGripInfo.action = m_rightGripAction;
        newState.inMenu.rightGrip = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRightGripInfo, &newState.inMenu.rightGrip), "Failed to get right grip action value!");

        if (newState.inMenu.rightGrip.currentState == XR_TRUE) {
            newState.inMenu.lastPickupSide = OpenXR::EyeSide::RIGHT;
        }

        XrActionStateGetInfo getMapAndInventoryInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getMapAndInventoryInfo.action = m_inMenu_mapAndInventoryAction;
        newState.inMenu.mapAndInventory = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getMapAndInventoryInfo, &newState.inMenu.mapAndInventory), "Failed to get back action value!");

        XrActionStateGetInfo getLeftTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getLeftTriggerInfo.action = m_inMenu_leftTriggerAction;
        getLeftTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inMenu.leftTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getLeftTriggerInfo, &newState.inMenu.leftTrigger), "Failed to get left trigger action value!");

        XrActionStateGetInfo getRightTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRightTriggerInfo.action = m_inMenu_rightTriggerAction;
        getRightTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inMenu.rightTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRightTriggerInfo, &newState.inMenu.rightTrigger), "Failed to get right trigger action value!");
    }
    else {
        for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
            XrActionStateGetInfo getPoseInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getPoseInfo.action = m_gripPoseAction;
            getPoseInfo.subactionPath = m_handPaths[side];
            newState.inGame.pose[side] = { XR_TYPE_ACTION_STATE_POSE };
            checkXRResult(xrGetActionStatePose(m_session, &getPoseInfo, &newState.inGame.pose[side]), "Failed to get pose of controller!");

            if (newState.inGame.pose[side].isActive) {
                {
                    XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
                    XrSpaceVelocity spaceVelocity = { XR_TYPE_SPACE_VELOCITY };
                    spaceLocation.next = &spaceVelocity;
                    newState.inGame.poseVelocity[side].linearVelocity = { 0.0f, 0.0f, 0.0f };
                    newState.inGame.poseVelocity[side].angularVelocity = { 0.0f, 0.0f, 0.0f };
                    checkXRResult(xrLocateSpace(m_handSpaces[side], m_stageSpace, predictedFrameTime, &spaceLocation), "Failed to get location from controllers!");
                    if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                        // raise/lower the tracked pose in stage space
                        spaceLocation.pose.position.y += playerHeightOffsetMeters;
                        newState.inGame.poseLocation[side] = spaceLocation;

                        if ((spaceLocation.locationFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) != 0) {
                            // rotate angular velocity to world space when it's using a buggy runtime
                            auto mode = CemuHooks::GetSettings().AngularVelocityFixer_GetMode();
                            bool isUsingQuestRuntime = m_capabilities.isOculusLinkRuntime;
                            if ((mode == data_VRSettingsIn::AngularVelocityFixerMode::AUTO && isUsingQuestRuntime) || mode == data_VRSettingsIn::AngularVelocityFixerMode::FORCED_ON) {
                                glm::vec3 angularVelocity = ToGLM(spaceVelocity.angularVelocity);
                                glm::fquat fix_angle = glm::fquat(0.924, -0.383, 0, 0);
                                angularVelocity = (ToGLM(spaceLocation.pose.orientation) * (fix_angle * angularVelocity)); // TOD: Contact other modders for similar issues with angular velocity being not on the grip rotation (quest 2) + Tune the angular velocity based on manually calculated on rotation positions
                                spaceVelocity.angularVelocity = { angularVelocity.x, angularVelocity.y, angularVelocity.z };
                            }

                            newState.inGame.poseVelocity[side] = spaceVelocity;
                        }
                    }
                }
                {
                    XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
                    checkXRResult(xrLocateSpace(m_handSpaces[side], m_headSpace, predictedFrameTime, &spaceLocation), "Failed to get location from controllers!");
                    if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                        spaceLocation.pose.position.y += playerHeightOffsetMeters;
                        newState.inGame.hmdRelativePoseLocation[side] = spaceLocation;
                    }
                }
            }

            XrActionStateGetInfo getGrabInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getGrabInfo.action = m_grabAction;
            getGrabInfo.subactionPath = m_handPaths[side];
            newState.inGame.grab[side] = { XR_TYPE_ACTION_STATE_FLOAT };
            checkXRResult(xrGetActionStateFloat(m_session, &getGrabInfo, &newState.inGame.grab[side]), "Failed to get grab action value!");

            auto& action = newState.inGame.grab[side];
            auto& buttonState = newState.inGame.grabState[side];
            if (action.isActive == XR_TRUE) {
                auto buttonPressed = action.currentState > 0.75f;
                CheckButtonState(buttonPressed, buttonState);
            }
        }

        XrActionStateGetInfo getMapAndInventoryInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getMapAndInventoryInfo.action = m_inGame_mapAndInventoryAction;
        getMapAndInventoryInfo.subactionPath = m_handPaths[1];
        newState.inGame.mapAndInventory = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getMapAndInventoryInfo, &newState.inGame.mapAndInventory), "Failed to get mapAndInventory action value!");

        auto& mapAndInventoryAction = newState.inGame.mapAndInventory;
        auto& mapAndInventoryButtonState = newState.inGame.mapAndInventoryState;
        if (mapAndInventoryAction.isActive == XR_TRUE) {
            auto buttonPressed = mapAndInventoryAction.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, mapAndInventoryButtonState);
        }

        //XrActionStateGetInfo getInventory = { XR_TYPE_ACTION_STATE_GET_INFO };
        //getInventory.action = m_inGame_inventoryAction;
        //newState.inGame.inventory = { XR_TYPE_ACTION_STATE_BOOLEAN };
        //checkXRResult(xrGetActionStateBoolean(m_session, &getInventory, &newState.inGame.inventory), "Failed to get inventory action value!");

        XrActionStateGetInfo getMoveInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getMoveInfo.action = m_moveAction;
        newState.inGame.move = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getMoveInfo, &newState.inGame.move), "Failed to get move action value!");

        XrActionStateGetInfo getCameraInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getCameraInfo.action = m_cameraAction;
        newState.inGame.camera = { XR_TYPE_ACTION_STATE_VECTOR2F };
        checkXRResult(xrGetActionStateVector2f(m_session, &getCameraInfo, &newState.inGame.camera), "Failed to get camera action value!");

        XrActionStateGetInfo getInteractInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getInteractInfo.action = m_interactAction;
        getInteractInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.interact = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getInteractInfo, &newState.inGame.interact), "Failed to get interact action value!");

        XrActionStateGetInfo getCancelInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getCancelInfo.action = m_cancelAction;
        getCancelInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.cancel = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getCancelInfo, &newState.inGame.cancel), "Failed to get cancel action value!");

        XrActionStateGetInfo getJumpInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getJumpInfo.action = m_jumpAction;
        getJumpInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.jump = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getJumpInfo, &newState.inGame.jump), "Failed to get jump action value!");

        XrActionStateGetInfo getCrouchInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getCrouchInfo.action = m_crouchAndScopeAction;
        getCrouchInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.crouchAndScope = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getCrouchInfo, &newState.inGame.crouchAndScope), "Failed to get crouch action value!");

        auto& crouchAndScopeAction = newState.inGame.crouchAndScope;
        auto& crouchAndScopeButtonState = newState.inGame.crouchAndScopeState;
        if (crouchAndScopeAction.isActive == XR_TRUE) {
            auto buttonPressed = crouchAndScopeAction.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, crouchAndScopeButtonState);
        }

        XrActionStateGetInfo getRunInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRunInfo.action = m_runAction;
        getRunInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.run = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRunInfo, &newState.inGame.run), "Failed to get run action value!");

        auto& runAction = newState.inGame.run;
        auto& runButtonState = newState.inGame.runState;
        if (runAction.isActive == XR_TRUE) {
            auto buttonPressed = runAction.currentState == XR_TRUE;
            CheckButtonState(buttonPressed, runButtonState);
        }

        XrActionStateGetInfo getAttackInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getAttackInfo.action = m_attackAction;
        getAttackInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.attack = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getAttackInfo, &newState.inGame.attack), "Failed to get attack action value!");

        XrActionStateGetInfo getUseRuneInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getUseRuneInfo.action = m_useRuneAction;
        getUseRuneInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.useRune = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getUseRuneInfo, &newState.inGame.useRune), "Failed to get useRune action value!");

        XrActionStateGetInfo getThrowWeaponInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getThrowWeaponInfo.action = m_throwWeaponAction;
        getThrowWeaponInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.throwWeapon = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getThrowWeaponInfo, &newState.inGame.throwWeapon), "Failed to get throwWeapon action value!");

        XrActionStateGetInfo getLeftTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getLeftTriggerInfo.action = m_inGame_leftTriggerAction;
        getLeftTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.leftTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getLeftTriggerInfo, &newState.inGame.leftTrigger), "Failed to get left trigger action value!");

        XrActionStateGetInfo getRightTriggerInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
        getRightTriggerInfo.action = m_inGame_rightTriggerAction;
        getRightTriggerInfo.subactionPath = XR_NULL_PATH;
        newState.inGame.rightTrigger = { XR_TYPE_ACTION_STATE_BOOLEAN };
        checkXRResult(xrGetActionStateBoolean(m_session, &getRightTriggerInfo, &newState.inGame.rightTrigger), "Failed to get right trigger action value!");
    }
    this->m_input.store(newState);
    return newState;
}


std::optional<XrSpaceLocation> OpenXR::UpdateSpaces(XrTime predictedDisplayTime) {
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

void OpenXR::ProcessEvents() {
    auto processSessionStateChangedEvent = [this](XrEventDataSessionStateChanged* stateChangedEvent) {
        switch (stateChangedEvent->state) {
            case XR_SESSION_STATE_IDLE:
                Log::print<VERBOSE>("OpenXR has indicated that the session is idle!");
                break;
            case XR_SESSION_STATE_READY: {
                Log::print<VERBOSE>("OpenXR has indicated that the session is ready!");
                if (m_renderer) {
                    Log::print<WARNING>("OpenXR has indicated that the session is ready, but we already have a renderer!");
                }
                else {
                    m_renderer = std::make_unique<RND_Renderer>(m_session);
                }
                break;
            }
            case XR_SESSION_STATE_SYNCHRONIZED:
                Log::print<VERBOSE>("OpenXR has indicated that the session is synchronized!");
                break;
            case XR_SESSION_STATE_FOCUSED:
                Log::print<VERBOSE>("OpenXR has indicated that the session is focused!");
                break;
            case XR_SESSION_STATE_VISIBLE:
                Log::print<VERBOSE>("OpenXR has indicated that the session should be visible!");
                break;
            case XR_SESSION_STATE_STOPPING:
                Log::print<VERBOSE>("OpenXR has indicated that the session should be ended!");
                //this->m_renderer.reset();
                break;
            case XR_SESSION_STATE_EXITING:
                Log::print<VERBOSE>("OpenXR has indicated that the session should be destroyed!");
                // an exception is thrown here instead of using exit() to allow Cemu to ideally gracefully shutdown
                //throw std::runtime_error("BetterVR mod has been requested to exit by OpenXR!");
                break;
            case XR_SESSION_STATE_LOSS_PENDING:
                Log::print<VERBOSE>("OpenXR has indicated that the session is going to be lost!");
                // todo: implement being able to continuously check if xrGetSystem returns and then reinitialize the session
                break;
            default:
                Log::print<VERBOSE>("OpenXR has indicated that an unknown session state has occurred!");
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
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                Log::print<WARNING>("OpenXR has indicated that the interaction profile has changed!");
                break;
            case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
                Log::print<WARNING>("OpenXR has indicated that the reference space has changed!");
                break;
            default:
                Log::print<WARNING>("OpenXR has indicated that an unknown event with type {} has occurred!", std::to_underlying(eventData.type));
                break;
        }

        eventData = { XR_TYPE_EVENT_DATA_BUFFER };
        result = xrPollEvent(m_instance, &eventData);
    }
}
