#include "openxr.h"
#include "instance.h"


static void XR_DebugUtilsMessengerCallback(XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageType, const XrDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData) {
    Log::print("[OpenXR Debug Utils] Function {}: {}", callbackData->functionName, callbackData->message);
}

OpenXR::OpenXR() {
    uint32_t xrExtensionCount = 0;
    xrEnumerateInstanceExtensionProperties(NULL, 0, &xrExtensionCount, NULL);
    std::vector<XrExtensionProperties> instanceExtensions;
    instanceExtensions.resize(xrExtensionCount, { XR_TYPE_EXTENSION_PROPERTIES, NULL });
    checkXRResult(xrEnumerateInstanceExtensionProperties(NULL, xrExtensionCount, &xrExtensionCount, instanceExtensions.data()), "Couldn't enumerate OpenXR extensions!");

    // Create instance with required extensions
    bool d3d12Supported = false;
    bool depthSupported = false;
    bool timeConvSupported = false;
    bool debugUtilsSupported = false;
    for (XrExtensionProperties& extensionProperties : instanceExtensions) {
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
            debugUtilsSupported = true;
        }
    }

    if (!d3d12Supported) {
        Log::print("OpenXR runtime doesn't support D3D12 (XR_KHR_D3D12_ENABLE)!");
        throw std::runtime_error("Current OpenXR runtime doesn't support Direct3D 12 (XR_KHR_D3D12_ENABLE). See the Github page's troubleshooting section for a solution!");
    }
    if (!depthSupported) {
        Log::print("OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH)!");
        throw std::runtime_error("Current OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH). See the Github page's troubleshooting section for a solution!");
    }
    if (!timeConvSupported) {
        Log::print("OpenXR runtime doesn't support converting time from/to XrTime (XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME)!");
        throw std::runtime_error("Current OpenXR runtime doesn't support converting time from/to XrTime (XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME). See the Github page's troubleshooting section for a solution!");
    }
    if (!debugUtilsSupported) {
        Log::print("OpenXR runtime doesn't support debug utils (XR_EXT_DEBUG_UTILS)! Errors/debug information will no longer be able to be shown!");
    }

    std::vector<const char*> enabledExtensions = { XR_KHR_D3D12_ENABLE_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME };
    if (debugUtilsSupported) enabledExtensions.emplace_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);

    XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
    xrInstanceCreateInfo.createFlags = 0;
    xrInstanceCreateInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
    xrInstanceCreateInfo.enabledExtensionNames = enabledExtensions.data();
    xrInstanceCreateInfo.enabledApiLayerCount = 0;
    xrInstanceCreateInfo.enabledApiLayerNames = NULL;
    xrInstanceCreateInfo.applicationInfo = { "BetterVR OpenXR", 1, "Cemu", 1, XR_CURRENT_API_VERSION };
    checkXRResult(xrCreateInstance(&xrInstanceCreateInfo, &m_instance), "Failed to initialize the OpenXR instance!");

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
        utilsMessengerCreateInfo.userCallback = (PFN_xrDebugUtilsMessengerCallbackEXT)&XR_DebugUtilsMessengerCallback;
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
    Log::print("Acquired system to be used:");
    Log::print(" - System Name: {}", xrSystemProperties.systemName);
    Log::print(" - Runtime Name: {}", properties.runtimeName);
    Log::print(" - Runtime Version: {}.{}.{}", XR_VERSION_MAJOR(properties.runtimeVersion), XR_VERSION_MINOR(properties.runtimeVersion), XR_VERSION_PATCH(properties.runtimeVersion));
    Log::print(" - Supports Mutable FOV: {}", m_capabilities.supportsMutatableFOV ? "Yes" : "No");
    Log::print(" - Supports Orientation Tracking: {}", xrSystemProperties.trackingProperties.orientationTracking ? "Yes" : "No");
    Log::print(" - Supports Positional Tracking: {}", xrSystemProperties.trackingProperties.positionTracking ? "Yes" : "No");
    Log::print(" - Supports D3D12 feature level {} or higher", graphicsRequirements.minFeatureLevel);
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

    Log::print("Swapchain configuration to be used:");
    Log::print(" - [Left] Max View Resolution: w={}, h={} with {} samples", xrViewConf[0].maxImageRectWidth, xrViewConf[0].maxImageRectHeight, xrViewConf[0].maxSwapchainSampleCount);
    Log::print(" - [Right] Max View Resolution: w={}, h={}  with {} samples", xrViewConf[1].maxImageRectWidth, xrViewConf[1].maxImageRectHeight, xrViewConf[1].maxSwapchainSampleCount);
    Log::print(" - [Left] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[0].recommendedImageRectWidth, xrViewConf[0].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    Log::print(" - [Right] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[1].recommendedImageRectWidth, xrViewConf[1].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    return xrViewConf;
}

void OpenXR::CreateSession(const XrGraphicsBindingD3D12KHR& d3d12Binding) {
    Log::print("Creating the OpenXR session...");

    XrSessionCreateInfo sessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    sessionCreateInfo.systemId = m_systemId;
    sessionCreateInfo.next = &d3d12Binding;
    sessionCreateInfo.createFlags = 0;
    checkXRResult(xrCreateSession(m_instance, &sessionCreateInfo, &m_session), "Failed to create Vulkan-based OpenXR session!");

    Log::print("Creating the OpenXR spaces...");
    constexpr XrPosef xrIdentityPose = { .orientation = { .x = 0, .y = 0, .z = 0, .w = 1 }, .position = { .x = 0, .y = 0, .z = 0 } };

    XrReferenceSpaceCreateInfo stageSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    stageSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
    stageSpaceCreateInfo.poseInReferenceSpace = xrIdentityPose;
    checkXRResult(xrCreateReferenceSpace(m_session, &stageSpaceCreateInfo, &m_stageSpace), "Failed to create reference space for stage!");

    XrReferenceSpaceCreateInfo headSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    headSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    headSpaceCreateInfo.poseInReferenceSpace = xrIdentityPose;
    checkXRResult(xrCreateReferenceSpace(m_session, &headSpaceCreateInfo, &m_headSpace), "Failed to create reference space for head!");
}

bool firstInit = true;
void OpenXR::UpdateTime(EyeSide side, XrTime predictedDisplayTime) {
    if (firstInit) {
        firstInit = false;
        m_frameTimes[EyeSide::LEFT] = predictedDisplayTime;
        m_frameTimes[EyeSide::RIGHT] = predictedDisplayTime;
        return;
    }
    m_frameTimes[side] = predictedDisplayTime;
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
                Log::print("OpenXR has indicated that the session is idle!");
                break;
            case XR_SESSION_STATE_READY: {
                Log::print("OpenXR has indicated that the session is ready!");
                if (m_renderer) {
                    Log::print("OpenXR has indicated that the session is ready, but we already have a renderer!");
                }
                else {
                    m_renderer = std::make_unique<RND_Renderer>(m_session);
                    m_renderer->StartFrame();
                }
                break;
            }
            case XR_SESSION_STATE_SYNCHRONIZED:
                Log::print("OpenXR has indicated that the session is synchronized!");
                break;
            case XR_SESSION_STATE_FOCUSED:
                Log::print("OpenXR has indicated that the session is focused!");
                break;
            case XR_SESSION_STATE_VISIBLE:
                Log::print("OpenXR has indicated that the session should be visible!");
                break;
            case XR_SESSION_STATE_STOPPING:
                Log::print("OpenXR has indicated that the session should be ended!");
                this->m_renderer->StopRendering();
                m_session = XR_NULL_HANDLE;
                break;
            case XR_SESSION_STATE_EXITING:
                Log::print("OpenXR has indicated that the session should be destroyed! todo: Close Cemu maybe?");
                break;
            case XR_SESSION_STATE_LOSS_PENDING:
                Log::print("OpenXR has indicated that the session is going to be lost!");
                // todo: implement being able to continuously check if xrGetSystem returns and then reinitialize the session
                break;
            default:
                Log::print("OpenXR has indicated that an unknown session state has occurred!");
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
                Log::print("OpenXR has indicated that the instance is going to be lost!");
                break;
            case XR_TYPE_EVENT_DATA_EVENTS_LOST:
                Log::print("OpenXR has indicated that events are being lost!");
                break;
            default:
                Log::print("OpenXR has indicated that an unknown event has occurred!");
                break;
        }

        eventData = { XR_TYPE_EVENT_DATA_BUFFER };
        result = xrPollEvent(m_instance, &eventData);
    }
}
