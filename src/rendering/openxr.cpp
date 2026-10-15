#include "pch.h"

#include "openxr.h"
#include "instance.h"
#include "game/game.h"
#include "cemu/host_window.h"
#include "cemu/vulkan_loader.h"

#ifndef XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME
#define XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME "XR_BD_ultra_controller_interaction"
#endif

#if defined(_WIN32)
static constexpr const char* kTimeBridgeExtensionName = XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME;
static constexpr const char* kTimeBridgeToXrTimeName = "xrConvertWin32PerformanceCounterToTimeKHR";
static constexpr const char* kTimeBridgeFromXrTimeName = "xrConvertTimeToWin32PerformanceCounterKHR";
#else
static constexpr const char* kTimeBridgeExtensionName = XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME;
static constexpr const char* kTimeBridgeToXrTimeName = "xrConvertTimespecTimeToTimeKHR";
static constexpr const char* kTimeBridgeFromXrTimeName = "xrConvertTimeToTimespecTimeKHR";
#endif

template <size_t N>
static void CopyFixedString(char (&destination)[N], const char* source) {
    std::snprintf(destination, N, "%s", source);
}

static XrBool32 XR_DebugUtilsMessengerCallback(XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageType, const XrDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData) {
    Log::print<XR_DEBUGUTILS>("Function {}: {}", callbackData->functionName, callbackData->message);
    return XR_FALSE;
}

static std::vector<std::string> SplitExtensionList(const std::string& extensions) {
    std::vector<std::string> result;
    std::string current;
    for (const char character : extensions) {
        if (character == ' ' || character == '\0') {
            if (!current.empty()) {
                result.emplace_back(std::move(current));
            }
            current.clear();
        }
        else {
            current.push_back(character);
        }
    }
    if (!current.empty()) {
        result.emplace_back(std::move(current));
    }
    return result;
}

// what the player can do about it, shown in Cemu's error dialog
static std::string RuntimeFailureHelp(const char* what, XrResult result) {
    std::string help = what;
    switch (result) {
        case XR_ERROR_RUNTIME_UNAVAILABLE:
            help += " No OpenXR runtime is set up. Pick one in your headset's software (SteamVR, the Meta app, Virtual Desktop's VDXR, or Monado/WiVRn on Linux) and set it as the active OpenXR runtime.";
            break;
        case XR_ERROR_RUNTIME_FAILURE:
            help += " The active OpenXR runtime failed to start. Restart it or your PC, and check that the right runtime is active.";
            break;
        case XR_ERROR_FORM_FACTOR_UNAVAILABLE:
            help += " Make sure the headset is connected, switched on, awake and seen by the runtime.";
            break;
        case XR_ERROR_FORM_FACTOR_UNSUPPORTED:
            help += " The active OpenXR runtime doesn't support headsets.";
            break;
        case XR_ERROR_INSTANCE_LOST:
            help += " The OpenXR runtime stopped. Restart it and try again.";
            break;
        default:
            break;
    }
    return help + fmt::format(" (error {})\n\nCemu keeps running without VR.", (int32_t)result);
}

OpenXR::OpenXR() {
    uint32_t xrExtensionCount = 0;
    xrEnumerateInstanceExtensionProperties(NULL, 0, &xrExtensionCount, NULL);
    std::vector<XrExtensionProperties> instanceExtensions;
    instanceExtensions.resize(xrExtensionCount, { XR_TYPE_EXTENSION_PROPERTIES, NULL });
    {
        XrResult result = xrEnumerateInstanceExtensionProperties(NULL, xrExtensionCount, &xrExtensionCount, instanceExtensions.data());
        checkXRResult(result, RuntimeFailureHelp("Couldn't reach the OpenXR runtime.", result).c_str());
    }

    bool vulkanEnable2Supported = false;
    bool vulkanEnableSupported = false;
    bool depthSupported = false;
    bool timeBridgeSupported = false;
    bool debugUtilsSupported = false;
    for (XrExtensionProperties& extensionProperties : instanceExtensions) {
        Log::print<VERBOSE>("Found available OpenXR extension: {}", extensionProperties.extensionName);
        if (strcmp(extensionProperties.extensionName, XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME) == 0) {
            vulkanEnable2Supported = true;
        }
        if (strcmp(extensionProperties.extensionName, XR_KHR_VULKAN_ENABLE_EXTENSION_NAME) == 0) {
            vulkanEnableSupported = true;
        }
        if (strcmp(extensionProperties.extensionName, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME) == 0) {
            depthSupported = true;
        }
        else if (strcmp(extensionProperties.extensionName, kTimeBridgeExtensionName) == 0) {
            timeBridgeSupported = true;
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
        else if (strcmp(extensionProperties.extensionName, XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME) == 0) {
            m_capabilities.supportsEyeGaze = true;
        }
    }

    if (!vulkanEnable2Supported && !vulkanEnableSupported) {
        Log::print<ERROR>("OpenXR runtime doesn't support Vulkan (neither XR_KHR_vulkan_enable2 nor XR_KHR_vulkan_enable)!");
        checkAssert(false, "Current OpenXR runtime doesn't support Vulkan (XR_KHR_vulkan_enable2). See the Github page's troubleshooting section for a solution!");
    }
    m_capabilities.usesVulkanEnable2 = vulkanEnable2Supported;
    if (!vulkanEnable2Supported) {
        Log::print<WARNING>("OpenXR runtime doesn't support XR_KHR_vulkan_enable2, falling back to XR_KHR_vulkan_enable.");
    }
    if (!depthSupported) {
        Log::print<ERROR>("OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH)!");
        checkAssert(false, "Current OpenXR runtime doesn't support depth composition layers (XR_KHR_COMPOSITION_LAYER_DEPTH). See the Github page's troubleshooting section for a solution!");
    }
    if (!timeBridgeSupported) {
        Log::print<WARNING>("OpenXR runtime doesn't support converting time from/to XrTime ({}). Not required, as of this version.", kTimeBridgeExtensionName);
    }
    if (!debugUtilsSupported && Log::isLogTypeEnabled<XR_DEBUGUTILS>()) {
        Log::print<INFO>("OpenXR runtime doesn't support debug utils (XR_EXT_DEBUG_UTILS)! Errors/debug information will no longer be able to be shown!");
    }

    std::vector<const char*> enabledExtensions = { vulkanEnable2Supported ? XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME : XR_KHR_VULKAN_ENABLE_EXTENSION_NAME, XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME };
    if (timeBridgeSupported) enabledExtensions.emplace_back(kTimeBridgeExtensionName);
    if (debugUtilsSupported) enabledExtensions.emplace_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (m_capabilities.supportsPicoController) enabledExtensions.emplace_back(XR_BD_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsPicoUltraController) enabledExtensions.emplace_back(XR_BD_ULTRA_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsCosmosController) enabledExtensions.emplace_back(XR_HTC_VIVE_COSMOS_CONTROLLER_INTERACTION_EXTENSION_NAME);
    if (m_capabilities.supportsHPMixedRealityController) enabledExtensions.emplace_back(XR_EXT_HP_MIXED_REALITY_CONTROLLER_EXTENSION_NAME);
    if (m_capabilities.supportsEyeGaze) enabledExtensions.emplace_back(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);

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

        checkXRResult(result, RuntimeFailureHelp("The OpenXR runtime didn't start.", result).c_str());
    }

    if (m_capabilities.usesVulkanEnable2) {
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanGraphicsRequirements2KHR", (PFN_xrVoidFunction*)&func_xrGetVulkanGraphicsRequirements2KHR);
        xrGetInstanceProcAddr(m_instance, "xrCreateVulkanInstanceKHR", (PFN_xrVoidFunction*)&func_xrCreateVulkanInstanceKHR);
        xrGetInstanceProcAddr(m_instance, "xrCreateVulkanDeviceKHR", (PFN_xrVoidFunction*)&func_xrCreateVulkanDeviceKHR);
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanGraphicsDevice2KHR", (PFN_xrVoidFunction*)&func_xrGetVulkanGraphicsDevice2KHR);
        checkAssert(func_xrGetVulkanGraphicsRequirements2KHR != nullptr && func_xrCreateVulkanInstanceKHR != nullptr && func_xrCreateVulkanDeviceKHR != nullptr && func_xrGetVulkanGraphicsDevice2KHR != nullptr,
            "The OpenXR runtime advertises XR_KHR_vulkan_enable2 but doesn't expose all of its functions!");
    }
    else {
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanGraphicsRequirementsKHR", (PFN_xrVoidFunction*)&func_xrGetVulkanGraphicsRequirementsKHR);
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanInstanceExtensionsKHR", (PFN_xrVoidFunction*)&func_xrGetVulkanInstanceExtensionsKHR);
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanDeviceExtensionsKHR", (PFN_xrVoidFunction*)&func_xrGetVulkanDeviceExtensionsKHR);
        xrGetInstanceProcAddr(m_instance, "xrGetVulkanGraphicsDeviceKHR", (PFN_xrVoidFunction*)&func_xrGetVulkanGraphicsDeviceKHR);
        checkAssert(func_xrGetVulkanGraphicsRequirementsKHR != nullptr && func_xrGetVulkanInstanceExtensionsKHR != nullptr && func_xrGetVulkanDeviceExtensionsKHR != nullptr && func_xrGetVulkanGraphicsDeviceKHR != nullptr,
            "The OpenXR runtime advertises XR_KHR_vulkan_enable but doesn't expose all of its functions!");
    }
    if (timeBridgeSupported) {
        xrGetInstanceProcAddr(m_instance, kTimeBridgeFromXrTimeName, (PFN_xrVoidFunction*)&func_xrConvertTimeToPlatformTime);
        xrGetInstanceProcAddr(m_instance, kTimeBridgeToXrTimeName, (PFN_xrVoidFunction*)&func_xrConvertPlatformTimeToTime);
    }
    if (debugUtilsSupported) {
        xrGetInstanceProcAddr(m_instance, "xrCreateDebugUtilsMessengerEXT", (PFN_xrVoidFunction*)&func_xrCreateDebugUtilsMessengerEXT);
        xrGetInstanceProcAddr(m_instance, "xrDestroyDebugUtilsMessengerEXT", (PFN_xrVoidFunction*)&func_xrDestroyDebugUtilsMessengerEXT);
    }

    if (debugUtilsSupported) {
        XrDebugUtilsMessengerCreateInfoEXT utilsMessengerCreateInfo = { XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        utilsMessengerCreateInfo.messageSeverities = XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        utilsMessengerCreateInfo.messageTypes = XR_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_CONFORMANCE_BIT_EXT;
        utilsMessengerCreateInfo.userCallback = &XR_DebugUtilsMessengerCallback;
        func_xrCreateDebugUtilsMessengerEXT(m_instance, &utilsMessengerCreateInfo, &m_debugMessengerHandle);
    }

    XrSystemGetInfo xrSystemGetInfo = { XR_TYPE_SYSTEM_GET_INFO };
    xrSystemGetInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    const XrResult systemResult = xrGetSystem(m_instance, &xrSystemGetInfo, &m_systemId);
    checkXRResult(systemResult, RuntimeFailureHelp("The OpenXR runtime has no headset to offer.", systemResult).c_str());

    XrSystemProperties xrSystemProperties = { XR_TYPE_SYSTEM_PROPERTIES };
    // the extension is there for the runtime, this says whether the headset has the cameras
    XrSystemEyeGazeInteractionPropertiesEXT eyeGazeProperties = { XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT };
    if (m_capabilities.supportsEyeGaze) xrSystemProperties.next = &eyeGazeProperties;
    checkXRResult(xrGetSystemProperties(m_instance, m_systemId, &xrSystemProperties), "Couldn't get system properties of the given VR headset!");
    m_capabilities.supportsEyeGaze = m_capabilities.supportsEyeGaze && eyeGazeProperties.supportsEyeGazeInteraction;
    m_capabilities.supportsOrientational = xrSystemProperties.trackingProperties.orientationTracking;
    m_capabilities.supportsPositional = xrSystemProperties.trackingProperties.positionTracking;

    XrInstanceProperties properties = { XR_TYPE_INSTANCE_PROPERTIES };
    checkXRResult(xrGetInstanceProperties(m_instance, &properties), "Failed to get runtime details using xrGetInstanceProperties!");

    XrViewConfigurationProperties stereoViewConfiguration = { XR_TYPE_VIEW_CONFIGURATION_PROPERTIES };
    checkXRResult(xrGetViewConfigurationProperties(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, &stereoViewConfiguration), "There's no VR headset available that allows stereo rendering!");
    m_capabilities.supportsMutatableFOV = stereoViewConfiguration.fovMutable;

    // has to happen before the first xrCreateVulkanInstanceKHR
    XrGraphicsRequirementsVulkan2KHR graphicsRequirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR };
    if (m_capabilities.usesVulkanEnable2) {
        checkXRResult(func_xrGetVulkanGraphicsRequirements2KHR(m_instance, m_systemId, &graphicsRequirements), "Couldn't get Vulkan requirements for the given VR headset!");
    }
    else {
        checkXRResult(func_xrGetVulkanGraphicsRequirementsKHR(m_instance, m_systemId, &graphicsRequirements), "Couldn't get Vulkan requirements for the given VR headset!");
    }
    m_capabilities.minVulkanApiVersion = graphicsRequirements.minApiVersionSupported;
    m_capabilities.maxVulkanApiVersion = graphicsRequirements.maxApiVersionSupported;

    Log::print<INFO>("Acquired system to be used:");
    Log::print<INFO>(" - System Name: {}", xrSystemProperties.systemName); // Oculus Quest2
    Log::print<INFO>(" - Runtime Name: {}", properties.runtimeName); // Oculus
    Log::print<INFO>(" - Runtime Version: {}.{}.{}", XR_VERSION_MAJOR(properties.runtimeVersion), XR_VERSION_MINOR(properties.runtimeVersion), XR_VERSION_PATCH(properties.runtimeVersion));
    Log::print<INFO>(" - Supports Mutable FOV: {}", m_capabilities.supportsMutatableFOV ? "Yes" : "No");
    Log::print<INFO>(" - Supports Orientation Tracking: {}", xrSystemProperties.trackingProperties.orientationTracking ? "Yes" : "No");
    Log::print<INFO>(" - Supports Positional Tracking: {}", xrSystemProperties.trackingProperties.positionTracking ? "Yes" : "No");
    Log::print<INFO>(" - Supports Eye Tracking: {}", m_capabilities.supportsEyeGaze ? "Yes" : "No");
    Log::print<INFO>(" - Vulkan handshake: {}", m_capabilities.usesVulkanEnable2 ? XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME : XR_KHR_VULKAN_ENABLE_EXTENSION_NAME);
    Log::print<INFO>(" - Supports Vulkan {}.{}.{} up to {}.{}.{}",
        XR_VERSION_MAJOR(m_capabilities.minVulkanApiVersion), XR_VERSION_MINOR(m_capabilities.minVulkanApiVersion), XR_VERSION_PATCH(m_capabilities.minVulkanApiVersion),
        XR_VERSION_MAJOR(m_capabilities.maxVulkanApiVersion), XR_VERSION_MINOR(m_capabilities.maxVulkanApiVersion), XR_VERSION_PATCH(m_capabilities.maxVulkanApiVersion));

    m_capabilities.isOculusLinkRuntime = std::string(properties.runtimeName) == "Oculus";
    Log::print<INFO>(" - Using Meta Quest Link OpenXR runtime: {}", m_capabilities.isOculusLinkRuntime ? "Yes" : "No");
    
    m_capabilities.isMetaSimulator = std::string(properties.runtimeName).find("Meta XR Simulator") != std::string::npos;
}

OpenXR::~OpenXR() {
    m_rumbleManager.reset(); // join the worker while the session and haptic actions still exist
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
    if (m_eyeGazeSpace != XR_NULL_HANDLE) {
        xrDestroySpace(m_eyeGazeSpace);
        m_eyeGazeSpace = XR_NULL_HANDLE;
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

std::vector<std::string> OpenXR::GetRequiredVulkanInstanceExtensions() const {
    if (func_xrGetVulkanInstanceExtensionsKHR == nullptr) {
        return {};
    }

    uint32_t bufferSize = 0;
    checkXRResult(func_xrGetVulkanInstanceExtensionsKHR(m_instance, m_systemId, 0, &bufferSize, nullptr), "Couldn't count the Vulkan instance extensions the OpenXR runtime needs!");
    std::string buffer(bufferSize, '\0');
    checkXRResult(func_xrGetVulkanInstanceExtensionsKHR(m_instance, m_systemId, bufferSize, &bufferSize, buffer.data()), "Couldn't get the Vulkan instance extensions the OpenXR runtime needs!");
    return SplitExtensionList(buffer);
}

std::vector<std::string> OpenXR::GetRequiredVulkanDeviceExtensions() const {
    if (func_xrGetVulkanDeviceExtensionsKHR == nullptr) {
        return {};
    }

    uint32_t bufferSize = 0;
    checkXRResult(func_xrGetVulkanDeviceExtensionsKHR(m_instance, m_systemId, 0, &bufferSize, nullptr), "Couldn't count the Vulkan device extensions the OpenXR runtime needs!");
    std::string buffer(bufferSize, '\0');
    checkXRResult(func_xrGetVulkanDeviceExtensionsKHR(m_instance, m_systemId, bufferSize, &bufferSize, buffer.data()), "Couldn't get the Vulkan device extensions the OpenXR runtime needs!");
    return SplitExtensionList(buffer);
}

uint32_t OpenXR::ClampVulkanApiVersion(uint32_t requestedApiVersion) const {
    const uint32_t minVersion = VK_MAKE_API_VERSION(0, XR_VERSION_MAJOR(m_capabilities.minVulkanApiVersion), XR_VERSION_MINOR(m_capabilities.minVulkanApiVersion), 0);
    const uint32_t maxVersion = VK_MAKE_API_VERSION(0, XR_VERSION_MAJOR(m_capabilities.maxVulkanApiVersion), XR_VERSION_MINOR(m_capabilities.maxVulkanApiVersion), 0);

    uint32_t clampedApiVersion = requestedApiVersion;
    if (requestedApiVersion < minVersion) {
        clampedApiVersion = minVersion;
    }
    // a runtime that caps below Vulkan 1.1 can't work anyway, clamping would only make Cemu abort instead
    else if (requestedApiVersion > maxVersion && maxVersion >= VK_API_VERSION_1_1) {
        clampedApiVersion = maxVersion;
    }

    if (clampedApiVersion != requestedApiVersion) {
        Log::print<WARNING>("Clamping the requested Vulkan {}.{}.{} to {}.{}.{} to stay inside the OpenXR runtime's supported range",
            VK_API_VERSION_MAJOR(requestedApiVersion), VK_API_VERSION_MINOR(requestedApiVersion), VK_API_VERSION_PATCH(requestedApiVersion),
            VK_API_VERSION_MAJOR(clampedApiVersion), VK_API_VERSION_MINOR(clampedApiVersion), VK_API_VERSION_PATCH(clampedApiVersion));
    }
    else if (requestedApiVersion > maxVersion) {
        Log::print<WARNING>("The OpenXR runtime reports Vulkan {}.{} as its maximum, which is below the v1.1 the mod needs, so Cemu's requested version is kept",
            XR_VERSION_MAJOR(m_capabilities.maxVulkanApiVersion), XR_VERSION_MINOR(m_capabilities.maxVulkanApiVersion));
    }
    return clampedApiVersion;
}

VkResult OpenXR::CreateVulkanInstance(const VkInstanceCreateInfo& createInfo, const VkAllocationCallbacks* allocator, VkInstance* pInstance) {
    checkAssert(func_xrCreateVulkanInstanceKHR != nullptr, "The OpenXR runtime's xrCreateVulkanInstanceKHR is missing!");

    XrVulkanInstanceCreateInfoKHR vulkanInstanceCreateInfo = { XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR };
    vulkanInstanceCreateInfo.systemId = m_systemId;
    vulkanInstanceCreateInfo.createFlags = 0;
    vulkanInstanceCreateInfo.pfnGetInstanceProcAddr = VulkanLoader::GetInstanceProcAddr();
    vulkanInstanceCreateInfo.vulkanCreateInfo = &createInfo;
    vulkanInstanceCreateInfo.vulkanAllocator = allocator;

    VkResult vulkanResult = VK_ERROR_INITIALIZATION_FAILED;
    if (XrResult result = func_xrCreateVulkanInstanceKHR(m_instance, &vulkanInstanceCreateInfo, pInstance, &vulkanResult); XR_FAILED(result)) {
        Log::print<ERROR>("xrCreateVulkanInstanceKHR failed with {}", result);
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    return vulkanResult;
}

VkResult OpenXR::CreateVulkanDevice(VkPhysicalDevice physicalDevice, const VkDeviceCreateInfo& createInfo, const VkAllocationCallbacks* allocator, VkDevice* pDevice) {
    checkAssert(func_xrCreateVulkanDeviceKHR != nullptr, "The OpenXR runtime's xrCreateVulkanDeviceKHR is missing!");

    XrVulkanDeviceCreateInfoKHR vulkanDeviceCreateInfo = { XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR };
    vulkanDeviceCreateInfo.systemId = m_systemId;
    vulkanDeviceCreateInfo.createFlags = 0;
    vulkanDeviceCreateInfo.pfnGetInstanceProcAddr = VulkanLoader::GetInstanceProcAddr();
    vulkanDeviceCreateInfo.vulkanPhysicalDevice = physicalDevice;
    vulkanDeviceCreateInfo.vulkanCreateInfo = &createInfo;
    vulkanDeviceCreateInfo.vulkanAllocator = allocator;

    VkResult vulkanResult = VK_ERROR_INITIALIZATION_FAILED;
    if (XrResult result = func_xrCreateVulkanDeviceKHR(m_instance, &vulkanDeviceCreateInfo, pDevice, &vulkanResult); XR_FAILED(result)) {
        Log::print<ERROR>("xrCreateVulkanDeviceKHR failed with {}", result);
        return VK_ERROR_INITIALIZATION_FAILED;
    }
    return vulkanResult;
}

VkPhysicalDevice OpenXR::GetVulkanGraphicsDevice(VkInstance vulkanInstance) {
    if (m_capabilities.vulkanPhysicalDevice != VK_NULL_HANDLE && m_capabilities.vulkanInstance == vulkanInstance) {
        return m_capabilities.vulkanPhysicalDevice;
    }

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    if (m_capabilities.usesVulkanEnable2) {
        XrVulkanGraphicsDeviceGetInfoKHR graphicsDeviceGetInfo = { XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR };
        graphicsDeviceGetInfo.systemId = m_systemId;
        graphicsDeviceGetInfo.vulkanInstance = vulkanInstance;
        checkXRResult(func_xrGetVulkanGraphicsDevice2KHR(m_instance, &graphicsDeviceGetInfo, &physicalDevice), "Couldn't get the Vulkan GPU that drives the VR headset!");
    }
    else {
        checkXRResult(func_xrGetVulkanGraphicsDeviceKHR(m_instance, m_systemId, vulkanInstance, &physicalDevice), "Couldn't get the Vulkan GPU that drives the VR headset!");
    }

    m_capabilities.vulkanInstance = vulkanInstance;
    m_capabilities.vulkanPhysicalDevice = physicalDevice;
    return physicalDevice;
}

std::array<XrViewConfigurationView, 2> OpenXR::GetViewConfigurations() {
    uint32_t eyeViewsConfigurationCount = 0;
    checkXRResult(xrEnumerateViewConfigurationViews(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &eyeViewsConfigurationCount, nullptr), "Can't get number of individual views for stereo view available");
    checkAssert(eyeViewsConfigurationCount == 2, fmt::format("Expected 2 views for the stereo configuration but got {} which is unsupported!", eyeViewsConfigurationCount).c_str());

    std::array<XrViewConfigurationView, 2> xrViewConf = { XrViewConfigurationView{ XR_TYPE_VIEW_CONFIGURATION_VIEW }, XrViewConfigurationView{ XR_TYPE_VIEW_CONFIGURATION_VIEW } };
    checkXRResult(xrEnumerateViewConfigurationViews(m_instance, m_systemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, eyeViewsConfigurationCount, &eyeViewsConfigurationCount, xrViewConf.data()), "Can't get individual views for stereo view available!");

    Log::print<INFO>("Swapchain configuration to be used:");
    Log::print<INFO>(" - [Left] Max View Resolution: w={}, h={} with {} samples", xrViewConf[0].maxImageRectWidth, xrViewConf[0].maxImageRectHeight, xrViewConf[0].maxSwapchainSampleCount);
    Log::print<INFO>(" - [Right] Max View Resolution: w={}, h={}  with {} samples", xrViewConf[1].maxImageRectWidth, xrViewConf[1].maxImageRectHeight, xrViewConf[1].maxSwapchainSampleCount);
    Log::print<INFO>(" - [Left] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[0].recommendedImageRectWidth, xrViewConf[0].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    Log::print<INFO>(" - [Right] Recommended View Resolution: w={}, h={}  with {} samples", xrViewConf[1].recommendedImageRectWidth, xrViewConf[1].recommendedImageRectHeight, xrViewConf[0].recommendedSwapchainSampleCount);
    return xrViewConf;
}

void OpenXR::CreateSession(const XrGraphicsBindingVulkan2KHR& vulkanBinding) {
    Log::print<INFO>("Creating the OpenXR session on queue family {} index {}...", vulkanBinding.queueFamilyIndex, vulkanBinding.queueIndex);

    XrSessionCreateInfo sessionCreateInfo = { XR_TYPE_SESSION_CREATE_INFO };
    sessionCreateInfo.systemId = m_systemId;
    sessionCreateInfo.next = &vulkanBinding;
    sessionCreateInfo.createFlags = 0;
    checkXRResult(xrCreateSession(m_instance, &sessionCreateInfo, &m_session), "Failed to create Vulkan-based OpenXR session!");

    Log::print<INFO>("Creating the OpenXR spaces...");
    m_hasSeatedHeightCalibration = false;
    m_seenFirstLoadingScreen = false;
    m_lastSeenPlayMode.reset();
    ReplaceStageSpace(0.0f);

    XrReferenceSpaceCreateInfo headSpaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
    headSpaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    headSpaceCreateInfo.poseInReferenceSpace = s_xrIdentityPose;
    checkXRResult(xrCreateReferenceSpace(m_session, &headSpaceCreateInfo, &m_headSpace), "Failed to create reference space for head!");
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

    if (!m_seenFirstLoadingScreen && Game::GetPresentation().loadingScreenVisible) {
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
        CopyFixedString(actionInfo.actionName, id);
        CopyFixedString(actionInfo.localizedActionName, name);
        actionInfo.countSubactionPaths = (uint32_t)m_handPaths.size();
        actionInfo.subactionPaths = m_handPaths.data();
        checkXRResult(xrCreateAction(actionSet, &actionInfo, &action), fmt::format("Failed to create action for {}", id).c_str());
    };

    {
        XrActionSetCreateInfo actionSetInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
        CopyFixedString(actionSetInfo.actionSetName, "gameplay_fps");
        CopyFixedString(actionSetInfo.localizedActionSetName, "Gameplay");
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
        CopyFixedString(actionSetInfo.actionSetName, "menu");
        CopyFixedString(actionSetInfo.localizedActionSetName, "Menu Navigation");
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

    std::vector actionSets = { m_gameplayActionSet, m_menuActionSet };
    if (m_capabilities.supportsEyeGaze) {
        XrActionSetCreateInfo actionSetInfo = { XR_TYPE_ACTION_SET_CREATE_INFO };
        CopyFixedString(actionSetInfo.actionSetName, "eye_gaze");
        CopyFixedString(actionSetInfo.localizedActionSetName, "Eye Tracking");
        checkXRResult(xrCreateActionSet(m_instance, &actionSetInfo, &m_eyeGazeActionSet), "Failed to create the eye tracking actions!");

        XrActionCreateInfo actionInfo = { XR_TYPE_ACTION_CREATE_INFO };
        actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
        CopyFixedString(actionInfo.actionName, "gaze");
        CopyFixedString(actionInfo.localizedActionName, "Eye Gaze");
        checkXRResult(xrCreateAction(m_eyeGazeActionSet, &actionInfo, &m_eyeGazeAction), "Failed to create action for eye gaze");

        const XrActionSuggestedBinding gazeBinding = { m_eyeGazeAction, GetXRPath("/user/eyes_ext/input/gaze_ext/pose") };
        SuggestProfileBindings(m_instance, "/interaction_profiles/ext/eye_gaze_interaction", &gazeBinding, 1, false);
        actionSets.emplace_back(m_eyeGazeActionSet);
    }

    XrSessionActionSetsAttachInfo attachInfo = { XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
    attachInfo.countActionSets = (uint32_t)actionSets.size();
    attachInfo.actionSets = actionSets.data();
    checkXRResult(xrAttachSessionActionSets(m_session, &attachInfo), "Failed to attach action sets to session!");

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

    if (m_eyeGazeAction != XR_NULL_HANDLE) {
        XrActionSpaceCreateInfo createInfo = { XR_TYPE_ACTION_SPACE_CREATE_INFO };
        createInfo.action = m_eyeGazeAction;
        createInfo.poseInActionSpace = s_xrIdentityPose;
        checkXRResult(xrCreateActionSpace(m_session, &createInfo, &m_eyeGazeSpace), "Failed to create action space for eye gaze!");
    }

    m_rumbleManager = std::make_unique<RumbleManager>(m_session, m_rumbleAction);
    m_rumbleManager.get()->initializeXrPathsAndStartTime(m_instance);

    m_capabilities.activeControllerType = DetectActiveControllerType(m_instance, m_session);
}

std::optional<OpenXR::InputState> OpenXR::UpdateActions(XrTime predictedFrameTime, bool inMenu) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::XRUpdateActions);

    const std::array activeActionSets = {
        XrActiveActionSet{ inMenu ? m_menuActionSet : m_gameplayActionSet, XR_NULL_PATH },
        XrActiveActionSet{ m_eyeGazeActionSet, XR_NULL_PATH }
    };

    XrActionsSyncInfo syncInfo = { XR_TYPE_ACTIONS_SYNC_INFO };
    syncInfo.countActiveActionSets = m_eyeGazeActionSet != XR_NULL_HANDLE ? 2 : 1;
    syncInfo.activeActionSets = activeActionSets.data();
    checkXRResult(xrSyncActions(m_session, &syncInfo), "Failed to sync actions!");


    InputState newState = {};
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
                *outVelocity = { XR_TYPE_SPACE_VELOCITY };
            }

            checkXRResult(xrLocateSpace(handSpace, m_stageSpace, predictedFrameTime, &spaceLocation), "Failed to get location from controllers!");
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 && (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
                outLocation = spaceLocation;
                outLocation.next = nullptr;

                if (outVelocity != nullptr) {
                    // rotate angular velocity to world space when it's using a buggy runtime
                    auto mode = GetSettings().AngularVelocityFixer_GetMode();
                    bool isUsingQuestRuntime = m_capabilities.isOculusLinkRuntime;
                    if ((spaceVelocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT) != 0 && ((mode == AngularVelocityFixerMode::AUTO && isUsingQuestRuntime) || mode == AngularVelocityFixerMode::FORCED_ON)) {
                        glm::vec3 angularVelocity = ToGLM(spaceVelocity.angularVelocity);
                        glm::fquat fix_angle = glm::fquat(0.924, -0.383, 0, 0);
                        angularVelocity = (ToGLM(spaceLocation.pose.orientation) * (fix_angle * angularVelocity));
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
    // each action is read from the controller given, or from the other one when a rebinding left nothing for it on the first
    auto readAction = [&](XrAction action, EyeSide controller, auto& state, XrStructureType type, auto getActionState, const char* errorContext) {
        for (EyeSide source : { controller, (EyeSide)(1 - controller) }) {
            XrActionStateGetInfo getInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getInfo.action = action;
            getInfo.subactionPath = m_handPaths[source];
            state = { type };
            checkXRResult(getActionState(m_session, &getInfo, &state), errorContext);
            if (state.isActive)
                return;
        }
    };
    auto readBoolean = [&](XrAction action, EyeSide hand, XrActionStateBoolean& state, const char* errorContext) {
        readAction(action, hand, state, XR_TYPE_ACTION_STATE_BOOLEAN, xrGetActionStateBoolean, errorContext);
    };
    auto readVector2f = [&](XrAction action, EyeSide hand, XrActionStateVector2f& state, const char* errorContext) {
        readAction(action, hand, state, XR_TYPE_ACTION_STATE_VECTOR2F, xrGetActionStateVector2f, errorContext);
    };

    readBoolean(newState.shared.in_game ? m_inGame_inventory_mapAction : m_inMenu_inventory_mapAction, EyeSide::RIGHT, newState.shared.inventory_map, "Failed to get inventory_help action value!");
    readBoolean(newState.shared.in_game ? m_inGame_modMenuAction : m_inMenu_modMenuAction, EyeSide::LEFT, newState.shared.modMenu, "Failed to get mod menu action value!");

    if (inMenu) {
        readVector2f(m_scrollAction, EyeSide::RIGHT, newState.inMenu.scroll, "Failed to get navigate action value!");
        readVector2f(m_navigateAction, EyeSide::LEFT, newState.inMenu.navigate, "Failed to get select action value!");
        readBoolean(m_selectAction, EyeSide::RIGHT, newState.inMenu.select, "Failed to get select action value!");
        readBoolean(m_backAction, EyeSide::RIGHT, newState.inMenu.back, "Failed to get back action value!");
        readBoolean(m_sortAction, EyeSide::LEFT, newState.inMenu.sort, "Failed to get sort action value!");
        readBoolean(m_holdAction, EyeSide::LEFT, newState.inMenu.hold, "Failed to get hold action value!");
        readBoolean(m_leftGripAction, EyeSide::LEFT, newState.inMenu.leftGrip, "Failed to get left grip action value!");
        readBoolean(m_rightGripAction, EyeSide::RIGHT, newState.inMenu.rightGrip, "Failed to get right grip action value!");
        readAction(m_leftTriggerAction, EyeSide::LEFT, newState.inMenu.leftTrigger, XR_TYPE_ACTION_STATE_BOOLEAN, xrGetActionStateBoolean, "Failed to get left trigger action value!");
        readAction(m_rightTriggerAction, EyeSide::RIGHT, newState.inMenu.rightTrigger, XR_TYPE_ACTION_STATE_BOOLEAN, xrGetActionStateBoolean, "Failed to get right trigger action value!");
    }
    else {
        for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
            XrActionStateGetInfo getGrabInfo = { XR_TYPE_ACTION_STATE_GET_INFO };
            getGrabInfo.action = m_grab_interactAction;
            getGrabInfo.subactionPath = m_handPaths[side];
            newState.inGame.grab[side] = { XR_TYPE_ACTION_STATE_FLOAT };
            checkXRResult(xrGetActionStateFloat(m_session, &getGrabInfo, &newState.inGame.grab[side]), "Failed to get grab action value!");
        }

        readBoolean(m_crouch_scopeAction, EyeSide::LEFT, newState.inGame.crouch_scope, "Failed to get crouch and map action value!");
        readVector2f(m_moveAction, EyeSide::LEFT, newState.inGame.move, "Failed to get move action value!");
        readVector2f(m_cameraAction, EyeSide::RIGHT, newState.inGame.camera, "Failed to get camera action value!");
        readBoolean(m_jumpAction, EyeSide::RIGHT, newState.inGame.jump_cancel, "Failed to get jump action value!");
        readBoolean(m_run_interactAction, EyeSide::RIGHT, newState.inGame.run_interact, "Failed to get run action value!");
        readBoolean(m_useRune_dpadMenu_Action, EyeSide::LEFT, newState.inGame.useRune_dpadMenu, "Failed to get use rune action value!");
        readBoolean(m_useRightItemAction, EyeSide::RIGHT, newState.inGame.useRightItem, "Failed to get useRightItem action value!");
        readBoolean(m_useLeftItemAction, EyeSide::LEFT, newState.inGame.useLeftItem, "Failed to get useLeftItem action value!");
    }
    this->m_input.store(newState);
    return newState;
}

std::optional<glm::fvec3> OpenXR::LocateEyeGaze(XrTime predictedDisplayTime) const {
    if (m_eyeGazeSpace == XR_NULL_HANDLE) {
        return std::nullopt;
    }

    XrSpaceLocation gazeLocation = { XR_TYPE_SPACE_LOCATION };
    if (XR_FAILED(xrLocateSpace(m_eyeGazeSpace, m_stageSpace, predictedDisplayTime, &gazeLocation))) {
        return std::nullopt;
    }
    // some runtimes keep the last gaze valid after losing track of the eyes
    constexpr XrSpaceLocationFlags tracked = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    if ((gazeLocation.locationFlags & tracked) != tracked) {
        return std::nullopt;
    }
    return ToGLM(gazeLocation.pose.orientation) * glm::fvec3(0.0f, 0.0f, -1.0f);
}

std::optional<XrSpaceLocation> OpenXR::UpdateSpaces(XrTime predictedDisplayTime) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::XRUpdateSpaces);

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
                if (m_renderer) {
                    m_renderer->EndSession();
                }
                break;
            case XR_SESSION_STATE_EXITING:
                Log::print<VERBOSE>("OpenXR has indicated that the session should be destroyed!");
                HostWindow::RequestShutdown();
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
}
