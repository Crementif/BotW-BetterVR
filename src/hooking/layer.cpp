#include "layer.h"
#include "instance.h"
#include <cstdlib>
#include <cstring>
#include <filesystem>

namespace {
    // Simple helpers to keep diagnostics opt-in
    bool isEnvEnabled(const char* name) {
        if (const char* value = std::getenv(name)) {
            return value[0] != '\0' && value[0] != '0';
        }
        return false;
    }

    // Helper to query functions from the Vulkan loader without adding a new link dependency
    template <typename TFunc>
    TFunc getLoaderFunction(const char* name) {
        static HMODULE vulkanModule = GetModuleHandleA("vulkan-1.dll");
        if (!vulkanModule) {
            vulkanModule = LoadLibraryA("vulkan-1.dll");
        }
        if (!vulkanModule) {
            return nullptr;
        }
        return reinterpret_cast<TFunc>(GetProcAddress(vulkanModule, name));
    }

    // Debug messenger for validation output (created only when explicitly enabled)
    VkDebugUtilsMessengerEXT g_debugMessenger = VK_NULL_HANDLE;
    PFN_vkDestroyDebugUtilsMessengerEXT g_destroyDebugUtilsMessenger = nullptr;

    struct DiagnosticSupport {
        bool validationLayer = false;
        bool debugUtilsExt = false;
        bool validationFeaturesExt = false;
        std::vector<std::string> layerNames;
        std::string sdkPath;
        bool sdkExists = false;
        bool validationJsonExists = false;
        bool validationDllExists = false;
    };

    DiagnosticSupport queryDiagnosticSupport() {
        DiagnosticSupport support{};

        try {
            auto enumerateLayers = getLoaderFunction<PFN_vkEnumerateInstanceLayerProperties>("vkEnumerateInstanceLayerProperties");
            if (enumerateLayers) {
                uint32_t layerCount = 0;
                if (enumerateLayers(&layerCount, nullptr) == VK_SUCCESS && layerCount > 0) {
                    std::vector<VkLayerProperties> layers(layerCount);
                    if (enumerateLayers(&layerCount, layers.data()) == VK_SUCCESS) {
                        support.layerNames.reserve(layerCount);
                        for (const auto& layer : layers) {
                            support.layerNames.emplace_back(layer.layerName);
                            if (std::strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0) {
                                support.validationLayer = true;
                                break;
                            }
                        }
                    }
                }
            }
        } catch (...) {
            // Ignore errors during layer enumeration
        }

        try {
            auto enumerateExtensions = getLoaderFunction<PFN_vkEnumerateInstanceExtensionProperties>("vkEnumerateInstanceExtensionProperties");
            if (enumerateExtensions) {
                uint32_t extCount = 0;
                if (enumerateExtensions(nullptr, &extCount, nullptr) == VK_SUCCESS && extCount > 0) {
                    std::vector<VkExtensionProperties> extensions(extCount);
                    if (enumerateExtensions(nullptr, &extCount, extensions.data()) == VK_SUCCESS) {
                        auto hasExt = [&extensions](const char* name) {
                            return std::find_if(extensions.begin(), extensions.end(), [name](const VkExtensionProperties& ext) {
                                return std::strcmp(ext.extensionName, name) == 0;
                            }) != extensions.end();
                        };
                        support.debugUtilsExt = hasExt(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
                        support.validationFeaturesExt = hasExt(VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);
                    }
                }
            }
        } catch (...) {
            // Ignore errors during extension enumeration
        }

        try {
            if (const char* sdk = std::getenv("VULKAN_SDK")) {
                support.sdkPath = sdk;
                std::filesystem::path sdkPath(sdk);
                support.sdkExists = std::filesystem::exists(sdkPath);
                support.validationJsonExists = std::filesystem::exists(sdkPath / "Bin" / "config" / "VkLayer_khronos_validation.json");
                support.validationDllExists = std::filesystem::exists(sdkPath / "Bin" / "VkLayer_khronos_validation.dll");
            }
        } catch (...) {
            // Ignore filesystem errors
        }

        return support;
    }

    // Vulkan validation callback -> forward into our logger
    VKAPI_ATTR VkBool32 VKAPI_CALL DebugUtilsMessengerCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT /*messageTypes*/,
        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
        void* /*pUserData*/) {

        const char* severity = "INFO";
        if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
            severity = "ERROR";
        } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
            severity = "WARN";
        } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT) {
            severity = "INFO";
        } else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) {
            severity = "VERBOSE";
        }

        // Route validation output into the normal log stream so it lands in user logs
        if (messageSeverity & (VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)) {
            Log::print<WARNING>("[Vulkan {}] {} (id {}): {}", severity,
                pCallbackData->pMessageIdName ? pCallbackData->pMessageIdName : "unknown",
                pCallbackData->messageIdNumber,
                pCallbackData->pMessage ? pCallbackData->pMessage : "");
        } else {
            Log::print<INFO>("[Vulkan {}] {} (id {}): {}", severity,
                pCallbackData->pMessageIdName ? pCallbackData->pMessageIdName : "unknown",
                pCallbackData->messageIdNumber,
                pCallbackData->pMessage ? pCallbackData->pMessage : "");
        }
        return VK_FALSE;
    }
}

VkResult VRLayer::VkInstanceOverrides::CreateInstance(PFN_vkCreateInstance createInstanceFunc, const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance) {
    // Early safety checks - if anything is null, just call the original function
    if (!createInstanceFunc || !pCreateInfo || !pInstance) {
        if (createInstanceFunc && pCreateInfo && pInstance) {
            return createInstanceFunc(pCreateInfo, pAllocator, pInstance);
        }
        return VK_ERROR_INITIALIZATION_FAILED;
    }

    // Check if we should skip all diagnostic/validation code (for compatibility)
    const bool skipDiagnostics = isEnvEnabled("BETTERVR_SKIP_DIAGNOSTICS");
    if (skipDiagnostics) {
        Log::print<INFO>("Skipping diagnostics due to BETTERVR_SKIP_DIAGNOSTICS");
        return createInstanceFunc(pCreateInfo, pAllocator, pInstance);
    }

    // If the application already requested validation, avoid modifying the instance create info.
    bool appRequestedValidation = false;
    for (uint32_t i = 0; i < pCreateInfo->enabledLayerCount; ++i) {
        if (pCreateInfo->ppEnabledLayerNames &&
            std::strcmp(pCreateInfo->ppEnabledLayerNames[i], "VK_LAYER_KHRONOS_validation") == 0) {
            appRequestedValidation = true;
            break;
        }
    }
    if (appRequestedValidation) {
        Log::print<INFO>("CreateInstance: app requested validation; skipping BetterVR validation/env tweaks for compatibility");
        return createInstanceFunc(pCreateInfo, pAllocator, pInstance);
    }

    Log::print<INFO>("CreateInstance called - createInstanceFunc={}, pCreateInfo={}, pInstance={}",
        (void*)createInstanceFunc, (void*)pCreateInfo, (void*)pInstance);       

    // Skip diagnostic query for now - it may be causing crashes with certain drivers/hooks
    // TODO: Re-enable once the crash is fixed
    DiagnosticSupport diagSupport{};
    // diagSupport = queryDiagnosticSupport();  // DISABLED - causing crashes

    const bool validationEnv = isEnvEnabled("BETTERVR_ENABLE_VK_VALIDATION");
    const bool enableValidationRequest = validationEnv;

    // Proactively set loader env so users don't have to: point VK_LAYER_PATH at the SDK
    // and disable the OBS layer to prevent it from intercepting our hooks.
    auto getEnvStr = [](const char* name) -> std::string {
        if (const char* v = std::getenv(name)) {
            return v;
        }
        return {};
    };

    // Skip SDK path setup for now - diagSupport is disabled
    // if (diagSupport.sdkExists) { ... }

    {
        const std::string currDisable = getEnvStr("VK_LOADER_LAYERS_DISABLE");
        const std::string obsLayer = "VK_LAYER_OBS_HOOK";
        if (currDisable.find(obsLayer) == std::string::npos) {
            std::string newDisable = currDisable;
            if (!newDisable.empty() && newDisable.back() != ';' && newDisable.back() != ',') {
                newDisable += ",";
            }
            newDisable += obsLayer;
            SetEnvironmentVariableA("VK_LOADER_LAYERS_DISABLE", newDisable.c_str());
            Log::print<INFO>("Disabling OBS Vulkan layer via VK_LOADER_LAYERS_DISABLE='{}'", newDisable);
        } else {
            Log::print<INFO>("VK_LOADER_LAYERS_DISABLE already includes '{}'", obsLayer);
        }
    }

    // Log key loader env vars for diagnostics
    Log::print<INFO>("VK env - VULKAN_SDK='{}', VK_LAYER_PATH='{}', VK_ADD_LAYER_PATH='{}', VK_INSTANCE_LAYERS='{}', VK_LOADER_LAYERS_DISABLE='{}'",
        getEnvStr("VULKAN_SDK").empty() ? "<unset>" : getEnvStr("VULKAN_SDK"),
        getEnvStr("VK_LAYER_PATH").empty() ? "<unset>" : getEnvStr("VK_LAYER_PATH"),
        getEnvStr("VK_ADD_LAYER_PATH").empty() ? "<unset>" : getEnvStr("VK_ADD_LAYER_PATH"),
        getEnvStr("VK_INSTANCE_LAYERS").empty() ? "<unset>" : getEnvStr("VK_INSTANCE_LAYERS"),
        getEnvStr("VK_LOADER_LAYERS_DISABLE").empty() ? "<unset>" : getEnvStr("VK_LOADER_LAYERS_DISABLE"));

    if (isEnvEnabled("BETTERVR_ENABLE_VK_LOADER_DEBUG")) {
        SetEnvironmentVariableA("VK_LOADER_DEBUG", "all");
    }

    // Modify VkInstance with needed extensions/layers
    VkInstanceCreateInfo modifiedCreateInfo = *pCreateInfo;

    std::vector<const char*> modifiedExtensions;
    modifiedExtensions.reserve(pCreateInfo->enabledExtensionCount + 4);
    if (pCreateInfo->ppEnabledExtensionNames) {
        for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
            if (pCreateInfo->ppEnabledExtensionNames[i]) {
                modifiedExtensions.push_back(pCreateInfo->ppEnabledExtensionNames[i]);
            }
        }
    }
    // Use const char* directly since we're passing string literals which have static storage
    auto tryAddExt = [&modifiedExtensions](const char* ext) {
        if (!ext) return;  // Safety check
        bool found = false;
        for (const char* e : modifiedExtensions) {
            if (e && std::strcmp(e, ext) == 0) {
                found = true;
                break;
            }
        }
        if (!found) {
            modifiedExtensions.push_back(ext);
        }
    };

    std::vector<const char*> modifiedLayers;
    modifiedLayers.reserve(pCreateInfo->enabledLayerCount + 1);
    if (pCreateInfo->ppEnabledLayerNames) {
        for (uint32_t i = 0; i < pCreateInfo->enabledLayerCount; i++) {
            if (pCreateInfo->ppEnabledLayerNames[i]) {
                modifiedLayers.push_back(pCreateInfo->ppEnabledLayerNames[i]);
            }
        }
    }
    const bool hasValidationLayer = std::find_if(modifiedLayers.begin(), modifiedLayers.end(),
        [](const char* layer) { return layer && std::strcmp(layer, "VK_LAYER_KHRONOS_validation") == 0; }) != modifiedLayers.end();
    const bool validationLayerAvailable = hasValidationLayer || diagSupport.validationLayer;
    const bool validationEnabled = enableValidationRequest && validationLayerAvailable;

    const bool enableDebugUtils = validationEnabled && diagSupport.debugUtilsExt;
    const bool enableValidationFeatures = validationEnabled && diagSupport.validationFeaturesExt;
    if (enableDebugUtils) {
        tryAddExt(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }
    if (enableValidationFeatures) {
        tryAddExt(VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);
    }
    modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
    modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.data();

    Log::print<INFO>("Vulkan diagnostics support - validation layer present: {}, debug utils ext present: {}, validation features ext present: {}",
        diagSupport.validationLayer ? "yes" : "no",
        diagSupport.debugUtilsExt ? "yes" : "no",
        diagSupport.validationFeaturesExt ? "yes" : "no");
    if (!diagSupport.layerNames.empty()) {
        std::string allLayers;
        for (size_t i = 0; i < diagSupport.layerNames.size(); ++i) {
            allLayers += diagSupport.layerNames[i];
            if (i + 1 < diagSupport.layerNames.size()) {
                allLayers += ", ";
            }
        }
        Log::print<INFO>("Vulkan layers detected: {}", allLayers);
    } else {
        Log::print<INFO>("Vulkan layers detected: none");
    }
    if (!diagSupport.sdkPath.empty()) {
        Log::print<INFO>("VULKAN_SDK='{}' (exists: {}, validation json: {}, validation dll: {})",
            diagSupport.sdkPath.empty() ? "<not set>" : diagSupport.sdkPath,
            diagSupport.sdkExists ? "yes" : "no",
            diagSupport.validationJsonExists ? "yes" : "no",
            diagSupport.validationDllExists ? "yes" : "no");
    }
    if (enableValidationRequest && !validationLayerAvailable) {
        Log::print<WARNING>("Requested Vulkan validation but VK_LAYER_KHRONOS_validation is not available on this system; continuing without validation.");
    }
    if (validationEnabled && !hasValidationLayer) {
        modifiedLayers.push_back("VK_LAYER_KHRONOS_validation");
    }
    modifiedCreateInfo.enabledLayerCount = (uint32_t)modifiedLayers.size();
    modifiedCreateInfo.ppEnabledLayerNames = modifiedLayers.data();

    // Enable a couple of validation features when requested
    VkValidationFeaturesEXT validationFeatures = { VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT };
    std::vector<VkValidationFeatureEnableEXT> validationEnables;
    if (enableValidationFeatures) {
        validationEnables = {
            VK_VALIDATION_FEATURE_ENABLE_BEST_PRACTICES_EXT,
            VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT
        };
        validationFeatures.enabledValidationFeatureCount = (uint32_t)validationEnables.size();
        validationFeatures.pEnabledValidationFeatures = validationEnables.data();
        validationFeatures.pNext = const_cast<void*>(modifiedCreateInfo.pNext);
        modifiedCreateInfo.pNext = &validationFeatures;

        Log::print<INFO>("Enabling Vulkan validation (auto-available: {}, env: {}) | debug utils ext: {} | validation features ext: {}",
            diagSupport.validationLayer ? "yes" : "no",
            validationEnv ? "yes" : "no",
            enableDebugUtils ? "yes" : "no",
            enableValidationFeatures ? "yes" : "no");
    }

    VkResult result = createInstanceFunc(&modifiedCreateInfo, pAllocator, pInstance);
    if (validationEnabled && (result == VK_ERROR_LAYER_NOT_PRESENT || result == VK_ERROR_EXTENSION_NOT_PRESENT)) {
        Log::print<WARNING>("Validation/debug extensions were requested but not available; retrying without them.");
        result = createInstanceFunc(pCreateInfo, pAllocator, pInstance);
    }

    if (result == VK_SUCCESS && validationEnabled && enableDebugUtils) {
        auto getInstanceProcAddr = getLoaderFunction<PFN_vkGetInstanceProcAddr>("vkGetInstanceProcAddr");
        if (getInstanceProcAddr) {
            auto createDebugUtilsMessenger = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(getInstanceProcAddr(*pInstance, "vkCreateDebugUtilsMessengerEXT"));
            g_destroyDebugUtilsMessenger = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(getInstanceProcAddr(*pInstance, "vkDestroyDebugUtilsMessengerEXT"));

            if (createDebugUtilsMessenger && g_destroyDebugUtilsMessenger) {
                VkDebugUtilsMessengerCreateInfoEXT dbgCreateInfo = { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
                dbgCreateInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                    VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT |
                    VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT;
                dbgCreateInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                    VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                    VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
                dbgCreateInfo.pfnUserCallback = DebugUtilsMessengerCallback;

                if (createDebugUtilsMessenger(*pInstance, &dbgCreateInfo, pAllocator, &g_debugMessenger) != VK_SUCCESS) {
                    Log::print<WARNING>("Failed to create Vulkan debug utils messenger; validation messages may not appear in logs.");
                }
            } else {
                Log::print<WARNING>("Could not resolve debug utils functions; validation layer output may not be visible.");
            }
        } else {
            Log::print<WARNING>("vkGetInstanceProcAddr could not be resolved; skipping debug utils messenger.");
        }
    }

    VRManager::instance().vkVersion = modifiedCreateInfo.pApplicationInfo->apiVersion;

    Log::print<INFO>("Created Vulkan instance (using Vulkan {}.{}.{}) successfully!", VK_API_VERSION_MAJOR(modifiedCreateInfo.pApplicationInfo->apiVersion), VK_API_VERSION_MINOR(modifiedCreateInfo.pApplicationInfo->apiVersion), VK_API_VERSION_PATCH(modifiedCreateInfo.pApplicationInfo->apiVersion));
    checkAssert(VK_VERSION_MINOR(modifiedCreateInfo.pApplicationInfo->apiVersion) != 0 || VK_VERSION_MAJOR(modifiedCreateInfo.pApplicationInfo->apiVersion) > 1, "Vulkan version needs to be v1.1 or higher!");
    return result;
}

void VRLayer::VkInstanceOverrides::DestroyInstance(const vkroots::VkInstanceDispatch* pDispatch, VkInstance instance, const VkAllocationCallbacks* pAllocator) {
    if (g_debugMessenger != VK_NULL_HANDLE && g_destroyDebugUtilsMessenger) {
        g_destroyDebugUtilsMessenger(instance, g_debugMessenger, pAllocator);
        g_debugMessenger = VK_NULL_HANDLE;
    }
    return pDispatch->DestroyInstance(instance, pAllocator);
}


VkResult VRLayer::VkInstanceOverrides::EnumeratePhysicalDevices(const vkroots::VkInstanceDispatch* pDispatch, VkInstance instance, uint32_t* pPhysicalDeviceCount, VkPhysicalDevice* pPhysicalDevices) {
    // Proceed to get all devices
    uint32_t internalCount = 0;
    checkVkResult(pDispatch->EnumeratePhysicalDevices(instance, &internalCount, nullptr), "Failed to retrieve number of vulkan physical devices!");
    std::vector<VkPhysicalDevice> internalDevices(internalCount);
    checkVkResult(pDispatch->EnumeratePhysicalDevices(instance, &internalCount, internalDevices.data()), "Failed to retrieve vulkan physical devices!");

    VkPhysicalDevice matchedDevice = VK_NULL_HANDLE;
    VkPhysicalDevice fallbackDevice = VK_NULL_HANDLE;

    for (const VkPhysicalDevice& device : internalDevices) {
        VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
        VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        properties.pNext = &deviceId;
        pDispatch->GetPhysicalDeviceProperties2(device, &properties);

        if (deviceId.deviceLUIDValid && memcmp(&VRManager::instance().XR->m_capabilities.adapter, deviceId.deviceLUID, VK_LUID_SIZE) == 0) {
            matchedDevice = device;
            break;
        }

        // Keep track of the first discrete GPU as fallback for drivers that don't report valid LUIDs
        if (fallbackDevice == VK_NULL_HANDLE && properties.properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            fallbackDevice = device;
        }
    }

    // Use matched device, or fallback to first discrete GPU if no LUID match found
    VkPhysicalDevice selectedDevice = matchedDevice != VK_NULL_HANDLE ? matchedDevice : fallbackDevice;

    // Last resort: use first available device
    if (selectedDevice == VK_NULL_HANDLE && !internalDevices.empty()) {
        selectedDevice = internalDevices[0];
        Log::print<WARNING>("No device matched OpenXR LUID and no discrete GPU found, using first available device");
    }

    if (selectedDevice != VK_NULL_HANDLE) {
        VkPhysicalDeviceProperties props = {};
        pDispatch->GetPhysicalDeviceProperties(selectedDevice, &props);
        const bool luidMatched = matchedDevice != VK_NULL_HANDLE;
        Log::print<INFO>("Selected Vulkan GPU: '{}' (vendor=0x{:04X}, device=0x{:04X}, type={}, driver={}.{}.{}, api={}.{}.{}) | LUID match: {}",
            props.deviceName,
            props.vendorID,
            props.deviceID,
            (int)props.deviceType,
            VK_VERSION_MAJOR(props.driverVersion), VK_VERSION_MINOR(props.driverVersion), VK_VERSION_PATCH(props.driverVersion),
            VK_VERSION_MAJOR(props.apiVersion), VK_VERSION_MINOR(props.apiVersion), VK_VERSION_PATCH(props.apiVersion),
            luidMatched ? "yes" : "no");

        if (pPhysicalDevices != nullptr) {
            if (*pPhysicalDeviceCount < 1) {
                *pPhysicalDeviceCount = 1;
                return VK_INCOMPLETE;
            }
            *pPhysicalDeviceCount = 1;
            pPhysicalDevices[0] = selectedDevice;
            return VK_SUCCESS;
        }
        else {
            *pPhysicalDeviceCount = 1;
            return VK_SUCCESS;
        }
    }

    *pPhysicalDeviceCount = 0;
    return VK_SUCCESS;
}

// Some layers (OBS vulkan layer) will skip the vkEnumeratePhysicalDevices hook
// Therefor we also override vkGetPhysicalDeviceProperties to make any non-compatible VkPhysicalDevice use Vulkan 1.0 which Cemu won't list due to it being too low
void VRLayer::VkInstanceOverrides::GetPhysicalDeviceProperties(const vkroots::VkInstanceDispatch* pDispatch, VkPhysicalDevice physicalDevice, VkPhysicalDeviceProperties* pProperties) {
    // Do original query
    pDispatch->GetPhysicalDeviceProperties(physicalDevice, pProperties);

    // Do a seperate internal query to make sure that we also query the LUID
    {
        VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
        VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        properties.pNext = &deviceId;
        pDispatch->GetPhysicalDeviceProperties2(physicalDevice, &properties);

        if (deviceId.deviceLUIDValid && memcmp(&VRManager::instance().XR->m_capabilities.adapter, deviceId.deviceLUID, VK_LUID_SIZE) != 0) {
            pProperties->apiVersion = VK_API_VERSION_1_0;
        }
    }
}

// Some layers (OBS vulkan layer) will skip the vkEnumeratePhysicalDevices hook
// Therefor we also override vkGetPhysicalDeviceQueueFamilyProperties to make any non-VR-compatible VkPhysicalDevice have 0 queues
void VRLayer::VkInstanceOverrides::GetPhysicalDeviceQueueFamilyProperties(const vkroots::VkInstanceDispatch* pDispatch, VkPhysicalDevice physicalDevice, uint32_t* pQueueFamilyPropertyCount, VkQueueFamilyProperties* pQueueFamilyProperties) {
    // Check whether this VkPhysicalDevice matches the LUID that OpenXR returns
    VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
    VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
    properties.pNext = &deviceId;
    pDispatch->GetPhysicalDeviceProperties2(physicalDevice, &properties);

    if (deviceId.deviceLUIDValid && memcmp(&VRManager::instance().XR->m_capabilities.adapter, deviceId.deviceLUID, VK_LUID_SIZE) != 0) {
        *pQueueFamilyPropertyCount = 0;
        return;
    }

    return pDispatch->GetPhysicalDeviceQueueFamilyProperties(physicalDevice, pQueueFamilyPropertyCount, pQueueFamilyProperties);
}

const std::vector<std::string> additionalDeviceExtensions = {
    VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME,
    VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME,
    VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME,
    VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME,
    VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME,
    VK_KHR_SYNCHRONIZATION_2_EXTENSION_NAME,
#if ENABLE_VK_ROBUSTNESS
    VK_EXT_DEVICE_FAULT_EXTENSION_NAME,
    VK_EXT_ROBUSTNESS_2_EXTENSION_NAME,
    VK_EXT_IMAGE_ROBUSTNESS_EXTENSION_NAME
#endif
};

VkResult VRLayer::VkInstanceOverrides::CreateDevice(const vkroots::VkInstanceDispatch* pDispatch, VkPhysicalDevice gpu, const VkDeviceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDevice* pDevice) {
    // Query available extensions for this device
    uint32_t extensionCount = 0;
    pDispatch->EnumerateDeviceExtensionProperties(gpu, nullptr, &extensionCount, nullptr);
    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    pDispatch->EnumerateDeviceExtensionProperties(gpu, nullptr, &extensionCount, availableExtensions.data());

    auto isExtensionSupported = [&availableExtensions](const std::string& extName) {
        return std::find_if(availableExtensions.begin(), availableExtensions.end(),
            [&extName](const VkExtensionProperties& ext) {
                return extName == ext.extensionName;
            }) != availableExtensions.end();
    };

    // Modify VkDevice with needed extensions
    std::vector<const char*> modifiedExtensions;
    for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
        modifiedExtensions.push_back(pCreateInfo->ppEnabledExtensionNames[i]);
    }
    for (const std::string& extension : additionalDeviceExtensions) {
        if (std::find(modifiedExtensions.begin(), modifiedExtensions.end(), extension) == modifiedExtensions.end()) {
            if (isExtensionSupported(extension)) {
                modifiedExtensions.push_back(extension.c_str());
            } else {
                Log::print<WARNING>("Device extension {} is not supported, skipping", extension);
            }
        }
    }

    // Query supported features from the GPU
    VkPhysicalDeviceTimelineSemaphoreFeatures supportedTimelineSemaphoreFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES };
    VkPhysicalDeviceFeatures2 supportedFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    supportedFeatures.pNext = &supportedTimelineSemaphoreFeatures;

#if ENABLE_VK_ROBUSTNESS
    VkPhysicalDeviceImageRobustnessFeatures supportedImageRobustnessFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_ROBUSTNESS_FEATURES };
    VkPhysicalDeviceRobustness2FeaturesEXT supportedRobustness2Features = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ROBUSTNESS_2_FEATURES_EXT };
    supportedTimelineSemaphoreFeatures.pNext = &supportedImageRobustnessFeatures;
    supportedImageRobustnessFeatures.pNext = &supportedRobustness2Features;
#endif

    pDispatch->GetPhysicalDeviceFeatures2(gpu, &supportedFeatures);

    // Test if timeline semaphores are already enabled in the create info
    bool timelineSemaphoresEnabled = false;
    bool imageRobustnessEnabled = false;
    bool robustness2Enabled = false;
    const void* current_pNext = pCreateInfo->pNext;
    while (current_pNext) {
        const VkBaseInStructure* base = static_cast<const VkBaseInStructure*>(current_pNext);
        if (base->sType == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES) {
            timelineSemaphoresEnabled = true;
        }
#if ENABLE_VK_ROBUSTNESS
        if (base->sType == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_ROBUSTNESS_FEATURES) {
            imageRobustnessEnabled = true;
        }
        if (base->sType == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ROBUSTNESS_2_FEATURES_EXT) {
            robustness2Enabled = true;
        }
#endif
        current_pNext = base->pNext;
    }

    VkPhysicalDeviceTimelineSemaphoreFeatures createSemaphoreFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES };
    createSemaphoreFeatures.timelineSemaphore = true;

    VkPhysicalDeviceImageRobustnessFeatures createImageRobustnessFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_ROBUSTNESS_FEATURES };
    createImageRobustnessFeatures.robustImageAccess = true;

    VkPhysicalDeviceRobustness2FeaturesEXT createRobustness2Features = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ROBUSTNESS_2_FEATURES_EXT };
    createRobustness2Features.robustBufferAccess2 = true;
    createRobustness2Features.robustImageAccess2 = true;
    createRobustness2Features.nullDescriptor = true;

    void* nextChain = const_cast<void*>(pCreateInfo->pNext);

#if ENABLE_VK_ROBUSTNESS
    if (!robustness2Enabled && isExtensionSupported(VK_EXT_ROBUSTNESS_2_EXTENSION_NAME)) {
        // Only enable features that are actually supported
        createRobustness2Features.robustBufferAccess2 = supportedRobustness2Features.robustBufferAccess2;
        createRobustness2Features.robustImageAccess2 = supportedRobustness2Features.robustImageAccess2;
        createRobustness2Features.nullDescriptor = supportedRobustness2Features.nullDescriptor;
        if (createRobustness2Features.robustBufferAccess2 || createRobustness2Features.robustImageAccess2 || createRobustness2Features.nullDescriptor) {
            createRobustness2Features.pNext = nextChain;
            nextChain = &createRobustness2Features;
        }
    }

    if (!imageRobustnessEnabled && isExtensionSupported(VK_EXT_IMAGE_ROBUSTNESS_EXTENSION_NAME) && supportedImageRobustnessFeatures.robustImageAccess) {
        createImageRobustnessFeatures.pNext = nextChain;
        nextChain = &createImageRobustnessFeatures;
    }
#endif

    if (!timelineSemaphoresEnabled && supportedTimelineSemaphoreFeatures.timelineSemaphore) {
        createSemaphoreFeatures.pNext = nextChain;
        nextChain = &createSemaphoreFeatures;
    } else if (!timelineSemaphoresEnabled) {
        Log::print<ERROR>("Timeline semaphores are not supported by this GPU! VR functionality may not work.");
    }

    VkDeviceCreateInfo modifiedCreateInfo = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    modifiedCreateInfo.pNext = nextChain;
    modifiedCreateInfo.flags = pCreateInfo->flags;
    modifiedCreateInfo.queueCreateInfoCount = pCreateInfo->queueCreateInfoCount;
    modifiedCreateInfo.pQueueCreateInfos = pCreateInfo->pQueueCreateInfos;
    modifiedCreateInfo.enabledLayerCount = pCreateInfo->enabledLayerCount;
    modifiedCreateInfo.ppEnabledLayerNames = pCreateInfo->ppEnabledLayerNames;
    modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
    modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.data();
    // AMD GPU FIX: Preserve pEnabledFeatures from original create info
    // Dropping this can cause AMD drivers to disable features the application needs
    modifiedCreateInfo.pEnabledFeatures = pCreateInfo->pEnabledFeatures;

    // Log queue family selection for diagnostics
    uint32_t queueFamilyCount = 0;
    pDispatch->GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    if (queueFamilyCount > 0) {
        pDispatch->GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, queueFamilies.data());
    }

    Log::print<INFO>("Creating Vulkan device with {} queue infos", modifiedCreateInfo.queueCreateInfoCount);
    for (uint32_t i = 0; i < modifiedCreateInfo.queueCreateInfoCount; i++) {
        const auto& qci = modifiedCreateInfo.pQueueCreateInfos[i];
        VkQueueFlags familyFlags = 0;
        if (qci.queueFamilyIndex < queueFamilies.size()) {
            familyFlags = queueFamilies[qci.queueFamilyIndex].queueFlags;
        }
        Log::print<INFO>(" - Queue family {}: {} queues, familyFlags=0x{:X}, createFlags=0x{:X}",
            qci.queueFamilyIndex, qci.queueCount, familyFlags, qci.flags);
    }

    VkResult result = pDispatch->CreateDevice(gpu, &modifiedCreateInfo, pAllocator, pDevice);
    if (result != VK_SUCCESS) {
        Log::print<ERROR>("Failed to create Vulkan device! Error {}", result);
        return result;
    }

    // Initialize VRManager late if neither vkEnumeratePhysicalDevices and vkGetPhysicalDeviceProperties were called and used to filter the device
    if (!VRManager::instance().VK) {
        Log::print<WARNING>("Wasn't able to filter OpenXR-compatible devices for this instance!");
        Log::print<WARNING>("You might encounter an error if you've selected a GPU that's not connected to the VR headset in Cemu's settings. Usually this error is fine as long as this is the case.");
        Log::print<WARNING>("This issue appears due to OBS's Vulkan layer being installed which skips some calls used to hide GPUs that aren't compatible with your VR headset.");
    }

    return result;
}

void VRLayer::VkDeviceOverrides::DestroyDevice(const vkroots::VkDeviceDispatch* pDispatch, VkDevice device, const VkAllocationCallbacks* pAllocator) {
    return pDispatch->DestroyDevice(device, pAllocator);
}

VKROOTS_DEFINE_LAYER_INTERFACES(VRLayer::VkInstanceOverrides, VRLayer::VkPhysicalDeviceOverrides, VRLayer::VkDeviceOverrides);

// todo: These methods aren't required since we already use the negotiatelayerinterface function, thus can be removed for simplicity
VK_LAYER_EXPORT PFN_vkVoidFunction VKAPI_CALL VRLayer_GetInstanceProcAddr(VkInstance instance, const char* pName) {
    return vkroots::GetInstanceProcAddr<VRLayer::VkInstanceOverrides, VRLayer::VkPhysicalDeviceOverrides, VRLayer::VkDeviceOverrides>(instance, pName);
}

VK_LAYER_EXPORT PFN_vkVoidFunction VKAPI_CALL VRLayer_GetDeviceProcAddr(VkDevice device, const char* pName) {
    return vkroots::GetDeviceProcAddr<VRLayer::VkInstanceOverrides, VRLayer::VkPhysicalDeviceOverrides, VRLayer::VkDeviceOverrides>(device, pName);
}
