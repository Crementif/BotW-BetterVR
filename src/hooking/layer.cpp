#include "pch.h"
#include "layer.h"
#include "instance.h"


static bool s_deviceFaultLoggingEnabled = false;

#ifdef _DEBUG
static VkInstance s_debugMessengerInstance = VK_NULL_HANDLE;
static VkDebugUtilsMessengerEXT s_debugMessenger = VK_NULL_HANDLE;

static VKAPI_ATTR VkBool32 VKAPI_CALL VulkanDebugUtilsMessengerCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT /*messageTypes*/,
    const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
    void* /*pUserData*/) {

    const char* messageIdName = pCallbackData && pCallbackData->pMessageIdName ? pCallbackData->pMessageIdName : "unknown";
    const char* message = pCallbackData && pCallbackData->pMessage ? pCallbackData->pMessage : "";
    const int32_t messageIdNumber = pCallbackData ? pCallbackData->messageIdNumber : 0;

    if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        Log::print<ERROR>("[Vulkan Debug] {} (id {}): {}", messageIdName, messageIdNumber, message);
    }
    else if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        Log::print<WARNING>("[Vulkan Debug] {} (id {}): {}", messageIdName, messageIdNumber, message);
    }
    else {
        Log::print<VERBOSE>("[Vulkan Debug] {} (id {}): {}", messageIdName, messageIdNumber, message);
    }

    return VK_FALSE;
}
#endif

VkResult VRLayer::VkInstanceOverrides::CreateInstance(PFN_vkCreateInstance createInstanceFunc, const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance) {
    if (!createInstanceFunc || !pCreateInfo || !pInstance) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }

    VkInstanceCreateInfo modifiedCreateInfo = *pCreateInfo;
    std::vector<const char*> modifiedExtensions;
    modifiedExtensions.reserve(pCreateInfo->enabledExtensionCount + 1);
    if (pCreateInfo->ppEnabledExtensionNames) {
        for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
            modifiedExtensions.push_back(pCreateInfo->ppEnabledExtensionNames[i]);
        }
    }

#ifdef _DEBUG
    bool debugUtilsEnabled = false;
    bool addedDebugUtilsExtension = false;
    VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo = { VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
    for (const char* extensionName : modifiedExtensions) {
        if (extensionName && std::strcmp(extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
            debugUtilsEnabled = true;
            break;
        }
    }

    if (!debugUtilsEnabled) {
        modifiedExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        debugUtilsEnabled = true;
        addedDebugUtilsExtension = true;
    }

    if (debugUtilsEnabled) {
        debugCreateInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        debugCreateInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
            VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        debugCreateInfo.pfnUserCallback = VulkanDebugUtilsMessengerCallback;
        debugCreateInfo.pNext = const_cast<void*>(modifiedCreateInfo.pNext);
        modifiedCreateInfo.pNext = &debugCreateInfo;
    }
#endif

    // dependencies of the device-side VK_EXT_swapchain_maintenance1 used for present throttling
    const size_t extensionCountBeforeSurfaceMaintenance = modifiedExtensions.size();
    for (const char* wanted : { VK_KHR_GET_SURFACE_CAPABILITIES_2_EXTENSION_NAME, VK_EXT_SURFACE_MAINTENANCE_1_EXTENSION_NAME }) {
        bool alreadyEnabled = false;
        for (const char* extensionName : modifiedExtensions) {
            if (extensionName && std::strcmp(extensionName, wanted) == 0) {
                alreadyEnabled = true;
                break;
            }
        }
        if (!alreadyEnabled) {
            modifiedExtensions.push_back(wanted);
        }
    }

    modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
    modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.empty() ? nullptr : modifiedExtensions.data();

    VkResult result = createInstanceFunc(&modifiedCreateInfo, pAllocator, pInstance);
    if (result == VK_ERROR_EXTENSION_NOT_PRESENT && modifiedExtensions.size() != extensionCountBeforeSurfaceMaintenance) {
        // retry without the surface-maintenance extensions on older loaders/drivers
        modifiedExtensions.resize(extensionCountBeforeSurfaceMaintenance);
        modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
        modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.empty() ? nullptr : modifiedExtensions.data();
        result = createInstanceFunc(&modifiedCreateInfo, pAllocator, pInstance);
    }
#ifdef _DEBUG
    if (result == VK_ERROR_EXTENSION_NOT_PRESENT && addedDebugUtilsExtension) {
        debugUtilsEnabled = false;
        result = createInstanceFunc(pCreateInfo, pAllocator, pInstance);
    }
#endif
    if (result != VK_SUCCESS) {
        return result;
    }

#ifdef _DEBUG
    if (debugUtilsEnabled) {
        s_debugMessengerInstance = VK_NULL_HANDLE;
        s_debugMessenger = VK_NULL_HANDLE;

        const vkroots::VkInstanceDispatch* instanceDispatch = vkroots::LookupDispatch(*pInstance);
        if (instanceDispatch) {
            if (instanceDispatch->CreateDebugUtilsMessengerEXT(*pInstance, &debugCreateInfo, pAllocator, &s_debugMessenger) == VK_SUCCESS) {
                s_debugMessengerInstance = *pInstance;
            }
            else {
                Log::print<WARNING>("Failed to create Vulkan debug messenger.");
            }
        }
    }
#endif

    VRManager::instance().vkVersion = modifiedCreateInfo.pApplicationInfo->apiVersion;

    Log::print<INFO>("Created Vulkan instance (using Vulkan {}.{}.{}) successfully!", VK_API_VERSION_MAJOR(modifiedCreateInfo.pApplicationInfo->apiVersion), VK_API_VERSION_MINOR(modifiedCreateInfo.pApplicationInfo->apiVersion), VK_API_VERSION_PATCH(modifiedCreateInfo.pApplicationInfo->apiVersion));
    checkAssert(VK_VERSION_MINOR(modifiedCreateInfo.pApplicationInfo->apiVersion) != 0 || VK_VERSION_MAJOR(modifiedCreateInfo.pApplicationInfo->apiVersion) > 1, "Vulkan version needs to be v1.1 or higher!");

    Log::OnInstanceCreated();
    return result;
}


VkResult VRLayer::VkInstanceOverrides::EnumeratePhysicalDevices(const vkroots::VkInstanceDispatch& pDispatch, VkInstance instance, uint32_t* pPhysicalDeviceCount, VkPhysicalDevice* pPhysicalDevices) {
    // Proceed to get all devices
    uint32_t internalCount = 0;
    checkVkResult(pDispatch.EnumeratePhysicalDevices(instance, &internalCount, nullptr), "Failed to retrieve number of vulkan physical devices!");
    std::vector<VkPhysicalDevice> internalDevices(internalCount);
    checkVkResult(pDispatch.EnumeratePhysicalDevices(instance, &internalCount, internalDevices.data()), "Failed to retrieve vulkan physical devices!");

    VkPhysicalDevice matchedDevice = VK_NULL_HANDLE;
    VkPhysicalDevice fallbackDevice = VK_NULL_HANDLE;

    for (const VkPhysicalDevice& device : internalDevices) {
        VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
        VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        properties.pNext = &deviceId;
        pDispatch.GetPhysicalDeviceProperties2(device, &properties);

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
        pDispatch.GetPhysicalDeviceProperties(selectedDevice, &props);
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
void VRLayer::VkInstanceOverrides::GetPhysicalDeviceProperties(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, VkPhysicalDeviceProperties* pProperties) {
    // Do original query
    pDispatch.GetPhysicalDeviceProperties(physicalDevice, pProperties);

    // Do a seperate internal query to make sure that we also query the LUID
    {
        VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
        VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
        properties.pNext = &deviceId;
        pDispatch.GetPhysicalDeviceProperties2(physicalDevice, &properties);

        if (deviceId.deviceLUIDValid && memcmp(&VRManager::instance().XR->m_capabilities.adapter, deviceId.deviceLUID, VK_LUID_SIZE) != 0) {
            pProperties->apiVersion = VK_API_VERSION_1_0;
        }
    }
}

// Some layers (OBS vulkan layer) will skip the vkEnumeratePhysicalDevices hook
// Therefor we also override vkGetPhysicalDeviceQueueFamilyProperties to make any non-VR-compatible VkPhysicalDevice have 0 queues
void VRLayer::VkInstanceOverrides::GetPhysicalDeviceQueueFamilyProperties(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, uint32_t* pQueueFamilyPropertyCount, VkQueueFamilyProperties* pQueueFamilyProperties) {
    // Check whether this VkPhysicalDevice matches the LUID that OpenXR returns
    VkPhysicalDeviceIDProperties deviceId = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES };
    VkPhysicalDeviceProperties2 properties = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
    properties.pNext = &deviceId;
    pDispatch.GetPhysicalDeviceProperties2(physicalDevice, &properties);

    if (deviceId.deviceLUIDValid && memcmp(&VRManager::instance().XR->m_capabilities.adapter, deviceId.deviceLUID, VK_LUID_SIZE) != 0) {
        *pQueueFamilyPropertyCount = 0;
        return;
    }

    return pDispatch.GetPhysicalDeviceQueueFamilyProperties(physicalDevice, pQueueFamilyPropertyCount, pQueueFamilyProperties);
}

// set when VK_EXT_swapchain_maintenance1 was enabled on the device, allowing the
// present throttle in QueuePresentKHR to hand skipped swapchain images back
bool g_swapchainMaintenance1Enabled = false;

const std::vector<std::string> additionalDeviceExtensions = {
    VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME,
    VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME,
    VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME,
    VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME,
    VK_KHR_EXTERNAL_SEMAPHORE_WIN32_EXTENSION_NAME,
    VK_KHR_SYNCHRONIZATION_2_EXTENSION_NAME,
    VK_EXT_SWAPCHAIN_MAINTENANCE_1_EXTENSION_NAME,
#if ENABLE_VK_DEVICE_FAULT
    VK_EXT_DEVICE_FAULT_EXTENSION_NAME,
#endif
#if ENABLE_VK_ROBUSTNESS
    VK_EXT_ROBUSTNESS_2_EXTENSION_NAME,
    VK_EXT_IMAGE_ROBUSTNESS_EXTENSION_NAME
#endif
};

VkResult VRLayer::VkInstanceOverrides::CreateDevice(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice gpu, const VkDeviceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDevice* pDevice) {
    // Query available extensions for this device
    uint32_t extensionCount = 0;
    pDispatch.EnumerateDeviceExtensionProperties(gpu, nullptr, &extensionCount, nullptr);
    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    pDispatch.EnumerateDeviceExtensionProperties(gpu, nullptr, &extensionCount, availableExtensions.data());

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
            }
            else {
                Log::print<WARNING>("Device extension {} is not supported, skipping", extension);
            }
        }
    }

    // Query supported features from the GPU
    VkPhysicalDeviceTimelineSemaphoreFeatures supportedTimelineSemaphoreFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES };
    VkPhysicalDeviceFeatures2 supportedFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2 };
    supportedFeatures.pNext = &supportedTimelineSemaphoreFeatures;

    void* supportedChainTail = &supportedTimelineSemaphoreFeatures;

#if ENABLE_VK_ROBUSTNESS
    VkPhysicalDeviceImageRobustnessFeatures supportedImageRobustnessFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_ROBUSTNESS_FEATURES };
    VkPhysicalDeviceRobustness2FeaturesEXT supportedRobustness2Features = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ROBUSTNESS_2_FEATURES_EXT };
    static_cast<VkBaseOutStructure*>(supportedChainTail)->pNext = reinterpret_cast<VkBaseOutStructure*>(&supportedImageRobustnessFeatures);
    supportedImageRobustnessFeatures.pNext = &supportedRobustness2Features;
    supportedChainTail = &supportedRobustness2Features;
#endif

#if ENABLE_VK_DEVICE_FAULT
    VkPhysicalDeviceFaultFeaturesEXT supportedFaultFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FAULT_FEATURES_EXT };
    static_cast<VkBaseOutStructure*>(supportedChainTail)->pNext = reinterpret_cast<VkBaseOutStructure*>(&supportedFaultFeatures);
    supportedChainTail = &supportedFaultFeatures;
#endif

    pDispatch.GetPhysicalDeviceFeatures2(gpu, &supportedFeatures);

    // Test if timeline semaphores are already enabled in the create info
    bool timelineSemaphoresEnabled = false;
    bool imageRobustnessEnabled = false;
    bool robustness2Enabled = false;
    bool deviceFaultEnabled = false;
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
#if ENABLE_VK_DEVICE_FAULT
        if (base->sType == VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FAULT_FEATURES_EXT) {
            deviceFaultEnabled = true;
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

    VkPhysicalDeviceFaultFeaturesEXT createFaultFeatures = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FAULT_FEATURES_EXT };
    createFaultFeatures.deviceFault = true;

    void* nextChain = const_cast<void*>(pCreateInfo->pNext);

#if ENABLE_VK_DEVICE_FAULT
    if (!deviceFaultEnabled && isExtensionSupported(VK_EXT_DEVICE_FAULT_EXTENSION_NAME) && supportedFaultFeatures.deviceFault) {
        createFaultFeatures.pNext = nextChain;
        nextChain = &createFaultFeatures;
        s_deviceFaultLoggingEnabled = true;
    }
#endif

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
    }
    else if (!timelineSemaphoresEnabled) {
        Log::print<ERROR>("Timeline semaphores are not supported by this GPU! VR functionality may not work.");
    }

    // swapchain maintenance1 lets the present throttle release acquired-but-unpresented images
    static VkPhysicalDeviceSwapchainMaintenance1FeaturesEXT createSwapchainMaintenance1Features = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SWAPCHAIN_MAINTENANCE_1_FEATURES_EXT };
    g_swapchainMaintenance1Enabled = isExtensionSupported(VK_EXT_SWAPCHAIN_MAINTENANCE_1_EXTENSION_NAME);
    if (g_swapchainMaintenance1Enabled) {
        createSwapchainMaintenance1Features.swapchainMaintenance1 = VK_TRUE;
        createSwapchainMaintenance1Features.pNext = nextChain;
        nextChain = &createSwapchainMaintenance1Features;
        Log::print<INFO>("Enabling VK_EXT_swapchain_maintenance1 for present throttling");
    }
    else {
        Log::print<WARNING>("VK_EXT_swapchain_maintenance1 not supported; window presents can pace the game on slow displays");
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
    pDispatch.GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    if (queueFamilyCount > 0) {
        pDispatch.GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, queueFamilies.data());
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

    VkResult result = pDispatch.CreateDevice(gpu, &modifiedCreateInfo, pAllocator, pDevice);
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

void VRLayer::VkInstanceOverrides::DestroyInstance(const vkroots::VkInstanceDispatch& pDispatch, VkInstance instance, const VkAllocationCallbacks* pAllocator) {
#ifdef _DEBUG
    if (instance == s_debugMessengerInstance && s_debugMessenger != VK_NULL_HANDLE) {
        pDispatch.DestroyDebugUtilsMessengerEXT(instance, s_debugMessenger, pAllocator);
        s_debugMessengerInstance = VK_NULL_HANDLE;
        s_debugMessenger = VK_NULL_HANDLE;
    }
#endif

    PFN_vkDestroyInstance ptr_vkDestroyInstance = (PFN_vkDestroyInstance)pDispatch.GetInstanceProcAddr(instance, "vkDestroyInstance");
    vkroots::tables::DestroyDispatchTable(instance);
    ptr_vkDestroyInstance(instance, pAllocator);

    Log::OnInstanceDestroyed();
}

void VRLayer::VkDeviceOverrides::DestroyDevice(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkAllocationCallbacks* pAllocator) {
    PFN_vkDestroyDevice ptr_vkDestroyDeviceFn = (PFN_vkDestroyDevice)pDispatch.GetDeviceProcAddr(device, "vkDestroyDevice");
    vkroots::tables::DestroyDispatchTable(device);
    ptr_vkDestroyDeviceFn(device, pAllocator);
}

void VRLayer::LogDeviceFaultInfo() {
    if (!s_deviceFaultLoggingEnabled) {
        return;
    }

    static std::atomic<bool> s_alreadyReported = false;
    if (s_alreadyReported.exchange(true)) {
        return;
    }

    RND_Vulkan* vk = VRManager::instance().VK.get();
    if (!vk) {
        return;
    }
    const vkroots::VkDeviceDispatch* dispatch = vk->GetDeviceDispatch();
    VkDevice device = vk->GetDevice();
    if (!dispatch || device == VK_NULL_HANDLE) {
        return;
    }

    VkDeviceFaultCountsEXT faultCounts = { VK_STRUCTURE_TYPE_DEVICE_FAULT_COUNTS_EXT };
    VkResult result = dispatch->GetDeviceFaultInfoEXT(device, &faultCounts, nullptr);
    if (result < 0) {
        Log::print<ERROR>("vkGetDeviceFaultInfoEXT failed to report fault counts with error {}", result);
        return;
    }

    std::vector<VkDeviceFaultAddressInfoEXT> addressInfos(faultCounts.addressInfoCount);
    std::vector<VkDeviceFaultVendorInfoEXT> vendorInfos(faultCounts.vendorInfoCount);
    faultCounts.vendorBinarySize = 0;

    VkDeviceFaultInfoEXT faultInfo = { VK_STRUCTURE_TYPE_DEVICE_FAULT_INFO_EXT };
    faultInfo.pAddressInfos = addressInfos.empty() ? nullptr : addressInfos.data();
    faultInfo.pVendorInfos = vendorInfos.empty() ? nullptr : vendorInfos.data();
    faultInfo.pVendorBinaryData = nullptr;

    result = dispatch->GetDeviceFaultInfoEXT(device, &faultCounts, &faultInfo);
    if (result < 0) {
        Log::print<ERROR>("vkGetDeviceFaultInfoEXT failed to report fault info with error {}", result);
        return;
    }

    Log::print<ERROR>("VK_EXT_device_fault reports {}", faultInfo.description);
    for (uint32_t i = 0; i < faultCounts.addressInfoCount; i++) {
        const VkDeviceFaultAddressInfoEXT& addressInfo = addressInfos[i];
        uint64_t alignedAddress = addressInfo.addressPrecision ? (addressInfo.reportedAddress & ~(addressInfo.addressPrecision - 1)) : addressInfo.reportedAddress;
        Log::print<ERROR>(" - Address fault: {} at 0x{:X} (precision 0x{:X}, aligned 0x{:X})", vkroots::helpers::enumString(addressInfo.addressType), addressInfo.reportedAddress, addressInfo.addressPrecision, alignedAddress);
    }
    for (uint32_t i = 0; i < faultCounts.vendorInfoCount; i++) {
        const VkDeviceFaultVendorInfoEXT& vendorInfo = vendorInfos[i];
        Log::print<ERROR>(" - Vendor fault: {} (code 0x{:X}, data 0x{:X})", vendorInfo.description, vendorInfo.vendorFaultCode, vendorInfo.vendorFaultData);
    }
    if (faultCounts.addressInfoCount == 0 && faultCounts.vendorInfoCount == 0) {
        Log::print<ERROR>("The driver did not report any address or vendor fault details.");
    }
}

VKROOTS_DEFINE_LAYER_INTERFACES(VRLayer::VkInstanceOverrides, VRLayer::VkDeviceOverrides);
