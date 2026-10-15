#include "pch.h"
#include "vulkan_device.h"
#include "instance.h"
#include "vulkan_loader.h"

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

VkResult VulkanDevice::CreateInstance(PFN_vkCreateInstance createInstanceFunc, const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance) {
    if (!createInstanceFunc || !pCreateInfo || !pInstance) {
        return VK_ERROR_INITIALIZATION_FAILED;
    }

    // OpenXR's graphics requirements have to be queried before the runtime creates the Vulkan instance
    OpenXR* xr = VRManager::instance().XR.get();
    checkAssert(xr != nullptr, "The OpenXR instance wasn't available while creating the Vulkan instance!");

    VkInstanceCreateInfo modifiedCreateInfo = *pCreateInfo;
    std::vector<const char*> modifiedExtensions;
    modifiedExtensions.reserve(pCreateInfo->enabledExtensionCount + 1);
    if (pCreateInfo->ppEnabledExtensionNames) {
        for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
            modifiedExtensions.push_back(pCreateInfo->ppEnabledExtensionNames[i]);
        }
    }

    VkApplicationInfo modifiedApplicationInfo = {};
    if (pCreateInfo->pApplicationInfo != nullptr) {
        modifiedApplicationInfo = *pCreateInfo->pApplicationInfo;
        modifiedApplicationInfo.apiVersion = xr->ClampVulkanApiVersion(modifiedApplicationInfo.apiVersion);
        modifiedCreateInfo.pApplicationInfo = &modifiedApplicationInfo;
    }

    const std::vector<std::string> runtimeExtensions = xr->UsesVulkanEnable2() ? std::vector<std::string>{} : xr->GetRequiredVulkanInstanceExtensions();
    for (const std::string& runtimeExtension : runtimeExtensions) {
        if (std::ranges::none_of(modifiedExtensions, [&runtimeExtension](const char* name) { return name != nullptr && runtimeExtension == name; })) {
            modifiedExtensions.push_back(runtimeExtension.c_str());
            Log::print<INFO>("Adding the OpenXR runtime's required instance extension {}", runtimeExtension);
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

    modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
    modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.empty() ? nullptr : modifiedExtensions.data();

    // with enable2 the runtime creates the instance itself, so vkroots needs its dispatch table built here
    auto buildDispatchTable = [&]() {
        PFN_vkGetInstanceProcAddr realGetInstanceProcAddr = VulkanLoader::GetInstanceProcAddr();
        vkroots::tables::CreateDispatchTable(realGetInstanceProcAddr, realGetInstanceProcAddr, *pInstance);
    };

    auto plainCreate = [&](const VkInstanceCreateInfo& createInfo) -> VkResult {
        static const auto realCreateInstance = (PFN_vkCreateInstance)VulkanLoader::GetInstanceProcAddr()(VK_NULL_HANDLE, "vkCreateInstance");
        const VkResult result = realCreateInstance(&createInfo, pAllocator, pInstance);
        if (result == VK_SUCCESS) {
            buildDispatchTable();
        }
        return result;
    };

    auto terminalCreate = [&](const VkInstanceCreateInfo& createInfo) -> VkResult {
        if (!xr->UsesVulkanEnable2()) {
            return plainCreate(createInfo);
        }

        const VkResult xrResult = xr->CreateVulkanInstance(createInfo, pAllocator, pInstance);
        if (xrResult == VK_SUCCESS) {
            buildDispatchTable();
            Log::print<INFO>("The OpenXR runtime created this Vulkan instance through xrCreateVulkanInstanceKHR");
            return VK_SUCCESS;
        }
        if (xrResult == VK_ERROR_EXTENSION_NOT_PRESENT) {
            return xrResult;
        }

        Log::print<WARNING>("The OpenXR runtime wouldn't create this Vulkan instance (error {}), creating it without the runtime instead", xrResult);
        return plainCreate(createInfo);
    };

    VkResult result = terminalCreate(modifiedCreateInfo);
#ifdef _DEBUG
    if (result == VK_ERROR_EXTENSION_NOT_PRESENT && addedDebugUtilsExtension) {
        debugUtilsEnabled = false;
        modifiedExtensions.pop_back();
        modifiedCreateInfo.pNext = pCreateInfo->pNext;
        modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
        modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.empty() ? nullptr : modifiedExtensions.data();
        result = terminalCreate(modifiedCreateInfo);
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

    VRManager::instance().vkVersion = modifiedApplicationInfo.apiVersion;

    Log::print<INFO>("Created Vulkan instance (using Vulkan {}.{}.{}) successfully!", VK_API_VERSION_MAJOR(modifiedApplicationInfo.apiVersion), VK_API_VERSION_MINOR(modifiedApplicationInfo.apiVersion), VK_API_VERSION_PATCH(modifiedApplicationInfo.apiVersion));
    checkAssert(VK_VERSION_MINOR(modifiedApplicationInfo.apiVersion) != 0 || VK_VERSION_MAJOR(modifiedApplicationInfo.apiVersion) > 1, "Vulkan version needs to be v1.1 or higher!");

    Log::OnInstanceCreated();
    return result;
}


const std::vector<std::string> additionalDeviceExtensions = {
    VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME,
    VK_KHR_SYNCHRONIZATION_2_EXTENSION_NAME,
#if ENABLE_VK_DEVICE_FAULT
    VK_EXT_DEVICE_FAULT_EXTENSION_NAME,
#endif
#if ENABLE_VK_ROBUSTNESS
    VK_EXT_ROBUSTNESS_2_EXTENSION_NAME,
    VK_EXT_IMAGE_ROBUSTNESS_EXTENSION_NAME
#endif
};

VkResult VulkanDevice::CreateDevice(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice gpu, const VkDeviceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDevice* pDevice) {
    OpenXR* xr = VRManager::instance().XR.get();
    checkAssert(xr != nullptr, "The OpenXR instance wasn't available while creating the Vulkan device!");

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

    std::vector<const char*> modifiedExtensions;
    for (uint32_t i = 0; i < pCreateInfo->enabledExtensionCount; i++) {
        modifiedExtensions.push_back(pCreateInfo->ppEnabledExtensionNames[i]);
    }
    auto addExtension = [&](const std::string& extension) {
        if (std::find(modifiedExtensions.begin(), modifiedExtensions.end(), extension) == modifiedExtensions.end()) {
            if (isExtensionSupported(extension)) {
                modifiedExtensions.push_back(extension.c_str());
            }
            else {
                Log::print<WARNING>("Device extension {} is not supported, skipping", extension);
            }
        }
    };

    for (const std::string& extension : additionalDeviceExtensions) {
        addExtension(extension);
    }

    const std::vector<std::string> runtimeExtensions = xr->UsesVulkanEnable2() ? std::vector<std::string>{} : xr->GetRequiredVulkanDeviceExtensions();
    for (const std::string& extension : runtimeExtensions) {
        Log::print<INFO>("Adding the OpenXR runtime's required device extension {}", extension);
        addExtension(extension);
    }

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

    VkDeviceCreateInfo modifiedCreateInfo = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    modifiedCreateInfo.pNext = nextChain;
    modifiedCreateInfo.flags = pCreateInfo->flags;
    modifiedCreateInfo.enabledLayerCount = pCreateInfo->enabledLayerCount;
    modifiedCreateInfo.ppEnabledLayerNames = pCreateInfo->ppEnabledLayerNames;
    modifiedCreateInfo.enabledExtensionCount = (uint32_t)modifiedExtensions.size();
    modifiedCreateInfo.ppEnabledExtensionNames = modifiedExtensions.data();
    // AMD GPU FIX: Preserve pEnabledFeatures from original create info
    // Dropping this can cause AMD drivers to disable features the application needs
    modifiedCreateInfo.pEnabledFeatures = pCreateInfo->pEnabledFeatures;

    uint32_t queueFamilyCount = 0;
    pDispatch.GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    if (queueFamilyCount > 0) {
        pDispatch.GetPhysicalDeviceQueueFamilyProperties(gpu, &queueFamilyCount, queueFamilies.data());
    }

    // the mod's queue has to be in Cemu's graphics family since the layer textures use exclusive sharing
    std::vector<VkDeviceQueueCreateInfo> modifiedQueueCreateInfos(pCreateInfo->pQueueCreateInfos, pCreateInfo->pQueueCreateInfos + pCreateInfo->queueCreateInfoCount);
    checkAssert(!modifiedQueueCreateInfos.empty(), "Cemu created a Vulkan device without a single queue!");

    uint32_t graphicsQueueInfoIdx = UINT32_MAX;
    for (uint32_t i = 0; i < (uint32_t)modifiedQueueCreateInfos.size(); i++) {
        const uint32_t familyIndex = modifiedQueueCreateInfos[i].queueFamilyIndex;
        if (familyIndex < queueFamilies.size() && (queueFamilies[familyIndex].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
            graphicsQueueInfoIdx = i;
            break;
        }
    }
    if (graphicsQueueInfoIdx == UINT32_MAX) {
        graphicsQueueInfoIdx = 0;
        Log::print<WARNING>("None of Cemu's queue families reports graphics support, falling back to its first queue family for the mod's own submits");
    }

    VkDeviceQueueCreateInfo& graphicsQueueCreateInfo = modifiedQueueCreateInfos[graphicsQueueInfoIdx];
    const uint32_t familyCapacity = graphicsQueueCreateInfo.queueFamilyIndex < queueFamilies.size() ? queueFamilies[graphicsQueueCreateInfo.queueFamilyIndex].queueCount : graphicsQueueCreateInfo.queueCount;

    std::vector<float> modQueuePriorities;
    ModQueueBinding modQueue = {};
    modQueue.familyIndex = graphicsQueueCreateInfo.queueFamilyIndex;
    if (graphicsQueueCreateInfo.queueCount < familyCapacity) {
        modQueuePriorities.assign(graphicsQueueCreateInfo.pQueuePriorities, graphicsQueueCreateInfo.pQueuePriorities + graphicsQueueCreateInfo.queueCount);
        modQueuePriorities.push_back(1.0f);

        modQueue.queueIndex = graphicsQueueCreateInfo.queueCount;
        modQueue.shared = false;

        graphicsQueueCreateInfo.queueCount += 1;
        graphicsQueueCreateInfo.pQueuePriorities = modQueuePriorities.data();
    }
    else {
        modQueue.queueIndex = 0;
        modQueue.shared = true;
        Log::print<WARNING>("Queue family {} is already fully claimed by Cemu ({} of {} queues), so the mod shares Cemu's queue instead", modQueue.familyIndex, graphicsQueueCreateInfo.queueCount, familyCapacity);
    }
    VRManager::instance().modQueue = modQueue;

    modifiedCreateInfo.queueCreateInfoCount = (uint32_t)modifiedQueueCreateInfos.size();
    modifiedCreateInfo.pQueueCreateInfos = modifiedQueueCreateInfos.data();

    Log::print<INFO>("Creating Vulkan device with {} queue infos, mod queue on family {} index {}{}", modifiedCreateInfo.queueCreateInfoCount, modQueue.familyIndex, modQueue.queueIndex, modQueue.shared ? " (shared with Cemu)" : "");
    for (uint32_t i = 0; i < modifiedCreateInfo.queueCreateInfoCount; i++) {
        const auto& qci = modifiedCreateInfo.pQueueCreateInfos[i];
        VkQueueFlags familyFlags = 0;
        if (qci.queueFamilyIndex < queueFamilies.size()) {
            familyFlags = queueFamilies[qci.queueFamilyIndex].queueFlags;
        }
        Log::print<INFO>(" - Queue family {}: {} queues, familyFlags=0x{:X}, createFlags=0x{:X}",
            qci.queueFamilyIndex, qci.queueCount, familyFlags, qci.flags);
    }

    // with enable2 the runtime creates the device itself, so vkroots never builds its dispatch table
    VkResult result;
    if (xr->UsesVulkanEnable2()) {
        // the runtime creates the device on the GPU it named itself
        if (xr->GetVulkanGraphicsDevice(pDispatch.pInstanceDispatch->Instance) != gpu) {
            Log::print<WARNING>("The OpenXR runtime names a different Vulkan GPU than the one Cemu is creating its device on");
        }
        result = xr->CreateVulkanDevice(gpu, modifiedCreateInfo, pAllocator, pDevice);
        if (result == VK_SUCCESS) {
            vkroots::tables::CreateDispatchTable(&modifiedCreateInfo, VulkanLoader::GetDeviceProcAddr(), gpu, *pDevice);
            Log::print<INFO>("The OpenXR runtime created this Vulkan device through xrCreateVulkanDeviceKHR");
        }
        else {
            Log::print<WARNING>("The OpenXR runtime wouldn't create the Vulkan device (error {}), creating it without the runtime instead", result);
            result = pDispatch.CreateDevice(gpu, &modifiedCreateInfo, pAllocator, pDevice);
        }
    }
    else {
        result = pDispatch.CreateDevice(gpu, &modifiedCreateInfo, pAllocator, pDevice);
    }

    if (result != VK_SUCCESS) {
        Log::print<ERROR>("Failed to create Vulkan device! Error {}", result);
        return result;
    }

    return result;
}

void VulkanDevice::LogDeviceFaultInfo() {
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
