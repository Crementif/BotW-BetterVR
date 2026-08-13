#include "vulkan.h"
#include "hooking/layer.h"
#include "instance.h"
#include "utils/logger.h"

RND_Vulkan::RND_Vulkan(VkInstance vkInstance, VkPhysicalDevice vkPhysDevice, VkDevice vkDevice): m_instance(vkInstance), m_physicalDevice(vkPhysDevice), m_device(vkDevice) {
    m_instanceDispatch = vkroots::tables::InstanceDispatches.find(vkInstance);
    m_physicalDeviceDispatch = vkroots::tables::PhysicalDeviceDispatches.find(vkPhysDevice);
    m_deviceDispatch = vkroots::tables::DeviceDispatches.find(vkDevice);

    // AMD GPU FIX: Initialize sType before calling vkGetPhysicalDeviceMemoryProperties2
    m_memoryProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MEMORY_PROPERTIES_2;
    m_memoryProperties.pNext = nullptr;
    m_physicalDeviceDispatch->GetPhysicalDeviceMemoryProperties2KHR(vkPhysDevice, &m_memoryProperties);

    VkPhysicalDeviceProperties props{};
    m_instanceDispatch->GetPhysicalDeviceProperties(vkPhysDevice, &props);

    uint64_t localVramBytes = 0;
    for (uint32_t i = 0; i < m_memoryProperties.memoryProperties.memoryHeapCount; ++i) {
        if ((m_memoryProperties.memoryProperties.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0) {
            localVramBytes += m_memoryProperties.memoryProperties.memoryHeaps[i].size;
        }
    }

    Log::print<INFO>("GPU: {} (vendor={:#06x}, device={:#06x}, driver={})", props.deviceName, props.vendorID, props.deviceID, props.driverVersion);
    if (localVramBytes > 0) {
        Log::print<INFO>("GPU VRAM (device local): {:.2f} GiB", double(localVramBytes) / (1024.0 * 1024.0 * 1024.0));
    }
}

RND_Vulkan::~RND_Vulkan() {
}

uint32_t RND_Vulkan::FindMemoryType(uint32_t memoryTypeBitsRequirement, VkMemoryPropertyFlags requirementsMask) {
    // AMD GPU FIX: Use actual memoryTypeCount instead of VK_MAX_MEMORY_TYPES to avoid reading uninitialized data
    const uint32_t memoryTypeCount = m_memoryProperties.memoryProperties.memoryTypeCount;
    for (uint32_t i = 0; i < memoryTypeCount; i++) {
        const uint32_t memoryTypeBits = (1u << i);
        const bool isRequiredMemoryType = (memoryTypeBitsRequirement & memoryTypeBits) != 0;
        const bool satisfiesFlags = (m_memoryProperties.memoryProperties.memoryTypes[i].propertyFlags & requirementsMask) == requirementsMask;

        if (isRequiredMemoryType && satisfiesFlags) {
            return i;
        }
    }
    checkAssert(false, "Failed to find suitable memory type");
    return 0;
}


VkResult VRLayer::VkDeviceOverrides::GetPhysicalDeviceSurfacePresentModesKHR(const vkroots::VkDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, uint32_t* pPresentModeCount, VkPresentModeKHR* pPresentModes) {
    // check all supported present modes
    uint32_t testPresentModes = 0;
    VkResult result = pDispatch.GetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &testPresentModes, VK_NULL_HANDLE);

    std::vector<VkPresentModeKHR> supportedPresentModes;
    supportedPresentModes.resize(testPresentModes);

    result = pDispatch.GetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &testPresentModes, supportedPresentModes.data());

    checkAssert(result == VK_SUCCESS, "Failed to get physical device surface present modes!");

    // prefer MAILBOX over IMMEDIATE: on virtual/streamed displays (Virtual Desktop, Parsec)
    // even IMMEDIATE presents can block at the display's refresh rate, which would pace the
    // whole game loop at ~60Hz; MAILBOX replaces the queued image and never blocks
    for (VkPresentModeKHR preferred : { VK_PRESENT_MODE_MAILBOX_KHR, VK_PRESENT_MODE_IMMEDIATE_KHR }) {
        for (uint32_t i = 0; i < testPresentModes; i++) {
            if (supportedPresentModes[i] == preferred) {
                if (pPresentModes != VK_NULL_HANDLE) {
                    pPresentModes[0] = preferred;
                }
                else {
                    *pPresentModeCount = 1;
                }
                return VK_SUCCESS;
            }
        }
    }

    // otherwise, we just try and use whatever mode is available
    return pDispatch.GetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, pPresentModeCount, pPresentModes);
}

VkResult VRLayer::VkDeviceOverrides::CreateSwapchainKHR(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkSwapchainCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkSwapchainKHR* pSwapchain) {
    // force MAILBOX when supported (see GetPhysicalDeviceSurfacePresentModesKHR above); also
    // give the swapchain a third image so a queued-but-unconsumed image never stalls acquire
    VkSwapchainCreateInfoKHR modifiedCreateInfo = *pCreateInfo;
    uint32_t supportedCount = 0;
    if (pDispatch.GetPhysicalDeviceSurfacePresentModesKHR(pDispatch.PhysicalDevice, pCreateInfo->surface, &supportedCount, nullptr) == VK_SUCCESS && supportedCount > 0) {
        std::vector<VkPresentModeKHR> supported(supportedCount);
        if (pDispatch.GetPhysicalDeviceSurfacePresentModesKHR(pDispatch.PhysicalDevice, pCreateInfo->surface, &supportedCount, supported.data()) == VK_SUCCESS) {
            if (std::ranges::find(supported, VK_PRESENT_MODE_MAILBOX_KHR) != supported.end()) {
                modifiedCreateInfo.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
                modifiedCreateInfo.minImageCount = std::max(modifiedCreateInfo.minImageCount, 3u);
                Log::print<INFO>("Forcing MAILBOX present mode for Cemu's swapchain (was mode {})", (int)pCreateInfo->presentMode);
            }
        }
    }
    return pDispatch.CreateSwapchainKHR(device, &modifiedCreateInfo, pAllocator, pSwapchain);
}