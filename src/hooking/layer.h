#pragma once

namespace VRLayer {
    void LogDeviceFaultInfo();

    class VkInstanceOverrides {
    public:
        static VkResult CreateInstance(PFN_vkCreateInstance createInstanceFunc, const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance);
        static void DestroyInstance(const vkroots::VkInstanceDispatch& pDispatch, VkInstance instance, const VkAllocationCallbacks* pAllocator);

        static VkResult EnumeratePhysicalDevices(const vkroots::VkInstanceDispatch& pDispatch, VkInstance instance, uint32_t* pPhysicalDeviceCount, VkPhysicalDevice* pPhysicalDevices);
        static void GetPhysicalDeviceProperties(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, VkPhysicalDeviceProperties* pProperties);
        static void GetPhysicalDeviceQueueFamilyProperties(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, uint32_t* pQueueFamilyPropertyCount, VkQueueFamilyProperties* pQueueFamilyProperties);

        static VkResult CreateDevice(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice gpu, const VkDeviceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDevice* pDevice);
    };


    class VkDeviceOverrides {
    public:
        static void DestroyDevice(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkAllocationCallbacks* pAllocator);

        // Overrides used for finding framebuffer
        static VkResult CreateImage(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkImageCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkImage* pImage);
        static void DestroyImage(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, VkImage image, const VkAllocationCallbacks* pAllocator);
        static void CmdClearColorImage(const vkroots::VkCommandBufferDispatch& pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearColorValue* pColor, uint32_t rangeCount, const VkImageSubresourceRange* pRanges);
        static void CmdClearDepthStencilImage(const vkroots::VkCommandBufferDispatch& pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearDepthStencilValue* pDepthStencil, uint32_t rangeCount, const VkImageSubresourceRange* pRanges);
        static VkResult QueuePresentKHR(const vkroots::VkQueueDispatch& pDispatch, VkQueue queue, const VkPresentInfoKHR* pPresentInfo);


        // frame manager
        static VkResult CreateSwapchainKHR(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkSwapchainCreateInfoKHR* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkSwapchainKHR* pSwapchain);
        static void DestroySwapchainKHR(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, VkSwapchainKHR swapchain, const VkAllocationCallbacks* pAllocator);
        static VkResult GetPhysicalDeviceSurfacePresentModesKHR(const vkroots::VkDeviceDispatch& pDispatch, VkPhysicalDevice physicalDevice, VkSurfaceKHR surface, uint32_t* pPresentModeCount, VkPresentModeKHR* pPresentModes);
        static VkResult QueueSubmit(const vkroots::VkQueueDispatch& pDispatch, VkQueue queue, uint32_t submitCount, const VkSubmitInfo* pSubmits, VkFence fence);
    };
}