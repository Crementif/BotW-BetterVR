#pragma once

namespace VulkanDevice {
    VkResult CreateInstance(PFN_vkCreateInstance createInstanceFunc, const VkInstanceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkInstance* pInstance);
    VkResult CreateDevice(const vkroots::VkPhysicalDeviceDispatch& pDispatch, VkPhysicalDevice gpu, const VkDeviceCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkDevice* pDevice);
    void LogDeviceFaultInfo();
}
