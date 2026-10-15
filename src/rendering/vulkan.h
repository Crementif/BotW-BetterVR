#pragma once
#include "openxr.h"
#include "texture.h"


class RND_Vulkan {
public:
    RND_Vulkan(VkInstance vkInstance, VkPhysicalDevice vkPhysDevice, VkDevice vkDevice);
    ~RND_Vulkan();

    uint32_t FindMemoryType(uint32_t memoryTypeBitsRequirement, VkMemoryPropertyFlags requirementsMask);
    VkInstance GetInstance() { return m_instance; }
    VkDevice GetDevice() { return m_device; }
    VkPhysicalDevice GetPhysicalDevice() { return m_physicalDevice; }

    const vkroots::VkInstanceDispatch* GetInstanceDispatch() const { return m_instanceDispatch; }
    const vkroots::VkDeviceDispatch* GetDeviceDispatch() const { return m_deviceDispatch; }

private:
    VkInstance m_instance;
    VkPhysicalDevice m_physicalDevice;
    VkDevice m_device;
    VkPhysicalDeviceMemoryProperties2 m_memoryProperties = {};

    const vkroots::VkInstanceDispatch* m_instanceDispatch;
    const vkroots::VkPhysicalDeviceDispatch* m_physicalDeviceDispatch;
    const vkroots::VkDeviceDispatch* m_deviceDispatch;
};