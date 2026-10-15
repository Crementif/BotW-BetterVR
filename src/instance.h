#pragma once

#include <pch.h>

#include "cemu/cemu_host.h"
#include "game/botw/debug/entity_debugger.h"
#include "rendering/openxr.h"
#include "rendering/renderer.h"
#include "rendering/vk_backend.h"
#include "rendering/vulkan.h"

class VRManager {
public:
    static VRManager& instance() {
        static VRManager singletonInstance;
        return singletonInstance;
    }

    VRManager(VRManager const&) = delete;
    void operator=(VRManager const&) = delete;

    void Init(VkInstance instance, VkPhysicalDevice physicalDevice, VkDevice device) {
        VK = std::make_unique<RND_Vulkan>(instance, physicalDevice, device);
        Backend = std::make_unique<RND_VkBackend>(instance, physicalDevice, device, modQueue);
        Log::print<INFO>("Initialized VRManager instance...");
    }

    void InitSession() {
        XrGraphicsBindingVulkan2KHR vulkanBinding = { XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR };
        vulkanBinding.instance = VK->GetInstance();
        vulkanBinding.physicalDevice = VK->GetPhysicalDevice();
        vulkanBinding.device = VK->GetDevice();
        vulkanBinding.queueFamilyIndex = Backend->GetQueueBinding().familyIndex;
        vulkanBinding.queueIndex = Backend->GetQueueBinding().queueIndex;
        XR->CreateSession(vulkanBinding);
        XR->CreateActions();
        Cemu = std::make_unique<CemuHost>();
    }

    std::unique_ptr<OpenXR> XR;
    std::unique_ptr<RND_Vulkan> VK;
    std::unique_ptr<RND_VkBackend> Backend;
    std::unique_ptr<CemuHost> Cemu;
    std::unique_ptr<EntityDebugger> Debugger;

    uint32_t vkVersion = 0;
    ModQueueBinding modQueue = {};

private:
    VRManager() {
        m_logger = std::make_unique<Log>();
        XR = std::make_unique<OpenXR>();
    };

    ~VRManager() {
        // note: OpenXR gets to destroy its swapchains and layer resources before the backend and the device wrapper they were built on
        Debugger.reset();
        XR.reset();
        Backend.reset();
        VK.reset();

        m_logger.reset();
    };

    std::unique_ptr<Log> m_logger;
};
