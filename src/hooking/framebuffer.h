#pragma once

#include "rendering/texture.h"
#include "rendering/openxr.h"

struct CaptureTexture {
    bool initialized;
    VkExtent2D foundSize;
    VkImage foundImage;
    VkFormat format;
    VkExtent2D minSize;
    std::array<std::unique_ptr<SharedTexture>, 2> sharedTextures = { nullptr, nullptr };
    std::array<bool, 2> isCapturingSharedTextures = { false, false };

    // current frame state
    VkCommandBuffer captureCmdBuffer = VK_NULL_HANDLE;
};