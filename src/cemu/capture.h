#pragma once

#include "rendering/openxr.h"
#include "rendering/texture.h"

namespace Capture {
    struct Surface {
        VkImage image = VK_NULL_HANDLE;
        VkExtent2D extent = {};
        VkFormat format = VK_FORMAT_UNDEFINED;
        VkImageAspectFlags aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        uint32_t sliceIndex = 0;
    };

    struct FrameState {
        OpenXR::EyeSide side = OpenXR::EyeSide::LEFT;
        long frameIdx = 0;
        bool monoCapture = false;
    };

    std::optional<FrameState> ReadFrameState();

    // rightSurface is null when Cemu didn't draw the right eye in the same replay
    void On3DColor(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface, const Surface* rightSurface);
    void On3DDepth(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface, const Surface* rightSurface);
    void OnHud(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface);

    void OnFrameSubmitted();
}
