#pragma once
#include "pch.h"


namespace VulkanUtils {
    static constexpr VkImageAspectFlags GetAspectMaskForFormat(VkFormat format) {
        switch (format) {
            case VK_FORMAT_D16_UNORM:
            case VK_FORMAT_D32_SFLOAT:
            case VK_FORMAT_X8_D24_UNORM_PACK32:
                return VK_IMAGE_ASPECT_DEPTH_BIT;

            case VK_FORMAT_D16_UNORM_S8_UINT:
            case VK_FORMAT_D24_UNORM_S8_UINT:
            case VK_FORMAT_D32_SFLOAT_S8_UINT:
                return VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;

            case VK_FORMAT_S8_UINT:
                return VK_IMAGE_ASPECT_STENCIL_BIT;

            default:
                return VK_IMAGE_ASPECT_COLOR_BIT;
        }
    }

    static constexpr bool IsDepthFormat(VkFormat format) {
        switch (format) {
            case VK_FORMAT_D16_UNORM:
            case VK_FORMAT_D32_SFLOAT:
            case VK_FORMAT_X8_D24_UNORM_PACK32:
            case VK_FORMAT_D16_UNORM_S8_UINT:
            case VK_FORMAT_D24_UNORM_S8_UINT:
            case VK_FORMAT_D32_SFLOAT_S8_UINT:
                return true;
            default:
                return false;
        }
    }

    // Check if a Vulkan format applies the sRGB transfer function on attachment writes
    static constexpr bool IsSrgbFormat(VkFormat format) {
        switch (format) {
            case VK_FORMAT_R8_SRGB:
            case VK_FORMAT_R8G8_SRGB:
            case VK_FORMAT_R8G8B8_SRGB:
            case VK_FORMAT_B8G8R8_SRGB:
            case VK_FORMAT_R8G8B8A8_SRGB:
            case VK_FORMAT_B8G8R8A8_SRGB:
            case VK_FORMAT_A8B8G8R8_SRGB_PACK32:
                return true;
            default:
                return false;
        }
    }

    static void TransitionLayout(VkCommandBuffer cmdBuffer, VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout, VkImageAspectFlags aspectMask = VK_IMAGE_ASPECT_COLOR_BIT) {
        // AMD GPU FIX: Skip redundant transitions - some AMD drivers are strict about this
        if (oldLayout == newLayout) {
            return;
        }

        VkImageMemoryBarrier barrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
        barrier.srcAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        // AMD GPU FIX: Use VK_REMAINING to cover all subresources
        barrier.subresourceRange = {
            .aspectMask = aspectMask,
            .baseMipLevel = 0,
            .levelCount = VK_REMAINING_MIP_LEVELS,
            .baseArrayLayer = 0,
            .layerCount = VK_REMAINING_ARRAY_LAYERS
        };

        vkroots::tables::CommandBufferDispatches.find(cmdBuffer)->CmdPipelineBarrier(cmdBuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    }

    static void DebugPipelineBarrier(VkCommandBuffer cmdBuffer) {
        VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        memoryBarrier.srcAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT |
            VK_ACCESS_INDEX_READ_BIT |
            VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT |
            VK_ACCESS_UNIFORM_READ_BIT |
            VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
            VK_ACCESS_SHADER_READ_BIT |
            VK_ACCESS_SHADER_WRITE_BIT |
            VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_TRANSFER_READ_BIT |
            VK_ACCESS_TRANSFER_WRITE_BIT |
            VK_ACCESS_HOST_READ_BIT |
            VK_ACCESS_HOST_WRITE_BIT |
            VK_ACCESS_MEMORY_READ_BIT |
            VK_ACCESS_MEMORY_WRITE_BIT;

        memoryBarrier.dstAccessMask = VK_ACCESS_INDIRECT_COMMAND_READ_BIT |
            VK_ACCESS_INDEX_READ_BIT |
            VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT |
            VK_ACCESS_UNIFORM_READ_BIT |
            VK_ACCESS_INPUT_ATTACHMENT_READ_BIT |
            VK_ACCESS_SHADER_READ_BIT |
            VK_ACCESS_SHADER_WRITE_BIT |
            VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
            VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT |
            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
            VK_ACCESS_TRANSFER_READ_BIT |
            VK_ACCESS_TRANSFER_WRITE_BIT |
            VK_ACCESS_HOST_READ_BIT |
            VK_ACCESS_HOST_WRITE_BIT |
            VK_ACCESS_MEMORY_READ_BIT |
            VK_ACCESS_MEMORY_WRITE_BIT;

        // ALL_COMMANDS does not cover the host, so the HOST_* access bits need the host stage alongside it
        constexpr VkPipelineStageFlags stageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT | VK_PIPELINE_STAGE_HOST_BIT;
        vkroots::tables::CommandBufferDispatches.find(cmdBuffer)->CmdPipelineBarrier(cmdBuffer, stageMask, stageMask, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
    }

}
