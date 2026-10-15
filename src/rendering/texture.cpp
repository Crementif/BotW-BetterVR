#include "texture.h"
#include "instance.h"
#include "utils/vulkan_utils.h"


BaseVulkanTexture::~BaseVulkanTexture() {
    vkReleaseStaging();
    if (m_vkImage != VK_NULL_HANDLE) {
        VRManager::instance().VK->GetDeviceDispatch()->DestroyImage(VRManager::instance().VK->GetDevice(), m_vkImage, nullptr);
        m_vkImage = VK_NULL_HANDLE;
    }
    if (m_vkMemory != VK_NULL_HANDLE) {
        VRManager::instance().VK->GetDeviceDispatch()->FreeMemory(VRManager::instance().VK->GetDevice(), m_vkMemory, nullptr);
        m_vkMemory = VK_NULL_HANDLE;
    }
}

void BaseVulkanTexture::vkPipelineBarrier(VkCommandBuffer cmdBuffer) {
    return VulkanUtils::DebugPipelineBarrier(cmdBuffer);
}

VkImageAspectFlags BaseVulkanTexture::GetAspectMask() const {
    return VulkanUtils::GetAspectMaskForFormat(m_vkFormat);
}

void BaseVulkanTexture::vkTransitionLayout(VkCommandBuffer cmdBuffer, VkImageLayout newLayout) {
    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
    VulkanUtils::TransitionLayout(cmdBuffer, m_vkImage, m_vkCurrLayout, newLayout, GetAspectMask());
    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
    m_vkCurrLayout = newLayout;
}

void BaseVulkanTexture::vkCopyToImage(VkCommandBuffer cmdBuffer, VkImage dstImage) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    VkImageAspectFlags aspectMask = GetAspectMask();

    const VkImageCopy region = {
        .srcSubresource = { aspectMask, 0, 0, 1 },
        .srcOffset = { 0, 0, 0 },
        .dstSubresource = { aspectMask, 0, 0, 1 },
        .dstOffset = { 0, 0, 0 },
        .extent = {
            .width = m_width,
            .height = m_height,
            .depth = 1
        }
    };

    // the only caller hands over a Cemu image that the capture hook already moved to GENERAL
    dispatch->CmdCopyImage(cmdBuffer, m_vkImage, m_vkCurrLayout, dstImage, VK_IMAGE_LAYOUT_GENERAL, 1, &region);

    // AMD GPU FIX: Add memory barrier after copy to ensure data is visible
    VkMemoryBarrier postCopyBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
    postCopyBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    postCopyBarrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;

    dispatch->CmdPipelineBarrier(cmdBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &postCopyBarrier, 0, nullptr, 0, nullptr);
}

void BaseVulkanTexture::vkClear(VkCommandBuffer cmdBuffer, VkClearColorValue color) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    if (VulkanUtils::IsDepthFormat(m_vkFormat)) {
        Log::print<WARNING>("vkClear called on depth image - use vkClearDepth instead");
        return;
    }

    if (m_vkCurrLayout != VK_IMAGE_LAYOUT_GENERAL && m_vkCurrLayout != VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
        VulkanUtils::TransitionLayout(cmdBuffer, m_vkImage, m_vkCurrLayout, VK_IMAGE_LAYOUT_GENERAL);
        m_vkCurrLayout = VK_IMAGE_LAYOUT_GENERAL;
    }

    const VkImageSubresourceRange range = {
        .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
        .baseMipLevel = 0,
        .levelCount = VK_REMAINING_MIP_LEVELS,
        .baseArrayLayer = 0,
        .layerCount = VK_REMAINING_ARRAY_LAYERS
    };

    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
    dispatch->CmdClearColorImage(cmdBuffer, m_vkImage, m_vkCurrLayout, &color, 1, &range);
    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
}

void BaseVulkanTexture::vkClearDepth(VkCommandBuffer cmdBuffer, float depth, uint32_t stencil) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    if (!VulkanUtils::IsDepthFormat(m_vkFormat)) {
        Log::print<WARNING>("vkClearDepth called on color image - use vkClear instead");
        return;
    }

    // AMD GPU FIX: Transition to GENERAL if not already in a valid clear layout
    // CmdClearDepthStencilImage requires GENERAL or TRANSFER_DST_OPTIMAL
    if (m_vkCurrLayout != VK_IMAGE_LAYOUT_GENERAL && m_vkCurrLayout != VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
        VulkanUtils::TransitionLayout(cmdBuffer, m_vkImage, m_vkCurrLayout, VK_IMAGE_LAYOUT_GENERAL, GetAspectMask());
        m_vkCurrLayout = VK_IMAGE_LAYOUT_GENERAL;
    }

    VkClearDepthStencilValue clearValue = {
        .depth = depth,
        .stencil = stencil
    };

    const VkImageSubresourceRange range = {
        .aspectMask = GetAspectMask(),
        .baseMipLevel = 0,
        .levelCount = VK_REMAINING_MIP_LEVELS,
        .baseArrayLayer = 0,
        .layerCount = VK_REMAINING_ARRAY_LAYERS
    };

    dispatch->CmdClearDepthStencilImage(cmdBuffer, m_vkImage, m_vkCurrLayout, &clearValue, 1, &range);
}

void BaseVulkanTexture::vkCopyFromImage(VkCommandBuffer cmdBuffer, VkImage srcImage, uint32_t srcLayer) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    VkImageAspectFlags aspectMask = GetAspectMask();
    const VkImageCopy region = {
        .srcSubresource = { aspectMask, 0, srcLayer, 1 },
        .srcOffset = { 0, 0, 0 },
        .dstSubresource = { aspectMask, 0, 0, 1 },
        .dstOffset = { 0, 0, 0 },
        .extent = {
            .width = m_width,
            .height = m_height,
            .depth = 1
        }
    };

    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
    dispatch->CmdCopyImage(cmdBuffer, srcImage, VK_IMAGE_LAYOUT_GENERAL, m_vkImage, VK_IMAGE_LAYOUT_GENERAL, 1, &region);
    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
}

void BaseVulkanTexture::vkUpload(VkCommandBuffer cmdBuffer, const void* data, size_t size) {
    vkReleaseStaging();

    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    VkDevice device = VRManager::instance().VK->GetDevice();

    VkBufferCreateInfo bufferInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.size = size;
    bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    bufferInfo.queueFamilyIndexCount = 0;
    bufferInfo.pQueueFamilyIndices = nullptr;

    checkVkResult(dispatch->CreateBuffer(device, &bufferInfo, nullptr, &m_stagingBuffer), "Failed to create staging buffer!");

    VkMemoryRequirements memRequirements;
    dispatch->GetBufferMemoryRequirements(device, m_stagingBuffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = VRManager::instance().VK->FindMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    checkVkResult(dispatch->AllocateMemory(device, &allocInfo, nullptr, &m_stagingMemory), "Failed to allocate staging buffer memory!");

    checkVkResult(dispatch->BindBufferMemory(device, m_stagingBuffer, m_stagingMemory, 0), "Failed to bind staging buffer memory!");

    void* mappedData;
    checkVkResult(dispatch->MapMemory(device, m_stagingMemory, 0, size, 0, &mappedData), "Failed to map staging buffer memory!");
    memcpy(mappedData, data, size);
    dispatch->UnmapMemory(device, m_stagingMemory);

    VkBufferImageCopy region = {};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = GetAspectMask();
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = { .x = 0, .y = 0, .z = 0 };
    region.imageExtent = {
        .width = m_width,
        .height = m_height,
        .depth = 1
    };

    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
    dispatch->CmdCopyBufferToImage(cmdBuffer, m_stagingBuffer, m_vkImage, VK_IMAGE_LAYOUT_GENERAL, 1, &region);
    VulkanUtils::DebugPipelineBarrier(cmdBuffer);
}

void BaseVulkanTexture::vkReleaseStaging() {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    VkDevice device = VRManager::instance().VK->GetDevice();
    if (m_stagingBuffer != VK_NULL_HANDLE) {
        dispatch->DestroyBuffer(device, m_stagingBuffer, nullptr);
        m_stagingBuffer = VK_NULL_HANDLE;
    }
    if (m_stagingMemory != VK_NULL_HANDLE) {
        dispatch->FreeMemory(device, m_stagingMemory, nullptr);
        m_stagingMemory = VK_NULL_HANDLE;
    }
}

VulkanTexture::VulkanTexture(uint32_t width, uint32_t height, VkFormat format, VkImageUsageFlags usage, bool disableAlphaThroughSwizzling): BaseVulkanTexture(width, height, format) {
    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    VkImageCreateInfo imageCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageCreateInfo.flags = 0;
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = format;
    imageCreateInfo.extent = {
        .width = m_width,
        .height = m_height,
        .depth = 1
    };
    imageCreateInfo.mipLevels = 1;
    imageCreateInfo.arrayLayers = 1;
    imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.usage = usage;
    imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageCreateInfo.queueFamilyIndexCount = 0;
    imageCreateInfo.pQueueFamilyIndices = nullptr;
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    checkVkResult(dispatch->CreateImage(VRManager::instance().VK->GetDevice(), &imageCreateInfo, nullptr, &m_vkImage), "Failed to create image!");

    VkMemoryRequirements memRequirements;
    dispatch->GetImageMemoryRequirements(VRManager::instance().VK->GetDevice(), m_vkImage, &memRequirements);

    uint32_t memoryTypeIndex = VRManager::instance().VK->FindMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    VkMemoryAllocateInfo allocInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = memoryTypeIndex;
    checkVkResult(dispatch->AllocateMemory(VRManager::instance().VK->GetDevice(), &allocInfo, nullptr, &m_vkMemory), "Failed to allocate memory!");

    checkVkResult(dispatch->BindImageMemory(VRManager::instance().VK->GetDevice(), m_vkImage, m_vkMemory, 0), "Failed to bind memory to image!");

    VkImageViewCreateInfo imageViewCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    imageViewCreateInfo.image = m_vkImage;
    imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    imageViewCreateInfo.format = format;
    imageViewCreateInfo.components = {
        .r = VK_COMPONENT_SWIZZLE_IDENTITY,
        .g = VK_COMPONENT_SWIZZLE_IDENTITY,
        .b = VK_COMPONENT_SWIZZLE_IDENTITY,
        .a = disableAlphaThroughSwizzling ? VK_COMPONENT_SWIZZLE_ONE : VK_COMPONENT_SWIZZLE_IDENTITY
    };
    imageViewCreateInfo.subresourceRange = {
        .aspectMask = VulkanUtils::GetAspectMaskForFormat(format),
        .baseMipLevel = 0,
        .levelCount = 1,
        .baseArrayLayer = 0,
        .layerCount = 1
    };
    checkVkResult(dispatch->CreateImageView(VRManager::instance().VK->GetDevice(), &imageViewCreateInfo, nullptr, &m_vkImageView), "Failed to create image view!");
}

VulkanTexture::~VulkanTexture() {
    if (m_vkImageView != VK_NULL_HANDLE) {
        VRManager::instance().VK->GetDeviceDispatch()->DestroyImageView(VRManager::instance().VK->GetDevice(), m_vkImageView, nullptr);
        m_vkImageView = VK_NULL_HANDLE;
    }
}

VulkanFramebuffer::VulkanFramebuffer(uint32_t width, uint32_t height, VkFormat format, VkRenderPass renderPass): VulkanTexture(width, height, format, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT) {
    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    VkFramebufferCreateInfo framebufferInfo = { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
    framebufferInfo.renderPass = renderPass;
    framebufferInfo.attachmentCount = 1;
    framebufferInfo.pAttachments = &m_vkImageView;
    framebufferInfo.width = width;
    framebufferInfo.height = height;
    framebufferInfo.layers = 1;
    checkVkResult(dispatch->CreateFramebuffer(dispatch->Device, &framebufferInfo, nullptr, &m_framebuffer), "Failed to create framebuffer!");
}

VulkanFramebuffer::~VulkanFramebuffer() {
    if (m_framebuffer != VK_NULL_HANDLE)
        VRManager::instance().VK->GetDeviceDispatch()->DestroyFramebuffer(VRManager::instance().VK->GetDevice(), m_framebuffer, nullptr);
}

FrameSync::FrameSync() {
    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    VkSemaphoreTypeCreateInfo timelineCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO };
    timelineCreateInfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
    timelineCreateInfo.initialValue = 0;

    VkSemaphoreCreateInfo semaphoreCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    semaphoreCreateInfo.pNext = &timelineCreateInfo;
    checkVkResult(dispatch->CreateSemaphore(VRManager::instance().VK->GetDevice(), &semaphoreCreateInfo, nullptr, &m_vkSemaphore), "Failed to create timeline semaphore for frame sync!");
}

FrameSync::~FrameSync() {
    if (m_vkSemaphore != VK_NULL_HANDLE) {
        VRManager::instance().VK->GetDeviceDispatch()->DestroySemaphore(VRManager::instance().VK->GetDevice(), m_vkSemaphore, nullptr);
        m_vkSemaphore = VK_NULL_HANDLE;
    }
}
