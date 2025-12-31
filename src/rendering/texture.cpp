#include "texture.h"
#include "../utils/d3d12_utils.h"
#include "instance.h"
#include "utils/vulkan_utils.h"


BaseVulkanTexture::~BaseVulkanTexture() {
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
    VulkanUtils::TransitionLayout(cmdBuffer, m_vkImage, m_vkCurrLayout, newLayout, GetAspectMask());
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

    dispatch->CmdCopyImage(cmdBuffer, m_vkImage, m_vkCurrLayout, dstImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    // AMD GPU FIX: Add memory barrier after copy to ensure data is visible
    VkMemoryBarrier2 postCopyBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
    postCopyBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    postCopyBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    postCopyBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    postCopyBarrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;

    VkDependencyInfo depInfo = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    depInfo.memoryBarrierCount = 1;
    depInfo.pMemoryBarriers = &postCopyBarrier;
    dispatch->CmdPipelineBarrier2(cmdBuffer, &depInfo);
}

void BaseVulkanTexture::vkClear(VkCommandBuffer cmdBuffer, VkClearColorValue color) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    // Only use CmdClearColorImage for color images
    if (VulkanUtils::IsDepthFormat(m_vkFormat)) {
        Log::print<WARNING>("vkClear called on depth image - use vkClearDepth instead");
        return;
    }

    // AMD GPU FIX: Transition to GENERAL if not already in a valid clear layout
    // CmdClearColorImage requires GENERAL or TRANSFER_DST_OPTIMAL
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

    dispatch->CmdClearColorImage(cmdBuffer, m_vkImage, m_vkCurrLayout, &color, 1, &range);
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

void BaseVulkanTexture::vkCopyFromImage(VkCommandBuffer cmdBuffer, VkImage srcImage, VkImageLayout srcLayout) {
    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    VkImageAspectFlags aspectMask = GetAspectMask();

    // AMD GPU FIX: If srcLayout is already TRANSFER_SRC_OPTIMAL, the caller has managed the transition
    // (e.g., via ensureSrcLayout in framebuffer.cpp). Skip internal transitions to avoid conflicts.
    bool callerManagedLayout = (srcLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

    if (!callerManagedLayout) {
        // Pre-copy barrier: transition source to TRANSFER_SRC_OPTIMAL
        VkImageMemoryBarrier2 srcPreBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        srcPreBarrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        srcPreBarrier.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        srcPreBarrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        srcPreBarrier.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        srcPreBarrier.oldLayout = srcLayout;
        srcPreBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        srcPreBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPreBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPreBarrier.image = srcImage;
        // AMD GPU FIX: Use VK_REMAINING to cover all subresources
        srcPreBarrier.subresourceRange = {
            .aspectMask = aspectMask,
            .baseMipLevel = 0,
            .levelCount = VK_REMAINING_MIP_LEVELS,
            .baseArrayLayer = 0,
            .layerCount = VK_REMAINING_ARRAY_LAYERS
        };

        VkDependencyInfo preBarrierDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        preBarrierDep.imageMemoryBarrierCount = 1;
        preBarrierDep.pImageMemoryBarriers = &srcPreBarrier;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &preBarrierDep);
    }

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

    dispatch->CmdCopyImage(cmdBuffer, srcImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, m_vkImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    if (!callerManagedLayout) {
        // Post-copy barrier: transition source back to original layout
        VkImageMemoryBarrier2 srcPostBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        srcPostBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        srcPostBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        srcPostBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        srcPostBarrier.dstAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        srcPostBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        srcPostBarrier.newLayout = srcLayout;
        srcPostBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPostBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPostBarrier.image = srcImage;
        // AMD GPU FIX: Use VK_REMAINING to cover all subresources
        srcPostBarrier.subresourceRange = {
            .aspectMask = aspectMask,
            .baseMipLevel = 0,
            .levelCount = VK_REMAINING_MIP_LEVELS,
            .baseArrayLayer = 0,
            .layerCount = VK_REMAINING_ARRAY_LAYERS
        };

        VkDependencyInfo postBarrierDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        postBarrierDep.imageMemoryBarrierCount = 1;
        postBarrierDep.pImageMemoryBarriers = &srcPostBarrier;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &postBarrierDep);
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
        .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
        .baseMipLevel = 0,
        .levelCount = 1,
        .baseArrayLayer = 0,
        .layerCount = 1
    };
    checkVkResult(dispatch->CreateImageView(VRManager::instance().VK->GetDevice(), &imageViewCreateInfo, nullptr, &m_vkImageView), "Failed to create image view!");
}

VulkanTexture::~VulkanTexture() {
    // AMD GPU FIX: Only destroy the image view here.
    // DO NOT destroy m_vkImage/m_vkMemory - BaseVulkanTexture::~BaseVulkanTexture() handles them.
    // Double-destroy was causing undefined behavior and AMD-specific crashes.
    if (m_vkImageView != VK_NULL_HANDLE) {
        VRManager::instance().VK->GetDeviceDispatch()->DestroyImageView(VRManager::instance().VK->GetDevice(), m_vkImageView, nullptr);
        m_vkImageView = VK_NULL_HANDLE;
    }
}

VulkanFramebuffer::VulkanFramebuffer(uint32_t width, uint32_t height, VkFormat format, VkRenderPass renderPass): VulkanTexture(width, height, format, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT) {
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

Texture::Texture(uint32_t width, uint32_t height, DXGI_FORMAT format): m_d3d12Format(format) {
    // Use typeless format for depth resources to allow both DSV and SRV creation
    // This is required for AMD compatibility
    DXGI_FORMAT resourceFormat = D3D12Utils::IsDepthFormat(format) ? D3D12Utils::ToTypelessDepthFormat(format) : format;

    // AMD GPU FIX: Shared resources require:
    // 1. 64KB alignment (D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT) for AMD interop
    // 2. ALLOW_SIMULTANEOUS_ACCESS for non-depth to disable DCC compression so Vulkan can read
    D3D12_RESOURCE_FLAGS flags = D3D12Utils::IsDepthFormat(format)
        ? D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL
        : (D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET | D3D12_RESOURCE_FLAG_ALLOW_SIMULTANEOUS_ACCESS);

    // clang-format off
    D3D12_RESOURCE_DESC textureDesc = {
        .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
        .Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,  // AMD GPU FIX: Force 64KB alignment
        .Width = width,
        .Height = height,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = resourceFormat,
        .SampleDesc = {
            .Count = 1,
            .Quality = 0
        },
        .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
        .Flags = flags
    };
    // clang-format on

    D3D12_HEAP_PROPERTIES heapProp = {
        .Type = D3D12_HEAP_TYPE_DEFAULT,
        .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
        .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN,
        .CreationNodeMask = 1,
        .VisibleNodeMask = 1
    };

    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateCommittedResource(&heapProp, D3D12_HEAP_FLAG_SHARED, &textureDesc, D3D12_RESOURCE_STATE_COMMON, nullptr, IID_PPV_ARGS(&m_d3d12Texture)), "Failed to create texture!");
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateSharedHandle(m_d3d12Texture.Get(), nullptr, GENERIC_ALL, nullptr, &m_d3d12TextureHandle), "Failed to create shared handle to texture!");

    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateFence(0, D3D12_FENCE_FLAG_SHARED, IID_PPV_ARGS(&m_d3d12Fence)), "Failed to create fence for texture!");
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateSharedHandle(m_d3d12Fence.Get(), nullptr, GENERIC_ALL, nullptr, &m_d3d12FenceHandle), "Failed to create shared handle to fence!");
}

void Texture::d3d12SignalFence(uint64_t value) {
    // Check current fence value before signaling
    uint64_t currentValue = m_d3d12Fence->GetCompletedValue();
    static uint32_t s_d3d12SignalCount = 0;
    s_d3d12SignalCount++;
    if (s_d3d12SignalCount % 500 == 0) {
        Log::print<INFO>("D3D12 Signal #{}: texture={}, current={}, signaling to {}",
            s_d3d12SignalCount, (void*)this, currentValue, value);
    }
    SetLastSignalledValue(value);
    HRESULT hr = VRManager::instance().D3D12->GetCommandQueue()->Signal(m_d3d12Fence.Get(), value);
    if (FAILED(hr)) {
        Log::print<ERROR>("D3D12 Signal FAILED! hr=0x{:X}, texture={}, value={}", (uint32_t)hr, (void*)this, value);
    }
}

void Texture::d3d12WaitForFence(uint64_t value) {
    uint64_t currentValue = m_d3d12Fence->GetCompletedValue();
    static uint32_t s_d3d12WaitCount = 0;
    s_d3d12WaitCount++;
    if (s_d3d12WaitCount % 500 == 0) {
        Log::print<INFO>("D3D12 Wait #{}: texture={}, current={}, waiting for {}",
            s_d3d12WaitCount, (void*)this, currentValue, value);
    }
    if (currentValue == UINT64_MAX) {
        Log::print<ERROR>("D3D12 fence in ERROR state! texture={}", (void*)this);
    }
    SetLastAwaitedValue(value);
    HRESULT hr = VRManager::instance().D3D12->GetCommandQueue()->Wait(m_d3d12Fence.Get(), value);
    if (FAILED(hr)) {
        Log::print<ERROR>("D3D12 Wait FAILED! hr=0x{:X}, texture={}, value={}", (uint32_t)hr, (void*)this, value);
    }
}

Texture::~Texture() {
    if (m_d3d12TextureHandle != nullptr)
        CloseHandle(m_d3d12TextureHandle);
    if (m_d3d12FenceHandle != nullptr)
        CloseHandle(m_d3d12FenceHandle);
}

void Texture::d3d12TransitionLayout(ID3D12GraphicsCommandList* cmdList, D3D12_RESOURCE_STATES state) {
    // AMD GPU FIX: Skip redundant transitions - transitioning to same state is invalid
    if (m_currState == state) {
        return;
    }

    // clang-format off
    D3D12_RESOURCE_BARRIER barrier = {
        .Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION,
        .Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
        .Transition = {
            .pResource = m_d3d12Texture.Get(),
            .Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES,
            .StateBefore = m_currState,
            .StateAfter = state
        }
    };
    // clang-format on
    m_currState = state;
    cmdList->ResourceBarrier(1, &barrier);
}

SharedTexture::SharedTexture(uint32_t width, uint32_t height, VkFormat vkFormat, DXGI_FORMAT d3d12Format): Texture(width, height, d3d12Format), BaseVulkanTexture(width, height, vkFormat) {
    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();

    // create image
    VkExternalMemoryImageCreateInfo externalMemoryImageCreateInfo = { VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO };
    externalMemoryImageCreateInfo.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;

    VkImageCreateInfo imageCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageCreateInfo.pNext = &externalMemoryImageCreateInfo;
    imageCreateInfo.flags = 0;
    imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imageCreateInfo.format = vkFormat;
    imageCreateInfo.extent = {
        .width = m_width,
        .height = m_height,
        .depth = 1
    };
    imageCreateInfo.mipLevels = 1;
    imageCreateInfo.arrayLayers = 1;
    imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.usage = (D3D12Utils::IsDepthFormat(d3d12Format) ? VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageCreateInfo.queueFamilyIndexCount = 0;
    imageCreateInfo.pQueueFamilyIndices = nullptr;
    imageCreateInfo.initialLayout = m_vkCurrLayout;
    checkVkResult(dispatch->CreateImage(VRManager::instance().VK->GetDevice(), &imageCreateInfo, nullptr, &m_vkImage), "Failed to create image for shared texture!");

    // get memory requirements
    VkImageMemoryRequirementsInfo2 requirementInfo = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2 };
    requirementInfo.image = m_vkImage;
    VkMemoryRequirements2 requirements = { VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2 };
    dispatch->GetImageMemoryRequirements2(VRManager::instance().VK->GetDevice(), &requirementInfo, &requirements);

    VkMemoryWin32HandlePropertiesKHR win32HandleProperties = { VK_STRUCTURE_TYPE_MEMORY_WIN32_HANDLE_PROPERTIES_KHR };
    checkVkResult(dispatch->GetMemoryWin32HandlePropertiesKHR(VRManager::instance().VK->GetDevice(), VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT, m_d3d12TextureHandle, &win32HandleProperties), "Failed to get properties of win32 texture handle!");

    // import memory from handle
    VkImportMemoryWin32HandleInfoKHR win32ImportMemoryHandleInfo = { VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR };
    win32ImportMemoryHandleInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT;
    win32ImportMemoryHandleInfo.handle = m_d3d12TextureHandle;

    VkMemoryDedicatedAllocateInfo dedicatedMemoryAllocateInfo = { VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO };
    dedicatedMemoryAllocateInfo.pNext = &win32ImportMemoryHandleInfo;
    dedicatedMemoryAllocateInfo.image = m_vkImage;
    dedicatedMemoryAllocateInfo.buffer = VK_NULL_HANDLE;

    VkMemoryAllocateInfo allocateInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocateInfo.pNext = &dedicatedMemoryAllocateInfo;
    allocateInfo.allocationSize = requirements.memoryRequirements.size;
    allocateInfo.memoryTypeIndex = VRManager::instance().VK->FindMemoryType(win32HandleProperties.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    checkVkResult(dispatch->AllocateMemory(VRManager::instance().VK->GetDevice(), &allocateInfo, nullptr, &m_vkMemory), "Failed to allocate memory for shared texture!");

    // bind memory to VkImage
    VkBindImageMemoryInfo bindImageInfo = { VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_INFO };
    bindImageInfo.image = m_vkImage;
    bindImageInfo.memory = m_vkMemory;
    checkVkResult(dispatch->BindImageMemory2(VRManager::instance().VK->GetDevice(), 1, &bindImageInfo), "Failed to bind memory to image for shared texture!");

    // create imported semaphore
    VkSemaphoreTypeCreateInfo timelineCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO };
    timelineCreateInfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
    timelineCreateInfo.initialValue = 0;

    VkSemaphoreCreateInfo semaphoreCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    semaphoreCreateInfo.pNext = &timelineCreateInfo;
    checkVkResult(dispatch->CreateSemaphore(VRManager::instance().VK->GetDevice(), &semaphoreCreateInfo, nullptr, &m_vkSemaphore), "Failed to create timeline semaphore for shared texture!");

    VkImportSemaphoreWin32HandleInfoKHR importSemaphoreInfo = { VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_WIN32_HANDLE_INFO_KHR };
    importSemaphoreInfo.semaphore = m_vkSemaphore;
    importSemaphoreInfo.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT;
    importSemaphoreInfo.handle = m_d3d12FenceHandle;
    checkVkResult(dispatch->ImportSemaphoreWin32HandleKHR(VRManager::instance().VK->GetDevice(), &importSemaphoreInfo), "Failed to import semaphore for shared texture!");
}

SharedTexture::~SharedTexture() {
    if (m_vkSemaphore != VK_NULL_HANDLE)
        VRManager::instance().VK->GetDeviceDispatch()->DestroySemaphore(VRManager::instance().VK->GetDevice(), m_vkSemaphore, nullptr);
}

void SharedTexture::CopyFromVkImage(VkCommandBuffer cmdBuffer, VkImage srcImage, VkImageLayout srcImageLayout) {
    static uint32_t s_copyCount = 0;
    s_copyCount++;

    auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    // AMD GPU FIX: Use GetAspectMask() from Vulkan format, NOT D3D12 format detection.
    // D3D12 depth resources are created as typeless (e.g., DXGI_FORMAT_R32_TYPELESS),
    // which would incorrectly return false for IsDepthFormat().
    VkImageAspectFlags aspectMask = GetAspectMask();

    // Validate state before copy
    if (srcImage == VK_NULL_HANDLE) {
        Log::print<ERROR>("CopyFromVkImage #{}: srcImage is NULL!", s_copyCount);
        return;
    }
    if (this->m_vkImage == VK_NULL_HANDLE) {
        Log::print<ERROR>("CopyFromVkImage #{}: destination m_vkImage is NULL!", s_copyCount);
        return;
    }

    // AMD GPU FIX: If srcImageLayout is already TRANSFER_SRC_OPTIMAL, the caller has managed the transition
    // (e.g., via ensureSrcLayout in framebuffer.cpp). Skip source transitions to avoid layout conflicts.
    bool callerManagedLayout = (srcImageLayout == VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

    // Log every 500 copies for debugging
    if (s_copyCount % 500 == 0) {
        auto desc = this->m_d3d12Texture->GetDesc();
        Log::print<INFO>("CopyFromVkImage #{}: src={}, dst={}, size={}x{}, srcLayout={}, dstLayout={}, callerManaged={}",
            s_copyCount, (void*)srcImage, (void*)this->m_vkImage,
            desc.Width, desc.Height, (int)srcImageLayout, (int)m_vkCurrLayout, callerManagedLayout);
    }

    // Pre-copy barrier: destination always needs transition, source only if not caller-managed
    VkImageMemoryBarrier2 dstBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    dstBarrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    dstBarrier.srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    dstBarrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    dstBarrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    dstBarrier.oldLayout = m_vkCurrLayout;
    dstBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    dstBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstBarrier.image = this->m_vkImage;
    // AMD GPU FIX: Use VK_REMAINING to cover all subresources
    dstBarrier.subresourceRange = {
        .aspectMask = aspectMask,
        .baseMipLevel = 0,
        .levelCount = VK_REMAINING_MIP_LEVELS,
        .baseArrayLayer = 0,
        .layerCount = VK_REMAINING_ARRAY_LAYERS
    };

    if (callerManagedLayout) {
        // Only transition destination
        VkDependencyInfo preCopyDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        preCopyDep.imageMemoryBarrierCount = 1;
        preCopyDep.pImageMemoryBarriers = &dstBarrier;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &preCopyDep);
    } else {
        // Transition both source and destination
        VkImageMemoryBarrier2 srcBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        srcBarrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        srcBarrier.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        srcBarrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        srcBarrier.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        srcBarrier.oldLayout = srcImageLayout;
        srcBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        srcBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcBarrier.image = srcImage;
        // AMD GPU FIX: Use VK_REMAINING to cover all subresources
        srcBarrier.subresourceRange = {
            .aspectMask = aspectMask,
            .baseMipLevel = 0,
            .levelCount = VK_REMAINING_MIP_LEVELS,
            .baseArrayLayer = 0,
            .layerCount = VK_REMAINING_ARRAY_LAYERS
        };

        VkImageMemoryBarrier2 barriers[2] = { srcBarrier, dstBarrier };
        VkDependencyInfo preCopyDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        preCopyDep.imageMemoryBarrierCount = 2;
        preCopyDep.pImageMemoryBarriers = barriers;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &preCopyDep);
    }

    m_vkCurrLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;

    VkImageCopy copyRegion = {
        .srcSubresource = { aspectMask, 0, 0, 1 },
        .srcOffset = { 0, 0, 0 },
        .dstSubresource = { aspectMask, 0, 0, 1 },
        .dstOffset = { 0, 0, 0 },
        .extent = { (uint32_t)this->m_d3d12Texture->GetDesc().Width, (uint32_t)this->m_d3d12Texture->GetDesc().Height, 1 }
    };

    // Copy using the correct layouts
    dispatch->CmdCopyImage(cmdBuffer, srcImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, this->m_vkImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

    // Post-copy barrier: destination always transitions to GENERAL, source only if not caller-managed
    VkImageMemoryBarrier2 dstPostBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    dstPostBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    dstPostBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    dstPostBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    dstPostBarrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    dstPostBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    dstPostBarrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    dstPostBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstPostBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    dstPostBarrier.image = this->m_vkImage;
    // AMD GPU FIX: Use VK_REMAINING to cover all subresources
    dstPostBarrier.subresourceRange = {
        .aspectMask = aspectMask,
        .baseMipLevel = 0,
        .levelCount = VK_REMAINING_MIP_LEVELS,
        .baseArrayLayer = 0,
        .layerCount = VK_REMAINING_ARRAY_LAYERS
    };

    if (callerManagedLayout) {
        // Only transition destination - source stays in TRANSFER_SRC_OPTIMAL for caller to restore
        VkDependencyInfo postCopyDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        postCopyDep.imageMemoryBarrierCount = 1;
        postCopyDep.pImageMemoryBarriers = &dstPostBarrier;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &postCopyDep);
    } else {
        // Transition both source and destination
        VkImageMemoryBarrier2 srcPostBarrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
        srcPostBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
        srcPostBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
        srcPostBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        srcPostBarrier.dstAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
        srcPostBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        srcPostBarrier.newLayout = srcImageLayout;  // Restore to ORIGINAL layout for Cemu
        srcPostBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPostBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        srcPostBarrier.image = srcImage;
        // AMD GPU FIX: Use VK_REMAINING to cover all subresources
        srcPostBarrier.subresourceRange = {
            .aspectMask = aspectMask,
            .baseMipLevel = 0,
            .levelCount = VK_REMAINING_MIP_LEVELS,
            .baseArrayLayer = 0,
            .layerCount = VK_REMAINING_ARRAY_LAYERS
        };

        VkImageMemoryBarrier2 postBarriers[2] = { srcPostBarrier, dstPostBarrier };
        VkDependencyInfo postCopyDep = { VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
        postCopyDep.imageMemoryBarrierCount = 2;
        postCopyDep.pImageMemoryBarriers = postBarriers;
        dispatch->CmdPipelineBarrier2(cmdBuffer, &postCopyDep);
    }

    m_vkCurrLayout = VK_IMAGE_LAYOUT_GENERAL;
}