#include "texture.h"
#include "instance.h"
#include "../utils/d3d12_utils.h"

Texture::Texture(uint32_t width, uint32_t height, DXGI_FORMAT format): m_d3d12Format(format) {
    // clang-format off
    D3D12_RESOURCE_DESC textureDesc = {
        .Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D,
        .Alignment = 0,
        .Width = width,
        .Height = height,
        .DepthOrArraySize = 1,
        .MipLevels = 1,
        .Format = format,
        .SampleDesc = {
            .Count = 1,
            .Quality = 0
        },
        .Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN,
        .Flags = D3D12Utils::IsDepthFormat(format) ? D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL : D3D12_RESOURCE_FLAG_NONE
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
    VRManager::instance().D3D12->GetCommandQueue()->Signal(m_d3d12Fence.Get(), value);
}

void Texture::d3d12WaitForFence(uint64_t value) {
    VRManager::instance().D3D12->GetCommandQueue()->Wait(m_d3d12Fence.Get(), value);
}

Texture::~Texture() {
    if (m_d3d12TextureHandle != nullptr)
        CloseHandle(m_d3d12TextureHandle);
    if (m_d3d12FenceHandle != nullptr)
        CloseHandle(m_d3d12FenceHandle);
}

void Texture::d3d12TransitionLayout(ID3D12GraphicsCommandList* cmdList, D3D12_RESOURCE_STATES state) {
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

bool Texture::CopyFromSharedTexture(ID3D12GraphicsCommandList* cmdList, SharedTexture* texture) {
    return false;
}

SharedTexture::SharedTexture(uint32_t width, uint32_t height, VkFormat vkFormat, DXGI_FORMAT d3d12Format): Texture(width, height, d3d12Format) {
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
        .width = width,
        .height = height,
        .depth = 1
    };
    imageCreateInfo.mipLevels = 1;
    imageCreateInfo.arrayLayers = 1;
    imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageCreateInfo.usage = (D3D12Utils::IsDepthFormat(d3d12Format) ? VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT) | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageCreateInfo.queueFamilyIndexCount = 0;
    imageCreateInfo.pQueueFamilyIndices = nullptr;
    imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
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
    if (m_vkImage != VK_NULL_HANDLE)
        VRManager::instance().VK->GetDeviceDispatch()->DestroyImage(VRManager::instance().VK->GetDevice(), m_vkImage, nullptr);
    if (m_vkMemory != VK_NULL_HANDLE)
        VRManager::instance().VK->GetDeviceDispatch()->FreeMemory(VRManager::instance().VK->GetDevice(), m_vkMemory, nullptr);
}


void SharedTexture::vkTransitionLayout(VkCommandBuffer cmdBuffer, VkImageLayout layout) {
    VkImageMemoryBarrier barrier = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
    barrier.srcAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT;
    barrier.oldLayout = m_currLayout;
    barrier.newLayout = layout;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = m_vkImage;
    barrier.subresourceRange = {
        .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
        .baseMipLevel = 0,
        .levelCount = 1,
        .baseArrayLayer = 0,
        .layerCount = 1
    };
    VRManager::instance().VK->GetDeviceDispatch()->CmdPipelineBarrier(cmdBuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT, 0, 0, nullptr, 0, nullptr, 1, &barrier);
    m_currLayout = layout;
}

void SharedTexture::CopyFromVkImage(VkCommandBuffer cmdBuffer, VkImage srcImage) {
    VkMemoryBarrier2KHR memoryBarrier = {
        .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2_KHR,
        .srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT_KHR,
        .srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT_KHR | VK_ACCESS_2_MEMORY_WRITE_BIT_KHR,
        .dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT_KHR,
        .dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT_KHR | VK_ACCESS_2_MEMORY_WRITE_BIT_KHR,
    };

    VkDependencyInfoKHR dependencyInfo = {
        .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO_KHR,
        .pNext = nullptr,
        .dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT,
        .memoryBarrierCount = 1,
        .pMemoryBarriers = &memoryBarrier,
        .bufferMemoryBarrierCount = 0,
        .pBufferMemoryBarriers = nullptr,
        .imageMemoryBarrierCount = 0,
        .pImageMemoryBarriers = nullptr,
    };

    VRManager::instance().VK->GetDeviceDispatch()->CmdPipelineBarrier2KHR(cmdBuffer, &dependencyInfo);

    VkImageCopy copyRegion = {
        .srcSubresource = { (VkImageAspectFlags)(D3D12Utils::IsDepthFormat(this->d3d12GetTexture()->GetDesc().Format) ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT), 0, 0, 1 },
        .srcOffset = { 0, 0, 0 },
        .dstSubresource = { (VkImageAspectFlags)(D3D12Utils::IsDepthFormat(this->d3d12GetTexture()->GetDesc().Format) ? VK_IMAGE_ASPECT_DEPTH_BIT : VK_IMAGE_ASPECT_COLOR_BIT), 0, 0, 1 },
        .dstOffset = { 0, 0, 0 },
        .extent = { (uint32_t)this->m_d3d12Texture->GetDesc().Width, (uint32_t)this->m_d3d12Texture->GetDesc().Height, 1 }
    };

    VRManager::instance().VK->GetDeviceDispatch()->CmdCopyImage(cmdBuffer, srcImage, VK_IMAGE_LAYOUT_GENERAL, this->m_vkImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

    VRManager::instance().VK->GetDeviceDispatch()->CmdPipelineBarrier2KHR(cmdBuffer, &dependencyInfo);
}