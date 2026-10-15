#include "pch.h"

#include "vk_backend.h"
#include "instance.h"
#include "renderer.h"

#include "shaders_spirv.h"

RND_VkBackend::RND_VkBackend(VkInstance vkInstance, VkPhysicalDevice vkPhysicalDevice, VkDevice vkDevice, const ModQueueBinding& modQueue): m_instance(vkInstance), m_physicalDevice(vkPhysicalDevice), m_device(vkDevice), m_modQueue(modQueue) {
    m_dispatch = vkroots::tables::DeviceDispatches.find(vkDevice);
    checkAssert(m_dispatch != nullptr, "The Vulkan backend couldn't find the device dispatch table!");

    m_dispatch->GetDeviceQueue(m_device, m_modQueue.familyIndex, m_modQueue.queueIndex, &m_queue);
    checkAssert(m_queue != VK_NULL_HANDLE, "The Vulkan backend failed to retrieve its own queue!");
    Log::print<INFO>("Vulkan backend submitting on queue family {} index {}{}", m_modQueue.familyIndex, m_modQueue.queueIndex, m_modQueue.shared ? " (shared with Cemu)" : "");

    VkSemaphoreTypeCreateInfo timelineCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO };
    timelineCreateInfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
    timelineCreateInfo.initialValue = 0;

    VkSemaphoreCreateInfo semaphoreCreateInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    semaphoreCreateInfo.pNext = &timelineCreateInfo;
    checkVkResult(m_dispatch->CreateSemaphore(m_device, &semaphoreCreateInfo, nullptr, &m_timeline), "Failed to create the mod queue timeline semaphore!");

    auto createCommandPool = [this](VkCommandPool& commandPool) {
        VkCommandPoolCreateInfo commandPoolCreateInfo = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        commandPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        commandPoolCreateInfo.queueFamilyIndex = m_modQueue.familyIndex;
        checkVkResult(m_dispatch->CreateCommandPool(m_device, &commandPoolCreateInfo, nullptr, &commandPool), "Failed to create a mod command pool!");
    };

    auto allocateCommandBuffers = [this](VkCommandPool commandPool, VkCommandBuffer* commandBuffers, uint32_t count) {
        VkCommandBufferAllocateInfo allocateInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        allocateInfo.commandPool = commandPool;
        allocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocateInfo.commandBufferCount = count;
        checkVkResult(m_dispatch->AllocateCommandBuffers(m_device, &allocateInfo, commandBuffers), "Failed to allocate mod command buffers!");
    };

    for (FrameContext& frameContext : m_frameContexts) {
        createCommandPool(frameContext.commandPool);
        allocateCommandBuffers(frameContext.commandPool, frameContext.commandBuffers.data(), (uint32_t)frameContext.commandBuffers.size());
    }

    createCommandPool(m_immediateContext.commandPool);
    allocateCommandBuffers(m_immediateContext.commandPool, &m_immediateContext.commandBuffer, 1);

    VkDescriptorPoolSize poolSizes[] = {
        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 128 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 32 }
    };

    VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
        .flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT,
        .maxSets = 64,
        .poolSizeCount = (uint32_t)std::size(poolSizes),
        .pPoolSizes = poolSizes
    };
    checkVkResult(m_dispatch->CreateDescriptorPool(m_device, &descriptorPoolCreateInfo, nullptr, &m_descriptorPool), "Failed to create the mod descriptor pool!");

    VkSamplerCreateInfo samplerCreateInfo = { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerCreateInfo.magFilter = VK_FILTER_NEAREST;
    samplerCreateInfo.minFilter = VK_FILTER_NEAREST;
    samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerCreateInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCreateInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCreateInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerCreateInfo.mipLodBias = 0.0f;
    samplerCreateInfo.anisotropyEnable = VK_FALSE;
    samplerCreateInfo.maxAnisotropy = 1.0f;
    samplerCreateInfo.compareEnable = VK_FALSE;
    samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
    samplerCreateInfo.minLod = 0.0f;
    samplerCreateInfo.maxLod = VK_LOD_CLAMP_NONE;
    samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
    samplerCreateInfo.unnormalizedCoordinates = VK_FALSE;
    checkVkResult(m_dispatch->CreateSampler(m_device, &samplerCreateInfo, nullptr, &m_pointSampler), "Failed to create the mod point sampler!");
}

RND_VkBackend::~RND_VkBackend() {
    if (m_dispatch == nullptr || m_device == VK_NULL_HANDLE) {
        return;
    }

    if (m_queue != VK_NULL_HANDLE) {
        m_dispatch->QueueWaitIdle(m_queue);
    }

    FlushFramebufferCache();

    for (const CachedRenderPass& cachedRenderPass : m_renderPasses) {
        m_dispatch->DestroyRenderPass(m_device, cachedRenderPass.renderPass, nullptr);
    }
    m_renderPasses.clear();

    if (m_pointSampler != VK_NULL_HANDLE) {
        m_dispatch->DestroySampler(m_device, m_pointSampler, nullptr);
        m_pointSampler = VK_NULL_HANDLE;
    }
    if (m_descriptorPool != VK_NULL_HANDLE) {
        m_dispatch->DestroyDescriptorPool(m_device, m_descriptorPool, nullptr);
        m_descriptorPool = VK_NULL_HANDLE;
    }

    for (FrameContext& frameContext : m_frameContexts) {
        if (frameContext.commandPool != VK_NULL_HANDLE) {
            m_dispatch->DestroyCommandPool(m_device, frameContext.commandPool, nullptr);
            frameContext.commandPool = VK_NULL_HANDLE;
        }
    }
    if (m_immediateContext.commandPool != VK_NULL_HANDLE) {
        m_dispatch->DestroyCommandPool(m_device, m_immediateContext.commandPool, nullptr);
        m_immediateContext.commandPool = VK_NULL_HANDLE;
    }

    if (m_timeline != VK_NULL_HANDLE) {
        m_dispatch->DestroySemaphore(m_device, m_timeline, nullptr);
        m_timeline = VK_NULL_HANDLE;
    }
}

void RND_VkBackend::StartFrame() {
    m_currentFrameContextIndex = (m_currentFrameContextIndex + 1) % kFrameContextCount;

    FrameContext& frameContext = GetCurrentFrameContext();
    if (frameContext.completionTimelineValue != 0) {
        WaitForTimeline(frameContext.completionTimelineValue);
        frameContext.completionTimelineValue = 0;
    }

    checkVkResult(m_dispatch->ResetCommandPool(m_device, frameContext.commandPool, 0), "Failed to reset a frame command pool!");
    frameContext.nextCommandBufferIndex = 0;
}

void RND_VkBackend::EndFrame() {
    GetCurrentFrameContext().completionTimelineValue = m_lastSubmittedTimelineValue;
}

void RND_VkBackend::WaitForSubmittedWork() {
    WaitForTimeline(m_lastSubmittedTimelineValue);
}

RND_VkBackend::FrameContext& RND_VkBackend::GetCurrentFrameContext() {
    return m_frameContexts[m_currentFrameContextIndex];
}

VkCommandBuffer RND_VkBackend::AcquireFrameCommandBuffer() {
    FrameContext& frameContext = GetCurrentFrameContext();
    checkAssert(frameContext.nextCommandBufferIndex < frameContext.commandBuffers.size(), "Exceeded the reusable frame command buffer pool!");

    VkCommandBuffer commandBuffer = frameContext.commandBuffers[frameContext.nextCommandBufferIndex++];

    VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVkResult(m_dispatch->BeginCommandBuffer(commandBuffer, &beginInfo), "Failed to begin a reusable frame command buffer!");
    return commandBuffer;
}

VkCommandBuffer RND_VkBackend::AcquireImmediateCommandBuffer() {
    checkVkResult(m_dispatch->ResetCommandPool(m_device, m_immediateContext.commandPool, 0), "Failed to reset the immediate command pool!");

    VkCommandBufferBeginInfo beginInfo = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVkResult(m_dispatch->BeginCommandBuffer(m_immediateContext.commandBuffer, &beginInfo), "Failed to begin the immediate command buffer!");
    return m_immediateContext.commandBuffer;
}

uint64_t RND_VkBackend::SubmitCommandBuffer(VkCommandBuffer commandBuffer, const std::vector<std::pair<FrameSync*, uint64_t>>& waitFor, const std::vector<std::pair<FrameSync*, uint64_t>>& signalTo) {
    AssertRenderThread();

    checkVkResult(m_dispatch->EndCommandBuffer(commandBuffer), "Failed to end a mod command buffer!");

    std::vector<VkSemaphore> waitSemaphores;
    std::vector<uint64_t> waitValues;
    std::vector<VkPipelineStageFlags> waitStageMasks;
    waitSemaphores.reserve(waitFor.size());
    waitValues.reserve(waitFor.size());
    waitStageMasks.reserve(waitFor.size());
    for (const auto& [sync, value] : waitFor) {
        waitSemaphores.emplace_back(sync->GetSemaphoreForWait(value));
        waitValues.emplace_back(value);
        waitStageMasks.emplace_back(VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);
    }

    std::vector<VkSemaphore> signalSemaphores;
    std::vector<uint64_t> signalValues;
    signalSemaphores.reserve(signalTo.size() + 1);
    signalValues.reserve(signalTo.size() + 1);
    for (const auto& [sync, value] : signalTo) {
        signalSemaphores.emplace_back(sync->GetSemaphoreForSignal(value));
        signalValues.emplace_back(value);
    }

    const uint64_t completionTimelineValue = m_nextTimelineValue++;
    signalSemaphores.emplace_back(m_timeline);
    signalValues.emplace_back(completionTimelineValue);

    VkTimelineSemaphoreSubmitInfo timelineSubmitInfo = { VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO };
    timelineSubmitInfo.waitSemaphoreValueCount = (uint32_t)waitValues.size();
    timelineSubmitInfo.pWaitSemaphoreValues = waitValues.data();
    timelineSubmitInfo.signalSemaphoreValueCount = (uint32_t)signalValues.size();
    timelineSubmitInfo.pSignalSemaphoreValues = signalValues.data();

    VkSubmitInfo submitInfo = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.pNext = &timelineSubmitInfo;
    submitInfo.waitSemaphoreCount = (uint32_t)waitSemaphores.size();
    submitInfo.pWaitSemaphores = waitSemaphores.data();
    submitInfo.pWaitDstStageMask = waitStageMasks.data();
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;
    submitInfo.signalSemaphoreCount = (uint32_t)signalSemaphores.size();
    submitInfo.pSignalSemaphores = signalSemaphores.data();

    checkVkResult(m_dispatch->QueueSubmit(m_queue, 1, &submitInfo, VK_NULL_HANDLE), "Failed to submit a mod command buffer!");

    m_lastSubmittedTimelineValue = completionTimelineValue;
    return completionTimelineValue;
}

void RND_VkBackend::WaitForTimeline(uint64_t timelineValue) {
    if (timelineValue == 0) {
        return;
    }

    VkSemaphoreWaitInfo waitInfo = { VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO };
    waitInfo.semaphoreCount = 1;
    waitInfo.pSemaphores = &m_timeline;
    waitInfo.pValues = &timelineValue;
    checkVkResult(m_dispatch->WaitSemaphores(m_device, &waitInfo, UINT64_MAX), "Failed to wait for the mod queue timeline!");
}

void RND_VkBackend::AssertRenderThread() {
    const std::thread::id currentThreadId = std::this_thread::get_id();
    if (!m_submitThreadLatched) {
        m_submitThreadId = currentThreadId;
        m_submitThreadLatched = true;
        return;
    }
    if (currentThreadId == m_submitThreadId) {
        return;
    }

    checkAssert(!m_modQueue.shared, "The mod shares Cemu's queue but submitted from a second thread, which breaks the queue's external synchronization!");

    static bool s_warnedAboutSubmitThread = false;
    if (!s_warnedAboutSubmitThread) {
        s_warnedAboutSubmitThread = true;
        Log::print<WARNING>("The mod queue is being submitted to from more than one thread");
    }
}

VkRenderPass RND_VkBackend::AcquireRenderPass(VkFormat colorFormat, VkFormat depthFormat, bool preserveContents) {
    for (const CachedRenderPass& cachedRenderPass : m_renderPasses) {
        if (cachedRenderPass.colorFormat == colorFormat && cachedRenderPass.depthFormat == depthFormat && cachedRenderPass.preserveContents == preserveContents) {
            return cachedRenderPass.renderPass;
        }
    }

    const bool hasDepth = depthFormat != VK_FORMAT_UNDEFINED;
    const VkAttachmentLoadOp loadOp = preserveContents ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_DONT_CARE;

    // OpenXR hands out swapchain images in their attachment layout and wants them back that way, so they never get barriers
    std::array<VkAttachmentDescription, 2> attachments = {};
    attachments[0] = {
        .format = colorFormat,
        .samples = VK_SAMPLE_COUNT_1_BIT,
        .loadOp = loadOp,
        .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
        .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
        .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
        .initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        .finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
    };
    attachments[1] = {
        .format = depthFormat,
        .samples = VK_SAMPLE_COUNT_1_BIT,
        .loadOp = loadOp,
        .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
        .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
        .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
        .initialLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
        .finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
    };

    const VkAttachmentReference colorAttachmentRef = { .attachment = 0, .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    const VkAttachmentReference depthAttachmentRef = { .attachment = 1, .layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

    VkSubpassDescription subpass = {
        .pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS,
        .colorAttachmentCount = 1,
        .pColorAttachments = &colorAttachmentRef,
        .pDepthStencilAttachment = hasDepth ? &depthAttachmentRef : nullptr
    };

    std::array<VkSubpassDependency, 2> dependencies = {};
    dependencies[0] = {
        .srcSubpass = VK_SUBPASS_EXTERNAL,
        .dstSubpass = 0,
        .srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        .srcAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
        .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
        .dependencyFlags = 0
    };
    dependencies[1] = {
        .srcSubpass = 0,
        .dstSubpass = VK_SUBPASS_EXTERNAL,
        .srcStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        .dstStageMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
        .srcAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
        .dstAccessMask = VK_ACCESS_MEMORY_READ_BIT | VK_ACCESS_MEMORY_WRITE_BIT,
        .dependencyFlags = 0
    };

    VkRenderPassCreateInfo renderPassCreateInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    renderPassCreateInfo.attachmentCount = hasDepth ? 2 : 1;
    renderPassCreateInfo.pAttachments = attachments.data();
    renderPassCreateInfo.subpassCount = 1;
    renderPassCreateInfo.pSubpasses = &subpass;
    renderPassCreateInfo.dependencyCount = (uint32_t)dependencies.size();
    renderPassCreateInfo.pDependencies = dependencies.data();

    VkRenderPass renderPass = VK_NULL_HANDLE;
    checkVkResult(m_dispatch->CreateRenderPass(m_device, &renderPassCreateInfo, nullptr, &renderPass), "Failed to create a mod render pass!");

    m_renderPasses.emplace_back(CachedRenderPass{ colorFormat, depthFormat, preserveContents, renderPass });
    return renderPass;
}

VkFramebuffer RND_VkBackend::AcquireFramebuffer(VkRenderPass renderPass, VkImageView colorView, VkImageView depthView, VkExtent2D extent) {
    for (const CachedFramebuffer& cachedFramebuffer : m_framebuffers) {
        if (cachedFramebuffer.renderPass == renderPass && cachedFramebuffer.colorView == colorView && cachedFramebuffer.depthView == depthView && cachedFramebuffer.extent.width == extent.width && cachedFramebuffer.extent.height == extent.height) {
            return cachedFramebuffer.framebuffer;
        }
    }

    std::array<VkImageView, 2> views = { colorView, depthView };

    VkFramebufferCreateInfo framebufferCreateInfo = { VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
    framebufferCreateInfo.renderPass = renderPass;
    framebufferCreateInfo.attachmentCount = depthView != VK_NULL_HANDLE ? 2 : 1;
    framebufferCreateInfo.pAttachments = views.data();
    framebufferCreateInfo.width = extent.width;
    framebufferCreateInfo.height = extent.height;
    framebufferCreateInfo.layers = 1;

    VkFramebuffer framebuffer = VK_NULL_HANDLE;
    checkVkResult(m_dispatch->CreateFramebuffer(m_device, &framebufferCreateInfo, nullptr, &framebuffer), "Failed to create a mod framebuffer!");

    m_framebuffers.emplace_back(CachedFramebuffer{ renderPass, colorView, depthView, extent, framebuffer });
    return framebuffer;
}

void RND_VkBackend::FlushFramebufferCache() {
    if (m_framebuffers.empty()) {
        return;
    }

    if (m_queue != VK_NULL_HANDLE) {
        m_dispatch->QueueWaitIdle(m_queue);
    }
    for (const CachedFramebuffer& cachedFramebuffer : m_framebuffers) {
        m_dispatch->DestroyFramebuffer(m_device, cachedFramebuffer.framebuffer, nullptr);
    }
    m_framebuffers.clear();
}

VkShaderModule RND_VkBackend::CreateShaderModule(const uint32_t* spirvWords, size_t spirvByteSize) const {
    VkShaderModuleCreateInfo shaderModuleCreateInfo = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    shaderModuleCreateInfo.codeSize = spirvByteSize;
    shaderModuleCreateInfo.pCode = spirvWords;

    VkShaderModule shaderModule = VK_NULL_HANDLE;
    checkVkResult(m_dispatch->CreateShaderModule(m_device, &shaderModuleCreateInfo, nullptr, &shaderModule), "Failed to create a mod shader module!");
    return shaderModule;
}

RND_VkBackend::HostBuffer RND_VkBackend::CreateHostBuffer(uint32_t size, VkBufferUsageFlags usage) const {
    HostBuffer hostBuffer = {};
    hostBuffer.capacity = size;

    VkBufferCreateInfo bufferCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferCreateInfo.size = size;
    bufferCreateInfo.usage = usage;
    bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    checkVkResult(m_dispatch->CreateBuffer(m_device, &bufferCreateInfo, nullptr, &hostBuffer.buffer), "Failed to create a host visible buffer!");

    VkMemoryRequirements memoryRequirements;
    m_dispatch->GetBufferMemoryRequirements(m_device, hostBuffer.buffer, &memoryRequirements);

    VkMemoryAllocateInfo allocateInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocateInfo.allocationSize = memoryRequirements.size;
    allocateInfo.memoryTypeIndex = VRManager::instance().VK->FindMemoryType(memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    checkVkResult(m_dispatch->AllocateMemory(m_device, &allocateInfo, nullptr, &hostBuffer.memory), "Failed to allocate host visible buffer memory!");
    checkVkResult(m_dispatch->BindBufferMemory(m_device, hostBuffer.buffer, hostBuffer.memory, 0), "Failed to bind host visible buffer memory!");
    checkVkResult(m_dispatch->MapMemory(m_device, hostBuffer.memory, 0, VK_WHOLE_SIZE, 0, &hostBuffer.mapped), "Failed to map host visible buffer memory!");
    return hostBuffer;
}

void RND_VkBackend::DestroyHostBuffer(HostBuffer& hostBuffer) const {
    if (hostBuffer.mapped != nullptr) {
        m_dispatch->UnmapMemory(m_device, hostBuffer.memory);
        hostBuffer.mapped = nullptr;
    }
    if (hostBuffer.buffer != VK_NULL_HANDLE) {
        m_dispatch->DestroyBuffer(m_device, hostBuffer.buffer, nullptr);
        hostBuffer.buffer = VK_NULL_HANDLE;
    }
    if (hostBuffer.memory != VK_NULL_HANDLE) {
        m_dispatch->FreeMemory(m_device, hostBuffer.memory, nullptr);
        hostBuffer.memory = VK_NULL_HANDLE;
    }
    hostBuffer.capacity = 0;
}

template <bool depth>
RND_VkBackend::PresentPipeline<depth>::PresentPipeline(RND_VkBackend* backend, RND_Renderer* pRenderer): m_backend(backend), m_renderer(pRenderer) {
    checkAssert(m_backend != nullptr, "The present pipeline is missing its renderer backend!");
    const auto* dispatch = m_backend->GetDispatch();

    m_vertexShader = m_backend->CreateShaderModule(present_vert_spv, sizeof(present_vert_spv));
    if constexpr (depth) {
        m_fragmentShader = m_backend->CreateShaderModule(present_depth_frag_spv, sizeof(present_depth_frag_spv));
    }
    else {
        m_fragmentShader = m_backend->CreateShaderModule(present_frag_spv, sizeof(present_frag_spv));
    }

    const VkSampler pointSampler = m_backend->GetPointSampler();
    std::array<VkDescriptorSetLayoutBinding, kAttachmentCount> bindings = {};
    for (uint32_t i = 0; i < kAttachmentCount; i++) {
        bindings[i] = {
            .binding = i,
            .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            .descriptorCount = 1,
            .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
            .pImmutableSamplers = &pointSampler
        };
    }

    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    descriptorSetLayoutCreateInfo.bindingCount = (uint32_t)bindings.size();
    descriptorSetLayoutCreateInfo.pBindings = bindings.data();
    checkVkResult(dispatch->CreateDescriptorSetLayout(m_backend->GetDevice(), &descriptorSetLayoutCreateInfo, nullptr, &m_descriptorSetLayout), "Failed to create the present descriptor set layout!");

    const VkPushConstantRange pushConstantRange = {
        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
        .offset = 0,
        .size = sizeof(presentSettings)
    };

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutCreateInfo.setLayoutCount = 1;
    pipelineLayoutCreateInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;
    checkVkResult(dispatch->CreatePipelineLayout(m_backend->GetDevice(), &pipelineLayoutCreateInfo, nullptr, &m_pipelineLayout), "Failed to create the present pipeline layout!");

    std::array<VkDescriptorSetLayout, kFrameContextCount> setLayouts = {};
    setLayouts.fill(m_descriptorSetLayout);

    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    descriptorSetAllocateInfo.descriptorPool = m_backend->GetDescriptorPool();
    descriptorSetAllocateInfo.descriptorSetCount = (uint32_t)setLayouts.size();
    descriptorSetAllocateInfo.pSetLayouts = setLayouts.data();
    checkVkResult(dispatch->AllocateDescriptorSets(m_backend->GetDevice(), &descriptorSetAllocateInfo, m_descriptorSets.data()), "Failed to allocate the present descriptor sets!");
}

template <bool depth>
RND_VkBackend::PresentPipeline<depth>::~PresentPipeline() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    m_backend->WaitForSubmittedWork();

    if (m_pipeline != VK_NULL_HANDLE) {
        dispatch->DestroyPipeline(device, m_pipeline, nullptr);
    }
    if (m_pipelineLayout != VK_NULL_HANDLE) {
        dispatch->DestroyPipelineLayout(device, m_pipelineLayout, nullptr);
    }
    if (m_descriptorSetLayout != VK_NULL_HANDLE) {
        dispatch->DestroyDescriptorSetLayout(device, m_descriptorSetLayout, nullptr);
    }
    if (m_vertexShader != VK_NULL_HANDLE) {
        dispatch->DestroyShaderModule(device, m_vertexShader, nullptr);
    }
    if (m_fragmentShader != VK_NULL_HANDLE) {
        dispatch->DestroyShaderModule(device, m_fragmentShader, nullptr);
    }
}

template <bool depth>
void RND_VkBackend::PresentPipeline<depth>::BindAttachment(uint32_t attachmentIdx, VkImageView srcView) {
    checkAssert(attachmentIdx < kAttachmentCount, "Present pipeline attachment index is out of range!");
    m_attachmentViews[attachmentIdx] = srcView;
}

template <bool depth>
void RND_VkBackend::PresentPipeline<depth>::BindSettings(float screenWidth, float screenHeight, const RenderUtils::UvTransform& uvTransform) {
    m_renderWidth = screenWidth;
    m_renderHeight = screenHeight;
    m_uvTransform = uvTransform;
}

template <bool depth>
presentSettings RND_VkBackend::PresentPipeline<depth>::BuildSettings(const RenderTarget& target) const {
    presentSettings settings = {
        .renderWidth = m_renderWidth,
        .renderHeight = m_renderHeight,
        .swapchainWidth = (float)target.extent.width,
        .swapchainHeight = (float)target.extent.height,
        .uvOffsetX = m_uvTransform.offsetX,
        .uvOffsetY = m_uvTransform.offsetY,
        .uvScaleX = m_uvTransform.scaleX,
        .uvScaleY = m_uvTransform.scaleY,
        .customFadeAmount = 0.0f,
        .customFadeColorR = 0.0f,
        .customFadeColorG = 0.0f,
        .customFadeColorB = 0.0f,
        .isFadeActive = 0.0f,
        .srgbEncode = target.srgbEncodeInShader ? 1.0f : 0.0f,
    };

    if constexpr (depth) {
        const RND_Renderer::CustomFade fade = m_renderer != nullptr ? m_renderer->GetCustomFade() : RND_Renderer::CustomFade{};
        settings.customFadeAmount = fade.amount;
        settings.customFadeColorR = fade.color.x;
        settings.customFadeColorG = fade.color.y;
        settings.customFadeColorB = fade.color.z;
        settings.isFadeActive = m_renderer != nullptr && m_renderer->IsFadeActive() ? 1.0f : 0.0f;
    }

    return settings;
}

template <bool depth>
void RND_VkBackend::PresentPipeline<depth>::RecreatePipeline() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    if (m_pipeline != VK_NULL_HANDLE) {
        m_backend->WaitForSubmittedWork();
        dispatch->DestroyPipeline(device, m_pipeline, nullptr);
        m_pipeline = VK_NULL_HANDLE;
    }

    const std::array<VkPipelineShaderStageCreateInfo, 2> stages = {
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = m_vertexShader, .pName = "main" },
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = m_fragmentShader, .pName = "main" }
    };

    const VkPipelineVertexInputStateCreateInfo vertexInputState = { VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };

    const VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
    };

    const VkPipelineViewportStateCreateInfo viewportState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .viewportCount = 1,
        .scissorCount = 1
    };

    const VkPipelineRasterizationStateCreateInfo rasterizationState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .polygonMode = VK_POLYGON_MODE_FILL,
        .cullMode = VK_CULL_MODE_NONE,
        .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
        .lineWidth = 1.0f
    };

    const VkPipelineMultisampleStateCreateInfo multisampleState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
    };

    const VkPipelineDepthStencilStateCreateInfo depthStencilState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .depthTestEnable = depth ? VK_TRUE : VK_FALSE,
        .depthWriteEnable = depth ? VK_TRUE : VK_FALSE,
        .depthCompareOp = VK_COMPARE_OP_ALWAYS
    };

    const VkPipelineColorBlendAttachmentState blendAttachment = {
        .blendEnable = VK_FALSE,
        .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
    };

    const VkPipelineColorBlendStateCreateInfo colorBlendState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .attachmentCount = 1,
        .pAttachments = &blendAttachment
    };

    const std::array<VkDynamicState, 2> dynamicStates = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    const VkPipelineDynamicStateCreateInfo dynamicState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .dynamicStateCount = (uint32_t)dynamicStates.size(),
        .pDynamicStates = dynamicStates.data()
    };

    VkGraphicsPipelineCreateInfo pipelineCreateInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineCreateInfo.stageCount = (uint32_t)stages.size();
    pipelineCreateInfo.pStages = stages.data();
    pipelineCreateInfo.pVertexInputState = &vertexInputState;
    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.layout = m_pipelineLayout;
    pipelineCreateInfo.renderPass = m_renderPass;
    pipelineCreateInfo.subpass = 0;
    checkVkResult(dispatch->CreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, nullptr, &m_pipeline), "Failed to create the present graphics pipeline!");
}

template <bool depth>
void RND_VkBackend::PresentPipeline<depth>::Render(VkCommandBuffer commandBuffer, const RenderTarget& target) {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    const VkImageView depthTargetView = depth ? target.depthView : VK_NULL_HANDLE;
    const VkFormat depthTargetFormat = depth ? target.depthFormat : VK_FORMAT_UNDEFINED;

    if (VkRenderPass renderPass = m_backend->AcquireRenderPass(target.colorFormat, depthTargetFormat, false); renderPass != m_renderPass || m_pipeline == VK_NULL_HANDLE) {
        m_renderPass = renderPass;
        RecreatePipeline();
    }

    VkDescriptorSet descriptorSet = m_descriptorSets[m_backend->GetFrameContextIndex()];

    std::array<VkDescriptorImageInfo, kAttachmentCount> imageInfos = {};
    std::array<VkWriteDescriptorSet, kAttachmentCount> descriptorWrites = {};
    for (uint32_t i = 0; i < kAttachmentCount; i++) {
        checkAssert(m_attachmentViews[i] != VK_NULL_HANDLE, "The present pipeline is missing one of its attachments!");
        imageInfos[i] = {
            .sampler = VK_NULL_HANDLE,
            .imageView = m_attachmentViews[i],
            .imageLayout = VK_IMAGE_LAYOUT_GENERAL
        };
        descriptorWrites[i] = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        descriptorWrites[i].dstSet = descriptorSet;
        descriptorWrites[i].dstBinding = i;
        descriptorWrites[i].descriptorCount = 1;
        descriptorWrites[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        descriptorWrites[i].pImageInfo = &imageInfos[i];
    }
    dispatch->UpdateDescriptorSets(device, (uint32_t)descriptorWrites.size(), descriptorWrites.data(), 0, nullptr);

    const presentSettings settings = BuildSettings(target);

    VkRenderPassBeginInfo renderPassBeginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    renderPassBeginInfo.renderPass = m_renderPass;
    renderPassBeginInfo.framebuffer = m_backend->AcquireFramebuffer(m_renderPass, target.colorView, depthTargetView, target.extent);
    renderPassBeginInfo.renderArea = { { 0, 0 }, target.extent };
    dispatch->CmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport = { 0.0f, 0.0f, (float)target.extent.width, (float)target.extent.height, 0.0f, 1.0f };
    const VkRect2D scissor = { { 0, 0 }, target.extent };
    dispatch->CmdSetViewport(commandBuffer, 0, 1, &viewport);
    dispatch->CmdSetScissor(commandBuffer, 0, 1, &scissor);

    dispatch->CmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline);
    dispatch->CmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
    dispatch->CmdPushConstants(commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(settings), &settings);
    dispatch->CmdDraw(commandBuffer, 6, 1, 0, 0);

    dispatch->CmdEndRenderPass(commandBuffer);
}

RND_VkBackend::DebugDrawPipeline::DebugDrawPipeline(RND_VkBackend* backend): m_backend(backend) {
    checkAssert(m_backend != nullptr, "The debug draw pipeline is missing its renderer backend!");
    const auto* dispatch = m_backend->GetDispatch();

    m_vertexShader = m_backend->CreateShaderModule(debug_draw_vert_spv, sizeof(debug_draw_vert_spv));
    m_fragmentShader = m_backend->CreateShaderModule(debug_draw_frag_spv, sizeof(debug_draw_frag_spv));

    const VkSampler pointSampler = m_backend->GetPointSampler();
    const VkDescriptorSetLayoutBinding sceneDepthBinding = {
        .binding = 0,
        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
        .descriptorCount = 1,
        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
        .pImmutableSamplers = &pointSampler
    };

    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    descriptorSetLayoutCreateInfo.bindingCount = 1;
    descriptorSetLayoutCreateInfo.pBindings = &sceneDepthBinding;
    checkVkResult(dispatch->CreateDescriptorSetLayout(m_backend->GetDevice(), &descriptorSetLayoutCreateInfo, nullptr, &m_descriptorSetLayout), "Failed to create the debug draw descriptor set layout!");

    const VkPushConstantRange pushConstantRange = {
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        .offset = 0,
        .size = sizeof(debugDrawLineSettings)
    };

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutCreateInfo.setLayoutCount = 1;
    pipelineLayoutCreateInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;
    checkVkResult(dispatch->CreatePipelineLayout(m_backend->GetDevice(), &pipelineLayoutCreateInfo, nullptr, &m_pipelineLayout), "Failed to create the debug draw pipeline layout!");

    for (auto& eyeSets : m_sceneDepthSets) {
        std::array<VkDescriptorSetLayout, 2> setLayouts = {};
        setLayouts.fill(m_descriptorSetLayout);

        VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        descriptorSetAllocateInfo.descriptorPool = m_backend->GetDescriptorPool();
        descriptorSetAllocateInfo.descriptorSetCount = (uint32_t)setLayouts.size();
        descriptorSetAllocateInfo.pSetLayouts = setLayouts.data();
        checkVkResult(dispatch->AllocateDescriptorSets(m_backend->GetDevice(), &descriptorSetAllocateInfo, eyeSets.data()), "Failed to allocate the debug draw descriptor sets!");
    }
}

RND_VkBackend::DebugDrawPipeline::~DebugDrawPipeline() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    m_backend->WaitForSubmittedWork();

    for (HostBuffer& vertexBuffer : m_vertexBuffers) {
        m_backend->DestroyHostBuffer(vertexBuffer);
    }

    for (VkPipeline pipeline : { m_trianglePipeline, m_linePipeline, m_xrayTrianglePipeline, m_xrayLinePipeline }) {
        if (pipeline != VK_NULL_HANDLE) {
            dispatch->DestroyPipeline(device, pipeline, nullptr);
        }
    }
    if (m_pipelineLayout != VK_NULL_HANDLE) {
        dispatch->DestroyPipelineLayout(device, m_pipelineLayout, nullptr);
    }
    if (m_descriptorSetLayout != VK_NULL_HANDLE) {
        dispatch->DestroyDescriptorSetLayout(device, m_descriptorSetLayout, nullptr);
    }
    if (m_vertexShader != VK_NULL_HANDLE) {
        dispatch->DestroyShaderModule(device, m_vertexShader, nullptr);
    }
    if (m_fragmentShader != VK_NULL_HANDLE) {
        dispatch->DestroyShaderModule(device, m_fragmentShader, nullptr);
    }
}

void RND_VkBackend::DebugDrawPipeline::RecreatePipelines() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    if (m_trianglePipeline != VK_NULL_HANDLE) {
        m_backend->WaitForSubmittedWork();
        for (VkPipeline* pipeline : { &m_trianglePipeline, &m_linePipeline, &m_xrayTrianglePipeline, &m_xrayLinePipeline }) {
            if (*pipeline != VK_NULL_HANDLE) {
                dispatch->DestroyPipeline(device, *pipeline, nullptr);
                *pipeline = VK_NULL_HANDLE;
            }
        }
    }

    const std::array<VkPipelineShaderStageCreateInfo, 2> stages = {
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = m_vertexShader, .pName = "main" },
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = m_fragmentShader, .pName = "main" }
    };

    const VkVertexInputBindingDescription vertexBinding = {
        .binding = 0,
        .stride = sizeof(DebugDrawVertex),
        .inputRate = VK_VERTEX_INPUT_RATE_VERTEX
    };

    const std::array<VkVertexInputAttributeDescription, 2> vertexAttributes = {
        VkVertexInputAttributeDescription{ .location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(DebugDrawVertex, position) },
        VkVertexInputAttributeDescription{ .location = 1, .binding = 0, .format = VK_FORMAT_R8G8B8A8_UNORM, .offset = offsetof(DebugDrawVertex, color) }
    };

    const VkPipelineVertexInputStateCreateInfo vertexInputState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .vertexBindingDescriptionCount = 1,
        .pVertexBindingDescriptions = &vertexBinding,
        .vertexAttributeDescriptionCount = (uint32_t)vertexAttributes.size(),
        .pVertexAttributeDescriptions = vertexAttributes.data()
    };

    const VkPipelineInputAssemblyStateCreateInfo triangleInputAssemblyState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
    };
    const VkPipelineInputAssemblyStateCreateInfo lineInputAssemblyState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST
    };

    const VkPipelineViewportStateCreateInfo viewportState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .viewportCount = 1,
        .scissorCount = 1
    };

    const VkPipelineRasterizationStateCreateInfo rasterizationState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .polygonMode = VK_POLYGON_MODE_FILL,
        .cullMode = VK_CULL_MODE_NONE,
        .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
        .lineWidth = 1.0f
    };

    const VkPipelineMultisampleStateCreateInfo multisampleState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
    };

    const VkPipelineDepthStencilStateCreateInfo depthStencilState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .depthTestEnable = VK_TRUE,
        .depthWriteEnable = VK_TRUE,
        .depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL
    };
    const VkPipelineDepthStencilStateCreateInfo xrayDepthStencilState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .depthTestEnable = VK_FALSE,
        .depthWriteEnable = VK_FALSE,
        .depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL
    };

    const VkPipelineColorBlendAttachmentState blendAttachment = {
        .blendEnable = VK_TRUE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
        .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .colorBlendOp = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .alphaBlendOp = VK_BLEND_OP_ADD,
        .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
    };

    const VkPipelineColorBlendStateCreateInfo colorBlendState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .attachmentCount = 1,
        .pAttachments = &blendAttachment
    };

    const std::array<VkDynamicState, 2> dynamicStates = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    const VkPipelineDynamicStateCreateInfo dynamicState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .dynamicStateCount = (uint32_t)dynamicStates.size(),
        .pDynamicStates = dynamicStates.data()
    };

    VkGraphicsPipelineCreateInfo baseCreateInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    baseCreateInfo.stageCount = (uint32_t)stages.size();
    baseCreateInfo.pStages = stages.data();
    baseCreateInfo.pVertexInputState = &vertexInputState;
    baseCreateInfo.pInputAssemblyState = &triangleInputAssemblyState;
    baseCreateInfo.pViewportState = &viewportState;
    baseCreateInfo.pRasterizationState = &rasterizationState;
    baseCreateInfo.pMultisampleState = &multisampleState;
    baseCreateInfo.pDepthStencilState = &depthStencilState;
    baseCreateInfo.pColorBlendState = &colorBlendState;
    baseCreateInfo.pDynamicState = &dynamicState;
    baseCreateInfo.layout = m_pipelineLayout;
    baseCreateInfo.renderPass = m_renderPass;
    baseCreateInfo.subpass = 0;

    std::array<VkGraphicsPipelineCreateInfo, 4> pipelineCreateInfos = { baseCreateInfo, baseCreateInfo, baseCreateInfo, baseCreateInfo };
    pipelineCreateInfos[1].pInputAssemblyState = &lineInputAssemblyState;
    pipelineCreateInfos[2].pDepthStencilState = &xrayDepthStencilState;
    pipelineCreateInfos[3].pInputAssemblyState = &lineInputAssemblyState;
    pipelineCreateInfos[3].pDepthStencilState = &xrayDepthStencilState;

    std::array<VkPipeline, 4> pipelines = {};
    checkVkResult(dispatch->CreateGraphicsPipelines(device, VK_NULL_HANDLE, (uint32_t)pipelineCreateInfos.size(), pipelineCreateInfos.data(), nullptr, pipelines.data()), "Failed to create the debug draw graphics pipelines!");

    m_trianglePipeline = pipelines[0];
    m_linePipeline = pipelines[1];
    m_xrayTrianglePipeline = pipelines[2];
    m_xrayLinePipeline = pipelines[3];
}

void RND_VkBackend::DebugDrawPipeline::EnsureVertexBuffer(uint32_t frameContextIndex, uint32_t requiredBytes) {
    HostBuffer& vertexBuffer = m_vertexBuffers[frameContextIndex];
    if (requiredBytes == 0 || requiredBytes <= vertexBuffer.capacity) {
        return;
    }

    if (vertexBuffer.buffer != VK_NULL_HANDLE) {
        m_backend->WaitForSubmittedWork();
        m_backend->DestroyHostBuffer(vertexBuffer);
    }

    const uint32_t newCapacity = std::max(requiredBytes, std::max(vertexBuffer.capacity * 2, 4096u));
    vertexBuffer = m_backend->CreateHostBuffer(newCapacity, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

void RND_VkBackend::DebugDrawPipeline::UploadRenderData(const DebugDrawRenderData& renderData) {
    const uint32_t triangleVertexCount = (uint32_t)renderData.triangleVertices.size();
    const uint32_t lineVertexCount = (uint32_t)renderData.lineVertices.size();
    const uint32_t xrayTriangleVertexCount = (uint32_t)renderData.xrayTriangleVertices.size();
    const uint32_t xrayLineVertexCount = (uint32_t)renderData.xrayLineVertices.size();
    const uint32_t totalVertexCount = triangleVertexCount + lineVertexCount + xrayTriangleVertexCount + xrayLineVertexCount;

    const uint32_t frameContextIndex = m_backend->GetFrameContextIndex();
    m_uploadedVertexCounts[frameContextIndex] = 0;
    EnsureVertexBuffer(frameContextIndex, totalVertexCount * sizeof(DebugDrawVertex));

    DebugDrawVertex* dstVertices = (DebugDrawVertex*)m_vertexBuffers[frameContextIndex].mapped;
    if (dstVertices == nullptr) {
        return;
    }

    if (triangleVertexCount > 0) {
        memcpy(dstVertices, renderData.triangleVertices.data(), triangleVertexCount * sizeof(DebugDrawVertex));
    }
    if (lineVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount, renderData.lineVertices.data(), lineVertexCount * sizeof(DebugDrawVertex));
    }
    if (xrayTriangleVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount + lineVertexCount, renderData.xrayTriangleVertices.data(), xrayTriangleVertexCount * sizeof(DebugDrawVertex));
    }
    if (xrayLineVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount + lineVertexCount + xrayTriangleVertexCount, renderData.xrayLineVertices.data(), xrayLineVertexCount * sizeof(DebugDrawVertex));
    }

    m_uploadedVertexCounts[frameContextIndex] = totalVertexCount;
}

void RND_VkBackend::DebugDrawPipeline::RenderVertices(VkCommandBuffer commandBuffer, VkPipeline pipeline, uint32_t vertexOffset, uint32_t vertexCount) {
    if (pipeline == VK_NULL_HANDLE || vertexCount == 0) {
        return;
    }

    const auto* dispatch = m_backend->GetDispatch();
    dispatch->CmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    dispatch->CmdDraw(commandBuffer, vertexCount, 1, vertexOffset, 0);
}

void RND_VkBackend::DebugDrawPipeline::Render(OpenXR::EyeSide side, VkCommandBuffer commandBuffer, VkImageView sceneDepthView, const RenderTarget& target, const DebugDrawRenderData& renderData, const glm::mat4& viewProjection) {
    if (renderData.IsEmpty()) {
        return;
    }

    checkAssert(target.depthView != VK_NULL_HANDLE && target.depthFormat != VK_FORMAT_UNDEFINED, "The debug draw pipeline needs a depth target to test against!");

    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    if (VkRenderPass renderPass = m_backend->AcquireRenderPass(target.colorFormat, target.depthFormat, true); renderPass != m_renderPass || m_trianglePipeline == VK_NULL_HANDLE) {
        m_renderPass = renderPass;
        RecreatePipelines();
    }

    const uint32_t triangleVertexCount = (uint32_t)renderData.triangleVertices.size();
    const uint32_t lineVertexCount = (uint32_t)renderData.lineVertices.size();
    const uint32_t xrayTriangleVertexCount = (uint32_t)renderData.xrayTriangleVertices.size();
    const uint32_t xrayLineVertexCount = (uint32_t)renderData.xrayLineVertices.size();
    const uint32_t totalVertexCount = triangleVertexCount + lineVertexCount + xrayTriangleVertexCount + xrayLineVertexCount;

    const uint32_t frameContextIndex = m_backend->GetFrameContextIndex();
    if (m_uploadedVertexCounts[frameContextIndex] < totalVertexCount) {
        UploadRenderData(renderData);
    }

    const HostBuffer& vertexBuffer = m_vertexBuffers[frameContextIndex];
    if (vertexBuffer.buffer == VK_NULL_HANDLE) {
        return;
    }

    VkDescriptorSet descriptorSet = m_sceneDepthSets[frameContextIndex][side];
    const VkDescriptorImageInfo sceneDepthInfo = {
        .sampler = VK_NULL_HANDLE,
        .imageView = sceneDepthView,
        .imageLayout = VK_IMAGE_LAYOUT_GENERAL
    };
    VkWriteDescriptorSet descriptorWrite = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    descriptorWrite.dstSet = descriptorSet;
    descriptorWrite.dstBinding = 0;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrite.pImageInfo = &sceneDepthInfo;
    dispatch->UpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);

    debugDrawLineSettings settings = {
        .viewProjection = viewProjection,
        .targetWidth = (float)target.extent.width,
        .targetHeight = (float)target.extent.height,
        .depthBias = 0.0005f,
        .xrayAlphaScale = 0.0f,
    };

    VkRenderPassBeginInfo renderPassBeginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    renderPassBeginInfo.renderPass = m_renderPass;
    renderPassBeginInfo.framebuffer = m_backend->AcquireFramebuffer(m_renderPass, target.colorView, target.depthView, target.extent);
    renderPassBeginInfo.renderArea = { { 0, 0 }, target.extent };
    dispatch->CmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport = { 0.0f, 0.0f, (float)target.extent.width, (float)target.extent.height, 0.0f, 1.0f };
    const VkRect2D scissor = { { 0, 0 }, target.extent };
    dispatch->CmdSetViewport(commandBuffer, 0, 1, &viewport);
    dispatch->CmdSetScissor(commandBuffer, 0, 1, &scissor);

    const VkDeviceSize vertexBufferOffset = 0;
    dispatch->CmdBindVertexBuffers(commandBuffer, 0, 1, &vertexBuffer.buffer, &vertexBufferOffset);
    dispatch->CmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
    dispatch->CmdPushConstants(commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(settings), &settings);

    RenderVertices(commandBuffer, m_trianglePipeline, 0, triangleVertexCount);
    RenderVertices(commandBuffer, m_linePipeline, triangleVertexCount, lineVertexCount);

    if (xrayTriangleVertexCount > 0 || xrayLineVertexCount > 0) {
        const uint32_t xrayTriangleOffset = triangleVertexCount + lineVertexCount;
        const uint32_t xrayLineOffset = xrayTriangleOffset + xrayTriangleVertexCount;

        settings.xrayAlphaScale = 0.35f;
        dispatch->CmdPushConstants(commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(settings), &settings);

        RenderVertices(commandBuffer, m_xrayTrianglePipeline, xrayTriangleOffset, xrayTriangleVertexCount);
        RenderVertices(commandBuffer, m_xrayLinePipeline, xrayLineOffset, xrayLineVertexCount);
    }

    dispatch->CmdEndRenderPass(commandBuffer);
}

template class RND_VkBackend::PresentPipeline<false>;
template class RND_VkBackend::PresentPipeline<true>;
