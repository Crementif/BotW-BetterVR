#pragma once

#include "openxr.h"
#include "texture.h"
#include "shader.h"
#include "utils/render_utils.h"
#include "utils/debug_draw.h"

class RND_Renderer;

// the queue the mod submits its own work to
// shared means Cemu uses the same queue, which is only safe because everything happens on Cemu's render thread
struct ModQueueBinding {
    uint32_t familyIndex = 0;
    uint32_t queueIndex = 0;
    bool shared = false;
};

class RND_VkBackend {
    friend class RND_Renderer;

public:
    static constexpr uint32_t kFrameContextCount = 3;
    static constexpr uint32_t kFrameCommandBufferCount = 4;

    RND_VkBackend(VkInstance vkInstance, VkPhysicalDevice vkPhysicalDevice, VkDevice vkDevice, const ModQueueBinding& modQueue);
    ~RND_VkBackend();

    VkDevice GetDevice() const { return m_device; };
    VkQueue GetQueue() const { return m_queue; };
    const ModQueueBinding& GetQueueBinding() const { return m_modQueue; };
    const vkroots::VkDeviceDispatch* GetDispatch() const { return m_dispatch; };
    VkDescriptorPool GetDescriptorPool() const { return m_descriptorPool; };
    VkSampler GetPointSampler() const { return m_pointSampler; };
    uint32_t GetFrameContextIndex() const { return m_currentFrameContextIndex; };

    void StartFrame();
    void EndFrame();
    void WaitForSubmittedWork();

    struct RenderTarget {
        VkImageView colorView = VK_NULL_HANDLE;
        VkFormat colorFormat = VK_FORMAT_UNDEFINED;
        VkImageView depthView = VK_NULL_HANDLE;
        VkFormat depthFormat = VK_FORMAT_UNDEFINED;
        VkExtent2D extent = {};
        bool srgbEncodeInShader = false;
    };

    // Passing VK_FORMAT_UNDEFINED as the depth format creates a color-only pass
    VkRenderPass AcquireRenderPass(VkFormat colorFormat, VkFormat depthFormat, bool preserveContents);
    VkFramebuffer AcquireFramebuffer(VkRenderPass renderPass, VkImageView colorView, VkImageView depthView, VkExtent2D extent);
    void FlushFramebufferCache();

    VkShaderModule CreateShaderModule(const uint32_t* spirvWords, size_t spirvByteSize) const;

    struct HostBuffer {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        void* mapped = nullptr;
        uint32_t capacity = 0;
    };
    HostBuffer CreateHostBuffer(uint32_t size, VkBufferUsageFlags usage) const;
    void DestroyHostBuffer(HostBuffer& hostBuffer) const;

    template <bool depth>
    class PresentPipeline {
    public:
        PresentPipeline(RND_VkBackend* backend, RND_Renderer* pRenderer);
        ~PresentPipeline();

        void BindAttachment(uint32_t attachmentIdx, VkImageView srcView);
        void BindSettings(float screenWidth, float screenHeight, const RenderUtils::UvTransform& uvTransform = {});
        void SetUvTransform(const RenderUtils::UvTransform& uvTransform) { m_uvTransform = uvTransform; }
        void Render(VkCommandBuffer commandBuffer, const RenderTarget& target);

    private:
        static constexpr uint32_t kAttachmentCount = depth ? 3 : 1;

        void RecreatePipeline();
        presentSettings BuildSettings(const RenderTarget& target) const;

        RND_VkBackend* m_backend = nullptr;
        RND_Renderer* m_renderer = nullptr;
        float m_renderWidth = 0.0f;
        float m_renderHeight = 0.0f;
        RenderUtils::UvTransform m_uvTransform = {};

        VkShaderModule m_vertexShader = VK_NULL_HANDLE;
        VkShaderModule m_fragmentShader = VK_NULL_HANDLE;
        VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
        VkRenderPass m_renderPass = VK_NULL_HANDLE;
        VkPipeline m_pipeline = VK_NULL_HANDLE;

        std::array<VkDescriptorSet, kFrameContextCount> m_descriptorSets = {};
        std::array<VkImageView, kAttachmentCount> m_attachmentViews = {};
    };

    template <bool blockTillExecuted>
    class CommandContext {
    public:
        template <typename F>
        CommandContext(RND_VkBackend* backend, F&& recordCallback): m_backend(backend) {
            checkAssert(m_backend != nullptr, "Vulkan command context is missing its renderer backend!");
            m_waitFor.reserve(4);
            m_signalTo.reserve(4);

            if constexpr (blockTillExecuted) {
                m_commandBuffer = m_backend->AcquireImmediateCommandBuffer();
            }
            else {
                m_commandBuffer = m_backend->AcquireFrameCommandBuffer();
            }

            recordCallback(this);
        }

        ~CommandContext() {
            const uint64_t completionValue = m_backend->SubmitCommandBuffer(m_commandBuffer, m_waitFor, m_signalTo);

            if constexpr (blockTillExecuted) {
                m_backend->WaitForTimeline(completionValue);
            }
        }

        VkCommandBuffer GetCommandBuffer() { return m_commandBuffer; }
        void WaitFor(FrameSync* sync, uint64_t value) { m_waitFor.push_back({ sync, value }); }
        void Signal(FrameSync* sync, uint64_t value) { m_signalTo.push_back({ sync, value }); }

    private:
        RND_VkBackend* m_backend = nullptr;
        VkCommandBuffer m_commandBuffer = VK_NULL_HANDLE;
        std::vector<std::pair<FrameSync*, uint64_t>> m_waitFor;
        std::vector<std::pair<FrameSync*, uint64_t>> m_signalTo;
    };

    class DebugDrawPipeline {
    public:
        explicit DebugDrawPipeline(RND_VkBackend* backend);
        ~DebugDrawPipeline();

        void UploadRenderData(const DebugDrawRenderData& renderData);
        void Render(OpenXR::EyeSide side, VkCommandBuffer commandBuffer, VkImageView sceneDepthView, const RenderTarget& target, const DebugDrawRenderData& renderData, const glm::mat4& viewProjection);

    private:
        void RecreatePipelines();
        void EnsureVertexBuffer(uint32_t frameContextIndex, uint32_t requiredBytes);
        void RenderVertices(VkCommandBuffer commandBuffer, VkPipeline pipeline, uint32_t vertexOffset, uint32_t vertexCount);

        RND_VkBackend* m_backend = nullptr;

        VkShaderModule m_vertexShader = VK_NULL_HANDLE;
        VkShaderModule m_fragmentShader = VK_NULL_HANDLE;
        VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
        VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
        VkRenderPass m_renderPass = VK_NULL_HANDLE;
        VkPipeline m_trianglePipeline = VK_NULL_HANDLE;
        VkPipeline m_linePipeline = VK_NULL_HANDLE;
        VkPipeline m_xrayTrianglePipeline = VK_NULL_HANDLE;
        VkPipeline m_xrayLinePipeline = VK_NULL_HANDLE;

        std::array<std::array<VkDescriptorSet, 2>, kFrameContextCount> m_sceneDepthSets = {};
        std::array<HostBuffer, kFrameContextCount> m_vertexBuffers = {};
        std::array<uint32_t, kFrameContextCount> m_uploadedVertexCounts = {};
    };

private:
    struct FrameContext {
        VkCommandPool commandPool = VK_NULL_HANDLE;
        std::array<VkCommandBuffer, kFrameCommandBufferCount> commandBuffers = {};
        uint32_t nextCommandBufferIndex = 0;
        uint64_t completionTimelineValue = 0;
    };

    struct ImmediateContext {
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    };

    struct CachedRenderPass {
        VkFormat colorFormat = VK_FORMAT_UNDEFINED;
        VkFormat depthFormat = VK_FORMAT_UNDEFINED;
        bool preserveContents = false;
        VkRenderPass renderPass = VK_NULL_HANDLE;
    };

    struct CachedFramebuffer {
        VkRenderPass renderPass = VK_NULL_HANDLE;
        VkImageView colorView = VK_NULL_HANDLE;
        VkImageView depthView = VK_NULL_HANDLE;
        VkExtent2D extent = {};
        VkFramebuffer framebuffer = VK_NULL_HANDLE;
    };

    FrameContext& GetCurrentFrameContext();
    VkCommandBuffer AcquireFrameCommandBuffer();
    VkCommandBuffer AcquireImmediateCommandBuffer();
    uint64_t SubmitCommandBuffer(VkCommandBuffer commandBuffer, const std::vector<std::pair<FrameSync*, uint64_t>>& waitFor, const std::vector<std::pair<FrameSync*, uint64_t>>& signalTo);
    void WaitForTimeline(uint64_t timelineValue);
    void AssertRenderThread();

    VkInstance m_instance = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    const vkroots::VkDeviceDispatch* m_dispatch = nullptr;
    ModQueueBinding m_modQueue = {};
    VkQueue m_queue = VK_NULL_HANDLE;

    std::array<FrameContext, kFrameContextCount> m_frameContexts = {};
    ImmediateContext m_immediateContext = {};
    uint32_t m_currentFrameContextIndex = kFrameContextCount - 1;

    VkSemaphore m_timeline = VK_NULL_HANDLE;
    uint64_t m_nextTimelineValue = 1;
    uint64_t m_lastSubmittedTimelineValue = 0;

    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkSampler m_pointSampler = VK_NULL_HANDLE;

    std::vector<CachedRenderPass> m_renderPasses;
    std::vector<CachedFramebuffer> m_framebuffers;

    std::thread::id m_submitThreadId = {};
    bool m_submitThreadLatched = false;
};
