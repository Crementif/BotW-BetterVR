#pragma once

#include "openxr.h"
#include "texture.h"
#include "utils/render_utils.h"
#include "utils/debug_draw.h"

class RND_Renderer;

class RND_D3D12 {
    friend class RND_Renderer;

public:
    RND_D3D12();
    ~RND_D3D12();

    ID3D12Device* GetDevice() { return m_device.Get(); };
    ID3D12CommandQueue* GetCommandQueue() { return m_queue.Get(); };

    // fence progress accessors for non-stalling readbacks (telemetry)
    uint64_t GetLastSignaledFenceValue() const { return m_nextFenceValue - 1; }
    uint64_t GetFenceCompletedValue() const { return m_fence->GetCompletedValue(); }

    void StartFrame();
    void EndFrame();

    // todo: extract most to a base pipeline class if other pipelines are needed
    template <bool depth>
    class PresentPipeline {
        friend class Texture;

    public:
        explicit PresentPipeline(RND_Renderer* pRenderer);
        ~PresentPipeline() = default;

        void BindAttachment(uint32_t attachmentIdx, ID3D12Resource* srcTexture, DXGI_FORMAT overwriteFormat = DXGI_FORMAT_UNKNOWN);
        void BindTarget(uint32_t targetIdx, ID3D12Resource* dstTexture, DXGI_FORMAT overwriteFormat = DXGI_FORMAT_UNKNOWN);
        void BindDepthTarget(ID3D12Resource* dstTexture, DXGI_FORMAT overwriteFormat);
        void BindSettings(float screenWidth, float screenHeight, const RenderUtils::UvTransform& uvTransform = {});
        void SetUvTransform(const RenderUtils::UvTransform& uvTransform) { m_uvTransform = uvTransform; }
        // when set, the pixel shader inverse-warps the bound (left-eye) texture into this eye's view
        void SetReprojection(const glm::mat4* leftToRightClipMtx) {
            m_reprojectionEnabled = leftToRightClipMtx != nullptr;
            if (leftToRightClipMtx != nullptr) {
                m_reprojectionMatrix = *leftToRightClipMtx;
            }
        }
        void Render(ID3D12GraphicsCommandList* commandList, ID3D12Resource* swapchain);

    private:
        void UpdateSettingsBuffer(ID3D12Resource* swapchain);

        RND_Renderer* m_renderer = nullptr;
        float m_renderWidth = 0.0f;
        float m_renderHeight = 0.0f;
        RenderUtils::UvTransform m_uvTransform = {};
        bool m_reprojectionEnabled = false;
        glm::mat4 m_reprojectionMatrix = glm::mat4(1.0f);

        void RecreatePipeline();

        ComPtr<ID3DBlob> m_vertexShader;
        ComPtr<ID3DBlob> m_pixelShader;

        ComPtr<ID3D12Resource> m_screenIndicesBuffer;
        D3D12_INDEX_BUFFER_VIEW m_screenIndicesView = {};

        ComPtr<ID3D12Resource> m_settingsBuffer;

        ComPtr<ID3D12RootSignature> m_signature;
        ComPtr<ID3D12PipelineState> m_pipelineState;

        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, depth ? 3 : 1> m_attachmentHandles = {};
        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, 1> m_targetHandles = {};
        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, depth ? 1 : 0> m_depthTargetHandles = {};
        ComPtr<ID3D12DescriptorHeap> m_attachmentHeap;
        ComPtr<ID3D12DescriptorHeap> m_targetHeap;
        ComPtr<ID3D12DescriptorHeap> m_depthHeap;
        std::array<DXGI_FORMAT, 2> m_targetFormats = { DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_D32_FLOAT };
    };

    template <bool blockTillExecuted>
    class CommandContext {
    public:
        template <typename F>
        CommandContext(RND_D3D12* d3d12, F&& recordCallback): m_d3d12(d3d12) {
            checkAssert(m_d3d12 != nullptr, "D3D12 command context is missing its renderer backend!");
            m_waitFor.reserve(4);
            m_signalTo.reserve(4);

            if constexpr (blockTillExecuted) {
                m_cmdList = m_d3d12->AcquireImmediateCommandList();
            }
            else {
                m_cmdList = m_d3d12->AcquireFrameCommandList();
            }

            recordCallback(this);
        }

        ~CommandContext() {
            checkHResult(m_cmdList->Close(), "Failed to close D3D12 command list!");

            for (auto& [texture, value] : m_waitFor)
                texture->d3d12WaitForFence(value);

            m_d3d12->ExecuteCommandList(m_cmdList);

            for (auto& [texture, value] : m_signalTo)
                texture->d3d12SignalFence(value);

            if constexpr (blockTillExecuted) {
                const uint64_t fenceValue = m_d3d12->SignalQueueFence();
                m_d3d12->WaitForQueueFence(fenceValue);
            }
        }

        ID3D12GraphicsCommandList* GetRecordList() { return m_cmdList; }
        void WaitFor(Texture* texture, uint64_t value) { m_waitFor.push_back({ texture, value }); }
        void Signal(Texture* texture, uint64_t value) { m_signalTo.push_back({ texture, value }); }

    private:
        RND_D3D12* m_d3d12 = nullptr;
        ID3D12GraphicsCommandList* m_cmdList = nullptr;
        std::vector<std::pair<Texture*, uint64_t>> m_waitFor;
        std::vector<std::pair<Texture*, uint64_t>> m_signalTo;
    };

    class DebugDrawPipeline {
    public:
        DebugDrawPipeline();
        ~DebugDrawPipeline();

        void UploadRenderData(const DebugDrawRenderData& renderData);
        void Render(OpenXR::EyeSide side, ID3D12GraphicsCommandList* commandList, ID3D12Resource* sceneDepthTexture, ID3D12Resource* colorTarget, DXGI_FORMAT colorFormat, ID3D12Resource* depthTarget, DXGI_FORMAT depthFormat, const DebugDrawRenderData& renderData, const glm::mat4& viewProjection);

    private:
        void RecreatePipeline();
        void EnsureVertexBuffer(uint32_t requiredBytes);
        void UpdateSceneSettings(OpenXR::EyeSide side, uint32_t settingsIndex, ID3D12Resource* colorTarget, const glm::mat4& viewProjection, float xrayAlphaScale);
        void RenderVertices(ID3D12GraphicsCommandList* commandList, D3D12_PRIMITIVE_TOPOLOGY topology, ID3D12PipelineState* pipelineState, uint32_t vertexOffset, uint32_t vertexCount);

        ComPtr<ID3DBlob> m_vertexShader;
        ComPtr<ID3DBlob> m_pixelShader;
        ComPtr<ID3D12RootSignature> m_signature;
        ComPtr<ID3D12PipelineState> m_trianglePipelineState;
        ComPtr<ID3D12PipelineState> m_linePipelineState;
        ComPtr<ID3D12PipelineState> m_xrayTrianglePipelineState;
        ComPtr<ID3D12PipelineState> m_xrayLinePipelineState;

        std::array<std::array<ComPtr<ID3D12Resource>, 2>, 2> m_sceneSettingsBuffers;
        ComPtr<ID3D12Resource> m_vertexBuffer;
        D3D12_VERTEX_BUFFER_VIEW m_vertexBufferView = {};
        uint32_t m_vertexBufferCapacity = 0;
        DebugDrawVertex* m_mappedVertexBuffer = nullptr;

        ComPtr<ID3D12DescriptorHeap> m_sceneDepthHeap;
        ComPtr<ID3D12DescriptorHeap> m_targetHeap;
        ComPtr<ID3D12DescriptorHeap> m_depthHeap;
        std::array<std::array<uint8_t*, 2>, 2> m_sceneSettingsMappedData = {};
        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, 2> m_sceneDepthHandles = {};
        std::array<D3D12_GPU_DESCRIPTOR_HANDLE, 2> m_sceneDepthGpuHandles = {};
        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, 2> m_targetHandles = {};
        std::array<D3D12_CPU_DESCRIPTOR_HANDLE, 2> m_depthHandles = {};
        std::array<DXGI_FORMAT, 2> m_targetFormats = { DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_UNKNOWN };
    };

private:
    static constexpr uint32_t kFrameContextCount = 3;
    static constexpr uint32_t kFrameCommandListCount = 4;

    struct FrameContext {
        ComPtr<ID3D12CommandAllocator> allocator;
        std::array<ComPtr<ID3D12GraphicsCommandList>, kFrameCommandListCount> commandLists = {};
        uint32_t nextCommandListIndex = 0;
        uint64_t completionFenceValue = 0;
    };

    struct ImmediateContext {
        ComPtr<ID3D12CommandAllocator> allocator;
        ComPtr<ID3D12GraphicsCommandList> commandList;
    };

    FrameContext& GetCurrentFrameContext();
    ID3D12GraphicsCommandList* AcquireFrameCommandList();
    ID3D12GraphicsCommandList* AcquireImmediateCommandList();
    void ExecuteCommandList(ID3D12GraphicsCommandList* commandList);
    uint64_t SignalQueueFence();
    void WaitForQueueFence(uint64_t fenceValue);

    ComPtr<ID3D12Device> m_device;
    ComPtr<ID3D12CommandQueue> m_queue;
    std::array<FrameContext, kFrameContextCount> m_frameContexts = {};
    ImmediateContext m_immediateContext = {};
    uint32_t m_currentFrameContextIndex = kFrameContextCount - 1;
    ComPtr<ID3D12Fence> m_fence;
    HANDLE m_fenceEvent = nullptr;
    uint64_t m_nextFenceValue = 1;
};
