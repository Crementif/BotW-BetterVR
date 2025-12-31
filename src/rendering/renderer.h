#pragma once

#include "pch.h"
#include "d3d12.h"
#include "openxr.h"
#include "swapchain.h"
#include "texture.h"

class SharedTexture;

class RND_Renderer {
public:
    explicit RND_Renderer(XrSession xrSession);
    ~RND_Renderer();

    struct RenderFrame {
        std::optional<std::array<XrView, 2>> views;
        std::atomic_bool copiedColor[2] = { false, false };
        std::atomic_bool copiedDepth[2] = { false, false };
        std::atomic_bool copied2D = false;
        std::atomic_bool presented3D = false;

        std::unique_ptr<VulkanTexture> mainFramebuffer;
        std::unique_ptr<VulkanTexture> hudFramebuffer;
        std::unique_ptr<VulkanTexture> hudWithoutAlphaFramebuffer;
        std::unique_ptr<VulkanFramebuffer> imguiFramebuffer;
        VkDescriptorSet mainFramebufferDS = VK_NULL_HANDLE;
        VkDescriptorSet hudFramebufferDS = VK_NULL_HANDLE;
        VkDescriptorSet hudWithoutAlphaFramebufferDS = VK_NULL_HANDLE;
        float mainFramebufferAspectRatio = 1.0f;

        bool ranMotionAnalysis[2] = { false, false };

        bool Is3DComplete() const { return copiedColor[0] && copiedColor[1] && copiedDepth[0] && copiedDepth[1]; }
        bool Is2DComplete() const { return copied2D; }

        void Reset() {
            views = std::nullopt;
            copiedColor[0] = false;
            copiedColor[1] = false;
            copiedDepth[0] = false;
            copiedDepth[1] = false;
            copied2D = false;

            ranMotionAnalysis[0] = false;
            ranMotionAnalysis[1] = false;
        }
    };

    void StartFrame();
    void EndFrame();
    std::optional<std::array<XrView, 2>> UpdateViews(XrTime predictedDisplayTime);
    
    std::optional<std::array<XrView, 2>> GetPoses(long frameIdx = -1) const { 
        if (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) return m_renderFrames[frameIdx].views;
        return m_currViews; 
    }
    
    std::optional<XrFovf> GetFOV(OpenXR::EyeSide side, long frameIdx = -1) const { 
        const auto& views = (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) ? m_renderFrames[frameIdx].views : m_currViews;
        return views.transform([side](auto& views) { return views[side].fov; }); 
    }
    
    std::optional<XrPosef> GetPose(OpenXR::EyeSide side, long frameIdx = -1) const { 
        const auto& views = (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) ? m_renderFrames[frameIdx].views : m_currViews;
        return views.transform([side](auto& views) { return views[side].pose; }); 
    }
    
    std::optional<glm::fmat4> GetPoseAsMatrix(OpenXR::EyeSide side, long frameIdx = -1) const {
        const auto& views = (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) ? m_renderFrames[frameIdx].views : m_currViews;
        return views.transform([side](auto& views) {
            const XrPosef& pose = views[side].pose;
            return ToMat4(ToGLM(pose.position), ToGLM(pose.orientation));
        });
    };
    
    std::optional<glm::fmat4> GetMiddlePose(long frameIdx = -1) const {
        const auto& views = (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) ? m_renderFrames[frameIdx].views : m_currViews;
        if (!views.has_value()) return std::nullopt;
        const XrPosef& leftPose = views->at(OpenXR::EyeSide::LEFT).pose;
        const XrPosef& rightPose = views->at(OpenXR::EyeSide::RIGHT).pose;
        glm::fvec3 middlePos = (ToGLM(leftPose.position) + ToGLM(rightPose.position)) * 0.5f;
        glm::quat middleOri = glm::slerp(ToGLM(leftPose.orientation), ToGLM(rightPose.orientation), 0.5f);

        return ToMat4(middlePos, middleOri);
    };

    void On3DColorCopied(OpenXR::EyeSide side, long frameIdx) {
        m_renderFrames[frameIdx].copiedColor[side] = true;
        if (!m_renderFrames[frameIdx].views.has_value()) m_renderFrames[frameIdx].views = m_currViews;
    }

    void On3DDepthCopied(OpenXR::EyeSide side, long frameIdx) {
        m_renderFrames[frameIdx].copiedDepth[side] = true;
        if (!m_renderFrames[frameIdx].views.has_value()) m_renderFrames[frameIdx].views = m_currViews;
    }

    void On2DCopied(long frameIdx) {
        m_renderFrames[frameIdx].copied2D = true;
    }

    RenderFrame& GetFrame(long frameIdx) { return m_renderFrames[frameIdx]; }
    const RenderFrame& GetFrame(long frameIdx) const { return m_renderFrames[frameIdx]; }

    class Layer3D {
    public:
        explicit Layer3D(VkExtent2D extent);
        ~Layer3D();

        SharedTexture* CopyColorToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx, VkImageLayout srcImageLayout);
        SharedTexture* CopyDepthToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx, VkImageLayout srcImageLayout);
        void PrepareRendering(OpenXR::EyeSide side);
        void StartRendering();
        void Render(OpenXR::EyeSide side, long frameIdx);
        const std::array<XrCompositionLayerProjectionView, 2>& FinishRendering(long frameIdx);

        float GetAspectRatio(OpenXR::EyeSide side) const { return m_swapchains[side]->GetWidth() / (float)m_swapchains[side]->GetHeight(); }
        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }

    private:
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>, 2> m_swapchains;
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_D32_FLOAT>>, 2> m_depthSwapchains;
        std::array<std::unique_ptr<RND_D3D12::PresentPipeline<true>>, 2> m_presentPipelines;
        std::array<std::array<std::unique_ptr<SharedTexture>, 2>, 2> m_textures;
        std::array<std::array<std::unique_ptr<SharedTexture>, 2>, 2> m_depthTextures;

        std::array<XrCompositionLayerProjectionView, 2> m_projectionViews = {};
        std::array<XrCompositionLayerDepthInfoKHR, 2> m_projectionViewsDepthInfo = {};

        long m_currentFrameIdx = 0;
    };

    class Layer2D {
    public:
        explicit Layer2D(VkExtent2D extent);
        ~Layer2D();

        SharedTexture* CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx, VkImageLayout srcImageLayout);
        // AMD GPU FIX: With incrementing values, Vulkan signals odd values (1,3,5...), D3D12 signals even values (2,4,6...)
        // Texture is ready for D3D12 when Vulkan has signaled (odd value > 0)
        bool IsTextureReady(long frameIdx) const {
            uint64_t lastSignal = m_textures[frameIdx]->GetLastSignalledValue();
            return lastSignal > 0 && (lastSignal % 2 == 1);
        };
        void StartRendering() const;
        void Render(long frameIdx);
        std::vector<XrCompositionLayerQuad> FinishRendering(XrTime predictedDisplayTime, long frameIdx);
        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }

    private:
        std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>> m_swapchain;
        std::unique_ptr<RND_D3D12::PresentPipeline<false>> m_presentPipeline;
        std::array<std::unique_ptr<SharedTexture>, 2> m_textures;

        static constexpr float DISTANCE = 2.0f;
        static constexpr float LERP_SPEED = 0.05f;
        glm::quat m_currentOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

        long m_currentFrameIdx = 0;
    };

    class ImGuiOverlay {
    public:
        explicit ImGuiOverlay(VkCommandBuffer cb, uint32_t width, uint32_t height, VkFormat format);
        ~ImGuiOverlay();

        bool ShouldBlockGameInput() { return ImGui::GetIO().WantCaptureKeyboard; }

        void BeginFrame(long frameIdx, bool renderBackground);
        // AMD GPU FIX: Added srcLayout parameter to specify the actual source image layout
        static void Draw3DLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, float aspectRatio, long frameIdx, VkImageLayout srcLayout);
        static void DrawHUDLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, long frameIdx, VkImageLayout srcLayout);
        void Update();
        void Render();
        void DrawAndCopyToImage(VkCommandBuffer cb, VkImage destImage, long frameIdx);

    private:
        VkDescriptorPool m_descriptorPool;
        VkRenderPass m_renderPass;

        HWND m_cemuTopWindow = nullptr;
        HWND m_cemuRenderWindow = nullptr;

        VkSampler m_sampler = VK_NULL_HANDLE;
    };

    std::unique_ptr<Layer3D> m_layer3D;
    std::unique_ptr<Layer2D> m_layer2D;
    std::unique_ptr<ImGuiOverlay> m_imguiOverlay;

    bool IsRendering3D(long frameIdx) {
        return m_renderFrames[frameIdx].presented3D;
    }
    bool IsRendering2D() {
        return m_presented2DLastFrame;
    }
    bool IsInitialized() {
        return m_isInitialized;
    }

protected:
    XrSession m_session;
    XrFrameState m_frameState = { XR_TYPE_FRAME_STATE };
    std::optional<std::array<XrView, 2>> m_currViews;
    std::array<RenderFrame, 2> m_renderFrames;

    std::atomic_bool m_isInitialized = false;
    std::atomic_bool m_presented2DLastFrame = false;
};
