#pragma once

#include "d3d12.h"
#include "openxr.h"


class RND_Renderer {
public:
    explicit RND_Renderer(XrSession xrSession);
    ~RND_Renderer();
    void StopRendering();

    void StartFrame();
    void EndFrame();

    class Layer {
    public:
        Layer() = default;
        virtual ~Layer() = default;

        virtual void PrepareRendering() = 0;
        void UpdatePredictedTime(XrTime time) { m_predictedTime = time; };
        void UpdatePoses();
        void AddTexture(OpenXR::EyeSide side, class SharedTexture* texture) {
            checkAssert(m_status == Status::PREPARING || m_status == Status::BINDING, "Need to prepare the layer before adding textures to it");
            m_status = Status::BINDING;

            this->m_textures[side] = texture;
        }
        virtual bool ShouldRender() = 0;
        virtual void Render(OpenXR::EyeSide side) = 0;
        virtual void StartRendering() = 0;

        enum class Status {
            NOT_RENDERING,
            PREPARING,
            BINDING,
            RENDERING,
        };
        [[nodiscard]] Status GetStatus() const { return m_status; };
        void FlipSide() { this->m_currentSide = (this->m_currentSide == OpenXR::LEFT) ? OpenXR::RIGHT : OpenXR::LEFT; }
        [[nodiscard]] OpenXR::EyeSide GetCurrentSide() const { return this->m_currentSide; }
    protected:
        Status m_status = Status::NOT_RENDERING;
        OpenXR::EyeSide m_currentSide = OpenXR::EyeSide::LEFT;
        XrTime m_predictedTime = 0;
        std::array<XrView, 2> m_currViews = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
        std::array<class SharedTexture*, 2> m_textures = { nullptr, nullptr };
    };

protected:
    XrSession m_session;
    XrFrameState m_frameState = { XR_TYPE_FRAME_STATE };

    class Layer3D : public Layer {
    public:
        Layer3D();

        ~Layer3D() override {
            for (auto& swapchain : m_swapchains) {
                swapchain.reset();
            }
        };

        void PrepareRendering() override;
        void AddDepthTexture(OpenXR::EyeSide side, class SharedTexture* depthTexture) {
            checkAssert(m_status == Status::PREPARING || m_status == Status::BINDING, "Need to prepare the layer before adding textures to it");
            m_status = Status::BINDING;

            this->m_depthTextures[side] = depthTexture;
        };
        void StartRendering() override;
        void Render(OpenXR::EyeSide side) override;
        const std::array<XrCompositionLayerProjectionView, 2>& FinishRendering();

        bool ShouldRender() override { return m_textures[OpenXR::EyeSide::LEFT] != nullptr && m_textures[OpenXR::EyeSide::RIGHT] != nullptr; }
        [[nodiscard]] XrFovf GetCurrentFOV() const { return m_currViews[m_currentSide].fov; }
        [[nodiscard]] XrPosef GetCurrentPose() const { return m_currViews[m_currentSide].pose; }
        [[nodiscard]] XrPosef GetOtherPose() const { return m_currViews[(m_currentSide == OpenXR::EyeSide::LEFT) ? OpenXR::EyeSide::RIGHT : OpenXR::EyeSide::LEFT].pose; }
    private:
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>, 2> m_swapchains;
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_D32_FLOAT>>, 2> m_depthSwapchains;
        std::array<std::unique_ptr<RND_D3D12::PresentPipeline<true>>, 2> m_presentPipelines;
        std::array<class SharedTexture*, 2> m_depthTextures = { nullptr, nullptr };
        std::array<XrCompositionLayerProjectionView, 2> m_projectionViews = {};
        std::array<XrCompositionLayerDepthInfoKHR, 2> m_projectionViewsDepthInfo = {};
    };

    class Layer2D : public Layer {
    public:
        Layer2D();

        ~Layer2D() override {
            m_swapchain.reset();
        };

        void PrepareRendering() override;
        void StartRendering() override;
        void Render(OpenXR::EyeSide side) override;
        XrCompositionLayerQuad FinishRendering();

        bool ShouldRender() override { return m_textures[OpenXR::EyeSide::LEFT] != nullptr || m_textures[OpenXR::EyeSide::RIGHT] != nullptr; }
    private:
        std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>> m_swapchain;
        std::unique_ptr<RND_D3D12::PresentPipeline<false>> m_presentPipeline;
    };

public:
    Layer3D m_layer3D;
    Layer2D m_layer2D;
};
