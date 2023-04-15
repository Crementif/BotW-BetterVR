#include "renderer.h"
#include "instance.h"
#include "texture.h"


RND_Renderer::RND_Renderer(XrSession xrSession): m_session(xrSession) {
    XrSessionBeginInfo m_sessionCreateInfo = { XR_TYPE_SESSION_BEGIN_INFO };
    m_sessionCreateInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    checkXRResult(xrBeginSession(VRManager::instance().XR->GetSession(), &m_sessionCreateInfo), "Failed to begin OpenXR session!");

    m_presentPipeline = std::make_unique<RND_D3D12::PresentPipeline>();
}

RND_Renderer::~RND_Renderer() {
    checkXRResult(xrEndSession(VRManager::instance().XR->GetSession()), "Failed to end OpenXR session!");
}

void RND_Renderer::StartFrame() {
    checkAssert(m_textures.empty(), "Need to finish rendering the previous frame before starting a new one");
    m_stage = RenderStage::STARTED;

    XrFrameWaitInfo waitFrameInfo = { XR_TYPE_FRAME_WAIT_INFO };
    checkXRResult(xrWaitFrame(m_session, &waitFrameInfo, &m_frameState), "Failed to wait for next frame!");

    XrFrameBeginInfo beginFrameInfo = { XR_TYPE_FRAME_BEGIN_INFO };
    checkXRResult(xrBeginFrame(m_session, &beginFrameInfo), "Couldn't begin OpenXR frame!");

    VRManager::instance().XR->UpdateTime(VRManager::instance().Hooks->s_eyeSide, m_frameState.predictedDisplayTime);

    // only update one at a time to do alternating eye rendering
    VRManager::instance().XR->UpdatePoses(OpenXR::EyeSide::LEFT);
    VRManager::instance().XR->UpdatePoses(OpenXR::EyeSide::RIGHT);

    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::LEFT)->PrepareRendering();
    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::RIGHT)->PrepareRendering();

    m_frameProjectionViews = { { { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW }, { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW } } };
}

void RND_Renderer::Render(SharedTexture* texture) {
    m_textures.emplace_back(texture);
}

void RND_Renderer::EndFrame() {
    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::LEFT)->StartRendering();
    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::RIGHT)->StartRendering();

    if (m_frameState.shouldRender) {
        XrCompositionLayerProjection frameRenderLayer = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };

        for (uint32_t i = 0; i < m_frameProjectionViews.size(); i++) {
            OpenXR::EyeSide eye = (i == 0) ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;
            Swapchain* swapchain = VRManager::instance().XR->GetSwapchain(eye);
            XrView view = VRManager::instance().XR->GetPredictedView(eye);

            // fixme: semaphores need to be adjusted so that they can be rendered to two eyes in one frame, or two SharedTextures need to be rendered to one eye each
            for (SharedTexture* texture : m_textures) {
                ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
                ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
                RND_D3D12::CommandContext<false> renderSharedTexture(device, queue, [this, texture, swapchain](ID3D12GraphicsCommandList* cmdList) {
                    cmdList->SetName(L"RenderSharedTexture");
                    texture->d3d12WaitForFence(1);
                    texture->d3d12TransitionLayout(cmdList, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
                    m_presentPipeline->BindAttachment(0, texture->d3d12GetTexture());
                    m_presentPipeline->BindTarget(0, swapchain->GetTexture(), swapchain->GetFormat());
                    m_presentPipeline->Render(cmdList, swapchain->GetTexture());
                    texture->d3d12TransitionLayout(cmdList, D3D12_RESOURCE_STATE_COPY_DEST);
                    texture->d3d12SignalFence(0);
                });
            }

            // float leftHalfFOV = glm::degrees(frameViews[0].fov.angleLeft);
            // float rightHalfFOV = glm::degrees(frameViews[0].fov.angleRight);
            // float upHalfFOV = glm::degrees(frameViews[0].fov.angleUp);
            // float downHalfFOV = glm::degrees(frameViews[0].fov.angleDown);
            //
            // float horizontalHalfFOV = (float)(abs(frameViews[0].fov.angleLeft) + abs(frameViews[0].fov.angleRight)) * 0.5f;
            // float verticalHalfFOV = (float)(abs(frameViews[0].fov.angleUp) + abs(frameViews[0].fov.angleDown)) * 0.5f;
            m_frameProjectionViews[i].pose = view.pose;
            m_frameProjectionViews[i].fov = view.fov;
            m_frameProjectionViews[i].subImage.swapchain = swapchain->GetHandle();
            m_frameProjectionViews[i].subImage.imageRect.offset = { 0, 0 };
            m_frameProjectionViews[i].subImage.imageRect.extent = { (int32_t)swapchain->GetWidth(), (int32_t)swapchain->GetHeight() };
        }

        frameRenderLayer.layerFlags = NULL;
        frameRenderLayer.space = VRManager::instance().XR->m_headSpace;
        frameRenderLayer.viewCount = (uint32_t)m_frameProjectionViews.size();
        frameRenderLayer.views = m_frameProjectionViews.data();
        m_layers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&frameRenderLayer));
    }

    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::LEFT)->FinishRendering();
    VRManager::instance().XR->GetSwapchain(OpenXR::EyeSide::RIGHT)->FinishRendering();

    XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO };
    frameEndInfo.displayTime = m_frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount = (uint32_t)m_layers.size();
    frameEndInfo.layers = m_layers.data();
    checkXRResult(xrEndFrame(m_session, &frameEndInfo), "Failed to render texture!");

    m_textures.clear();
    m_layers.clear();
}
