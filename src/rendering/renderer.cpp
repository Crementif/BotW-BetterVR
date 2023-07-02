#include "renderer.h"
#include "instance.h"
#include "texture.h"

#include <glm/glm.hpp>


RND_Renderer::RND_Renderer(XrSession xrSession): m_session(xrSession) {
    XrSessionBeginInfo m_sessionCreateInfo = { XR_TYPE_SESSION_BEGIN_INFO };
    m_sessionCreateInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    checkXRResult(xrBeginSession(m_session, &m_sessionCreateInfo), "Failed to begin OpenXR session!");
}

RND_Renderer::~RND_Renderer() {
    StopRendering();
}

void RND_Renderer::StopRendering() {
    xrRequestExitSession(m_session);
    if (m_session != XR_NULL_HANDLE) {
        checkXRResult(xrEndSession(m_session), "Failed to end OpenXR session!");
        m_session = XR_NULL_HANDLE;
    }
}

void RND_Renderer::StartFrame() {
    XrFrameWaitInfo waitFrameInfo = { XR_TYPE_FRAME_WAIT_INFO };
    checkXRResult(xrWaitFrame(m_session, &waitFrameInfo, &m_frameState), "Failed to wait for next frame!");

    XrFrameBeginInfo beginFrameInfo = { XR_TYPE_FRAME_BEGIN_INFO };
    checkXRResult(xrBeginFrame(m_session, &beginFrameInfo), "Couldn't begin OpenXR frame!");

    VRManager::instance().D3D12->StartFrame();

    if (m_layer3D.GetStatus() != Layer::Status::PREPARING && m_layer3D.GetStatus() != Layer::Status::BINDING) {
        Log::print("Preparing for 3D rendering");
        m_layer3D.PrepareRendering();
    }
    if (m_layer2D.GetStatus() != Layer::Status::PREPARING && m_layer2D.GetStatus() != Layer::Status::BINDING) {
        Log::print("Preparing for 2D rendering");
        m_layer2D.PrepareRendering();
    }

    m_layer3D.UpdatePredictedTime(m_frameState.predictedDisplayTime);
    m_layer2D.UpdatePredictedTime(m_frameState.predictedDisplayTime);
    VRManager::instance().XR->UpdateSpaces(m_frameState.predictedDisplayTime);

    // currently we only support non-AER presenting, aka we render two textures with the same pose and then we present them
    if (CemuHooks::GetSettings().alternatingEyeRenderingSetting == 0) {
        m_layer3D.UpdatePoses();
    }
    else {
        checkAssert(false, "AER isn't a supported configuration yet!");
    }
}

void RND_Renderer::EndFrame() {
    if (m_layer3D.ShouldRender()) {
        Log::print("Rendering 3D");
        m_layer3D.StartRendering();
    }
    if (m_layer2D.ShouldRender()) {
        Log::print("Rendering 2D");
        m_layer2D.StartRendering();
    }

    std::vector<XrCompositionLayerBaseHeader*> compositionLayers;

    XrCompositionLayerQuad layer2D = { XR_TYPE_COMPOSITION_LAYER_QUAD };
    if (m_frameState.shouldRender && m_layer2D.GetStatus() == Layer::Status::RENDERING) {
        // The HUD/menus aren't eye-specific, so just present the most recent one for both eyes at once
        m_layer2D.FlipSide();
        m_layer2D.Render(m_layer2D.GetCurrentSide());
        layer2D = m_layer2D.FinishRendering();
        Log::print("Finished rendering 2D");
        m_layer2D.FlipSide();
        compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer2D));
    }

    XrCompositionLayerProjection layer3D = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
    std::array<XrCompositionLayerProjectionView, 2> layer3DViews = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
    if (m_frameState.shouldRender && m_layer3D.GetStatus() == Layer::Status::RENDERING) {
        m_layer3D.Render(OpenXR::EyeSide::LEFT);
        m_layer3D.Render(OpenXR::EyeSide::RIGHT);
        layer3DViews = m_layer3D.FinishRendering();
        Log::print("Finished rendering 3D");
        layer3D.layerFlags = NULL;
        layer3D.space = VRManager::instance().XR->m_stageSpace;
        layer3D.viewCount = (uint32_t)layer3DViews.size();
        layer3D.views = layer3DViews.data();
        compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer3D));
    }

    XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO };
    frameEndInfo.displayTime = m_frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount = (uint32_t)compositionLayers.size();
    frameEndInfo.layers = compositionLayers.data();
    checkXRResult(xrEndFrame(m_session, &frameEndInfo), "Failed to render texture!");

    VRManager::instance().D3D12->EndFrame();
}

void RND_Renderer::Layer::UpdatePoses() {
    // todo: should this be moved to Layer3D since it's only used there?
    std::array<XrView, 2> views = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
    XrViewLocateInfo viewLocateInfo = { XR_TYPE_VIEW_LOCATE_INFO };
    viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    viewLocateInfo.displayTime = m_predictedTime;
    viewLocateInfo.space = VRManager::instance().XR->m_stageSpace; // locate the rendering views relative to the room, not the headset center
    XrViewState viewState = { XR_TYPE_VIEW_STATE };
    uint32_t viewCount = (uint32_t)views.size();
    checkXRResult(xrLocateViews(VRManager::instance().XR->m_session, &viewLocateInfo, &viewState, viewCount, &viewCount, views.data()), "Failed to get view information!");
    if ((viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0)
        return; // what should occur when the orientation is invalid? keep rendering using old values?

    m_currViews = views;
//    {
//        glm::fvec3 positions{ (m_currViews[0].pose.position.x, m_currViews[0].pose.position.y, m_currViews[0].pose.position.z) };
//        positions = positions * 10.0f;
//        m_currViews[0].pose.position = { positions.x, positions.y, positions.z };
//    }
//    {
//        glm::fvec3 positions{ (m_currViews[1].pose.position.x, m_currViews[1].pose.position.y, m_currViews[1].pose.position.z) };
//        positions = positions * 10.0f;
//        m_currViews[1].pose.position = { positions.x, positions.y, positions.z };
//    }
}

RND_Renderer::Layer3D::Layer3D(): Layer() {
    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

    this->m_presentPipelines[OpenXR::EyeSide::LEFT] = std::make_unique<RND_D3D12::PresentPipeline<true>>(VRManager::instance().XR->GetRenderer());
    this->m_presentPipelines[OpenXR::EyeSide::RIGHT] = std::make_unique<RND_D3D12::PresentPipeline<true>>(VRManager::instance().XR->GetRenderer());

    // note: it's possible to make a swapchain that matches Cemu's internal resolution and let the headset downsample it, although I doubt there's a benefit
    this->m_swapchains[OpenXR::EyeSide::LEFT] = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight, viewConfs[0].recommendedSwapchainSampleCount);
    this->m_swapchains[OpenXR::EyeSide::RIGHT] = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(viewConfs[1].recommendedImageRectWidth, viewConfs[1].recommendedImageRectHeight, viewConfs[1].recommendedSwapchainSampleCount);
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT] = std::make_unique<Swapchain<DXGI_FORMAT_D32_FLOAT>>(viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight, viewConfs[0].recommendedSwapchainSampleCount);
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT] = std::make_unique<Swapchain<DXGI_FORMAT_D32_FLOAT>>(viewConfs[1].recommendedImageRectWidth, viewConfs[1].recommendedImageRectHeight, viewConfs[1].recommendedSwapchainSampleCount);

    this->m_presentPipelines[OpenXR::EyeSide::LEFT]->BindSettings((float)this->m_swapchains[OpenXR::EyeSide::LEFT]->GetWidth(), (float)this->m_swapchains[OpenXR::EyeSide::LEFT]->GetHeight());
    this->m_presentPipelines[OpenXR::EyeSide::RIGHT]->BindSettings((float)this->m_swapchains[OpenXR::EyeSide::RIGHT]->GetWidth(), (float)this->m_swapchains[OpenXR::EyeSide::RIGHT]->GetHeight());
}

void RND_Renderer::Layer3D::PrepareRendering() {
    checkAssert(m_status == Status::NOT_RENDERING, "Need to finish rendering the previous frame before starting a new one");
    m_status = Status::PREPARING;

    this->m_swapchains[OpenXR::EyeSide::LEFT]->PrepareRendering();
    this->m_swapchains[OpenXR::EyeSide::RIGHT]->PrepareRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->PrepareRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->PrepareRendering();
}

void RND_Renderer::Layer3D::StartRendering() {
    checkAssert(m_status == Status::BINDING, "Haven't attached any textures to the layer yet so there's nothing to start rendering");
    m_status = Status::RENDERING;

    checkAssert((this->m_textures[OpenXR::EyeSide::LEFT] == nullptr && this->m_textures[OpenXR::EyeSide::RIGHT] == nullptr) || (this->m_textures[OpenXR::EyeSide::LEFT] != nullptr && this->m_textures[OpenXR::EyeSide::RIGHT] != nullptr), "Both textures must be either null or not null");
    checkAssert((this->m_depthTextures[OpenXR::EyeSide::LEFT] == nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT] == nullptr) || (this->m_depthTextures[OpenXR::EyeSide::LEFT] != nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT] != nullptr), "Both depth textures must be either null or not null");

    this->m_swapchains[OpenXR::EyeSide::LEFT]->StartRendering();
    this->m_swapchains[OpenXR::EyeSide::RIGHT]->StartRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->StartRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->StartRendering();
}

void RND_Renderer::Layer3D::Render(OpenXR::EyeSide side) {
    ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
    ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
    ID3D12CommandAllocator* allocator = VRManager::instance().D3D12->GetFrameAllocator();

    RND_D3D12::CommandContext<false> renderSharedTexture(device, queue, allocator, [this, side](RND_D3D12::CommandContext<false>* context) {
        context->GetRecordList()->SetName(L"RenderSharedTexture");
        Log::print("[{}] texture 0: Waiting for semaphore value {}", side == OpenXR::EyeSide::LEFT ? "left" : "right", 1);
        context->WaitFor(m_textures[side], 1);
        Log::print("[{}] texture 1: Waiting for semaphore value {}", side == OpenXR::EyeSide::LEFT ? "left" : "right", 1);
        context->WaitFor(m_depthTextures[side], 1);
        m_textures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_depthTextures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

        Log::print("[{}] texture 0: Rendering to swapchain texture", side == OpenXR::EyeSide::LEFT ? "left" : "right");
        Log::print("[{}] texture 1: Rendering to swapchain texture", side == OpenXR::EyeSide::LEFT ? "left" : "right");
        m_presentPipelines[side]->BindAttachment(0, m_textures[side]->d3d12GetTexture());
        m_presentPipelines[side]->BindAttachment(1, m_depthTextures[side]->d3d12GetTexture(), DXGI_FORMAT_R32_FLOAT);
        m_presentPipelines[side]->BindTarget(0, m_swapchains[side]->GetTexture(), m_swapchains[side]->GetFormat());
        m_presentPipelines[side]->BindDepthTarget(m_depthSwapchains[side]->GetTexture(), m_depthSwapchains[side]->GetFormat());
        m_presentPipelines[side]->Render(context->GetRecordList(), m_swapchains[side]->GetTexture());

        m_textures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
        m_depthTextures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
        Log::print("[{}] texture 0: Signaling semaphore value {}", side == OpenXR::EyeSide::LEFT ? "left" : "right", 0);
        context->Signal(m_textures[side], 0);
        Log::print("[{}] texture 1: Signaling semaphore value {}", side == OpenXR::EyeSide::LEFT ? "left" : "right", 0);
        context->Signal(m_depthTextures[side], 0);
    });

    this->m_textures[side] = nullptr;
    this->m_depthTextures[side] = nullptr;
}

const std::array<XrCompositionLayerProjectionView, 2>& RND_Renderer::Layer3D::FinishRendering() {
    checkAssert(m_status == Status::RENDERING, "Should have rendered before ending it");
    m_status = Status::NOT_RENDERING;

    this->m_swapchains[OpenXR::EyeSide::LEFT]->FinishRendering();
    this->m_swapchains[OpenXR::EyeSide::RIGHT]->FinishRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->FinishRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->FinishRendering();

    m_projectionViews[OpenXR::EyeSide::LEFT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
        .next = &m_projectionViewsDepthInfo[OpenXR::EyeSide::LEFT],
        .pose = m_currViews[OpenXR::EyeSide::LEFT].pose,
        .fov = m_currViews[OpenXR::EyeSide::LEFT].fov,
        .subImage = {
            .swapchain = this->m_swapchains[OpenXR::EyeSide::LEFT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_swapchains[OpenXR::EyeSide::LEFT]->GetWidth(),
                    .height = (int32_t)this->m_swapchains[OpenXR::EyeSide::LEFT]->GetHeight()
                }
            }
        },
    };
    m_projectionViewsDepthInfo[OpenXR::EyeSide::LEFT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR,
        .subImage = {
            .swapchain = this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->GetWidth(),
                    .height = (int32_t)this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->GetHeight()
                }
            },
        },
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
        .nearZ = 0.1f,
        .farZ = 1000.0f,
    };
    m_projectionViews[OpenXR::EyeSide::RIGHT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
        .next = &m_projectionViewsDepthInfo[OpenXR::EyeSide::RIGHT],
        .pose = m_currViews[OpenXR::EyeSide::RIGHT].pose,
        .fov = m_currViews[OpenXR::EyeSide::RIGHT].fov,
        .subImage = {
            .swapchain = this->m_swapchains[OpenXR::EyeSide::RIGHT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_swapchains[OpenXR::EyeSide::RIGHT]->GetWidth(),
                    .height = (int32_t)this->m_swapchains[OpenXR::EyeSide::RIGHT]->GetHeight()
                }
            }
        },
    };
    m_projectionViewsDepthInfo[OpenXR::EyeSide::RIGHT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR,
        .subImage = {
            .swapchain = this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->GetWidth(),
                    .height = (int32_t)this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->GetHeight()
                }
            },
        },
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
        .nearZ = 0.1f,
        .farZ = 1000.0f,
    };
    return m_projectionViews;
}

RND_Renderer::Layer2D::Layer2D(): Layer() {
    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

    this->m_presentPipeline = std::make_unique<RND_D3D12::PresentPipeline<false>>(VRManager::instance().XR->GetRenderer());

    // note: it's possible to make a swapchain that matches Cemu's internal resolution and let the headset downsample it, although I doubt there's a benefit
    this->m_swapchain = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight, viewConfs[0].recommendedSwapchainSampleCount);

    this->m_presentPipeline->BindSettings((float)this->m_swapchain->GetWidth(), (float)this->m_swapchain->GetHeight());
}

void RND_Renderer::Layer2D::PrepareRendering() {
    checkAssert(m_status == Status::NOT_RENDERING, "Need to finish rendering the previous frame before starting a new one");
    m_status = Status::PREPARING;

    this->m_swapchain->PrepareRendering();
}

void RND_Renderer::Layer2D::StartRendering() {
    checkAssert(m_status == Status::BINDING, "Haven't attached any textures to the layer yet so there's nothing to start rendering");
    m_status = Status::RENDERING;

    checkAssert(this->m_textures[OpenXR::EyeSide::LEFT] != nullptr || this->m_textures[OpenXR::EyeSide::RIGHT] != nullptr, "Shouldn't start rendering when there's no texture to render to this layer!");

    this->m_swapchain->StartRendering();
}

void RND_Renderer::Layer2D::Render(OpenXR::EyeSide side) {
    ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
    ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
    ID3D12CommandAllocator* allocator = VRManager::instance().D3D12->GetFrameAllocator();

    RND_D3D12::CommandContext<false> renderSharedTexture(device, queue, allocator, [this, side](RND_D3D12::CommandContext<false>* context) {
        context->GetRecordList()->SetName(L"RenderSharedTexture");

        // wait for both since we only have one 2D swap buffer to render to
        if (m_textures[OpenXR::EyeSide::LEFT]) {
            Log::print("[left] texture 2: Waiting for semaphore value {}", 1);
            // fixme: Why do we signal to the global command list instead of the local one?!
            context->WaitFor(m_textures[OpenXR::EyeSide::LEFT], 1);
            m_textures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        }
        if (m_textures[OpenXR::EyeSide::RIGHT]) {
            Log::print("[right] texture 2: Waiting for semaphore value {}", 1);
            context->WaitFor(m_textures[OpenXR::EyeSide::RIGHT], 1);
            m_textures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        }

        Log::print("[left] texture 2: Rendering to swapchain texture");
        Log::print("[right] texture 2: Rendering to swapchain texture");
        m_presentPipeline->BindAttachment(0, m_textures[side]->d3d12GetTexture());
        m_presentPipeline->BindTarget(0, m_swapchain->GetTexture(), m_swapchain->GetFormat());
        m_presentPipeline->Render(context->GetRecordList(), m_swapchain->GetTexture());

        if (m_textures[OpenXR::EyeSide::LEFT] != nullptr) {
            Log::print("[left] texture 2: Signaling semaphore value {}", 0);
            m_textures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            context->Signal(m_textures[OpenXR::EyeSide::LEFT], 0);
        }
        if (m_textures[OpenXR::EyeSide::RIGHT] != nullptr) {
            Log::print("[right] texture 2: Signaling semaphore value {}", 0);
            m_textures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            context->Signal(m_textures[OpenXR::EyeSide::RIGHT], 0);
        }
    });
}

constexpr float QUAD_SIZE = 1.0f;
XrCompositionLayerQuad RND_Renderer::Layer2D::FinishRendering() {
    checkAssert(m_status == Status::RENDERING, "Should have rendered before ending it");
    m_status = Status::NOT_RENDERING;

    this->m_swapchain->FinishRendering();

    XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
    xrLocateSpace(VRManager::instance().XR->m_headSpace, VRManager::instance().XR->m_stageSpace, VRManager::instance().XR->GetRenderer()->m_frameState.predictedDisplayTime, &spaceLocation);

    spaceLocation.pose.position.z -= 2.0f;
    spaceLocation.pose.orientation = { 0.0f, 0.0f, 0.0f, 1.0f };

    float aspectRatio = (float)this->m_textures[this->GetCurrentSide()]->d3d12GetTexture()->GetDesc().Width / (float)this->m_textures[this->GetCurrentSide()]->d3d12GetTexture()->GetDesc().Height;

    m_textures[OpenXR::EyeSide::LEFT] = nullptr;
    m_textures[OpenXR::EyeSide::RIGHT] = nullptr;

    float width = aspectRatio > 1.0f ? aspectRatio : 1.0f;
    float height = aspectRatio <= 1.0f ? 1.0f / aspectRatio : 1.0f;

    return {
        .type = XR_TYPE_COMPOSITION_LAYER_QUAD,
        .layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT,
        .space = VRManager::instance().XR->m_stageSpace,
        .eyeVisibility = XR_EYE_VISIBILITY_BOTH,
        .subImage = {
            .swapchain = this->m_swapchain->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_swapchain->GetWidth(),
                    .height = (int32_t)this->m_swapchain->GetHeight()
                }
            }
        },
        .pose = spaceLocation.pose,
        .size = { width*QUAD_SIZE, height*QUAD_SIZE }
    };
}