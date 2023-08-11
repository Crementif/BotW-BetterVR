#include "renderer.h"
#include "instance.h"
#include "texture.h"
#include "utils/d3d12_utils.h"

#include <glm/glm.hpp>


RND_Renderer::RND_Renderer(XrSession xrSession): m_session(xrSession) {
    XrSessionBeginInfo m_sessionCreateInfo = { XR_TYPE_SESSION_BEGIN_INFO };
    m_sessionCreateInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    checkXRResult(xrBeginSession(m_session, &m_sessionCreateInfo), "Failed to begin OpenXR session!");
}

RND_Renderer::~RND_Renderer() {
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

    if (m_layer3D.GetStatus() == Layer3D::Status3D::NOT_RENDERING) {
        m_layer3D.PrepareRendering(OpenXR::EyeSide::LEFT);
    }
    else if (m_layer3D.GetStatus() == Layer3D::Status3D::LEFT_BINDING_DEPTH) {
        m_layer3D.PrepareRendering(OpenXR::EyeSide::RIGHT);
    }

    if (m_layer2D.GetStatus() == Layer2D::Status2D::NOT_RENDERING) {
        m_layer2D.PrepareRendering();
    }

    // todo: should this update the predicted time for both layers regardless of whether they were rendering?
    m_layer3D.UpdatePredictedTime(OpenXR::EyeSide::LEFT, m_frameState.predictedDisplayTime);
    m_layer3D.UpdatePredictedTime(OpenXR::EyeSide::RIGHT, m_frameState.predictedDisplayTime);
    m_layer2D.UpdatePredictedTime(m_frameState.predictedDisplayTime);
    VRManager::instance().XR->UpdateSpaces(m_frameState.predictedDisplayTime);

    // currently we only support non-AER presenting, aka we render two textures with the same pose and then we present them
    if (CemuHooks::GetSettings().alternatingEyeRenderingSetting == 0) {
        m_layer3D.UpdatePoses(OpenXR::EyeSide::LEFT);
        m_layer3D.UpdatePoses(OpenXR::EyeSide::RIGHT);
    }
    else {
        checkAssert(false, "AER isn't a supported configuration yet!");
    }
}

void RND_Renderer::EndFrame() {
    if (m_layer3D.GetStatus() == Layer3D::Status3D::RIGHT_BINDING_DEPTH) {
        m_layer3D.StartRendering();
    }
    if (m_layer2D.GetStatus() == Layer2D::Status2D::BINDING_COLOR) {
        m_layer2D.StartRendering();
    }

    std::vector<XrCompositionLayerBaseHeader*> compositionLayers;

    // todo: currently ignores m_frameState.shouldRender, but that's probably fine
    XrCompositionLayerQuad layer2D = { XR_TYPE_COMPOSITION_LAYER_QUAD };
    if (m_layer2D.GetStatus() == Layer2D::Status2D::RENDERING) {
        // The HUD/menus aren't eye-specific, so just present the most recent one for both eyes at once
        m_layer2D.Render();
        layer2D = m_layer2D.FinishRendering();
        compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer2D));
    }

    XrCompositionLayerProjection layer3D = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
    std::array<XrCompositionLayerProjectionView, 2> layer3DViews = {};
    if (m_layer3D.GetStatus() == Layer3D::Status3D::RENDERING) {
        m_layer3D.Render(OpenXR::EyeSide::LEFT);
        m_layer3D.Render(OpenXR::EyeSide::RIGHT);
        layer3DViews = m_layer3D.FinishRendering();
        layer3D.layerFlags = NULL;
        layer3D.space = VRManager::instance().XR->m_stageSpace;
        layer3D.viewCount = (uint32_t)layer3DViews.size();
        layer3D.views = layer3DViews.data();
        if (CemuHooks::GetFramesSinceLastCameraUpdate() < 2) {
            compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer3D));
        }
    }

    XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO };
    frameEndInfo.displayTime = m_frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount = (uint32_t)compositionLayers.size();
    frameEndInfo.layers = compositionLayers.data();
    checkXRResult(xrEndFrame(m_session, &frameEndInfo), "Failed to render texture!");

    VRManager::instance().D3D12->EndFrame();
}

RND_Renderer::Layer3D::Layer3D() {
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

RND_Renderer::Layer3D::~Layer3D() {
    for (auto& swapchain : m_swapchains) {
        swapchain.reset();
    }
}

void RND_Renderer::Layer3D::InitTextures(VkExtent2D extent) {
    checkAssert(m_status == Status3D::UNINITIALIZED, "Should only be initialized once!");
    m_status = Status3D::NOT_RENDERING;

    m_textures[OpenXR::EyeSide::LEFT] = std::make_unique<SharedTexture>(extent.width, extent.height, VK_FORMAT_B10G11R11_UFLOAT_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_B10G11R11_UFLOAT_PACK32));
    m_textures[OpenXR::EyeSide::RIGHT] = std::make_unique<SharedTexture>(extent.width, extent.height, VK_FORMAT_B10G11R11_UFLOAT_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_B10G11R11_UFLOAT_PACK32));
    m_depthTextures[OpenXR::EyeSide::LEFT] = std::make_unique<SharedTexture>(extent.width, extent.height, VK_FORMAT_D32_SFLOAT, D3D12Utils::ToDXGIFormat(VK_FORMAT_D32_SFLOAT));
    m_depthTextures[OpenXR::EyeSide::RIGHT] = std::make_unique<SharedTexture>(extent.width, extent.height, VK_FORMAT_D32_SFLOAT, D3D12Utils::ToDXGIFormat(VK_FORMAT_D32_SFLOAT));

    m_textures[OpenXR::EyeSide::LEFT]->d3d12GetTexture()->SetName(L"Layer3D - Left Color Texture");
    m_textures[OpenXR::EyeSide::RIGHT]->d3d12GetTexture()->SetName(L"Layer3D - Right Color Texture");
    m_depthTextures[OpenXR::EyeSide::LEFT]->d3d12GetTexture()->SetName(L"Layer3D - Left Depth Texture");
    m_depthTextures[OpenXR::EyeSide::RIGHT]->d3d12GetTexture()->SetName(L"Layer3D - Right Depth Texture");

    ComPtr<ID3D12CommandAllocator> cmdAllocator;
    {
        ID3D12Device* d3d12Device = VRManager::instance().D3D12->GetDevice();
        ID3D12CommandQueue* d3d12Queue = VRManager::instance().D3D12->GetCommandQueue();
        d3d12Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&cmdAllocator));

        RND_D3D12::CommandContext<true> transitionInitialTextures(d3d12Device, d3d12Queue, cmdAllocator.Get(), [this](RND_D3D12::CommandContext<true>* context) {
            context->GetRecordList()->SetName(L"transitionInitialTextures");
            m_textures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            m_textures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            m_depthTextures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            m_depthTextures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            context->Signal(m_textures[OpenXR::EyeSide::LEFT].get(), 0);
            context->Signal(m_textures[OpenXR::EyeSide::RIGHT].get(), 0);
            context->Signal(m_depthTextures[OpenXR::EyeSide::LEFT].get(), 0);
            context->Signal(m_depthTextures[OpenXR::EyeSide::RIGHT].get(), 0);
        });
    }
}

void RND_Renderer::Layer3D::PrepareRendering(OpenXR::EyeSide side) {
    checkAssert(m_status == Status3D::NOT_RENDERING || m_status == Status3D::LEFT_BINDING_DEPTH, "Need to finish rendering the previous frame before starting a new one");
    m_status = (m_status == Status3D::NOT_RENDERING) ? Status3D::LEFT_PREPARING : Status3D::RIGHT_PREPARING;

    this->m_swapchains[side]->PrepareRendering();
    this->m_depthSwapchains[side]->PrepareRendering();
}

SharedTexture* RND_Renderer::Layer3D::CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image) {
    checkAssert(m_status == Status3D::LEFT_PREPARING || m_status == Status3D::RIGHT_PREPARING, "Need to prepare the layer before adding textures to it");
    m_status = m_status == Status3D::LEFT_PREPARING ? Status3D::LEFT_BINDING_COLOR : Status3D::RIGHT_BINDING_COLOR;

    m_textures[m_status == Status3D::LEFT_BINDING_COLOR ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT]->CopyFromVkImage(copyCmdBuffer, image);
    return m_textures[m_status == Status3D::LEFT_BINDING_COLOR ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT].get();
}

SharedTexture* RND_Renderer::Layer3D::CopyDepthToLayer(VkCommandBuffer copyCmdBuffer, VkImage image) {
    checkAssert(m_status == Status3D::LEFT_BINDING_COLOR || m_status == Status3D::RIGHT_BINDING_COLOR, "Need to prepare the layer before adding textures to it");
    m_status = m_status == Status3D::LEFT_BINDING_COLOR ? Status3D::LEFT_BINDING_DEPTH : Status3D::RIGHT_BINDING_DEPTH;

    m_depthTextures[m_status == Status3D::LEFT_BINDING_DEPTH ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT]->CopyFromVkImage(copyCmdBuffer, image);
    return m_depthTextures[m_status == Status3D::LEFT_BINDING_DEPTH ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT].get();
}

void RND_Renderer::Layer3D::UpdatePoses(OpenXR::EyeSide side) {
    // fixme: allow updating the poses to be updated separately for each eye

    std::array<XrView, 2> views = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
    XrViewLocateInfo viewLocateInfo = { XR_TYPE_VIEW_LOCATE_INFO };
    viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    viewLocateInfo.displayTime = m_predictedTimes[side];
    viewLocateInfo.space = VRManager::instance().XR->m_stageSpace; // locate the rendering views relative to the room, not the headset center
    XrViewState viewState = { XR_TYPE_VIEW_STATE };
    uint32_t viewCount = (uint32_t)views.size();
    checkXRResult(xrLocateViews(VRManager::instance().XR->m_session, &viewLocateInfo, &viewState, viewCount, &viewCount, views.data()), "Failed to get view information!");
    if ((viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0)
        return; // what should occur when the orientation is invalid? keep rendering using old values?

    m_currViews = views;

    // todo: this increases the 3D effect but it also makes the camera clip into the ground and also causes the camera to control weirdly
    // {
    //     glm::fvec3 positions{ (m_currViews[0].pose.position.x, m_currViews[0].pose.position.y, m_currViews[0].pose.position.z) };
    //     positions = positions * 10.0f;
    //     m_currViews[0].pose.position = { positions.x, positions.y, positions.z };
    // }
    // {
    //     glm::fvec3 positions{ (m_currViews[1].pose.position.x, m_currViews[1].pose.position.y, m_currViews[1].pose.position.z) };
    //     positions = positions * 10.0f;
    //     m_currViews[1].pose.position = { positions.x, positions.y, positions.z };
    // }
}

void RND_Renderer::Layer3D::StartRendering() {
    checkAssert(m_status == Status3D::RIGHT_BINDING_DEPTH, "Haven't attached any textures to the layer yet so there's nothing to start rendering");
    m_status = Status3D::RENDERING;

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
        context->WaitFor(m_textures[side].get(), 1);
        context->WaitFor(m_depthTextures[side].get(), 1);
        m_textures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
        m_depthTextures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

        m_presentPipelines[side]->BindAttachment(0, m_textures[side]->d3d12GetTexture());
        m_presentPipelines[side]->BindAttachment(1, m_depthTextures[side]->d3d12GetTexture(), DXGI_FORMAT_R32_FLOAT);
        m_presentPipelines[side]->BindTarget(0, m_swapchains[side]->GetTexture(), m_swapchains[side]->GetFormat());
        m_presentPipelines[side]->BindDepthTarget(m_depthSwapchains[side]->GetTexture(), m_depthSwapchains[side]->GetFormat());
        m_presentPipelines[side]->Render(context->GetRecordList(), m_swapchains[side]->GetTexture());

        m_textures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
        m_depthTextures[side]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
        context->Signal(m_textures[side].get(), 0);
        context->Signal(m_depthTextures[side].get(), 0);
    });
}

const std::array<XrCompositionLayerProjectionView, 2>& RND_Renderer::Layer3D::FinishRendering() {
    checkAssert(m_status == Status3D::RENDERING, "Should have rendered before ending it");
    m_status = Status3D::NOT_RENDERING;

    this->m_swapchains[OpenXR::EyeSide::LEFT]->FinishRendering();
    this->m_swapchains[OpenXR::EyeSide::RIGHT]->FinishRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->FinishRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->FinishRendering();

    // clang-format off
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
    // clang-format on
    return m_projectionViews;
}


RND_Renderer::Layer2D::Layer2D() {
    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

    this->m_presentPipeline = std::make_unique<RND_D3D12::PresentPipeline<false>>(VRManager::instance().XR->GetRenderer());

    // note: it's possible to make a swapchain that matches Cemu's internal resolution and let the headset downsample it, although I doubt there's a benefit
    this->m_swapchain = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight, viewConfs[0].recommendedSwapchainSampleCount);

    this->m_presentPipeline->BindSettings((float)this->m_swapchain->GetWidth(), (float)this->m_swapchain->GetHeight());
}

RND_Renderer::Layer2D::~Layer2D() {
    m_swapchain.reset();
}

void RND_Renderer::Layer2D::InitTextures(VkExtent2D extent) {
    checkAssert(m_status == Status2D::UNINITIALIZED, "Should only be initialized once!");
    m_status = Status2D::NOT_RENDERING;

    m_texture = std::make_unique<SharedTexture>(extent.width, extent.height, VK_FORMAT_A2B10G10R10_UNORM_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_A2B10G10R10_UNORM_PACK32));
    m_texture->d3d12GetTexture()->SetName(L"Layer2D - Color Texture");

    ComPtr<ID3D12CommandAllocator> cmdAllocator;
    {
        ID3D12Device* d3d12Device = VRManager::instance().D3D12->GetDevice();
        ID3D12CommandQueue* d3d12Queue = VRManager::instance().D3D12->GetCommandQueue();
        d3d12Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&cmdAllocator));

        RND_D3D12::CommandContext<true> transitionInitialTextures(d3d12Device, d3d12Queue, cmdAllocator.Get(), [this](RND_D3D12::CommandContext<true>* context) {
            context->GetRecordList()->SetName(L"transitionInitialTextures");
            m_texture->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
            context->Signal(m_texture.get(), 0);
        });
    }
}

void RND_Renderer::Layer2D::PrepareRendering() {
    checkAssert(m_status == Status2D::NOT_RENDERING, "Need to finish rendering the previous frame before starting a new one");
    m_status = Status2D::PREPARING;

    this->m_swapchain->PrepareRendering();
}

SharedTexture* RND_Renderer::Layer2D::CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image) {
    checkAssert(m_status == Status2D::PREPARING, "Need to prepare the layer before acquiring the texture");
    m_status = Status2D::BINDING_COLOR;

    m_texture->CopyFromVkImage(copyCmdBuffer, image);
    return m_texture.get();
}

void RND_Renderer::Layer2D::StartRendering() {
    checkAssert(m_status == Status2D::BINDING_COLOR, "Haven't attached both textures to the layer yet so there's nothing to start rendering");
    m_status = Status2D::RENDERING;

    this->m_swapchain->StartRendering();
}

void RND_Renderer::Layer2D::Render() {
    checkAssert(m_status == Status2D::RENDERING, "There doesn't seem to be a texture to render?");

    ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
    ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
    ID3D12CommandAllocator* allocator = VRManager::instance().D3D12->GetFrameAllocator();

    RND_D3D12::CommandContext<false> renderSharedTexture(device, queue, allocator, [this](RND_D3D12::CommandContext<false>* context) {
        context->GetRecordList()->SetName(L"RenderSharedTexture");

        // wait for both since we only have one 2D swap buffer to render to
        // fixme: Why do we signal to the global command list instead of the local one?!
        context->WaitFor(m_texture.get(), 1);
        m_texture->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

        m_presentPipeline->BindAttachment(0, m_texture->d3d12GetTexture());
        m_presentPipeline->BindTarget(0, m_swapchain->GetTexture(), m_swapchain->GetFormat());
        m_presentPipeline->Render(context->GetRecordList(), m_swapchain->GetTexture());

        m_texture->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
        context->Signal(m_texture.get(), 0);
    });
}

constexpr float DISTANCE = 2.0f;
constexpr float QUAD_SIZE = 1.0f;
XrCompositionLayerQuad RND_Renderer::Layer2D::FinishRendering() {
    checkAssert(m_status == Status2D::RENDERING, "Should have rendered before ending it");
    m_status = Status2D::NOT_RENDERING;

    this->m_swapchain->FinishRendering();

    XrSpaceLocation spaceLocation = { XR_TYPE_SPACE_LOCATION };
    xrLocateSpace(VRManager::instance().XR->m_headSpace, VRManager::instance().XR->m_stageSpace, VRManager::instance().XR->GetRenderer()->m_frameState.predictedDisplayTime, &spaceLocation);

    spaceLocation.pose.position.z -= DISTANCE;
    spaceLocation.pose.orientation = { 0.0f, 0.0f, 0.0f, 1.0f };

    float aspectRatio = (float)this->m_texture->d3d12GetTexture()->GetDesc().Width / (float)this->m_texture->d3d12GetTexture()->GetDesc().Height;

    float width = aspectRatio > 1.0f ? aspectRatio : 1.0f;
    float height = aspectRatio <= 1.0f ? 1.0f / aspectRatio : 1.0f;

    // clang-format off
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
        .size = { width * QUAD_SIZE, height * QUAD_SIZE }
    };
    // clang-format on
}