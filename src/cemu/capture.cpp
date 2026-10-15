#include "pch.h"

#include "capture.h"
#include "instance.h"
#include "game/game.h"
#include "game/botw/pack_shared_memory.h"
#include "utils/vulkan_utils.h"
#include "utils/debug_draw.h"

using EyeSide = OpenXR::EyeSide;

std::optional<Capture::FrameState> Capture::ReadFrameState() {
    if (Game::GetSharedMemoryAddress() == 0) {
        return std::nullopt;
    }

    VRGpuTimelineMemory timelineMem = {};
    Guest::Read(Game::GetSharedMemoryAddress() + offsetof(VRSharedMemoryRoot, gpuTimeline), &timelineMem);
    return FrameState{
        .side = timelineMem.currentEye.getLE() == 0 ? EyeSide::LEFT : EyeSide::RIGHT,
        .frameIdx = (long)(timelineMem.currentFrameCounter.getLE() % 2),
        .monoCapture = timelineMem.monoCapture.getLE() != 0,
    };
}

// whichever capture comes first creates both layers
static void EnsureLayers(RND_Renderer* renderer, VkCommandBuffer commandBuffer, const Capture::Surface& surface) {
    if (renderer->m_layer2D) {
        return;
    }

    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();
    VkExtent2D renderRes = surface.extent;
    VkExtent2D swapchainRes = surface.extent;
    if (VRManager::instance().XR->m_capabilities.isMetaSimulator) {
        swapchainRes = VkExtent2D{ viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight };
    }

    renderer->m_gameRenderAspectRatio = (float)renderRes.width / (float)renderRes.height;
    renderer->m_layer3D = std::make_unique<RND_Renderer::Layer3D>(renderRes, swapchainRes);
    renderer->m_layer2D = std::make_unique<RND_Renderer::Layer2D>(renderRes, swapchainRes);
    renderer->m_layer3D->InitTextures(commandBuffer);
    renderer->m_layer2D->InitTextures(commandBuffer);

    Log::print<INFO>("Found rendering resolution {}x{} @ {}", renderRes.width, renderRes.height, surface.format);
    renderer->m_imguiOverlay = std::make_unique<RND_Renderer::ImGuiOverlay>(commandBuffer, renderRes, VK_FORMAT_A2B10G10R10_UNORM_PACK32);
#ifdef _DEBUG
    VRManager::instance().Debugger = std::make_unique<EntityDebugger>();
#endif
}

static VkClearColorValue ClearValue(bool disableAlpha) {
    return disableAlpha ? VkClearColorValue{ { 0.0f, 0.0f, 0.0f, 1.0f } } : VkClearColorValue{ { 0.0f, 0.0f, 0.0f, 0.0f } };
}

// the GPU timeline flag comes first since the CPU counter can lag behind, the CPU check also catches the pause menu
static bool UsesMonoCapture(const Capture::FrameState& state) {
    return state.monoCapture || Game::IsCapturingMono();
}

// hands the surface back to Cemu, cleared if the game's own clear should still happen
static void Release(VkCommandBuffer commandBuffer, const Capture::Surface& surface, std::optional<VkClearColorValue> clear) {
    if (!clear.has_value()) {
        return;
    }
    const VkImageSubresourceRange range = { surface.aspectMask, 0, 1, surface.sliceIndex, 1 };
    vkroots::LookupDispatch(commandBuffer)->CmdClearColorImage(commandBuffer, surface.image, VK_IMAGE_LAYOUT_GENERAL, &clear.value(), 1, &range);
}

static void AttachSync(FrameSync* sync) {
    if (sync == nullptr) {
        return;
    }
    const uint64_t waitValue = sync->GetWaitValue();
    const uint64_t signalValue = sync->GetSignalValue();
    CemuHost::AttachToNextSubmit(sync->GetSemaphoreForWait(waitValue), waitValue, sync->GetSemaphoreForSignal(signalValue), signalValue);
}

// still clear it like the game asked
static void Skip3DColor(const Capture::FrameState& state, VkCommandBuffer commandBuffer, const Capture::Surface& surface) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (UsesMonoCapture(state)) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }
    Release(commandBuffer, surface, ClearValue(!renderer->IsRendering3D(state.frameIdx)));
}

void Capture::On3DColor(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface, const Surface* rightSurface) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    EnsureLayers(renderer, commandBuffer, surface);
    if (!renderer->IsInitialized()) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }
    checkAssert(renderer->m_layer3D && renderer->m_layer2D, "Couldn't find 3D or 2D layer!");

    const EyeSide side = state.side;
    const long frameIdx = state.frameIdx;
    RND_Renderer::RenderFrame& frame = renderer->GetFrame(frameIdx);

    // don't clear the image if we're in the faux 2D mode
    if (Game::UsesBlackBars()) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }

    if (side == EyeSide::LEFT) {
        if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
            renderer->BeginStereoCaptureGeneration(frameIdx);
        }
    }
    else if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
        frame.isCorrupted = true;
        Log::print<RENDERING>("[{}] Ignoring right-eye 3D color before stereo generation start", frameIdx);
        return Skip3DColor(state, commandBuffer, surface);
    }

    if (frame.isCorrupted) {
        Log::print<RENDERING>("[{}] Ignoring 3D color capture because the stereo slot is already invalid/corrupted", frameIdx);
        return Skip3DColor(state, commandBuffer, surface);
    }

    const uint32_t colorMask = side == EyeSide::LEFT ? RND_Renderer::CaptureMask_ColorLeft : RND_Renderer::CaptureMask_ColorRight;
    if (!frame.TryAcceptCapture(colorMask, surface.image, frame.acceptedColorImages[side], frameIdx, side == EyeSide::LEFT ? "left-eye 3D color" : "right-eye 3D color")) {
        Release(commandBuffer, surface, UsesMonoCapture(state) ? std::nullopt : std::optional(ClearValue(false)));
        return;
    }
    frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();

    auto& layer3D = renderer->m_layer3D;
    AttachSync(layer3D->CopyColorToLayer(side, commandBuffer, surface.image, frameIdx, surface.sliceIndex));
    renderer->LatchCaptureViews(frameIdx);
    DebugDraw::instance().SnapshotEyeState(side, frameIdx);

    // falls back to the left image when there's no right eye copy
    const Surface& right = rightSurface ? *rightSurface : surface;
    if (side == EyeSide::LEFT && frame.TryAcceptCapture(RND_Renderer::CaptureMask_ColorRight, right.image, frame.acceptedColorImages[EyeSide::RIGHT], frameIdx, rightSurface ? "right-eye 3D color" : "right-eye 3D color (left stand-in)")) {
        AttachSync(layer3D->CopyColorToLayer(EyeSide::RIGHT, commandBuffer, right.image, frameIdx, right.sliceIndex));
        DebugDraw::instance().SnapshotEyeState(EyeSide::RIGHT, frameIdx);
    }

    if (UsesMonoCapture(state)) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }

    renderer->m_imguiOverlay->Draw3DLayerAsBackground(commandBuffer, surface.image, layer3D->GetAspectRatio(side), layer3D->GetPresentUvTransform(side), frameIdx);

    // clear the image to be transparent to allow for the HUD to be rendered on top of it which results in a transparent HUD layer
    Release(commandBuffer, surface, ClearValue(false));
}

void Capture::On3DDepth(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface, const Surface* rightSurface) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (!renderer->IsInitialized()) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }

    const EyeSide side = state.side;
    const long frameIdx = state.frameIdx;
    RND_Renderer::RenderFrame& frame = renderer->GetFrame(frameIdx);

    if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
        if (side == EyeSide::LEFT) {
            renderer->BeginStereoCaptureGeneration(frameIdx);
        }
        else {
            frame.isCorrupted = true;
            Log::print<RENDERING>("[{}] Ignoring right-eye depth capture before stereo generation start", frameIdx);
            Release(commandBuffer, surface, std::nullopt);
            return;
        }
    }

    if (frame.isCorrupted) {
        Log::print<RENDERING>("[{}] Ignoring 3D depth capture because the stereo slot is already invalid/corrupted", frameIdx);
        Release(commandBuffer, surface, std::nullopt);
        return;
    }

    const uint32_t depthMask = side == EyeSide::LEFT ? RND_Renderer::CaptureMask_DepthLeft : RND_Renderer::CaptureMask_DepthRight;
    if (!frame.TryAcceptCapture(depthMask, surface.image, frame.acceptedDepthImages[side], frameIdx, side == EyeSide::LEFT ? "left-eye 3D depth" : "right-eye 3D depth")) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }
    frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();

    auto& layer3D = renderer->m_layer3D;
    FrameSync* sync = layer3D->CopyDepthToLayer(side, commandBuffer, surface.image, frameIdx, surface.sliceIndex);
    Log::print<RENDERING>("[Capture] Depth 3D side={}, frameIdx={}, sync={}, counter={}", side == EyeSide::LEFT ? "left" : "right", frameIdx, (void*)sync, sync ? sync->GetWaitValue() : 0);
    renderer->LatchCaptureViews(frameIdx);
    AttachSync(sync);
    const Surface& right = rightSurface ? *rightSurface : surface;
    if (side == EyeSide::LEFT && frame.TryAcceptCapture(RND_Renderer::CaptureMask_DepthRight, right.image, frame.acceptedDepthImages[EyeSide::RIGHT], frameIdx, rightSurface ? "right-eye 3D depth" : "right-eye 3D depth (left stand-in)")) {
        AttachSync(layer3D->CopyDepthToLayer(EyeSide::RIGHT, commandBuffer, right.image, frameIdx, right.sliceIndex));
    }
    Release(commandBuffer, surface, std::nullopt);
}

void Capture::OnHud(const FrameState& state, VkCommandBuffer commandBuffer, const Surface& surface) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    EnsureLayers(renderer, commandBuffer, surface);
    if (!renderer->IsInitialized()) {
        Release(commandBuffer, surface, std::nullopt);
        return;
    }
    checkAssert(renderer->m_layer3D && renderer->m_layer2D, "Couldn't find 3D or 2D layer!");

    const long frameIdx = state.frameIdx;
    RND_Renderer::RenderFrame& frame = renderer->GetFrame(frameIdx);
    auto& imguiOverlay = renderer->m_imguiOverlay;
    const bool hudCopied = frame.Is2DComplete();

    if (state.side == EyeSide::LEFT) {
        if (!frame.IsStereoRecord() && frame.recordKind == RND_Renderer::CaptureRecordKind::None) {
            renderer->BeginHudCaptureGeneration(frameIdx);
        }

        if (!frame.TryAcceptCapture(RND_Renderer::CaptureMask_Hud, surface.image, frame.acceptedHudImage, frameIdx, "HUD", false)) {
            Release(commandBuffer, surface, ClearValue(false));
            return;
        }
        frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();
        const Game::Presentation game = Game::GetPresentation();
        frame.hudHoldsScene = UsesMonoCapture(state) && !game.flatCamera && !game.pauseMenuOpen;

        if (imguiOverlay) {
            imguiOverlay->DrawHUDLayerAsBackground(commandBuffer, surface.image, frameIdx);
            VulkanUtils::DebugPipelineBarrier(commandBuffer);

            imguiOverlay->Update();
            imguiOverlay->Render(frameIdx, false, false);
            imguiOverlay->DrawAndCopyToImage(commandBuffer, surface.image, frameIdx, false);
            VulkanUtils::DebugPipelineBarrier(commandBuffer);
        }

        FrameSync* sync = renderer->m_layer2D->CopyColorToLayer(commandBuffer, surface.image, frameIdx);
        Log::print<RENDERING>("[Capture] HUD 2D side=left, frameIdx={}, sync={}, counter={}", frameIdx, (void*)sync, sync ? sync->GetWaitValue() : 0);

        // there is no right-eye HUD capture, so the desktop pass happens here
        if (imguiOverlay) {
            VulkanUtils::DebugPipelineBarrier(commandBuffer);
            imguiOverlay->Render(frameIdx, true, true);
            imguiOverlay->Update();
            imguiOverlay->DrawAndCopyToImage(commandBuffer, surface.image, frameIdx, true);
        }

        AttachSync(sync);
        Release(commandBuffer, surface, std::nullopt);
        return;
    }

    Log::print<RENDERING>("[Capture] HUD 2D side=right, frameIdx={}", frameIdx);
    if (imguiOverlay) {
        imguiOverlay->Render(frameIdx, true, true);
        imguiOverlay->Update();
        imguiOverlay->DrawAndCopyToImage(commandBuffer, surface.image, frameIdx, true);
        Release(commandBuffer, surface, std::nullopt);
        return;
    }
    Release(commandBuffer, surface, hudCopied ? std::optional(ClearValue(false)) : std::nullopt);
}

void Capture::OnFrameSubmitted() {
    VRManager::instance().XR->ProcessEvents();

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer && renderer->m_layer3D && renderer->m_layer2D && renderer->m_imguiOverlay) {
        if (renderer->IsInitialized()) {
            renderer->EndFrame();
        }
        renderer->StartFrame();
    }
}
