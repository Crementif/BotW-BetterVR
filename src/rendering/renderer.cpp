#include "pch.h"

#include "renderer.h"
#include "instance.h"
#include "telemetry.h"
#include "texture.h"
#include "hooking/bow.h"
#include "hooking/cemu_hooks.h"
#include "utils/d3d12_utils.h"
#include "utils/ipc_control.h"
#include "utils/render_utils.h"
#include "hooking/imgui_menus.h"

static uint32_t s_frameCounter = 0; // host frame number shared with telemetry/IPC

std::atomic_bool RND_Renderer::Layer2D::s_isBowAimingActive = false;

extern std::atomic<double> g_lastVkPresentMs; // written by the layer's QueuePresentKHR hook
static double s_lastXrEndFrameMs = 0.0;

static float GetSnapTurnFadeAmount() {
    auto* xr = VRManager::instance().XR.get();
    if (xr == nullptr) {
        return 0.0f;
    }

    const uint64_t startNs = xr->m_snapTurnFadeStartNs.load(std::memory_order_relaxed);
    const uint64_t endNs = xr->m_snapTurnFadeUntilNs.load(std::memory_order_relaxed);
    const uint64_t nowNs = GetTimeStamp();
    if (startNs == 0 || endNs <= startNs || nowNs >= endNs) {
        return 0.0f;
    }

    const double durationNs = (double)(endNs - startNs);
    const double t = glm::clamp(((double)(nowNs - startNs)) / durationNs, 0.0, 1.0);
    const double triangle = t <= 0.5 ? (t * 2.0) : ((1.0 - t) * 2.0);
    const double eased = triangle * triangle * (3.0 - (2.0 * triangle));
    return (float)eased;
}

bool RND_Renderer::IsCurrent3DPresentationAllowed(const RenderFrame& frame) const {
    if (!CemuHooks::IsInGame()) {
        return false;
    }
    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return false;
    }
    if (CemuHooks::IsScreenOpen(ScreenId::PauseMenuInfo_00)) {
        return false;
    }
    return (frame.issueFlags & CaptureIssue_MonoCapture) == 0;
}

bool RND_Renderer::IsStable3DReuseAllowed(const RenderFrame& frame) const {
    const bool isMonoFallbackFrame = (frame.issueFlags & CaptureIssue_MonoCapture) != 0;
    if (!isMonoFallbackFrame && !CemuHooks::IsInGame()) {
        return false;
    }
    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return false;
    }
    return !CemuHooks::IsAnyFadeScreenVisible();
}

long RND_Renderer::SelectNewestHudReadyFrame() const {
    long selectedFrameIdx = -1;
    uint64_t newestOrdinal = 0;
    for (long frameIdx = 0; frameIdx < (long)m_renderFrames.size(); ++frameIdx) {
        const auto& frame = m_renderFrames[frameIdx];
        if (!frame.Is2DComplete()) {
            continue;
        }
        if (selectedFrameIdx == -1 || frame.lastActivityOrdinal > newestOrdinal) {
            selectedFrameIdx = frameIdx;
            newestOrdinal = frame.lastActivityOrdinal;
        }
    }
    return selectedFrameIdx;
}

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

    if (m_layer3D) {
        m_layer3D.reset();
    }
    if (m_layer2D) {
        m_layer2D.reset();
    }
}

void RND_Renderer::StartFrame() {
    bool shouldEnableProfiler = false;
    if (auto* xr = VRManager::instance().XR.get(); xr != nullptr) {
        const bool isMenuOpen = xr->m_isMenuOpen.load(std::memory_order_relaxed);
        const uint8_t currentTab = xr->m_currMenuTab.load(std::memory_order_relaxed);
        const bool isDebugTabOpen = isMenuOpen && GetSettings().enableDebuggerTools.load(std::memory_order_relaxed) && currentTab == ImGuiMenus::DEBUG_TAB;
        const bool isProfilerTabOpen = isMenuOpen && currentTab == ImGuiMenus::FPS_OVERLAY_TAB;
        const PerformanceOverlayMode performanceOverlay = GetSettings().performanceOverlay.load(std::memory_order_relaxed);
        const bool showsProfilerOverlay = !isMenuOpen && performanceOverlay == PerformanceOverlayMode::WINDOW_AND_VR_WITH_PROFILER;
        shouldEnableProfiler = isDebugTabOpen || isProfilerTabOpen || showsProfilerOverlay;
    }

    BetterVRProfiler::SetEnabled(shouldEnableProfiler);
    BetterVRProfiler::AdvanceFrame();
    m_isInitialized = true;

    // runtime control channel: poll for commands before this frame's flag push so a new
    // skip mask takes effect on the same frame it was requested
    ++s_frameCounter;
    IpcControl::instance().Tick(s_frameCounter);

    // push the host-controlled performance flags into PPC-visible memory from here (and not
    // only from hook_UpdateSettings) so they reliably reach the game on every screen,
    // including menus/title screens where parts of the game's frame loop don't run
    CemuHooks::setMemory<uint32_t>(0x10416BF8, CemuHooks::GetEffectiveRenderSkipMask());
    CemuHooks::setMemory<uint32_t>(0x10416BFC, CemuHooks::ShouldUseSynthesizedRightEye() ? 1u : 0u);

    XrFrameWaitInfo waitFrameInfo = { XR_TYPE_FRAME_WAIT_INFO };
    auto waitStart = std::chrono::high_resolution_clock::now();
    checkXRResult(xrWaitFrame(m_session, &waitFrameInfo, &m_frameState), "Failed to wait for next frame!");
    m_lastWaitTimeMs = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - waitStart).count();

    // Runtime predicted cadence
    m_predictedDisplayPeriodMs = (double)m_frameState.predictedDisplayPeriod / 1e6;

    // "Frame" as the runtime sees it: delta between predicted display times
    if (m_lastPredictedDisplayTime != 0 && m_frameState.predictedDisplayTime > m_lastPredictedDisplayTime) {
        const XrTime deltaNs = m_frameState.predictedDisplayTime - m_lastPredictedDisplayTime;
        m_lastFrameTimeMs = (double)deltaNs / 1e6;

        // Overhead beyond the runtime cadence (missed interval / late frame, etc.)
        const double overheadMs = m_lastFrameTimeMs - m_predictedDisplayPeriodMs;
        m_lastOverheadMs = overheadMs > 0.0 ? overheadMs : 0.0;
    }
    m_lastPredictedDisplayTime = m_frameState.predictedDisplayTime;

    m_frameStartTime = std::chrono::high_resolution_clock::now();

    XrFrameBeginInfo beginFrameInfo = { XR_TYPE_FRAME_BEGIN_INFO };
    checkXRResult(xrBeginFrame(m_session, &beginFrameInfo), "Couldn't begin OpenXR frame!");

    VRManager::instance().D3D12->StartFrame();
    VRManager::instance().XR->UpdateSpaces(m_frameState.predictedDisplayTime);
    m_frameViewsPending = true;
    m_frameInputsPending = true;
    m_frameViewLatchFailed = false;
    m_frameInputLatchFailed = false;
}


void RND_Renderer::EndFrame() {
    static uint32_t s_endFrameCount = 0;
    s_endFrameCount++;

    std::vector<XrCompositionLayerBaseHeader*> compositionLayers;

    m_presented2DLastFrame = false;
    m_lastPresented3D = false;

    XrCompositionLayerProjection layer3D = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
    std::array<XrCompositionLayerProjectionView, 2> layer3DViews = {};
    std::vector<XrCompositionLayerQuad> layer2DQuads;

    const long hudFrameIdx = SelectNewestHudReadyFrame();

    QueueBowAimingArcPreview(Layer2D::IsBowAimingActive(), hudFrameIdx);
    DebugDrawRenderData debugDrawData = hudFrameIdx != -1 ? DebugDraw::instance().TakeRenderData(hudFrameIdx) : DebugDraw::instance().TakeRenderData();

    const bool isFadeVisible = CemuHooks::IsAnyFadeScreenVisible();
    const float roomscaleFade = CemuHooks::GetRoomscaleFadeAmount();
    const float snapTurnFade = GetSnapTurnFadeAmount();
    m_isFadeActive.store(isFadeVisible, std::memory_order_relaxed);
    SetCustomFadeColor(glm::fvec3(0.0f));
    SetCustomFadeAmount(std::max(roomscaleFade, snapTurnFade));

    bool reusedStable3D = false;
    long presented3DFrameIdx = -1;
    const std::array<XrView, 2>* presented3DViews = nullptr;
    bool presented3DSynthRight = false;
    SynthRightEyeInfo presented3DSynthInfo = {};
    bool projectionPrepared = false;

    if (hudFrameIdx != -1) {
        RenderFrame& hudFrame = m_renderFrames[hudFrameIdx];

        bool currentFrameViewLatchFailed = false;
        if (hudFrame.Is3DComplete() && !hudFrame.views.has_value()) {
            currentFrameViewLatchFailed = !EnsureFrameViewsLatched();
        }

        if (hudFrame.Is3DComplete() && !hudFrame.views.has_value() && currentFrameViewLatchFailed) {
            if (hudFrame.AddIssue(CaptureIssue_MissingPoseSnapshot)) {
                NoteFatalSlotInvalidation();
            }
        }

        if (m_layer3D != nullptr) {
            if (hudFrame.Is3DComplete() && !hudFrame.HasFatalIssue() && hudFrame.views.has_value() && IsCurrent3DPresentationAllowed(hudFrame)) {
                presented3DFrameIdx = hudFrameIdx;
                presented3DViews = &hudFrame.views.value();
                presented3DSynthRight = hudFrame.synthesizedRight;
                if (presented3DSynthRight) {
                    presented3DSynthInfo.hasMatrix = hudFrame.synthReprojMtx.has_value();
                    presented3DSynthInfo.reprojMtx = hudFrame.synthReprojMtx.value_or(glm::fmat4(1.0f));
                    presented3DSynthInfo.sourceSide = hudFrame.synthSourceSide;
                }
            }
            else if (m_stable3D.valid && IsStable3DReuseAllowed(hudFrame)) {
                presented3DFrameIdx = m_stable3D.frameIdx;
                presented3DViews = &m_stable3D.views;
                presented3DSynthRight = m_stable3D.synthesizedRight;
                presented3DSynthInfo = m_stable3D.synthInfo;
                reusedStable3D = true;
            }
        }

        const bool shouldRender2D = m_layer2D != nullptr;
        const bool shouldRender3D = presented3DFrameIdx != -1 && presented3DViews != nullptr;

        if (shouldRender2D) {
            m_layer2D->PrepareRendering();
        }
        if (shouldRender3D) {
            m_layer3D->PrepareRendering(OpenXR::EyeSide::LEFT);
            m_layer3D->PrepareRendering(OpenXR::EyeSide::RIGHT);
        }

        if (shouldRender2D || shouldRender3D) {
            if (shouldRender2D) {
                m_layer2D->StartRendering();
            }
            if (shouldRender3D) {
                m_layer3D->StartRendering();
                m_layer3D->PrepareDebugDraw(debugDrawData);
            }

            SharedTexture* fadeTexture = shouldRender2D ? m_layer2D->GetSharedTextures()[hudFrameIdx].get() : nullptr;
            const SynthRightEyeInfo* synthRightEye = presented3DSynthRight ? &presented3DSynthInfo : nullptr;
            RND_D3D12::CommandContext<false> renderFrame(VRManager::instance().D3D12.get(), [this, shouldRender2D, shouldRender3D, hudFrameIdx, presented3DFrameIdx, presented3DViews, fadeTexture, &debugDrawData, synthRightEye](RND_D3D12::CommandContext<false>* context) {
                D3D12_SET_NAME(context->GetRecordList(), L"RenderXRFrame");

                if (shouldRender2D) {
                    m_layer2D->RecordRender(context, hudFrameIdx);
                }
                if (shouldRender3D) {
                    m_layer3D->RecordRender(context, OpenXR::EyeSide::LEFT, presented3DFrameIdx, *presented3DViews, fadeTexture, debugDrawData);
                    m_layer3D->RecordRender(context, OpenXR::EyeSide::RIGHT, presented3DFrameIdx, *presented3DViews, fadeTexture, debugDrawData, synthRightEye);
                    // Inspect the exact images submitted to OpenXR. This deliberately
                    // happens after both present passes so synthesized-right output and
                    // presentation-shader defects cannot hide behind clean source images.
                    m_layer3D->RecordFinalTelemetry(context);
                }
            });
        }

        // on-demand full eye-texture dump: runs after the frame's render commands were
        // submitted (in-order queue) and only consumes the request when 3D textures exist
        if (presented3DFrameIdx != -1 && IpcControl::instance().ConsumeDumpRequest()) {
            EyeTelemetry::instance().DumpEyeTextures(VRManager::instance().D3D12.get(),
                m_layer3D->GetFinalColor(OpenXR::EyeSide::LEFT),
                m_layer3D->GetFinalColor(OpenXR::EyeSide::RIGHT),
                s_frameCounter);
            EyeTelemetry::instance().DumpEyeTextures(VRManager::instance().D3D12.get(),
                m_layer3D->GetSharedColor(OpenXR::EyeSide::LEFT, presented3DFrameIdx),
                m_layer3D->GetSharedColor(OpenXR::EyeSide::RIGHT, presented3DFrameIdx),
                s_frameCounter, D3D12_RESOURCE_STATE_COMMON, "source");
        }

        if (shouldRender2D) {
            layer2DQuads = m_layer2D->FinishRendering(m_frameState.predictedDisplayTime, hudFrameIdx);
            m_presented2DLastFrame = !layer2DQuads.empty();
        }

        if (shouldRender3D) {
            layer3DViews = m_layer3D->FinishRendering(*presented3DViews);
            m_lastProjectionViews = layer3DViews;
            m_hasLastProjectionViews = true;
            layer3D.layerFlags = 0;
            layer3D.space = VRManager::instance().XR->m_stageSpace;
            layer3D.viewCount = (uint32_t)layer3DViews.size();
            layer3D.views = layer3DViews.data();
            projectionPrepared = true;

            if (reusedStable3D) {
                ++m_stable3DReusedCount;
            }
            else {
                ++m_current3DPresentedCount;
                PromoteStable3D(hudFrameIdx);
            }
        }

        hudFrame.Reset();
    }

    // A new game stereo capture can temporarily invalidate the shared-texture
    // reference while its left/right/depth pieces arrive. Submitting zero projection
    // layers during that gap makes the runtime alternate the world with black/HUD-only
    // frames. Re-submit the last released OpenXR projection images instead; OpenXR
    // swapchain subimages remain valid until replaced by a later release.
    const bool projectionExpected = m_layer3D != nullptr && CemuHooks::IsInGame() &&
        !CemuHooks::UseBlackBarsDuringEvents() && !isFadeVisible;
    if (!projectionPrepared && projectionExpected && m_hasLastProjectionViews) {
        layer3DViews = m_lastProjectionViews;
        layer3D.layerFlags = 0;
        layer3D.space = VRManager::instance().XR->m_stageSpace;
        layer3D.viewCount = (uint32_t)layer3DViews.size();
        layer3D.views = layer3DViews.data();
        projectionPrepared = true;
        ++m_cachedProjectionReuseCount;
    }

    if (projectionPrepared) {
        compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer3D));
        m_lastPresented3D = true;
        ++m_projectionSubmittedCount;
        m_consecutiveProjectionOmissions = 0;
    }
    else if (m_layer3D != nullptr) {
        ++m_3DSuppressedCount;
        if (projectionExpected) {
            ++m_projectionOmittedWhenExpectedCount;
            ++m_consecutiveProjectionOmissions;
            m_maxConsecutiveProjectionOmissions = std::max(m_maxConsecutiveProjectionOmissions, m_consecutiveProjectionOmissions);
        }
    }

    // Add 2D after projection so HUD quads remain on top, including cached-world frames.
    for (auto& layer : layer2DQuads) {
        compositionLayers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
    }

    ++m_compositorFrameCount;
    if (m_projectionPresenceInitialized && m_lastProjectionPresence != projectionPrepared) {
        ++m_projectionPresenceTransitions;
    }
    m_projectionPresenceInitialized = true;
    m_lastProjectionPresence = projectionPrepared;
    m_lastCompositionLayerCount = (uint32_t)compositionLayers.size();
    if (compositionLayers.empty()) {
        ++m_emptyCompositionFrameCount;
    }
    else if (!projectionPrepared && !layer2DQuads.empty()) {
        ++m_twoDOnlyCompositionFrameCount;
    }

    // decrement camera capture counter since its active only for a few frames
    if (m_cameraIsCapturing3DFrameBuffer > 0) {
        --m_cameraIsCapturing3DFrameBuffer;
    }

    m_lastFrameWorkTimeMs = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - m_frameStartTime).count();

    // Benchmark logging is deliberately gated to stable gameplay. Menu/title frames run at
    // the runtime cadence and would otherwise contaminate the first in-game window, while
    // fades can include transition-only stalls. Require 90 stable in-game frames
    // before arming, and reset the window whenever stability is lost.
    // "frame" is the delta between OpenXR predicted display times (the game/present cadence),
    // "work" is host time spent inside the frame, "xrWait" is time blocked in xrWaitFrame
    // (headroom left before the headset cadence throttles the game).
    {
        static double s_benchFrameAccum = 0.0;
        static double s_benchWorkAccum = 0.0;
        static double s_benchWaitAccum = 0.0;
        static double s_benchXrEndAccum = 0.0;
        static double s_benchVkPresentAccum = 0.0;
        static uint32_t s_benchSamples = 0;
        static uint32_t s_stableGameplayFrames = 0;
        static bool s_benchmarkArmed = false;
        bool sampledThisFrame = false;

        const auto resetWindow = [&]() {
            s_benchFrameAccum = 0.0;
            s_benchWorkAccum = 0.0;
            s_benchWaitAccum = 0.0;
            s_benchXrEndAccum = 0.0;
            s_benchVkPresentAccum = 0.0;
            s_benchSamples = 0;
        };

        const bool stableGameplay = CemuHooks::IsInGame() && !CemuHooks::IsAnyFadeScreenVisible();
        if (!stableGameplay) {
            s_stableGameplayFrames = 0;
            s_benchmarkArmed = false;
            resetWindow();
        }
        else if (!s_benchmarkArmed) {
            constexpr uint32_t kGameplayWarmupFrames = 90;
            if (++s_stableGameplayFrames >= kGameplayWarmupFrames) {
                s_benchmarkArmed = true;
                resetWindow();
                Log::print<INFO>("[bench] armed after {} stable in-game frames", kGameplayWarmupFrames);
            }
        }
        else {
            s_benchFrameAccum += m_lastFrameTimeMs;
            s_benchWorkAccum += m_lastFrameWorkTimeMs;
            s_benchWaitAccum += m_lastWaitTimeMs;
            s_benchXrEndAccum += s_lastXrEndFrameMs;
            s_benchVkPresentAccum += g_lastVkPresentMs.load(std::memory_order_relaxed);
            sampledThisFrame = true;
        }

        if (sampledThisFrame && ++s_benchSamples >= 600) {
            const double invSamples = 1.0 / (double)s_benchSamples;
            const double avgFrameMs = s_benchFrameAccum * invSamples;
            const auto telemetryL = EyeTelemetry::instance().GetStats(OpenXR::EyeSide::LEFT);
            const auto telemetryR = EyeTelemetry::instance().GetStats(OpenXR::EyeSide::RIGHT);
            Log::print<INFO>("[bench] {} frames: avg frame {:.2f} ms ({:.1f} FPS), avg work {:.2f} ms, avg xrWait {:.2f} ms, avg xrEnd {:.2f} ms, avg vkPresent {:.2f} ms, synthRightEye={}, skipDrc={}, mask=0x{:X}, whiteL={} whiteR={} eyeL={} eyeR={}",
                s_benchSamples, avgFrameMs, avgFrameMs > 0.0 ? 1000.0 / avgFrameMs : 0.0,
                s_benchWorkAccum * invSamples, s_benchWaitAccum * invSamples,
                s_benchXrEndAccum * invSamples, s_benchVkPresentAccum * invSamples,
                CemuHooks::ShouldUseSynthesizedRightEye() ? "on" : "off",
                GetSettings().ShouldSkipDrcRendering() ? "on" : "off",
                CemuHooks::GetEffectiveRenderSkipMask(),
                telemetryL.whiteFrames, telemetryR.whiteFrames,
                EyeTelemetry::StateName(telemetryL.state), EyeTelemetry::StateName(telemetryR.state));
            resetWindow();
        }
    }

    XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO };
    frameEndInfo.displayTime = m_frameState.predictedDisplayTime;
    frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    frameEndInfo.layerCount = (uint32_t)compositionLayers.size();
    frameEndInfo.layers = compositionLayers.data();

    if (s_endFrameCount % 500 == 0) {
        Log::print<INTEROP>("EndFrame #{}: hudFrame={}, layers={}, 3D={}, 2D={}, current3D={}, reused3D={}, suppressed3D={}, dupDrops={}, fatalSlots={}",
            s_endFrameCount, hudFrameIdx, compositionLayers.size(),
            m_lastPresented3D ? "yes" : "no",
            m_presented2DLastFrame ? "yes" : "no",
            m_current3DPresentedCount,
            m_stable3DReusedCount,
            m_3DSuppressedCount,
            m_duplicateCaptureDrops,
            m_fatalSlotInvalidations);
    }

    const auto xrEndStart = std::chrono::high_resolution_clock::now();
    XrResult xrResult = xrEndFrame(m_session, &frameEndInfo);
    s_lastXrEndFrameMs = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - xrEndStart).count();
    if (XR_FAILED(xrResult)) {
        Log::print<ERROR>("xrEndFrame #{} FAILED with result {}", s_endFrameCount, (int)xrResult);
    }

    VRManager::instance().D3D12->EndFrame();

    // stamp this frame's telemetry samples with the fence value D3D12->EndFrame just
    // signaled (covers all queue work) and decode any samples whose fence completed
    auto* d3d12 = VRManager::instance().D3D12.get();
    EyeTelemetry::instance().OnFrameSubmitted(d3d12->GetLastSignaledFenceValue(), s_frameCounter, CemuHooks::GetEffectiveRenderSkipMask());
    EyeTelemetry::instance().Collect(d3d12->GetFenceCompletedValue());
}

void RND_Renderer::LatchFrameCameraReference(long frameIdx) {
    auto& frame = m_renderFrames[frameIdx];
    if (!frame.cameraReferenceMtx.has_value() && frame.views.has_value()) {
        frame.cameraReferenceMtx = CemuHooks::s_lastCameraMtx;
    }
}

bool RND_Renderer::EnsureFrameViewsLatched() const {
    if (!m_frameViewsPending) {
        return !m_frameViewLatchFailed && m_currViews.has_value();
    }

    auto* self = const_cast<RND_Renderer*>(this);
    self->m_frameViewsPending = false;
    self->m_frameViewLatchFailed = false;

    if (!self->UpdateViews(self->m_frameState.predictedDisplayTime).has_value()) {
        self->m_frameViewLatchFailed = true;
        return false;
    }

    for (auto& frame : self->m_renderFrames) {
        if (!frame.views.has_value() && frame.IsStereoRecord() && frame.activeStereoGeneration != 0 && frame.acceptedCaptureMask != CaptureMask_None) {
            frame.views = self->m_currViews;
            if (!frame.cameraReferenceMtx.has_value()) {
                frame.cameraReferenceMtx = CemuHooks::s_lastCameraMtx;
            }
        }
    }

    return true;
}

bool RND_Renderer::EnsureFrameInputLatched() {
    if (!m_frameInputsPending) {
        return !m_frameInputLatchFailed;
    }

    m_frameInputsPending = false;
    m_frameInputLatchFailed = false;

    if (!EnsureFrameViewsLatched() || !m_currViews.has_value()) {
        m_frameInputLatchFailed = true;
        return false;
    }

    const XrPosef& leftPose = m_currViews->at(OpenXR::EyeSide::LEFT).pose;
    const XrPosef& rightPose = m_currViews->at(OpenXR::EyeSide::RIGHT).pose;
    glm::quat middleOrientation = glm::slerp(ToGLM(leftPose.orientation), ToGLM(rightPose.orientation), 0.5f);

    if (!VRManager::instance().XR->UpdateActions(m_frameState.predictedDisplayTime, middleOrientation, CemuHooks::IsShowingMenu()).has_value()) {
        m_frameInputLatchFailed = true;
        return false;
    }

    return true;
}

void RND_Renderer::BeginStereoCaptureGeneration(long frameIdx) {
    checkAssert(frameIdx >= 0 && frameIdx < (long)m_renderFrames.size(), "Stereo capture frame index is out of range!");
    const bool preserveMonoCapture = (m_renderFrames[frameIdx].issueFlags & CaptureIssue_MonoCapture) != 0;
    if (!preserveMonoCapture) {
        InvalidateStable3DForFrame(frameIdx);
    }
    RenderFrame& frame = m_renderFrames[frameIdx];
    frame.BeginStereoCapture(m_nextStereoGeneration++);
    if (preserveMonoCapture) {
        frame.AddIssue(CaptureIssue_MonoCapture);
    }
    frame.lastActivityOrdinal = NextCaptureActivityOrdinal();
}

void RND_Renderer::BeginHudCaptureGeneration(long frameIdx) {
    checkAssert(frameIdx >= 0 && frameIdx < (long)m_renderFrames.size(), "HUD capture frame index is out of range!");
    RenderFrame& frame = m_renderFrames[frameIdx];
    const bool preserveMonoCapture = (frame.issueFlags & CaptureIssue_MonoCapture) != 0;
    if (frame.recordKind == CaptureRecordKind::None) {
        frame.BeginHudCapture();
        if (preserveMonoCapture) {
            frame.AddIssue(CaptureIssue_MonoCapture);
        }
    }
    frame.lastActivityOrdinal = NextCaptureActivityOrdinal();
}

void RND_Renderer::InvalidateStable3DForFrame(long frameIdx) {
    if (m_stable3D.valid && m_stable3D.frameIdx == frameIdx) {
        m_stable3D.valid = false;
        m_stable3D.frameIdx = -1;
        m_stable3D.stereoGeneration = 0;
    }
}

void RND_Renderer::PromoteStable3D(long frameIdx) {
    checkAssert(frameIdx >= 0 && frameIdx < (long)m_renderFrames.size(), "Stable 3D frame index is out of range!");
    const RenderFrame& frame = m_renderFrames[frameIdx];
    if (!frame.Is3DComplete() || !frame.views.has_value() || frame.HasFatalIssue()) {
        return;
    }

    m_stable3D.frameIdx = frameIdx;
    m_stable3D.stereoGeneration = frame.activeStereoGeneration;
    m_stable3D.valid = true;
    m_stable3D.views = frame.views.value();
    m_stable3D.synthesizedRight = frame.synthesizedRight;
    m_stable3D.synthInfo.hasMatrix = frame.synthReprojMtx.has_value();
    m_stable3D.synthInfo.reprojMtx = frame.synthReprojMtx.value_or(glm::fmat4(1.0f));
    m_stable3D.synthInfo.sourceSide = frame.synthSourceSide;
}

RND_Renderer::Layer3D::Layer3D(VkExtent2D inputRes, VkExtent2D outputRes) {
    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

    const float inputAspectRatio = (float)inputRes.width / (float)inputRes.height;
    this->m_recommendedAspectRatios[OpenXR::EyeSide::LEFT] = inputAspectRatio;
    this->m_recommendedAspectRatios[OpenXR::EyeSide::RIGHT] = inputAspectRatio;

    for (int side = 0; side < 2; ++side) {
        std::optional<XrFovf> rawFov = VRManager::instance().XR->GetRenderer()->GetFOV((OpenXR::EyeSide)side);
        m_presentUvTransforms[side] = RenderUtils::GetPresentationUvTransform(rawFov, inputAspectRatio);
    }

    this->m_presentPipelines[OpenXR::EyeSide::LEFT] = std::make_unique<RND_D3D12::PresentPipeline<true>>(VRManager::instance().XR->GetRenderer());
    this->m_presentPipelines[OpenXR::EyeSide::RIGHT] = std::make_unique<RND_D3D12::PresentPipeline<true>>(VRManager::instance().XR->GetRenderer());
    this->m_debugDrawPipeline = std::make_unique<RND_D3D12::DebugDrawPipeline>();

    this->m_swapchains[OpenXR::EyeSide::LEFT] = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(outputRes.width, outputRes.height, viewConfs[0].recommendedSwapchainSampleCount);
    this->m_swapchains[OpenXR::EyeSide::RIGHT] = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(outputRes.width, outputRes.height, viewConfs[1].recommendedSwapchainSampleCount);
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT] = std::make_unique<Swapchain<DXGI_FORMAT_D32_FLOAT>>(outputRes.width, outputRes.height, viewConfs[0].recommendedSwapchainSampleCount);
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT] = std::make_unique<Swapchain<DXGI_FORMAT_D32_FLOAT>>(outputRes.width, outputRes.height, viewConfs[1].recommendedSwapchainSampleCount);

    this->m_presentPipelines[OpenXR::EyeSide::LEFT]->BindSettings((float)outputRes.width, (float)outputRes.height, m_presentUvTransforms[OpenXR::EyeSide::LEFT]);
    this->m_presentPipelines[OpenXR::EyeSide::RIGHT]->BindSettings((float)outputRes.width, (float)outputRes.height, m_presentUvTransforms[OpenXR::EyeSide::RIGHT]);

    // initialize textures
    for (int i = 0; i < 2; ++i) {
        this->m_textures[OpenXR::EyeSide::LEFT][i] = std::make_unique<SharedTexture>(inputRes.width, inputRes.height, VK_FORMAT_A2B10G10R10_UNORM_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_A2B10G10R10_UNORM_PACK32));
        this->m_textures[OpenXR::EyeSide::RIGHT][i] = std::make_unique<SharedTexture>(inputRes.width, inputRes.height, VK_FORMAT_A2B10G10R10_UNORM_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_A2B10G10R10_UNORM_PACK32));
        this->m_depthTextures[OpenXR::EyeSide::LEFT][i] = std::make_unique<SharedTexture>(inputRes.width, inputRes.height, VK_FORMAT_D32_SFLOAT, D3D12Utils::ToDXGIFormat(VK_FORMAT_D32_SFLOAT));
        this->m_depthTextures[OpenXR::EyeSide::RIGHT][i] = std::make_unique<SharedTexture>(inputRes.width, inputRes.height, VK_FORMAT_D32_SFLOAT, D3D12Utils::ToDXGIFormat(VK_FORMAT_D32_SFLOAT));

        D3D12_SET_NAME(this->m_textures[OpenXR::EyeSide::LEFT][i]->d3d12GetTexture(), L"Layer3D - Left Color Texture");
        D3D12_SET_NAME(this->m_textures[OpenXR::EyeSide::RIGHT][i]->d3d12GetTexture(), L"Layer3D - Right Color Texture");
        D3D12_SET_NAME(this->m_depthTextures[OpenXR::EyeSide::LEFT][i]->d3d12GetTexture(), L"Layer3D - Left Depth Texture");
        D3D12_SET_NAME(this->m_depthTextures[OpenXR::EyeSide::RIGHT][i]->d3d12GetTexture(), L"Layer3D - Right Depth Texture");
    }
}

RND_Renderer::Layer3D::~Layer3D() {
    for (auto& swapchain : m_swapchains) {
        swapchain.reset();
    }
    for (auto& swapchain : m_depthSwapchains) {
        swapchain.reset();
    }
}

SharedTexture* RND_Renderer::Layer3D::CopyColorToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx) {
    static uint32_t s_copyCount = 0;
    static VkImage s_lastSrcImage = VK_NULL_HANDLE;
    s_copyCount++;

    if (s_lastSrcImage != VK_NULL_HANDLE && s_lastSrcImage != image) {
        Log::print<WARNING>("Layer3D srcImage changed: {} -> {} at copy #{}", (void*)s_lastSrcImage, (void*)image, s_copyCount);
    }
    s_lastSrcImage = image;

    if (s_copyCount % 100 == 0) {
        Log::print<INTEROP>("Layer3D::CopyColorToLayer #{} - side={}, frameIdx={}, srcImage={}", s_copyCount, side == OpenXR::EyeSide::LEFT ? "L" : "R", frameIdx, (void*)image);
    }

    m_currentFrameIdx = frameIdx;
    m_textures[side][frameIdx]->CopyFromVkImage(copyCmdBuffer, image);
    return m_textures[side][frameIdx].get();
}

SharedTexture* RND_Renderer::Layer3D::CopyDepthToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx) {
    m_depthTextures[side][frameIdx]->CopyFromVkImage(copyCmdBuffer, image);
    return m_depthTextures[side][frameIdx].get();
}

void RND_Renderer::Layer3D::PrepareRendering(OpenXR::EyeSide side) {
    // Log::print("Preparing rendering for {} side", side == OpenXR::EyeSide::LEFT ? "left" : "right");
    m_swapchains[side]->PrepareRendering();
    m_depthSwapchains[side]->PrepareRendering();
}

std::optional<std::array<XrView, 2>> RND_Renderer::UpdateViews(XrTime predictedDisplayTime) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::XRLocateViews);

    std::array newViews = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
    XrViewLocateInfo viewLocateInfo = { XR_TYPE_VIEW_LOCATE_INFO };
    viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    viewLocateInfo.displayTime = predictedDisplayTime;
    viewLocateInfo.space = VRManager::instance().XR->m_stageSpace; // locate the rendering views relative to the room, not the headset center
    XrViewState viewState = { XR_TYPE_VIEW_STATE };
    uint32_t viewCount = (uint32_t)newViews.size();
    checkXRResult(xrLocateViews(VRManager::instance().XR->m_session, &viewLocateInfo, &viewState, viewCount, &viewCount, newViews.data()), "Failed to get view information!");
    if ((viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0)
        return std::nullopt; // what should occur when the orientation is invalid? keep rendering using old values?

    m_currViews = newViews;
    return m_currViews;
}

void RND_Renderer::Layer3D::StartRendering() {
    // checkAssert((this->m_textures[OpenXR::EyeSide::LEFT] == nullptr && this->m_textures[OpenXR::EyeSide::RIGHT] == nullptr) || (this->m_textures[OpenXR::EyeSide::LEFT] != nullptr && this->m_textures[OpenXR::EyeSide::RIGHT] != nullptr), "Both textures must be either null or not null");
    // checkAssert((this->m_depthTextures[OpenXR::EyeSide::LEFT] == nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT] == nullptr) || (this->m_depthTextures[OpenXR::EyeSide::LEFT] != nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT] != nullptr), "Both depth textures must be either null or not null");
    // checkAssert((this->m_textures[OpenXR::EyeSide::LEFT][0] == nullptr && this->m_textures[OpenXR::EyeSide::RIGHT][0] == nullptr) || (this->m_textures[OpenXR::EyeSide::LEFT][0] != nullptr && this->m_textures[OpenXR::EyeSide::RIGHT][0] != nullptr), "Both textures must be either null or not null");
    // checkAssert((this->m_depthTextures[OpenXR::EyeSide::LEFT][0] == nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT][0] == nullptr) || (this->m_depthTextures[OpenXR::EyeSide::LEFT][0] != nullptr && this->m_depthTextures[OpenXR::EyeSide::RIGHT][0] != nullptr), "Both depth textures must be either null or not null");

    this->m_swapchains[OpenXR::EyeSide::LEFT]->StartRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::LEFT]->StartRendering();
    this->m_swapchains[OpenXR::EyeSide::RIGHT]->StartRendering();
    this->m_depthSwapchains[OpenXR::EyeSide::RIGHT]->StartRendering();
}

void RND_Renderer::Layer3D::PrepareDebugDraw(const DebugDrawRenderData& debugDrawData) {
    if (m_debugDrawPipeline == nullptr || debugDrawData.IsEmpty()) {
        return;
    }

    m_debugDrawPipeline->UploadRenderData(debugDrawData);
}

void RND_Renderer::Layer3D::RecordRender(RND_D3D12::CommandContext<false>* context, OpenXR::EyeSide side, long frameIdx, const std::array<XrView, 2>& views, SharedTexture* fadeTexture, const DebugDrawRenderData& debugDrawData, const SynthRightEyeInfo* synthRightEye) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::Layer3DRender);

    checkAssert(context != nullptr, "Layer3D render context is missing!");

    // In single-pass mode the game renders one source eye. The other output is a
    // mirror unless the frame carries an explicitly validated reprojection matrix.
    const bool synthMode = synthRightEye != nullptr;
    const OpenXR::EyeSide sourceSide = synthMode ? synthRightEye->sourceSide : side;
    const bool synthesizeThisEye = synthMode && side != sourceSide;

    auto& texture = m_textures[sourceSide][frameIdx];
    auto& depthTexture = m_depthTextures[sourceSide][frameIdx];
    checkAssert(fadeTexture != nullptr, "Layer3D fade texture is missing!");

    m_presentPipelines[side]->SetUvTransform(RenderUtils::GetPresentationUvTransform(
        views[side].fov,
        VRManager::instance().XR->GetRenderer()->m_gameRenderAspectRatio
    ));
    m_presentPipelines[side]->SetReprojection(synthesizeThisEye && synthRightEye->hasMatrix ? &synthRightEye->reprojMtx : nullptr);

    // the left-eye pass in this same command context already waits on and signals the left
    // textures; adding a second wait/signal pair would desync the Vulkan<->D3D12 fence protocol
    if (!synthMode || side == OpenXR::EyeSide::LEFT) {
        context->WaitFor(texture.get(), texture->GetD3D12WaitValue());
        context->WaitFor(depthTexture.get(), depthTexture->GetD3D12WaitValue());
    }

    // swapchains are already in D3D12_RESOURCE_STATE_RENDER_TARGET and depth in D3D12_RESOURCE_STATE_DEPTH_WRITE according to OpenXR spec
    m_presentPipelines[side]->BindAttachment(0, texture->d3d12GetTexture());
    m_presentPipelines[side]->BindAttachment(1, depthTexture->d3d12GetTexture(), DXGI_FORMAT_R32_FLOAT);
    m_presentPipelines[side]->BindAttachment(2, fadeTexture->d3d12GetTexture());
    m_presentPipelines[side]->BindTarget(0, m_swapchains[side]->GetTexture(), m_swapchains[side]->GetFormat());
    m_presentPipelines[side]->BindDepthTarget(m_depthSwapchains[side]->GetTexture(), m_depthSwapchains[side]->GetFormat());
    m_presentPipelines[side]->Render(context->GetRecordList(), m_swapchains[side]->GetTexture());

    if (m_debugDrawPipeline != nullptr && debugDrawData.hasViewProjections[side]) {
        m_debugDrawPipeline->Render(
            side,
            context->GetRecordList(),
            depthTexture->d3d12GetTexture(),
            m_swapchains[side]->GetTexture(),
            m_swapchains[side]->GetFormat(),
            m_depthSwapchains[side]->GetTexture(),
            m_depthSwapchains[side]->GetFormat(),
            debugDrawData,
            debugDrawData.viewProjections[side]
        );
    }

    // no transition needed here as OpenXR requires the swapchain to be returned in RENDER_TARGET/DEPTH_WRITE too
    if (!synthMode || side == OpenXR::EyeSide::RIGHT) {
        context->Signal(texture.get(), texture->GetD3D12SignalValue());
        context->Signal(depthTexture.get(), depthTexture->GetD3D12SignalValue());
    }
    // Log::print("[D3D12 - 3D Layer] Rendering finished");
}

void RND_Renderer::Layer3D::RecordFinalTelemetry(RND_D3D12::CommandContext<false>* context) {
    auto* commandList = context->GetRecordList();
    auto* device = VRManager::instance().D3D12->GetDevice();
    for (int side = 0; side < 2; ++side) {
        ID3D12Resource* output = m_swapchains[side]->GetTexture();
        if (output == nullptr || output->GetDesc().SampleDesc.Count != 1) {
            continue;
        }
        D3D12_RESOURCE_BARRIER toCopy = {};
        toCopy.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        toCopy.Transition.pResource = output;
        toCopy.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        toCopy.Transition.StateBefore = D3D12_RESOURCE_STATE_RENDER_TARGET;
        toCopy.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
        commandList->ResourceBarrier(1, &toCopy);
        EyeTelemetry::instance().RecordSample(device, commandList, side, output);
        std::swap(toCopy.Transition.StateBefore, toCopy.Transition.StateAfter);
        commandList->ResourceBarrier(1, &toCopy);
    }
}

const std::array<XrCompositionLayerProjectionView, 2>& RND_Renderer::Layer3D::FinishRendering(const std::array<XrView, 2>& views) {
    this->m_swapchains[EyeSide::LEFT]->FinishRendering();
    this->m_depthSwapchains[EyeSide::LEFT]->FinishRendering();
    this->m_swapchains[EyeSide::RIGHT]->FinishRendering();
    this->m_depthSwapchains[EyeSide::RIGHT]->FinishRendering();

    // clang-format off
    m_projectionViews[EyeSide::LEFT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
        .next = &m_projectionViewsDepthInfo[EyeSide::LEFT],
        .pose = views[EyeSide::LEFT].pose,
        .fov = views[EyeSide::LEFT].fov,
        .subImage = {
            .swapchain = this->m_swapchains[EyeSide::LEFT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_swapchains[EyeSide::LEFT]->GetWidth(),
                    .height = (int32_t)this->m_swapchains[EyeSide::LEFT]->GetHeight()
                }
            }
        }
    };
    m_projectionViewsDepthInfo[EyeSide::LEFT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR,
        .subImage = {
            .swapchain = this->m_depthSwapchains[EyeSide::LEFT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_depthSwapchains[EyeSide::LEFT]->GetWidth(),
                    .height = (int32_t)this->m_depthSwapchains[EyeSide::LEFT]->GetHeight()
                }
            },
        },
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
        .nearZ = GetSettings().GetZNear(),
        .farZ = GetSettings().GetZFar(),
    };
    m_projectionViews[EyeSide::RIGHT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW,
        .next = &m_projectionViewsDepthInfo[EyeSide::RIGHT],
        .pose = views[EyeSide::RIGHT].pose,
        .fov = views[EyeSide::RIGHT].fov,
        .subImage = {
            .swapchain = this->m_swapchains[EyeSide::RIGHT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_swapchains[EyeSide::RIGHT]->GetWidth(),
                    .height = (int32_t)this->m_swapchains[EyeSide::RIGHT]->GetHeight()
                }
            }
        }
    };
    m_projectionViewsDepthInfo[EyeSide::RIGHT] = {
        .type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR,
        .subImage = {
            .swapchain = this->m_depthSwapchains[EyeSide::RIGHT]->GetHandle(),
            .imageRect = {
                .offset = { 0, 0 },
                .extent = {
                    .width = (int32_t)this->m_depthSwapchains[EyeSide::RIGHT]->GetWidth(),
                    .height = (int32_t)this->m_depthSwapchains[EyeSide::RIGHT]->GetHeight()
                }
            },
        },
        .minDepth = 0.0f,
        .maxDepth = 1.0f,
        .nearZ = GetSettings().GetZNear(),
        .farZ = GetSettings().GetZFar(),
    };
    // clang-format on
    return m_projectionViews;
}


RND_Renderer::Layer2D::Layer2D(VkExtent2D inputRes, VkExtent2D outputRes) {
    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

    this->m_presentPipeline = std::make_unique<RND_D3D12::PresentPipeline<false>>(VRManager::instance().XR->GetRenderer());

    this->m_swapchain = std::make_unique<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>(inputRes.width, inputRes.height, viewConfs[0].recommendedSwapchainSampleCount);

    this->m_presentPipeline->BindSettings(outputRes.width, outputRes.height);

    // initialize textures
    for (int i = 0; i < 2; ++i) {
        this->m_textures[i] = std::make_unique<SharedTexture>(inputRes.width, inputRes.height, VK_FORMAT_A2B10G10R10_UNORM_PACK32, D3D12Utils::ToDXGIFormat(VK_FORMAT_A2B10G10R10_UNORM_PACK32));
        D3D12_SET_NAME(this->m_textures[i]->d3d12GetTexture(), L"Layer2D - Color Texture");
    }

    {
        RND_D3D12::CommandContext<true> transitionInitialTextures(VRManager::instance().D3D12.get(), [this](RND_D3D12::CommandContext<true>* context) {
            D3D12_SET_NAME(context->GetRecordList(), L"transitionInitialTextures");
            for (int i = 0; i < 2; ++i) {
                this->m_textures[i]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COMMON);
            }
        });
    }
}

RND_Renderer::Layer2D::~Layer2D() {
    m_swapchain.reset();
}

SharedTexture* RND_Renderer::Layer2D::CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx) {
    static uint32_t s_copyCount = 0;
    s_copyCount++;
    if (s_copyCount % 100 == 0) {
        Log::print<INTEROP>("Layer2D::CopyColorToLayer #{} - frameIdx={}, srcImage={}", s_copyCount, frameIdx, (void*)image);
    }

    m_currentFrameIdx = frameIdx;
    m_textures[frameIdx]->CopyFromVkImage(copyCmdBuffer, image);
    return m_textures[frameIdx].get();
}

void RND_Renderer::Layer2D::StartRendering() const {
    m_swapchain->StartRendering();
}

void RND_Renderer::Layer2D::PrepareRendering() const {
    m_swapchain->PrepareRendering();
}

void RND_Renderer::Layer2D::RecordRender(RND_D3D12::CommandContext<false>* context, long frameIdx) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::Layer2DRender);

    checkAssert(context != nullptr, "Layer2D render context is missing!");

    // wait for both since we only have one 2D swap buffer to render to
    // fixme: Why do we signal to the global command list instead of the local one?!
    auto& texture = m_textures[frameIdx];
    context->WaitFor(texture.get(), texture->GetD3D12WaitValue());

    m_presentPipeline->BindAttachment(0, texture->d3d12GetTexture());
    m_presentPipeline->BindTarget(0, m_swapchain->GetTexture(), m_swapchain->GetFormat());
    m_presentPipeline->Render(context->GetRecordList(), m_swapchain->GetTexture());

    context->Signal(texture.get(), texture->GetD3D12SignalValue());
}

std::vector<XrCompositionLayerQuad> RND_Renderer::Layer2D::FinishRendering(XrTime predictedDisplayTime, long frameIdx) {
    this->m_swapchain->FinishRendering();

    auto poses = VRManager::instance().XR->GetRenderer()->GetPoses(frameIdx);
    if (!poses.has_value()) {
        return {};
    }

    const XrPosef& leftPose = poses->at(OpenXR::EyeSide::LEFT).pose;
    const XrPosef& rightPose = poses->at(OpenXR::EyeSide::RIGHT).pose;
    glm::vec3 headPosition = (ToGLM(leftPose.position) + ToGLM(rightPose.position)) * 0.5f;
    glm::quat headOrientation = glm::slerp(ToGLM(leftPose.orientation), ToGLM(rightPose.orientation), 0.5f);

    const float DISTANCE = GetSettings().hudDistance;
    constexpr float LERP_SPEED = 0.05f;

    XrPosef layerPose = { { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f } };

    auto inputState = VRManager::instance().XR->m_input.load();
    const bool wasBowAimingSet = IsBowAimingActive();
    const bool isBowAiming = wasBowAimingSet && inputState.shared.in_game;

    if (GetSettings().DoesUIFollowGaze() || isBowAiming) {
        m_currentOrientation = glm::slerp(m_currentOrientation, headOrientation, LERP_SPEED);
        glm::vec3 forwardDirection = headOrientation * glm::vec3(0.0f, 0.0f, -1.0f);

        // calculate new position forwards
        glm::vec3 targetPosition = headPosition + (DISTANCE * forwardDirection);

        // calculate rightward direction
        glm::vec3 rightDirection = glm::normalize(glm::cross(forwardDirection, glm::vec3(0.0f, 1.0f, 0.0f)));

        // recalculate up direction using right and forward direction
        glm::vec3 upDirection = glm::cross(rightDirection, forwardDirection);
        glm::quat userFacingOrientation = glm::quatLookAt(forwardDirection, upDirection);

        layerPose.orientation = ToXR(userFacingOrientation);
        layerPose.position = ToXR(targetPosition);
    }
    else {
        layerPose.position = ToXR(headPosition);
        layerPose.position.z -= DISTANCE;
        layerPose.orientation = { 0.0f, 0.0f, 0.0f, 1.0f };
    }

    //const float aspectRatio = (float)this->m_textures[frameIdx]->d3d12GetTexture()->GetDesc().Width / (float)this->m_textures[frameIdx]->d3d12GetTexture()->GetDesc().Height;
    const float aspectRatio = 16.0f / 9.0f;


    const float width = aspectRatio > 1.0f ? aspectRatio : 1.0f;
    const float height = aspectRatio <= 1.0f ? 1.0f / aspectRatio : 1.0f;

    // todo: change space to head space if we want to follow the head
    const float LAYER_SIZE = GetSettings().hudSize;

    std::vector<XrCompositionLayerQuad> layers;

    // clang-format off
    layers.push_back({
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
        .pose = layerPose,
        .size = { width * LAYER_SIZE, height * LAYER_SIZE }
    });
    // clang-format on

    // render layer twice to visualize the controller positions in debug mode
    auto inputs = VRManager::instance().XR->m_input.load();

    if (!(inputs.shared.in_game && inputs.shared.pose[OpenXR::EyeSide::LEFT].isActive && inputs.shared.pose[OpenXR::EyeSide::RIGHT].isActive)) {
        return layers;
    }

    return layers;
}
