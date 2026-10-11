#pragma once

#include "pch.h"
#include <memory>
#include "d3d12.h"
#include "openxr.h"
#include "swapchain.h"
#include "texture.h"
#include "utils/debug_draw.h"

class SharedTexture;

// Milliseconds on a monotonic clock, used to time how long the frame pump has been stuck.
inline long long SteadyNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class RND_Renderer {
public:
    explicit RND_Renderer(XrSession xrSession);
    ~RND_Renderer();

    // Begins the OpenXR session. Also used to resume a session the runtime stopped,
    // which is what happens when the headset goes to sleep and wakes up again.
    bool BeginSession();
    void EndSession();
    bool IsSessionRunning() const {
        return m_sessionRunning.load(std::memory_order_acquire);
    }

    // Set by the frame pump once the runtime refuses to produce frames for a session it claims
    // is running; OpenXR responds by rebuilding the session.
    void RequestSessionRestart() { m_needsSessionRestart.store(true, std::memory_order_release); }
    bool NeedsSessionRestart() const { return m_needsSessionRestart.load(std::memory_order_acquire); }
    // Throws the session away without talking to the runtime, which is what a session that is about
    // to be destroyed needs. The frame pump is retired the same way the stuck-pump path retires it,
    // so a thread still blocked inside xrWaitFrame cannot come back at this renderer.
    void AbandonSession() {
        m_sessionRunning.store(false, std::memory_order_release);
        ++m_framePumpToken->generation;
        {
            std::lock_guard<std::mutex> lock(m_framePumpMutex);
            m_framePumpStopRequested = true;
        }
        m_framePumpCv.notify_all();
    }
    // True while xrWaitFrame keeps coming back with a usable frame, used to decide whether a
    // rebuilt session has been healthy long enough to earn another set of attempts.
    bool ProducedFrameRecently() const { return SteadyNowMs() - m_lastSuccessfulWaitMs.load(std::memory_order_relaxed) < 5000; }

    enum CaptureMaskBits : uint32_t {
        CaptureMask_None = 0,
        CaptureMask_ColorLeft = 1u << 0,
        CaptureMask_ColorRight = 1u << 1,
        CaptureMask_DepthLeft = 1u << 2,
        CaptureMask_DepthRight = 1u << 3,
        CaptureMask_Hud = 1u << 4,
    };

    enum CaptureIssueBits : uint32_t {
        CaptureIssue_None = 0,
        CaptureIssue_Duplicate = 1u << 0,
        CaptureIssue_CompetingSource = 1u << 1,
        CaptureIssue_MonoCapture = 1u << 2,
        CaptureIssue_MissingPoseSnapshot = 1u << 3,
        CaptureIssue_UnexpectedSequence = 1u << 4,
    };

    static constexpr uint32_t CaptureIssue_FatalMask =
        CaptureIssue_CompetingSource |
        CaptureIssue_MonoCapture |
        CaptureIssue_MissingPoseSnapshot;

    enum class CaptureRecordKind : uint8_t {
        None,
        HudOnly,
        Stereo3D,
    };

    struct RenderFrame {
        std::optional<std::array<XrView, 2>> views;
        std::optional<glm::fmat4> cameraReferenceMtx;
        CaptureRecordKind recordKind = CaptureRecordKind::None;
        uint64_t activeStereoGeneration = 0;
        uint64_t lastActivityOrdinal = 0;
        uint32_t acceptedCaptureMask = CaptureMask_None;
        uint32_t issueFlags = CaptureIssue_None;
        std::array<VkImage, 2> acceptedColorImages = { VK_NULL_HANDLE, VK_NULL_HANDLE };
        std::array<VkImage, 2> acceptedDepthImages = { VK_NULL_HANDLE, VK_NULL_HANDLE };
        VkImage acceptedHudImage = VK_NULL_HANDLE;

        std::unique_ptr<VulkanTexture> mainFramebuffer;
        std::unique_ptr<VulkanTexture> hudFramebuffer;
        std::unique_ptr<VulkanTexture> hudWithoutAlphaFramebuffer;
        std::unique_ptr<VulkanFramebuffer> imguiFramebuffer;
        VkDescriptorSet mainFramebufferDS = VK_NULL_HANDLE;
        VkDescriptorSet hudFramebufferDS = VK_NULL_HANDLE;
        VkDescriptorSet hudWithoutAlphaFramebufferDS = VK_NULL_HANDLE;
        float mainFramebufferAspectRatio = 1.0f;
        RenderUtils::UvTransform mainFramebufferUvTransform = {};

        bool ranMotionAnalysis[2] = { false, false };

        bool IsStereoRecord() const { return recordKind == CaptureRecordKind::Stereo3D; }
        bool IsHudOnlyRecord() const { return recordKind == CaptureRecordKind::HudOnly; }
        bool HasAcceptedCapture(uint32_t captureMask) const { return (acceptedCaptureMask & captureMask) == captureMask; }
        bool HasFatalIssue() const { return (issueFlags & CaptureIssue_FatalMask) != 0; }
        bool Is3DComplete() const { return IsStereoRecord() && HasAcceptedCapture(CaptureMask_ColorLeft | CaptureMask_ColorRight | CaptureMask_DepthLeft | CaptureMask_DepthRight); }
        bool Is2DComplete() const { return HasAcceptedCapture(CaptureMask_Hud); }

        bool AddIssue(uint32_t issueFlag) {
            if ((issueFlags & issueFlag) != 0)
                return false;
            issueFlags |= issueFlag;
            return true;
        }

        bool TryAcceptCapture(uint32_t captureMask, VkImage image, VkImage& acceptedImage, long frameIdx, const char* roleName, bool competingSourceIsFatal = true) {
            if ((acceptedCaptureMask & captureMask) != 0) {
                if (acceptedImage == image) {
                    AddIssue(CaptureIssue_Duplicate);
                    Log::print<RENDERING>("[{}] Ignoring duplicate {} capture from image {}", frameIdx, roleName, (void*)image);
                }
                else if (competingSourceIsFatal) {
                    AddIssue(CaptureIssue_CompetingSource);
                    Log::print<WARNING>("[{}] Ignoring competing {} capture: {} != {}", frameIdx, roleName, (void*)image, (void*)acceptedImage);
                }
                return false;
            }
            acceptedImage = image;
            acceptedCaptureMask |= captureMask;
            return true;
        }

        void BeginStereoCapture(uint64_t stereoGeneration) {
            Reset();
            recordKind = CaptureRecordKind::Stereo3D;
            activeStereoGeneration = stereoGeneration;
        }

        void BeginHudCapture() {
            Reset();
            recordKind = CaptureRecordKind::HudOnly;
        }

        void Reset() {
            views = std::nullopt;
            cameraReferenceMtx = std::nullopt;
            recordKind = CaptureRecordKind::None;
            activeStereoGeneration = 0;
            lastActivityOrdinal = 0;
            acceptedCaptureMask = CaptureMask_None;
            issueFlags = CaptureIssue_None;
            acceptedColorImages[0] = VK_NULL_HANDLE;
            acceptedColorImages[1] = VK_NULL_HANDLE;
            acceptedDepthImages[0] = VK_NULL_HANDLE;
            acceptedDepthImages[1] = VK_NULL_HANDLE;
            acceptedHudImage = VK_NULL_HANDLE;

            ranMotionAnalysis[0] = false;
            ranMotionAnalysis[1] = false;
        }
    };

    struct Stable3DReference {
        long frameIdx = -1;
        uint64_t stereoGeneration = 0;
        bool valid = false;
        std::array<XrView, 2> views = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
    };

    void StartFrame();
    void EndFrame();
    std::optional<std::array<XrView, 2>> UpdateViews(XrTime predictedDisplayTime);

    bool EnsureFrameViewsLatched() const;
    bool EnsureFrameInputLatched();
    std::optional<XrTime> ComputeInputSampleTime() const;
    bool HasCurrentFrameViewsLatched() const { return !m_frameViewsPending && !m_frameViewLatchFailed && m_currViews.has_value(); }
    bool DidCurrentFrameViewLatchFail() const { return m_frameViewLatchFailed; }

    std::optional<std::array<XrView, 2>> GetPoses(long frameIdx = -1) const {
        if (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) return m_renderFrames[frameIdx].views;
        EnsureFrameViewsLatched();
        return m_currViews;
    }

    std::optional<XrFovf> GetFOV(OpenXR::EyeSide side, long frameIdx = -1) const {
        auto views = GetPoses(frameIdx);
        return views.transform([side](auto& value) { return value[side].fov; });
    }

    std::optional<XrPosef> GetPose(OpenXR::EyeSide side, long frameIdx = -1) const {
        auto views = GetPoses(frameIdx);
        return views.transform([side](auto& value) { return value[side].pose; });
    }

    std::optional<glm::fmat4> GetPoseAsMatrix(OpenXR::EyeSide side, long frameIdx = -1) const {
        auto views = GetPoses(frameIdx);
        return views.transform([side](auto& views) {
            const XrPosef& pose = views[side].pose;
            return ToMat4(ToGLM(pose.position), ToGLM(pose.orientation));
        });
    };

    std::optional<glm::fmat4> GetMiddlePose(long frameIdx = -1) const {
        auto views = GetPoses(frameIdx);
        if (!views.has_value()) return std::nullopt;
        const XrPosef& leftPose = views->at(OpenXR::EyeSide::LEFT).pose;
        const XrPosef& rightPose = views->at(OpenXR::EyeSide::RIGHT).pose;
        glm::fvec3 middlePos = (ToGLM(leftPose.position) + ToGLM(rightPose.position)) * 0.5f;
        glm::quat middleOri = glm::slerp(ToGLM(leftPose.orientation), ToGLM(rightPose.orientation), 0.5f);

        return ToMat4(middlePos, middleOri);
    };

    // nullopt when the frame never latched one, in which case the live reference matrix applies
    std::optional<glm::fmat4> GetCameraReferenceMtx(long frameIdx) const {
        if (frameIdx != -1) {
            return m_renderFrames[frameIdx].cameraReferenceMtx;
        }
        return std::nullopt;
    }

    double GetLastFrameWorkTimeMs() const { return m_lastFrameWorkTimeMs; }
    double GetLastWaitTimeMs() const { return m_lastWaitTimeMs; }
    double GetLastFrameTimeMs() const { return m_lastFrameTimeMs; }
    double GetPredictedDisplayPeriodMs() const { return m_predictedDisplayPeriodMs; }
    double GetLastOverheadMs() const { return m_lastOverheadMs; }

    void On3DColorCopied(OpenXR::EyeSide side, long frameIdx) {
        if (!m_renderFrames[frameIdx].views.has_value() && HasCurrentFrameViewsLatched()) m_renderFrames[frameIdx].views = m_currViews;
        LatchFrameCameraReference(frameIdx);
        DebugDraw::instance().SnapshotEyeState(side, frameIdx);
    }

    void On3DDepthCopied(OpenXR::EyeSide side, long frameIdx) {
        if (!m_renderFrames[frameIdx].views.has_value() && HasCurrentFrameViewsLatched()) m_renderFrames[frameIdx].views = m_currViews;
        LatchFrameCameraReference(frameIdx);
    }

    void On2DCopied(long frameIdx) {
    }

    void LatchFrameCameraReference(long frameIdx);
    void BeginStereoCaptureGeneration(long frameIdx);
    void BeginHudCaptureGeneration(long frameIdx);
    uint64_t NextCaptureActivityOrdinal() { return m_nextCaptureActivityOrdinal++; }
    void InvalidateStable3DForFrame(long frameIdx);
    void PromoteStable3D(long frameIdx);
    void NoteDuplicateCaptureDropped() { ++m_duplicateCaptureDrops; }
    void NoteFatalSlotInvalidation() { ++m_fatalSlotInvalidations; }

    RenderFrame& GetFrame(long frameIdx) { return m_renderFrames[frameIdx]; }
    const RenderFrame& GetFrame(long frameIdx) const { return m_renderFrames[frameIdx]; }

    class Layer3D {
    public:
        explicit Layer3D(RND_Renderer* owner, VkExtent2D inputRes, VkExtent2D outputRes);
        ~Layer3D();

        SharedTexture* CopyColorToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx);
        SharedTexture* CopyDepthToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx);
        void PrepareRendering(OpenXR::EyeSide side);
        void StartRendering();
        void PrepareDebugDraw(const DebugDrawRenderData& debugDrawData);
        void RecordRender(RND_D3D12::CommandContext<false>* context, OpenXR::EyeSide side, long frameIdx, const std::array<XrView, 2>& views, SharedTexture* fadeTexture, const DebugDrawRenderData& debugDrawData);
        const std::array<XrCompositionLayerProjectionView, 2>& FinishRendering(const std::array<XrView, 2>& views);

        float GetAspectRatio(OpenXR::EyeSide side) const { return m_recommendedAspectRatios[side]; }
        const RenderUtils::UvTransform& GetPresentUvTransform(OpenXR::EyeSide side) const { return m_presentUvTransforms[side]; }
        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }
        auto& GetSharedTextures() { return m_textures; }
        auto& GetDepthSharedTextures() { return m_depthTextures; }

        // Called when the session these swapchains were made from is about to be destroyed, which
        // destroys the swapchains themselves along with it.
        void AbandonSwapchains() {
            for (auto& swapchain : m_swapchains) {
                if (swapchain) swapchain->AbandonHandle();
            }
            for (auto& swapchain : m_depthSwapchains) {
                if (swapchain) swapchain->AbandonHandle();
            }
        }

    private:
        // the renderer this layer was built by: while a session is being rebuilt the manager has no
        // renderer at all, so the layer cannot look its owner up there
        RND_Renderer* m_owner;
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>>, 2> m_swapchains;
        std::array<std::unique_ptr<Swapchain<DXGI_FORMAT_D32_FLOAT>>, 2> m_depthSwapchains;
        std::array<std::unique_ptr<RND_D3D12::PresentPipeline<true>>, 2> m_presentPipelines;
        std::array<std::array<std::unique_ptr<SharedTexture>, 2>, 2> m_textures;
        std::array<std::array<std::unique_ptr<SharedTexture>, 2>, 2> m_depthTextures;
        std::array<float, 2> m_recommendedAspectRatios = { 1.0f, 1.0f };
        std::array<RenderUtils::UvTransform, 2> m_presentUvTransforms = {};
        std::unique_ptr<RND_D3D12::DebugDrawPipeline> m_debugDrawPipeline;

        std::array<XrCompositionLayerProjectionView, 2> m_projectionViews = {};
        std::array<XrCompositionLayerDepthInfoKHR, 2> m_projectionViewsDepthInfo = {};

        long m_currentFrameIdx = 0;
    };

    class Layer2D {
    public:
        explicit Layer2D(RND_Renderer* owner, VkExtent2D inputRes, VkExtent2D outputRes);
        ~Layer2D();

        SharedTexture* CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx);
        // AMD GPU FIX: With incrementing values, Vulkan signals odd values (1,3,5...), D3D12 signals even values (2,4,6...)
        // Texture is ready for D3D12 when Vulkan has signaled (odd value > 0)
        bool IsTextureReady(long frameIdx) const {
            uint64_t lastSignal = m_textures[frameIdx]->GetLastSignalledValue();
            return lastSignal > 0 && (lastSignal % 2 == 1);
        };
        void PrepareRendering() const;
        void StartRendering() const;
        void RecordRender(RND_D3D12::CommandContext<false>* context, long frameIdx);
        std::vector<XrCompositionLayerQuad> FinishRendering(XrTime predictedDisplayTime, long frameIdx);
        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }
        auto& GetSharedTextures() { return m_textures; }

        static void SetBowAimingActive(bool isActive) { s_isBowAimingActive = isActive; }
        static bool IsBowAimingActive() { return s_isBowAimingActive; }

        // Called when the session this swapchain was made from is about to be destroyed, which
        // destroys the swapchain itself along with it.
        void AbandonSwapchains() {
            if (m_swapchain) m_swapchain->AbandonHandle();
        }

    private:
        static std::atomic_bool s_isBowAimingActive;
        // the renderer this layer was built by, see Layer3D::m_owner
        RND_Renderer* m_owner;
        std::unique_ptr<Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>> m_swapchain;
        std::unique_ptr<RND_D3D12::PresentPipeline<false>> m_presentPipeline;
        std::array<std::unique_ptr<SharedTexture>, 2> m_textures;

        glm::quat m_currentOrientation = glm::identity<glm::fquat>();

        bool m_isGazeLocked = false;
        uint32_t m_gazeUnlockCounter = 0;
        glm::fvec3 m_lockedGazeForward = glm::fvec3(0.0f, 0.0f, -1.0f);

        long m_currentFrameIdx = 0;
    };

    class ImGuiOverlay {
    public:
        // the overlay is built for one renderer but can be handed to its replacement, so the owner is
        // only used while building: everything after that looks the current renderer up
        explicit ImGuiOverlay(RND_Renderer* owner, VkCommandBuffer cb, VkExtent2D fbRes, VkFormat framebufferFormat);
        ~ImGuiOverlay();

        bool ShouldBlockGameInput() { return ImGui::GetIO().WantCaptureKeyboard; }

        void Update();
        static void Draw3DLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, float aspectRatio, const RenderUtils::UvTransform& uvTransform, long frameIdx);
        static void DrawHUDLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, long frameIdx);
        void Render(long frameIdx, bool renderBackground, bool isDesktopView);
        void DrawAndCopyToImage(VkCommandBuffer cb, VkImage destImage, long frameIdx, bool isDesktopView);
        void DrawHelpMenu();
        void ProcessInputs(OpenXR::InputState& inputs, const VPADStatus& vpadStatus);
        int GetHelpImagePagesCount() const { return m_helpImages.size(); };

    private:
        void DrawPlaystylePage(bool* changed);
        void DrawComfortPage(bool* changed);
        void DrawCombatPage(bool* changed);
        void DrawControlsPage(bool* changed);
        void DrawInterfacePage(bool* changed);
        void DrawSystemPage(bool* changed);
        void DrawGuidePage(const char* icon, const char* title, size_t helpImageIndex);
        void DrawCreditsPage();
        void DrawDebugPage(bool* changed);

        VkDescriptorPool m_descriptorPool;
        VkRenderPass m_renderPass;
        struct HelpImage {
            VulkanTexture* m_image;
            VkDescriptorSet m_imageDS = VK_NULL_HANDLE;
        };
        std::vector<HelpImage> m_helpImages;

        VkSampler m_sampler = VK_NULL_HANDLE;
        VkExtent2D m_outputRes = {};
    };

    std::unique_ptr<Layer3D> m_layer3D;
    std::unique_ptr<Layer2D> m_layer2D;
    std::unique_ptr<ImGuiOverlay> m_imguiOverlay;

    // ImGui, ImGui_ImplVulkan and ImPlot each keep exactly one context per process, so a renderer
    // must never build an overlay next to one that is still alive: CreateContext() does not make a
    // second context the current one, and the older overlay's destructor then destroys the context
    // every ImGui call since uses, which shows up as a null pointer dereference in ImGui::GetIO().
    // The overlay is built from Cemu's game framebuffer and has nothing to do with the OpenXR
    // session, so a renderer that gets replaced hands it to its replacement instead of owning it.
    std::unique_ptr<ImGuiOverlay> ReleaseImGuiOverlay() { return std::move(m_imguiOverlay); }
    void SetImGuiOverlay(std::unique_ptr<ImGuiOverlay> overlay) { m_imguiOverlay = std::move(overlay); }

    // The overlay builds its framebuffers, textures and descriptor sets onto the frames of the
    // renderer it was created for, so those have to travel with the overlay to its replacement:
    // they are plain Vulkan objects of Cemu's device and outlive the OpenXR session.
    void TakeImGuiFrameStateFrom(RND_Renderer& other);

    // The layers own the OpenXR swapchains, and a session that is destroyed takes them with it: a
    // renderer whose session is about to go away has to say goodbye to those handles first, or the
    // swapchain destructors would hand handles of a dead session back to the runtime.
    void AbandonSessionSwapchains() {
        if (m_layer3D) m_layer3D->AbandonSwapchains();
        if (m_layer2D) m_layer2D->AbandonSwapchains();
    }

    float m_gameRenderAspectRatio = 16.0f / 9.0f;

    bool IsRendering3D(long frameIdx) {
        return m_lastPresented3D;
    }
    bool IsRendering2D() {
        return m_presented2DLastFrame;
    }
    bool IsInitialized() {
        return m_isInitialized;
    }
    bool IsGameCapturing3DFrameBuffer() const {
        return m_cameraIsCapturing3DFrameBuffer > 0;
    }
    void SignalGameCapturing3DFrameBuffer(long frameIdx = -1) {
        if (frameIdx >= 0 && frameIdx < (long)m_renderFrames.size()) {
            m_renderFrames[frameIdx].AddIssue(CaptureIssue_MonoCapture);
        }
        m_cameraIsCapturing3DFrameBuffer = 1;
    }

    void SetCustomFadeAmount(float amount) {
        m_customFadeAmount.store(glm::clamp(amount, 0.0f, 1.0f), std::memory_order_relaxed);
    }

    void SetCustomFadeColor(const glm::fvec3& color) {
        m_customFadeColorR.store(glm::clamp(color.x, 0.0f, 1.0f), std::memory_order_relaxed);
        m_customFadeColorG.store(glm::clamp(color.y, 0.0f, 1.0f), std::memory_order_relaxed);
        m_customFadeColorB.store(glm::clamp(color.z, 0.0f, 1.0f), std::memory_order_relaxed);
    }

    struct CustomFade {
        float amount = 0.0f;
        glm::fvec3 color = glm::fvec3(0.0f);
    };

    CustomFade GetCustomFade() const {
        return {
            .amount = m_customFadeAmount.load(std::memory_order_relaxed),
            .color = {
                m_customFadeColorR.load(std::memory_order_relaxed),
                m_customFadeColorG.load(std::memory_order_relaxed),
                m_customFadeColorB.load(std::memory_order_relaxed),
            },
        };
    }

    bool IsFadeActive() const { return m_isFadeActive.load(std::memory_order_relaxed); }

private:
    bool IsCurrent3DPresentationAllowed(const RenderFrame& frame) const;
    bool IsStable3DReuseAllowed(const RenderFrame& frame) const;
    long SelectNewestHudReadyFrame() const;

protected:
    // A pump that is stuck inside a blocked xrWaitFrame can neither be joined nor be allowed to keep
    // calling xrWaitFrame next to its replacement, so it is detached and asked to leave through the
    // generation counter. The token is shared with the retired thread, which keeps the check valid
    // even if the renderer itself is already gone by the time the blocked call finally returns.
    struct FramePumpToken {
        std::atomic<uint32_t> generation = 0;
    };

    void FramePumpLoop(std::shared_ptr<FramePumpToken> token, uint32_t generation);
    void StopFramePump();

    XrSession m_session;
    std::atomic_bool m_sessionRunning = false;
    XrFrameState m_frameState = { XR_TYPE_FRAME_STATE };
    std::chrono::steady_clock::time_point m_frameStateReceivedAt = {};
    XrTime m_lastInputSampleTime = 0;

    std::thread m_framePumpThread;
    std::shared_ptr<FramePumpToken> m_framePumpToken = std::make_shared<FramePumpToken>();
    std::atomic<long long> m_lastPumpActivityMs = 0;
    std::atomic<unsigned long> m_pumpReplacements = 0;
    std::atomic<unsigned long> m_waitFailures = 0;
    std::atomic_bool m_needsSessionRestart = false;
    std::atomic<long long> m_lastSuccessfulWaitMs = 0;
    std::mutex m_framePumpMutex;
    std::condition_variable m_framePumpCv;
    XrFrameState m_pumpedFrameState = { XR_TYPE_FRAME_STATE };
    std::chrono::steady_clock::time_point m_pumpedFrameStateReceivedAt = {};
    bool m_framePumpSlotReady = false;
    bool m_framePumpWaitRequested = false;
    bool m_framePumpStopRequested = false;
    std::atomic_bool m_framePumpThreadAlive = false;
    static constexpr long long kFramePumpStuckMs = 1000;
    // A session that was begun but fails this many waits in a row is not going to recover on its own
    static constexpr unsigned long kMaxWaitFailures = 8;
    // ... and so does this many pumps replaced without a single frame coming back
    static constexpr unsigned long kMaxPumpReplacements = 4;
    bool m_xrFrameActive = false;
    uint64_t m_skippedXrFrameCount = 0;
    mutable std::optional<std::array<XrView, 2>> m_currViews;
    std::array<RenderFrame, 2> m_renderFrames;
    Stable3DReference m_stable3D = {};

    std::atomic_bool m_isInitialized = false;
    std::atomic_bool m_presented2DLastFrame = false;
    std::atomic_bool m_lastPresented3D = false;
    std::atomic_uint8_t m_cameraIsCapturing3DFrameBuffer = 0;
    std::atomic_bool m_isFadeActive = false;
    std::atomic<float> m_customFadeAmount = 0.0f;
    std::atomic<float> m_customFadeColorR = 0.0f;
    std::atomic<float> m_customFadeColorG = 0.0f;
    std::atomic<float> m_customFadeColorB = 0.0f;

    // Full-frame timing derived from OpenXR timestamps (XrTime is in nanoseconds)
    XrTime m_lastPredictedDisplayTime = 0;
    uint64_t m_nextStereoGeneration = 1;
    uint64_t m_nextCaptureActivityOrdinal = 1;
    mutable bool m_frameViewsPending = false;
    bool m_frameInputLatched = false;
    mutable bool m_frameViewLatchFailed = false;
    uint64_t m_current3DPresentedCount = 0;
    uint64_t m_stable3DReusedCount = 0;
    uint64_t m_3DSuppressedCount = 0;
    uint64_t m_duplicateCaptureDrops = 0;
    uint64_t m_fatalSlotInvalidations = 0;

    std::chrono::high_resolution_clock::time_point m_frameStartTime;

    double m_lastFrameWorkTimeMs = 0.0;
    double m_lastWaitTimeMs = 0.0;

    // Derived from OpenXR timestamps
    double m_lastFrameTimeMs = 0.0;
    double m_predictedDisplayPeriodMs = 0.0;
    double m_lastOverheadMs = 0.0;
};
