#pragma once

#include "pch.h"
#include "openxr.h"
#include "swapchain.h"
#include "texture.h"
#include "vk_backend.h"
#include "utils/debug_draw.h"
#include "world_guide.h"

class RND_Renderer {
public:
    explicit RND_Renderer(XrSession xrSession);
    ~RND_Renderer();

    bool BeginSession();
    void EndSession();

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
        bool isMonoCapture = false;
        // the game skipped its clear to copy the composed scene out, so the HUD was drawn over the whole picture
        bool hudHoldsScene = false;
        bool isCorrupted = false;
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


        bool IsStereoRecord() const { return recordKind == CaptureRecordKind::Stereo3D; }
        bool HasAcceptedCapture(uint32_t captureMask) const { return (acceptedCaptureMask & captureMask) == captureMask; }
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

        void Reset() {
            views = std::nullopt;
            cameraReferenceMtx = std::nullopt;
            recordKind = CaptureRecordKind::None;
            activeStereoGeneration = 0;
            lastActivityOrdinal = 0;
            acceptedCaptureMask = CaptureMask_None;
            issueFlags = CaptureIssue_None;
            isMonoCapture = false;
            hudHoldsScene = false;
            isCorrupted = false;
            acceptedColorImages[0] = VK_NULL_HANDLE;
            acceptedColorImages[1] = VK_NULL_HANDLE;
            acceptedDepthImages[0] = VK_NULL_HANDLE;
            acceptedDepthImages[1] = VK_NULL_HANDLE;
            acceptedHudImage = VK_NULL_HANDLE;
        }
    };

    struct Stable3DReference {
        long frameIdx = -1;
        uint64_t stereoGeneration = 0;
        bool valid = false;
        std::array<XrView, 2> views = { XrView{ XR_TYPE_VIEW }, XrView{ XR_TYPE_VIEW } };
    };

    void StartFrame();
    void UpdateDisplayPeriodGuess(XrDuration periodNs);
    void EndFrame();
    void FinishGuestFrame();
    std::optional<std::array<XrView, 2>> UpdateViews(XrTime predictedDisplayTime);

    bool EnsureFrameInputLatched(bool inMenu);
    std::optional<XrTime> ComputeInputSampleTime() const;

    // hooks call this from the game's threads and the Vulkan thread, so it returns a copy
    std::optional<std::array<XrView, 2>> GetPoses(long frameIdx = -1) const {
        std::scoped_lock lock(m_frameLatchMutex);
        if (frameIdx != -1 && m_renderFrames[frameIdx].views.has_value()) return m_renderFrames[frameIdx].views;
        LatchFrameViewsLocked();
        return m_currViews;
    }

    // located together with this XR frame's views
    std::optional<glm::fvec3> GetEyeGaze() const {
        std::scoped_lock lock(m_frameLatchMutex);
        if (!LatchFrameViewsLocked()) return std::nullopt;
        return m_currEyeGaze;
    }

    // also written by LatchFrameViewsLocked on the game's threads, so the Vulkan thread has to use these
    void LatchCaptureViews(long frameIdx);
    std::optional<std::array<XrView, 2>> GetCaptureViews(long frameIdx) const {
        std::scoped_lock lock(m_frameLatchMutex);
        return m_renderFrames[frameIdx].views;
    }

    static glm::fmat4 MiddlePoseOf(const std::array<XrView, 2>& views) {
        const XrPosef& leftPose = views[OpenXR::EyeSide::LEFT].pose;
        const XrPosef& rightPose = views[OpenXR::EyeSide::RIGHT].pose;
        glm::fvec3 middlePos = (ToGLM(leftPose.position) + ToGLM(rightPose.position)) * 0.5f;
        glm::quat middleOri = glm::slerp(ToGLM(leftPose.orientation), ToGLM(rightPose.orientation), 0.5f);
        return ToMat4(middlePos, middleOri);
    }

    std::optional<XrFovf> GetFOV(OpenXR::EyeSide side, long frameIdx = -1) const {
        auto views = GetPoses(frameIdx);
        return views.transform([side](auto& value) { return value[side].fov; });
    }

    // nullopt when the frame never latched one, in which case the live reference matrix applies
    std::optional<glm::fmat4> GetCameraReferenceMtx(long frameIdx) const {
        if (frameIdx != -1) {
            std::scoped_lock lock(m_frameLatchMutex);
            return m_renderFrames[frameIdx].cameraReferenceMtx;
        }
        return std::nullopt;
    }

    double GetAverageFrameTimeMs() const { return m_averageFrameTimeMs; }
    float GetDisplayRefreshRate() const;

    void BeginStereoCaptureGeneration(long frameIdx);
    void BeginHudCaptureGeneration(long frameIdx);
    uint64_t NextCaptureActivityOrdinal() { return m_nextCaptureActivityOrdinal++; }

    RenderFrame& GetFrame(long frameIdx) { return m_renderFrames[frameIdx]; }
    const RenderFrame& GetFrame(long frameIdx) const { return m_renderFrames[frameIdx]; }

    struct LayerTarget {
        std::unique_ptr<VulkanTexture> texture;
        std::unique_ptr<FrameSync> sync;

        void Create(VkExtent2D res, VkFormat format, VkImageUsageFlags usage) {
            texture = std::make_unique<VulkanTexture>(res.width, res.height, format, usage);
            sync = std::make_unique<FrameSync>();
        }
    };

    class Layer3D {
    public:
        explicit Layer3D(VkExtent2D inputRes, VkExtent2D outputRes);
        ~Layer3D();

        void InitTextures(VkCommandBuffer cmdBuffer);
        FrameSync* CopyColorToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx, uint32_t layer = 0);
        FrameSync* CopyDepthToLayer(OpenXR::EyeSide side, VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx, uint32_t layer = 0);
        void PrepareRendering(OpenXR::EyeSide side);
        void StartRendering();
        void PrepareDebugDraw(const DebugDrawRenderData& debugDrawData);
        void PrepareWorldGuide(const std::array<XrView, 2>& views);
        void RecordRender(RND_VkBackend::CommandContext<false>* context, OpenXR::EyeSide side, long frameIdx, const std::array<XrView, 2>& views, VulkanTexture* fadeTexture, const DebugDrawRenderData& debugDrawData);
        const std::array<XrCompositionLayerProjectionView, 2>& FinishRendering(const std::array<XrView, 2>& views);

        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }
        LayerTarget& GetColorTarget(OpenXR::EyeSide side, long frameIdx) { return m_colorTargets[side][frameIdx]; }
        LayerTarget& GetDepthTarget(OpenXR::EyeSide side, long frameIdx) { return m_depthTargets[side][frameIdx]; }

    private:
        std::array<std::unique_ptr<Swapchain>, 2> m_swapchains;
        std::array<std::unique_ptr<Swapchain>, 2> m_depthSwapchains;
        std::array<std::unique_ptr<RND_VkBackend::PresentPipeline<true>>, 2> m_presentPipelines;
        std::array<std::array<LayerTarget, 2>, 2> m_colorTargets;
        std::array<std::array<LayerTarget, 2>, 2> m_depthTargets;
        std::array<RenderUtils::UvTransform, 2> m_presentUvTransforms = {};
        std::unique_ptr<RND_VkBackend::DebugDrawPipeline> m_debugDrawPipeline;
        std::unique_ptr<WorldGuide> m_worldGuide;
        bool m_drawWorldGuide = false;

        std::array<XrCompositionLayerProjectionView, 2> m_projectionViews = {};
        std::array<XrCompositionLayerDepthInfoKHR, 2> m_projectionViewsDepthInfo = {};

        long m_currentFrameIdx = 0;
    };

    struct HudQuad {
        glm::fvec3 position;
        glm::fquat orientation;
        glm::fvec2 size;
    };
    static std::optional<HudQuad> GetHudQuad();

    class Layer2D {
    public:
        explicit Layer2D(VkExtent2D inputRes, VkExtent2D outputRes);
        ~Layer2D();

        void InitTextures(VkCommandBuffer cmdBuffer);
        FrameSync* CopyColorToLayer(VkCommandBuffer copyCmdBuffer, VkImage image, long frameIdx);
        void PrepareRendering() const;
        void StartRendering() const;
        void RecordRender(RND_VkBackend::CommandContext<false>* context, long frameIdx);
        std::vector<XrCompositionLayerQuad> FinishRendering(long frameIdx);
        long GetCurrentFrameIdx() const { return m_currentFrameIdx; }
        LayerTarget& GetTarget(long frameIdx) { return m_targets[frameIdx]; }

    private:
        std::unique_ptr<Swapchain> m_swapchain;
        std::unique_ptr<RND_VkBackend::PresentPipeline<false>> m_presentPipeline;
        std::array<LayerTarget, 2> m_targets;

        glm::quat m_currentOrientation = glm::identity<glm::fquat>();

        bool m_isGazeLocked = false;
        uint32_t m_gazeUnlockCounter = 0;
        glm::fvec3 m_lockedGazeForward = glm::fvec3(0.0f, 0.0f, -1.0f);

        long m_currentFrameIdx = 0;
    };

    class ImGuiOverlay {
    public:
        explicit ImGuiOverlay(VkCommandBuffer cb,VkExtent2D fbRes, VkFormat framebufferFormat);
        ~ImGuiOverlay();

        void Update();
        static void Draw3DLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, long frameIdx);
        static void DrawHUDLayerAsBackground(VkCommandBuffer cb, VkImage srcImage, long frameIdx);
        void Render(long frameIdx, bool renderBackground, bool isDesktopView);
        void DrawAndCopyToImage(VkCommandBuffer cb, VkImage destImage, long frameIdx, bool isDesktopView);
        void DrawHelpMenu();
        void ProcessInputs(OpenXR::InputState& inputs, const VPADStatus& vpadStatus);
        bool WantsKeyboard() const;

    private:
        void DrawPlaystylePage(bool* changed);
        void DrawComfortPage(bool* changed);
        void DrawControlsPage(bool* changed);
        void DrawDisplayPage(bool* changed);
        void DrawSystemPage(bool* changed);
        void DrawGuidePage(size_t helpImageIndex);
        void DrawCreditsPage();
        void DrawDebugPage(bool* changed);

        // not necessarily the current one, since Cemu draws its own overlays with ImGui too
        ImGuiContext* m_context = nullptr;
        VkDescriptorPool m_descriptorPool;
        VkRenderPass m_renderPass;
        struct HelpImage {
            VulkanTexture* m_image;
            VkDescriptorSet m_imageDS = VK_NULL_HANDLE;
        };
        std::vector<HelpImage> m_helpImages;

        VkSampler m_sampler = VK_NULL_HANDLE;
        VkExtent2D m_outputRes = {};
        float m_fontScale = 1.0f;
    };

    std::unique_ptr<Layer3D> m_layer3D;
    std::unique_ptr<Layer2D> m_layer2D;
    std::unique_ptr<ImGuiOverlay> m_imguiOverlay;
    // set when the Graphics pack renders the eyes wider than the headset sees, for Cemu's window
    std::optional<float> m_widenedRenderAspectRatio;

    bool IsRendering3D(long frameIdx) {
        return m_lastPresented3D;
    }
    bool IsInitialized() const {
        return m_isInitialized;
    }
    bool IsGameCapturing3DFrameBuffer() const {
        return m_cameraIsCapturing3DFrameBuffer > 0;
    }
    void SignalGameCapturing3DFrameBuffer(long frameIdx = -1) {
        if (frameIdx >= 0 && frameIdx < (long)m_renderFrames.size()) {
            m_renderFrames[frameIdx].AddIssue(CaptureIssue_MonoCapture);
            m_renderFrames[frameIdx].isMonoCapture = true;
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
    bool LatchFrameViewsLocked() const;
    long SelectNewestHudReadyFrame() const;

protected:
    void FramePumpLoop();
    void StopFramePump();

    XrSession m_session;
    bool m_sessionRunning = false;
    XrFrameState m_frameState = { XR_TYPE_FRAME_STATE };
    std::chrono::steady_clock::time_point m_frameStateReceivedAt = {};
    XrTime m_lastInputSampleTime = 0;

    std::thread m_framePumpThread;
    std::mutex m_framePumpMutex;
    std::condition_variable m_framePumpCv;
    XrFrameState m_pumpedFrameState = { XR_TYPE_FRAME_STATE };
    std::chrono::steady_clock::time_point m_pumpedFrameStateReceivedAt = {};
    bool m_framePumpSlotReady = false;
    bool m_framePumpWaitRequested = false;
    bool m_framePumpStopRequested = false;
    bool m_xrFrameActive = false;
    uint64_t m_skippedXrFrameCount = 0;
    // guards the frame state, views and input sample that a frame latches on first use
    mutable std::mutex m_frameLatchMutex;
    uint64_t m_frameSerial = 0;
    mutable std::optional<std::array<XrView, 2>> m_currViews;
    std::optional<glm::fvec3> m_currEyeGaze;
    std::array<RenderFrame, 2> m_renderFrames;
    Stable3DReference m_stable3D = {};
    std::array<XrCompositionLayerProjectionView, 2> m_lastLayer3DViews = {};
    std::vector<XrCompositionLayerQuad> m_lastLayer2DQuads;

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

    // Derived from OpenXR timestamps
    double m_lastFrameTimeMs = 0.0;
    double m_averageFrameTimeMs = 0.0;
    XrTime m_frameTimeWindowNs = 0;
    uint32_t m_frameTimeWindowFrames = 0;
    double m_predictedDisplayPeriodMs = 0.0;
    std::atomic<XrDuration> m_displayPeriodGuessNs = 0;
    double m_lastOverheadMs = 0.0;
};
