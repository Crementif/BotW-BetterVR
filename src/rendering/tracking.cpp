#include "pch.h"
#include "tracking.h"
#include "instance.h"

static RND_Renderer* GetRenderer() {
    auto& xr = VRManager::instance().XR;
    return xr ? xr->GetRenderer() : nullptr;
}

glm::fmat4 Tracking::HeadViews::MiddlePose() const {
    return RND_Renderer::MiddlePoseOf(eyes);
}

bool Tracking::IsSessionRunning() {
    const RND_Renderer* renderer = GetRenderer();
    return renderer != nullptr && renderer->IsInitialized();
}

bool Tracking::SupportsPositionalTracking() {
    return VRManager::instance().XR->m_capabilities.supportsPositional;
}

std::optional<Tracking::HeadViews> Tracking::Views() {
    const RND_Renderer* renderer = GetRenderer();
    if (renderer == nullptr) {
        return std::nullopt;
    }
    return renderer->GetPoses().transform([](const std::array<XrView, 2>& views) { return HeadViews{ views }; });
}

std::optional<glm::fmat4> Tracking::HeadPose() {
    return Views().transform([](const HeadViews& views) { return views.MiddlePose(); });
}

std::optional<glm::fvec3> Tracking::EyeGaze() {
    const RND_Renderer* renderer = GetRenderer();
    return renderer != nullptr ? renderer->GetEyeGaze() : std::nullopt;
}

std::optional<glm::fmat4> Tracking::HeadPoseOfCapturedFrame(long frameIdx) {
    const RND_Renderer* renderer = GetRenderer();
    if (renderer == nullptr)
        return std::nullopt;
    return renderer->GetPoses(frameIdx).transform([](const std::array<XrView, 2>& views) { return RND_Renderer::MiddlePoseOf(views); });
}

std::optional<glm::fmat4> Tracking::CameraReferenceOfCapturedFrame(long frameIdx) {
    const RND_Renderer* renderer = GetRenderer();
    return renderer != nullptr ? renderer->GetCameraReferenceMtx(frameIdx) : std::nullopt;
}

OpenXR::InputState Tracking::Hands() {
    return VRManager::instance().XR->m_input.load();
}

OpenXR::InputState Tracking::SampleHands(bool inMenu) {
    if (RND_Renderer* renderer = GetRenderer()) {
        renderer->EnsureFrameInputLatched(inMenu);
    }
    return Hands();
}
