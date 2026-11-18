#pragma once

#include "openxr.h"

// the hands are sampled separately from the head and for a later time, so they never share a timestamp
namespace Tracking {
    struct HeadViews {
        std::array<XrView, 2> eyes;

        const XrPosef& EyePose(OpenXR::EyeSide side) const { return eyes[side].pose; }
        const XrFovf& Fov(OpenXR::EyeSide side) const { return eyes[side].fov; }
        glm::fmat4 MiddlePose() const;
    };

    bool IsSessionRunning();
    bool SupportsPositionalTracking();
    ControllerType ActiveControllerType();

    // take the views once per hook, two calls can end up in different XR frames
    std::optional<HeadViews> Views();
    std::optional<glm::fmat4> HeadPose();
    // the direction the eyes look in, in the same space as the views, while the headset tracks them
    std::optional<glm::fvec3> EyeGaze();

    // -1 is the current frame
    std::optional<glm::fmat4> HeadPoseOfCapturedFrame(long frameIdx);
    std::optional<glm::fmat4> CameraReferenceOfCapturedFrame(long frameIdx);

    OpenXR::InputState Hands();
    OpenXR::InputState SampleHands(bool inMenu);
}
