#include "pch.h"
#include "hand_aim.h"
#include "roomscale.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/camera.h"

glm::vec3 HandAim::RemoveHeadsetHorizontalOffset(glm::vec3 trackedPos) {
    if (GameState::IsFirstPerson()) {
        const glm::vec3 appliedHeadPos = Roomscale::GetAppliedHeadPosition();
        trackedPos.x -= appliedHeadPos.x;
        trackedPos.z -= appliedHeadPos.z;
    }
    return trackedPos;
}

static glm::mat4 GetHandCorrectionMtx(OpenXR::EyeSide side) {
    if (side == OpenXR::EyeSide::LEFT) {
        glm::fquat wristL = glm::identity<glm::fquat>();
        wristL *= glm::angleAxis(glm::radians(90.0f), glm::fvec3(0, 1, 0));
        wristL *= glm::angleAxis(glm::radians(-90.0f), glm::fvec3(0, 0, 1));
        wristL *= glm::angleAxis(glm::radians(30.0f), glm::fvec3(0, 0, 1));
        return glm::mat4_cast(wristL);
    }

    glm::fquat wristR = glm::identity<glm::fquat>();
    wristR *= glm::angleAxis(glm::radians(-90.0f), glm::fvec3(0, 0, 1));
    wristR *= glm::angleAxis(glm::radians(-180.0f), glm::fvec3(0, 1, 0));
    wristR *= glm::angleAxis(glm::radians(180.0f), glm::fvec3(1, 0, 0));
    wristR *= glm::angleAxis(glm::radians(30.0f), glm::fvec3(0, 0, 1));
    return glm::mat4_cast(wristR);
}

glm::mat4 HandAim::ResolveCameraReferenceMtx(long frameIdx) {
    if (frameIdx != -1) {
        if (auto frameMtx = Tracking::CameraReferenceOfCapturedFrame(frameIdx)) {
            return frameMtx.value();
        }
    }
    return Camera::GetFreshReferenceMtx();
}

bool HandAim::TryGetControllerWorldPose(const OpenXR::InputState& inputs, OpenXR::EyeSide side, bool useAimPose, glm::vec3& outPosition, glm::vec3& outForward, long frameIdx) {
    if (!inputs.shared.in_game) {
        return false;
    }

    const XrActionStatePose& poseState = useAimPose ? inputs.shared.aimPose[side] : inputs.shared.pose[side];
    if (!poseState.isActive) {
        return false;
    }

    const XrSpaceLocation& pose = useAimPose ? inputs.shared.aimPoseLocation[side] : inputs.shared.poseLocation[side];
    if ((pose.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0 || (pose.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) == 0) {
        return false;
    }

    const glm::vec3 trackedPos = RemoveHeadsetHorizontalOffset(ToGLM(pose.pose.position));
    const glm::quat trackedRot = ToGLM(pose.pose.orientation);
    glm::mat4 controllerMat = glm::translate(glm::identity<glm::mat4>(), trackedPos) * glm::mat4_cast(trackedRot);
    if (!useAimPose) {
        controllerMat *= GetHandCorrectionMtx(side);
    }
    const glm::mat4 controllerWorld = ResolveCameraReferenceMtx(frameIdx) * controllerMat;
    const glm::vec3 worldForward = glm::mat3(controllerWorld) * glm::vec3(0.0f, 0.0f, -1.0f);

    outPosition = glm::vec3(controllerWorld[3]);
    outForward = glm::length2(worldForward) > 0.000001f ? glm::normalize(worldForward) : glm::vec3(0.0f, 0.0f, -1.0f);
    return IsAllFinite(outPosition) && IsAllFinite(outForward);
}
