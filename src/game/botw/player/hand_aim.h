#pragma once

#include "rendering/tracking.h"

namespace HandAim {
    glm::vec3 RemoveHeadsetHorizontalOffset(glm::vec3 trackedPos);
    // -1 for the current frame
    glm::mat4 ResolveCameraReferenceMtx(long frameIdx);
    bool TryGetControllerWorldPose(const OpenXR::InputState& inputs, OpenXR::EyeSide side, bool useAimPose, glm::vec3& outPosition, glm::vec3& outForward, long frameIdx = -1);
}
