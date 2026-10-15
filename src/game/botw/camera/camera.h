#pragma once
#include "rendering/tracking.h"

namespace Camera {
    void RegisterHooks();
    void OnGameFrame();

    uint32_t GetFramesSinceLastUpdate();
    glm::mat4 GetReferenceMtx();
    void SetReferenceMtx(const glm::mat4& referenceMtx);
    // the reference matrix is still from the previous frame during the actor calc jobs
    // that also reads the live player matrix has to re-resolve it or it mixes two simulation steps
    glm::mat4 GetFreshReferenceMtx();

    std::pair<glm::fvec3, glm::fquat> ResolveGameplayBasePose(const BESeadLookAtCamera& camera);
    void UpdateReferenceMtx(const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot);

    // a hook should pass the same views to each call so everything comes from one XR frame
    std::pair<glm::fvec3, glm::fquat> BuildMiddlePose(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot);
    std::pair<glm::fvec3, glm::fquat> BuildEyePose(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot, OpenXR::EyeSide side);
    void UpdateDebugEyeViews(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot);

    bool AppliesFirstPersonTurning();
    void RequestSnapTurn(int direction);
    void CancelSnapTurn();
    void SetSmoothTurnInput(float stickDeflection);
    float GetSnapTurnFadeAmount();
}
