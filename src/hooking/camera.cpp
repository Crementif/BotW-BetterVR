#include "pch.h"

#include "cemu_hooks.h"
#include "instance.h"
#include "rendering/openxr.h"
#include "utils/debug_draw.h"
#include "utils/game_utils.h"
#include "utils/render_utils.h"

bool CemuHooks::UseMonoFrameBufferTemporarilyDuringMenusOrPictures() {
    return IsScreenOpen(ScreenId::PauseMenuInfo_00) || VRManager::instance().XR->GetRenderer()->IsGameCapturing3DFrameBuffer();
}

static std::optional<XrFovf> TryGetRenderFOV(OpenXR::EyeSide side, long frameIdx = -1) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return std::nullopt;
    }

    return RenderUtils::GetRenderFov(renderer->GetFOV(side, frameIdx), renderer->m_gameRenderAspectRatio);
}

void CemuHooks::hook_BeginCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[0] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    Log::print<RENDERING>("");
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);
}

static std::optional<XrFovf> GetGameProjectionFOV(OpenXR::EyeSide side, const BESeadPerspectiveProjection& perspectiveProjection, long frameIdx = -1) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return std::nullopt;
    }

    return RenderUtils::ResolveGameProjectionFov(
        renderer->GetFOV(side, frameIdx),
        renderer->m_gameRenderAspectRatio,
        perspectiveProjection.fovYRadiansOrAngle.getLE(),
        perspectiveProjection.aspect.getLE()
    );
}


float hardcodedSwimOffset = 0.0f;
float hardcodedSwimHeight = 1.73f;
float hardcodedRidingOffset = 0.65f;
float hardcodedCrouchOffset = 0.3f;
float hardcodedClimbWallClearance = 0.35f;

glm::fvec3 s_wsCameraPosition = glm::fvec3();
glm::fquat s_wsCameraRotation = glm::identity<glm::fquat>();
glm::fvec3 s_lastGameplayCameraTarget = glm::fvec3();
bool s_hasGameplayCameraTarget = false;
bool s_isSwimming = false;
bool s_isCrouching = false;
bool s_wasCrouching = false;
float actualCrouchOffset = 0.0f;
std::chrono::steady_clock::time_point crouch_state_change_time;

static float s_baseYawDegrees = 0.0f;
static bool s_hasBaseYaw = false;
static float s_pendingGameYawCorrection = 0.0f;
static float s_pendingGameStickYawDelta = 0.0f;

// the normal points out of the wall, towards Link
static glm::fvec3 s_climbWallPoint = glm::fvec3(0.0f);
static glm::fvec3 s_climbWallNormal = glm::fvec3(0.0f);
static uint64_t s_climbWallUpdateNs = 0;
static glm::fvec3 s_climbWallCameraOffset = glm::fvec3(0.0f);

static int TryConsumePendingSnapTurnDirection() {
    auto* xr = VRManager::instance().XR.get();
    if (xr == nullptr) {
        return 0;
    }

    const uint64_t requestUntilNs = xr->m_pendingSnapTurnRequestUntilNs.load(std::memory_order_relaxed);
    if (requestUntilNs == 0 || GetTimeStamp() > requestUntilNs) {
        xr->m_pendingSnapTurnDirection.store(0, std::memory_order_relaxed);
        xr->m_pendingSnapTurnRequestUntilNs.store(0, std::memory_order_relaxed);
        return 0;
    }

    const int direction = xr->m_pendingSnapTurnDirection.exchange(0, std::memory_order_relaxed);
    if (direction != 0) {
        xr->m_pendingSnapTurnRequestUntilNs.store(0, std::memory_order_relaxed);
    }
    return direction;
}

static float TryConsumePendingSmoothTurnDelta() {
    auto* xr = VRManager::instance().XR.get();
    if (xr == nullptr) {
        return 0.0f;
    }

    static uint64_t s_lastTimeNs = 0;
    const uint64_t nowNs = GetTimeStamp();
    const float dt = s_lastTimeNs == 0 ? 0.0f : (float)(nowNs - s_lastTimeNs) * 1.0e-9f;
    s_lastTimeNs = nowNs;

    const float stickDeflection = xr->m_smoothTurnStickDeflection.load(std::memory_order_relaxed);
    if (stickDeflection == 0.0f || dt <= 0.0f || dt > 0.5f) {
        return 0.0f;
    }

    return stickDeflection * GetSettings().GetSmoothTurnSpeed() * dt;
}

static glm::fvec3 SphericalToCameraTargetVector(const glm::fvec3& sphericalDegrees) {
    const float radius = sphericalDegrees.x;
    const float latRadians = glm::radians(sphericalDegrees.y);
    const float yawRadians = glm::radians(sphericalDegrees.z);
    const float horizontalRadius = radius * std::cos(latRadians);

    return {
        horizontalRadius * std::sin(yawRadians),
        radius * std::sin(latRadians),
        horizontalRadius * std::cos(yawRadians),
    };
}

static void SignalSnapTurnFade() {
    auto* xr = VRManager::instance().XR.get();
    if (xr == nullptr) {
        return;
    }

    constexpr auto kSnapTurnFadeDuration = std::chrono::milliseconds(110);
    const uint64_t startNs = GetTimeStamp();
    xr->m_snapTurnFadeStartNs.store(startNs, std::memory_order_relaxed);
    xr->m_snapTurnFadeUntilNs.store(startNs + (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(kSnapTurnFadeDuration).count(), std::memory_order_relaxed);
}

static float YawDegreesFromRotation(const glm::fquat& rot) {
    const glm::fquat twist = RenderUtils::swingTwistY(rot).second;
    return NormalizeDegrees(glm::degrees(2.0f * std::atan2(twist.y, twist.w)));
}

static bool ShouldAdoptGameCameraYaw() {
    return !s_hasBaseYaw || !CemuHooks::IsFirstPerson() || !CemuHooks::IsInGame() || CemuHooks::HasActiveCutscene();
}

static float ConsumePendingCameraYawDelta() {
    const int32_t snapAngle = GetSettings().GetSnapTurnAngle();
    if (snapAngle <= 0) {
        return TryConsumePendingSmoothTurnDelta();
    }

    const int direction = TryConsumePendingSnapTurnDirection();
    if (direction == 0) {
        return 0.0f;
    }

    SignalSnapTurnFade();
    return (float)direction * (float)snapAngle;
}

static float ConsumePendingGameYawCorrection() {
    const float correction = s_pendingGameYawCorrection;
    s_pendingGameYawCorrection = 0.0f;
    return correction;
}

static float ConsumeGameStickYawDelta() {
    const float delta = s_pendingGameStickYawDelta;
    s_pendingGameStickYawDelta = 0.0f;
    return delta;
}

static bool ApplyYawDeltaToComputedCameraPivot(PPCInterpreter_t* hCPU, float yawDeltaDegrees) {
    if (yawDeltaDegrees == 0.0f || hCPU->gpr[4] == 0 || hCPU->gpr[5] == 0) {
        return false;
    }

    BEVec3 cameraWorldPosBE = {};
    BEVec3 sphericalBE = {};
    CemuHooks::readMemory(hCPU->gpr[4], &cameraWorldPosBE);
    CemuHooks::readMemory(hCPU->gpr[5], &sphericalBE);

    const glm::fvec3 cameraWorldPos = cameraWorldPosBE.getLE();
    glm::fvec3 spherical = sphericalBE.getLE();
    if (!glm::all(glm::isfinite(cameraWorldPos)) || !glm::all(glm::isfinite(spherical)) || spherical.x <= 1.0e-4f) {
        return false;
    }

    const glm::fvec3 cameraToTarget = SphericalToCameraTargetVector(spherical);
    const glm::fvec3 targetWorldPos = cameraWorldPos + cameraToTarget;

    spherical.z = NormalizeDegrees(spherical.z - yawDeltaDegrees);
    const glm::fvec3 rotatedCameraToTarget = SphericalToCameraTargetVector(spherical);
    const glm::fvec3 rotatedCameraWorldPos = targetWorldPos - rotatedCameraToTarget;
    if (!glm::all(glm::isfinite(rotatedCameraWorldPos))) {
        return false;
    }

    BEVec3 rotatedCameraWorldPosBE(rotatedCameraWorldPos.x, rotatedCameraWorldPos.y, rotatedCameraWorldPos.z);
    BEVec3 rotatedSphericalBE(spherical.x, spherical.y, spherical.z);
    CemuHooks::writeMemory(hCPU->gpr[4], &rotatedCameraWorldPosBE);
    CemuHooks::writeMemory(hCPU->gpr[5], &rotatedSphericalBE);
    return true;
}

static glm::fvec3 GetAppliedHeadsetOffset(glm::fvec3 eyePos) {
    if (CemuHooks::IsFirstPerson()) {
        glm::fvec3 appliedHeadPos = CemuHooks::GetAppliedRoomscaleHeadPosition();
        eyePos.x -= appliedHeadPos.x;
        eyePos.z -= appliedHeadPos.z;
    }
    return eyePos;
}

static glm::mat4 GetAppliedHeadsetPose(glm::mat4 pose) {
    if (CemuHooks::IsFirstPerson()) {
        glm::fvec3 appliedHeadPos = CemuHooks::GetAppliedRoomscaleHeadPosition();
        pose[3].x -= appliedHeadPos.x;
        pose[3].z -= appliedHeadPos.z;
    }
    return pose;
}

static std::optional<std::pair<glm::fvec3, glm::fquat>> TryGetAppliedEyePose(OpenXR::EyeSide side) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return std::nullopt;
    }

    auto eyePoseOpt = renderer->GetPose(side);
    if (!eyePoseOpt.has_value()) {
        return std::nullopt;
    }

    return { { GetAppliedHeadsetOffset(ToGLM(eyePoseOpt.value().position)), ToGLM(eyePoseOpt.value().orientation) } };
}

static bool TryUpdateGameplayCameraTarget(const glm::fvec3& cameraPos, const glm::fvec3& cameraTarget, const glm::fvec3& cameraUp) {
    if (!glm::all(glm::isfinite(cameraPos)) || !glm::all(glm::isfinite(cameraTarget)) || !glm::all(glm::isfinite(cameraUp))) {
        return false;
    }

    glm::fvec3 forward = cameraTarget - cameraPos;
    if (glm::dot(forward, forward) <= 1.0e-6f || glm::dot(cameraUp, cameraUp) <= 1.0e-6f) {
        return false;
    }

    s_lastGameplayCameraTarget = cameraTarget;
    s_hasGameplayCameraTarget = true;
    return true;
}

// the two targets only disagree while swimming, where the rendered camera pins the eye to a fixed
// height above the water and the reference anchor keeps following the height setting
enum class HeightAdjustmentTarget {
    ReferenceAnchor,
    RenderedCamera,
};

static float ResolvePlayerHeightAdjustment(HeightAdjustmentTarget target) {
    const float heightOffset = GetSettings().GetPlayerHeightOffset();

    if (CemuHooks::IsRiding(true)) {
        // the height setting is deliberately left out so the saddle keeps the height it had up to 0.9.15
        return -hardcodedRidingOffset;
    }
    if (s_isSwimming) {
        if (target == HeightAdjustmentTarget::RenderedCamera) {
            if (auto* renderer = VRManager::instance().XR->GetRenderer()) {
                if (auto middlePose = renderer->GetMiddlePose()) {
                    return hardcodedSwimHeight - middlePose.value()[3].y;
                }
            }
        }
        return heightOffset + hardcodedSwimOffset;
    }
    return heightOffset - actualCrouchOffset;
}

static glm::fvec3 ResolveClimbWallCameraOffset(const glm::fvec3& playerPos, const glm::fquat& cameraRotation) {
    // the climb sweep runs every frame Link is on a wall, so a stale reading means he already let go
    constexpr uint64_t kClimbWallMaxAgeNs = 100'000'000;
    // guards against a normal left over from an earlier wall, which the game never clears
    constexpr float kClimbWallMaxContactDistance = 3.0f;

    if (s_climbWallUpdateNs == 0 || GetTimeStamp() - s_climbWallUpdateNs > kClimbWallMaxAgeNs) {
        return glm::fvec3(0.0f);
    }
    if (glm::distance(s_climbWallPoint, playerPos) > kClimbWallMaxContactDistance) {
        return glm::fvec3(0.0f);
    }
    // Link is always on the free side of the wall he hangs on, so anything else isn't that wall
    if (glm::dot(playerPos - s_climbWallPoint, s_climbWallNormal) <= 0.0f) {
        return glm::fvec3(0.0f);
    }

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return glm::fvec3(0.0f);
    }

    auto middlePose = renderer->GetMiddlePose();
    if (!middlePose.has_value()) {
        return glm::fvec3(0.0f);
    }

    const glm::fquat baseYaw = RenderUtils::swingTwistY(cameraRotation).second;
    const glm::fvec3 headPos = playerPos + baseYaw * glm::fvec3(GetAppliedHeadsetPose(middlePose.value())[3]);

    const float clearance = glm::dot(headPos - s_climbWallPoint, s_climbWallNormal);
    if (!std::isfinite(clearance) || clearance >= hardcodedClimbWallClearance) {
        return glm::fvec3(0.0f);
    }

    return s_climbWallNormal * (hardcodedClimbWallClearance - clearance);
}

// call once per frame, not per eye, or the two eyes end up offset by different amounts
static void UpdateClimbWallCameraOffset(const glm::fvec3& playerPos, const glm::fquat& cameraRotation) {
    constexpr float kClimbWallOffsetHalfLifeSeconds = 0.06f;

    static uint64_t s_lastTimeNs = 0;
    const uint64_t nowNs = GetTimeStamp();
    const float dt = s_lastTimeNs == 0 ? 0.0f : (float)(nowNs - s_lastTimeNs) * 1.0e-9f;
    s_lastTimeNs = nowNs;

    const glm::fvec3 target = ResolveClimbWallCameraOffset(playerPos, cameraRotation);
    const float blend = (dt <= 0.0f || dt > 0.5f) ? 1.0f : 1.0f - std::exp2(-dt / kClimbWallOffsetHalfLifeSeconds);
    s_climbWallCameraOffset = glm::mix(s_climbWallCameraOffset, target, blend);
}

static glm::fvec3 ResolveGameplayAnchorPosition(const glm::fvec3& gameplayPos) {
    glm::fvec3 newPos = gameplayPos;
    if (CemuHooks::IsFirstPerson()) {
        BEMatrix34 playerMtx = {};
        CemuHooks::readMemory(CemuHooks::s_playerMtxAddress, &playerMtx);
        glm::fvec3 playerPos = playerMtx.getPos().getLE();

        playerPos.y += ResolvePlayerHeightAdjustment(HeightAdjustmentTarget::ReferenceAnchor);

        newPos = playerPos + s_climbWallCameraOffset;
    }

    return newPos;
}

static glm::fquat ResolveCameraBaseYaw(const glm::fquat& gameplayRot, bool allowEventCameraRotationOverride = false) {
    glm::fquat baseYaw = RenderUtils::swingTwistY(gameplayRot).second;

    if (!allowEventCameraRotationOverride || !CemuHooks::IsFirstPerson()) {
        return baseYaw;
    }

    BEMatrix34 playerMtx = {};
    CemuHooks::readMemory(CemuHooks::s_playerMtxAddress, &playerMtx);

    if (auto settings = CemuHooks::GetFirstPersonSettingsForActiveEvent()) {
        if (settings->ignoreCameraRotation) {
            glm::fquat playerRot = playerMtx.getRotLE();
            return RenderUtils::swingTwistY(playerRot).second * glm::angleAxis(glm::radians(180.0f), glm::fvec3(0.0f, 1.0f, 0.0f));
        }
    }

    return baseYaw;
}

static std::pair<glm::vec3, glm::fquat> BuildCameraPoseFromBase(const glm::fvec3& basePos, const glm::fquat& gameplayRot, OpenXR::EyeSide side, bool allowEventCameraRotationOverride = false) {
    glm::fquat baseYaw = ResolveCameraBaseYaw(gameplayRot, allowEventCameraRotationOverride);

    auto eyePoseOpt = TryGetAppliedEyePose(side);
    if (!eyePoseOpt.has_value()) {
        return { basePos, gameplayRot };
    }

    auto [eyePos, eyeRot] = eyePoseOpt.value();
    return { basePos + (baseYaw * eyePos), baseYaw * eyeRot };
}

static std::pair<glm::vec3, glm::fquat> BuildGameplayCameraPose(const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot, OpenXR::EyeSide side) {
    return BuildCameraPoseFromBase(ResolveGameplayAnchorPosition(gameplayPos), gameplayRot, side);
}

static void UpdateDebugEyeViewsFromGameplayPose(const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot) {
    for (uint32_t eyeIndex = 0; eyeIndex < 2; ++eyeIndex) {
        auto [eyePos, eyeRot] = BuildGameplayCameraPose(gameplayPos, gameplayRot, (OpenXR::EyeSide)eyeIndex);
        glm::mat4 eyeWorld = glm::translate(glm::mat4(1.0f), eyePos) * glm::mat4_cast(eyeRot);
        DebugDraw::instance().UpdateEyeView(eyeIndex, glm::inverse(eyeWorld));
    }
}

static void UpdateGameplayReferenceCameraMtx(const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot) {
    glm::fvec3 basePos = ResolveGameplayAnchorPosition(gameplayPos);
    glm::fquat baseYaw = ResolveCameraBaseYaw(gameplayRot);
    CemuHooks::s_lastCameraMtx = glm::translate(glm::identity<glm::fmat4>(), basePos) * glm::mat4(baseYaw);
}

static std::pair<glm::fvec3, glm::fquat> ResolveGameplayBasePose(const BESeadLookAtCamera& camera) {
    glm::mat4x3 viewMatrix = camera.mtx.getLEMatrix();
    glm::mat4 worldGame = glm::inverse(glm::mat4(viewMatrix));
    glm::fvec3 basePos = glm::vec3(worldGame[3]);
    glm::fquat baseRot = glm::quat_cast(worldGame);

    if (!s_hasGameplayCameraTarget || CemuHooks::GetFramesSinceLastCameraUpdate() > 4) {
        return { basePos, baseRot };
    }

    return { s_wsCameraPosition, s_wsCameraRotation };
}

void CemuHooks::hook_UpdateCameraForGameplay(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (UseBlackBarsDuringEvents()) {
        return;
    }

    // read the camera matrix from the game's memory
    uint32_t ppc_cameraMatrixOffsetIn = hCPU->gpr[31];
    ActCamera actCam = {};
    readMemory(ppc_cameraMatrixOffsetIn, &actCam);

    // extract components from the existing camera matrix
    glm::fvec3 oldCameraPosition = actCam.finalCamMtx.pos.getLE();
    glm::fvec3 oldCameraTarget = actCam.finalCamMtx.target.getLE();
    glm::fvec3 oldCameraUp = actCam.finalCamMtx.up.getLE();
    float oldCameraDistance = glm::distance(oldCameraPosition, oldCameraTarget);

    Log::print<RENDERING>("Getting gameplay camera (pos = {})", oldCameraPosition);

    if (IsFirstPerson()) {
        // remove verticality from the camera position to avoid pitch changes that aren't from the VR headset
        oldCameraPosition.y = oldCameraTarget.y;
    }

    // construct glm matrix from the existing camera parameters
    glm::mat4 existingGameMtx = glm::lookAtRH(oldCameraPosition, oldCameraTarget, oldCameraUp);
    glm::fquat gameplayRotation = glm::quat_cast(glm::inverse(existingGameMtx));

    const float gameYawDegrees = YawDegreesFromRotation(gameplayRotation);
    const float gameStickYawDelta = ConsumeGameStickYawDelta();
    const bool adoptGameCameraYaw = ShouldAdoptGameCameraYaw();
    if (adoptGameCameraYaw) {
        s_baseYawDegrees = gameYawDegrees;
        s_hasBaseYaw = true;
    }
    else {
        // follow the turn the game already applied instead of correcting it away
        s_baseYawDegrees = NormalizeDegrees(s_baseYawDegrees + gameStickYawDelta - ConsumePendingCameraYawDelta());
        gameplayRotation = glm::angleAxis(glm::radians(s_baseYawDegrees), glm::fvec3(0.0f, 1.0f, 0.0f));
    }
    s_pendingGameYawCorrection = NormalizeDegrees(gameYawDegrees - s_baseYawDegrees);

    s_wsCameraPosition = oldCameraPosition;
    s_wsCameraRotation = gameplayRotation;

    // rebase the rotation to the player position
    if (IsFirstPerson()) {
        // check if player is swimming
        Player actor;
        readMemory(s_playerAddress, &actor);

        PlayerMoveBitFlags moveBits = actor.moveBitFlags.getLE();
        s_isSwimming = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_SWIMMING);
        s_isCrouching = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_CROUCHING);

        // Todo: move those and their hooks in controls.cpp ?
        auto gameState = VRManager::instance().XR->m_gameState.load();
        // Unreliable flag, need to investigate
        gameState.is_climbing = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_CLIMBING_WALL) || s_isLadderClimbing == 2;
        gameState.is_riding_mount = CemuHooks::IsRiding(false);
        gameState.is_paragliding = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_GLIDER_ACTIVE);
        VRManager::instance().XR->m_gameState.store(gameState);

        //static bool s_loggedAdoptGameCameraYaw = true;
        //if (std::abs(s_pendingGameYawCorrection) > 1.0f || adoptGameCameraYaw != s_loggedAdoptGameCameraYaw) {
        //    Log::print<INFO>("Yaw hold: game={:.1f} held={:.1f} correction={:.1f} adopt={} climbing={}", gameYawDegrees, s_baseYawDegrees, s_pendingGameYawCorrection, adoptGameCameraYaw, gameState.is_climbing);
        //}
        //s_loggedAdoptGameCameraYaw = adoptGameCameraYaw;

        auto now = std::chrono::steady_clock::now();
        std::chrono::milliseconds crouchLerpDuration{ 150 };
        if (s_isCrouching != s_wasCrouching) {
            crouch_state_change_time = now;
        }
        auto test = 0.8f;
        if (now <= crouch_state_change_time + crouchLerpDuration) {
            auto elapsed = std::chrono::duration<float>(now - crouch_state_change_time);
            auto duration = std::chrono::duration<float>(crouchLerpDuration);
            float t = elapsed.count() / duration.count();
            t = glm::clamp(t, 0.0f, 1.0f);
            if (s_isCrouching) {
                actualCrouchOffset = glm::mix(0.0f, test, t);
            }
            else {
                actualCrouchOffset = glm::mix(test, 0.0f, t);
            }
        }
        else {
            actualCrouchOffset = s_isCrouching ? test : 0.0f;
        }

        glm::fvec3 playerPos = actor.mtx.getPos().getLE();
        playerPos.y += ResolvePlayerHeightAdjustment(HeightAdjustmentTarget::RenderedCamera);

        UpdateClimbWallCameraOffset(playerPos, gameplayRotation);
        playerPos += s_climbWallCameraOffset;

        if (s_isLadderClimbing > 0) {
            s_isLadderClimbing--;
        }
        if (s_isRiding > 0) {
            s_isRiding--;
        }
        if (s_isRidingSandSeal > 0) {
            s_isRidingSandSeal--;
        }

        s_wasCrouching = s_isCrouching;

        glm::mat4 playerMtx4 = glm::inverse(glm::translate(glm::identity<glm::mat4>(), playerPos) * glm::mat4(gameplayRotation));
        existingGameMtx = playerMtx4;
    }

    UpdateDebugEyeViewsFromGameplayPose(s_wsCameraPosition, s_wsCameraRotation);

    // current VR headset camera matrix
    auto viewsOpt = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (!viewsOpt) {
        Log::print<ERROR>("hook_UpdateCameraForGameplay: No views available for the middle pose.");
        return;
    }
    glm::mat4 views = GetAppliedHeadsetPose(viewsOpt.value());

    // calculate final camera matrix
    glm::mat4 finalPose = glm::inverse(existingGameMtx) * views;

    // extract camera up, forward and position from the final matrix
    glm::fvec3 camPos = glm::fvec3(finalPose[3]);
    glm::fvec3 forward = -glm::normalize(glm::fvec3(finalPose[2]));
    glm::fvec3 up = glm::normalize(glm::fvec3(finalPose[1]));
    glm::fvec3 target = camPos + forward * oldCameraDistance;

    if (!TryUpdateGameplayCameraTarget(camPos, target, up)) {
        return;
    }

    UpdateGameplayReferenceCameraMtx(s_wsCameraPosition, s_wsCameraRotation);

    actCam.finalCamMtx.pos = camPos;
    actCam.finalCamMtx.target = target;
    actCam.finalCamMtx.up = up;

    // write back the modified camera matrix to the game's memory
    uint32_t ppc_cameraMatrixOffsetOut = hCPU->gpr[31];
    writeMemory(ppc_cameraMatrixOffsetOut, &actCam);
    s_framesSinceLastCameraUpdate = 0;
}

// the gameplay camera is just a look-at camera that doesn't seem to have a pivot point at the point we hook it
// so this hook adjusts it afterwards to have properly working gameplay camera rotations
void CemuHooks::hook_AdjustGameplayCameraPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson() || !s_hasGameplayCameraTarget || GetFramesSinceLastCameraUpdate() > 4 || UseBlackBarsDuringEvents()) {
        return;
    }

    uint32_t pivotPtr = hCPU->gpr[4];
    if (pivotPtr == 0) {
        return;
    }

    BEVec3 pivot = {};
    readMemory(pivotPtr, &pivot);

    glm::fvec3 currentPivot = pivot.getLE();
    glm::fvec3 pivotDelta = s_lastGameplayCameraTarget - currentPivot;
    if (!std::isfinite(pivotDelta.x) || !std::isfinite(pivotDelta.y) || !std::isfinite(pivotDelta.z) || glm::dot(pivotDelta, pivotDelta) <= 1.0e-6f) {
        return;
    }

    pivot = currentPivot + pivotDelta;
    writeMemory(pivotPtr, &pivot);
}

// only the VR controller's stick is withheld from the game, so this is in practice the rotation a
// real controller's right stick asked for
void CemuHooks::hook_AddGameCameraStickYaw(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const float yawDelta = (float)hCPU->fpr[1].fp0;
    if (yawDelta == 0.0f || !std::isfinite(yawDelta) || ShouldAdoptGameCameraYaw()) {
        return;
    }

    s_pendingGameStickYawDelta = NormalizeDegrees(s_pendingGameStickYawDelta + yawDelta);
}

void CemuHooks::hook_CaptureClimbWallSurface(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    constexpr uint32_t kClimbWallNormalOffset = 0x54;
    constexpr uint32_t kClimbWallPointOffset = 0x60;

    const uint32_t wallStatePtr = hCPU->gpr[3];
    if (wallStatePtr == 0) {
        return;
    }

    const glm::fvec3 normal = getMemory<BEVec3>(wallStatePtr + kClimbWallNormalOffset).getLE();
    const glm::fvec3 point = getMemory<BEVec3>(wallStatePtr + kClimbWallPointOffset).getLE();
    // the game leaves the normal at zero until the sweep has actually hit something
    if (!IsAllFinite(normal) || !IsAllFinite(point) || glm::dot(normal, normal) < 0.5f) {
        return;
    }

    s_climbWallNormal = glm::normalize(normal);
    s_climbWallPoint = point;
    s_climbWallUpdateNs = GetTimeStamp();
}

void CemuHooks::hook_SnapTurnCameraPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!IsFirstPerson() || HasActiveCutscene()) {
        return;
    }

    ApplyYawDeltaToComputedCameraPivot(hCPU, ConsumePendingGameYawCorrection());
}

static bool ApplyYawDeltaToCameraTailPivot(PPCInterpreter_t* hCPU, float yawDeltaDegrees) {
    if (yawDeltaDegrees == 0.0f || hCPU->gpr[3] == 0 || hCPU->gpr[4] == 0) {
        return false;
    }

    constexpr uint32_t kCameraTailFloat98Offset = 0x98;
    constexpr uint32_t kSphericalYawOffset = 8;

    uint32_t sphericalYawAddr = hCPU->gpr[4] + kSphericalYawOffset;
    float sphericalYaw = CemuHooks::getMemory<BEType<float>>(sphericalYawAddr).getLE();
    if (!std::isfinite(sphericalYaw)) {
        return false;
    }
    CemuHooks::setMemory<BEType<float>>(sphericalYawAddr, NormalizeDegrees(sphericalYaw - yawDeltaDegrees));

    uint32_t float98Addr = hCPU->gpr[3] + kCameraTailFloat98Offset;
    float float98 = CemuHooks::getMemory<BEType<float>>(float98Addr).getLE();
    if (std::isfinite(float98)) {
        CemuHooks::setMemory<BEType<float>>(float98Addr, NormalizeDegrees(float98 - yawDeltaDegrees));
    }

    return true;
}

void CemuHooks::hook_SnapTurnCameraTailPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!IsFirstPerson() || HasActiveCutscene()) {
        return;
    }

    ApplyYawDeltaToCameraTailPivot(hCPU, ConsumePendingGameYawCorrection());
}

void CemuHooks::hook_FixStaminaGaugeScreenPosition(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson()) {
        return;
    }

    BEVec2 staminaGauge2DPos;
    readMemory(hCPU->gpr[4], &staminaGauge2DPos);
    BEVec3 playerPosToCameraPos;
    readMemory(hCPU->gpr[5], &playerPosToCameraPos);

    //Log::print<PPC>("Fixing stamina gauge position (oldPos = {}, playerPosToCameraPos = {})", staminaGauge2DPos.getLE(), playerPosToCameraPos);

    staminaGauge2DPos = BEVec2{ -482.0f, +170.0f }; // nested under hearts
    //staminaGauge2DPos = BEVec2{ 380.0f, 300.0f }; // above the status bars but at the top of the screen
    //staminaGauge2DPos = BEVec2{ -220.0f, 300.0f }; // above the status bars but at the top of the screen
    playerPosToCameraPos = BEVec3{ 0.0f, 0.0, 0.0f };

    writeMemory(hCPU->gpr[4], &staminaGauge2DPos);
    writeMemory(hCPU->gpr[5], &playerPosToCameraPos);
}

void CemuHooks::hook_FixExtraStaminaGaugeIconPositions(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // original instruction that got replaced
    hCPU->fpr[29].fp0 = 1.0f;

    if (IsThirdPerson()) {
        return;
    }

    // manually jump
    hCPU->instructionPointer = 0x02FB29C4;
}

glm::mat4 CemuHooks::s_lastCameraMtx = glm::mat4(1.0f);

// Latest per-eye view/projection matrices captured for the synthesized-right-eye mode.
// The left entries are exactly what the game rendered its (only) eye with; the right
// entries are what the game WOULD have used, computed from the same camera anchor.
namespace {
    struct SynthStereoMatrices {
        std::mutex lock;
        std::array<glm::fmat4, 2> view = { glm::fmat4(1.0f), glm::fmat4(1.0f) };
        std::array<glm::fmat4, 2> proj = { glm::fmat4(1.0f), glm::fmat4(1.0f) };
        bool viewValid = false;
        bool projValid = false;
    };
    SynthStereoMatrices s_synthStereo;
}

uint32_t CemuHooks::GetEffectiveRenderSkipMask() {
    uint32_t mask = GetSettings().GetRenderSkipMask();
    // loading screens and cutscenes hang when the right-eye calc functions are skipped
    // (their completion handshake runs through the calcView/queue path), so suspend
    // everything except the DRC skip while a fade or event is active
    if ((mask & ~1u) != 0 && (IsAnyFadeScreenVisible() || HasActiveCutscene() || !IsInGame())) {
        mask &= 1u;
    }
    return mask;
}

bool CemuHooks::ShouldUseSynthesizedRightEye() {
    if (!GetSettings().UseSynthesizedRightEye()) {
        return false;
    }
    auto* xr = VRManager::instance().XR.get();
    auto* renderer = xr != nullptr ? xr->GetRenderer() : nullptr;
    if (renderer == nullptr || !renderer->IsInitialized()) {
        return false;
    }
    // the game's own photo/save mono captures and cutscene black bars need the regular stereo path
    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return false;
    }
    if (UseBlackBarsDuringEvents()) {
        return false;
    }
    // cutscene and loading-screen progression still depends on the second render pass in
    // ways the single-pass mode doesn't cover yet (the intro event and save-loads hang),
    // so fall back to true stereo whenever an event or fade is active
    if (HasActiveCutscene()) {
        return false;
    }
    if (IsAnyFadeScreenVisible()) {
        return false;
    }
    return true;
}

std::optional<glm::fmat4> CemuHooks::GetSynthReprojectionMatrix() {
    std::lock_guard lk(s_synthStereo.lock);
    if (!s_synthStereo.viewValid || !s_synthStereo.projValid) {
        return std::nullopt;
    }
    const glm::fmat4 leftClip = s_synthStereo.proj[OpenXR::EyeSide::LEFT] * s_synthStereo.view[OpenXR::EyeSide::LEFT];
    const glm::fmat4 rightClip = s_synthStereo.proj[OpenXR::EyeSide::RIGHT] * s_synthStereo.view[OpenXR::EyeSide::RIGHT];
    return rightClip * glm::inverse(leftClip);
}

// s_lastCameraMtx is still the previous frame's anchor while the actor calc jobs run, so anything
// that also reads the live player matrix has to re-resolve it or it mixes two simulation steps
glm::mat4 CemuHooks::GetFreshCameraReferenceMtx() {
    glm::mat4 refMtx = s_lastCameraMtx;
    refMtx[3] = glm::fvec4(ResolveGameplayAnchorPosition(glm::fvec3(refMtx[3])), 1.0f);
    return refMtx;
}

void CemuHooks::hook_GetRenderCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t cameraIn = hCPU->gpr[3];
    uint32_t cameraOut = hCPU->gpr[12];
    EyeSide side = hCPU->gpr[11] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    if (UseBlackBarsDuringEvents()) {
        return;
    }

    BESeadLookAtCamera camera = {};
    readMemory(cameraIn, &camera);

    Log::print<RENDERING>("[{}] Getting render camera", side);
    auto [gameplayPos, gameplayRot] = ResolveGameplayBasePose(camera);
    UpdateGameplayReferenceCameraMtx(gameplayPos, gameplayRot);
    auto [newPos, newRot] = BuildGameplayCameraPose(gameplayPos, gameplayRot, side);

    glm::mat4 newWorldVR = glm::translate(glm::mat4(1.0f), newPos) * glm::mat4_cast(newRot);
    glm::mat4 newViewVR = glm::inverse(newWorldVR);
    DebugDraw::instance().UpdateEyeView(side, newViewVR);

    if (side == EyeSide::LEFT && GetSettings().UseSynthesizedRightEye()) {
        auto [rightPos, rightRot] = BuildGameplayCameraPose(gameplayPos, gameplayRot, EyeSide::RIGHT);
        glm::mat4 rightWorld = glm::translate(glm::mat4(1.0f), rightPos) * glm::mat4_cast(rightRot);
        std::lock_guard lk(s_synthStereo.lock);
        s_synthStereo.view[EyeSide::LEFT] = newViewVR;
        s_synthStereo.view[EyeSide::RIGHT] = glm::inverse(rightWorld);
        s_synthStereo.viewValid = true;
    }

    camera.mtx.setLEMatrix(newViewVR);

    camera.pos = newPos;

    // Set look-at point by offsetting position in view direction
    glm::vec3 viewDir = -glm::vec3(newViewVR[2]); // Forward direction is -Z in view space
    camera.at = newPos + viewDir;

    // Transform world up vector by new rotation
    glm::vec3 upDir = glm::vec3(newViewVR[1]); // Up direction is +Y in view space
    camera.up = upDir;


    //glm::mat4 workingMtx = glm::inverse(glm::lookAtRH(newPos, newPos + glm::vec3(newViewVR[2]), glm::fvec3(0, 1, 0)));
    //s_lastCameraMtx = workingMtx;

    writeMemory(cameraOut, &camera);
    hCPU->gpr[3] = cameraOut;
}

constexpr uint32_t seadOrthoProjection = 0x1027B5BC;
constexpr uint32_t seadPerspectiveProjection = 0x1027B54C;

void CemuHooks::hook_GetRenderProjection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (UseBlackBarsDuringEvents()) {
        return;
    }

    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }


    uint32_t projectionIn = hCPU->gpr[3];
    uint32_t projectionOut = hCPU->gpr[12];
    OpenXR::EyeSide side = hCPU->gpr[0] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionIn, &perspectiveProjection);

    Log::print<RENDERING>("[{}] Render Proj. (LR: {:08X}): {}", side, hCPU->sprNew.LR, perspectiveProjection);

    if (perspectiveProjection.zFar == 10000.0f) {
        return;
    }

    perspectiveProjection.zFar = GetSettings().GetZFar();
    perspectiveProjection.zNear = GetSettings().GetZNear();

    auto currFovOpt = GetGameProjectionFOV(side, perspectiveProjection);
    if (!currFovOpt.has_value()) {
        return;
    }
    XrFovf currFOV = currFovOpt.value();
    auto newProjection = RenderUtils::CalculateFOVAndOffset(currFOV);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), currFOV);
    perspectiveProjection.matrix = newMatrix;

    // calculate device matrix
    glm::fmat4 newDeviceMatrix = newMatrix;

    float zScale = perspectiveProjection.deviceZScale.getLE();
    float zOffset = perspectiveProjection.deviceZOffset.getLE();

    newDeviceMatrix[2][0] *= zScale;
    newDeviceMatrix[2][1] *= zScale;
    newDeviceMatrix[2][2] = (newDeviceMatrix[2][2] + newDeviceMatrix[3][2] * zOffset) * zScale;
    newDeviceMatrix[2][3] = newDeviceMatrix[2][3] * zScale + newDeviceMatrix[3][3] * zOffset;

    if (side == EyeSide::LEFT && GetSettings().UseSynthesizedRightEye()) {
        auto rightFovOpt = GetGameProjectionFOV(EyeSide::RIGHT, perspectiveProjection);
        if (rightFovOpt.has_value()) {
            glm::fmat4 rightDeviceMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), rightFovOpt.value());
            rightDeviceMatrix[2][0] *= zScale;
            rightDeviceMatrix[2][1] *= zScale;
            rightDeviceMatrix[2][2] = (rightDeviceMatrix[2][2] + rightDeviceMatrix[3][2] * zOffset) * zScale;
            rightDeviceMatrix[2][3] = rightDeviceMatrix[2][3] * zScale + rightDeviceMatrix[3][3] * zOffset;

            std::lock_guard lk(s_synthStereo.lock);
            s_synthStereo.proj[EyeSide::LEFT] = newDeviceMatrix;
            s_synthStereo.proj[EyeSide::RIGHT] = rightDeviceMatrix;
            s_synthStereo.projValid = true;
        }
    }

    {
        auto* rendererForDebugDraw = VRManager::instance().XR->GetRenderer();
        if (rendererForDebugDraw != nullptr) {
            auto rawFov = rendererForDebugDraw->GetFOV(side);
            if (rawFov.has_value()) {
                glm::fmat4 debugProjMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), rawFov.value());
                glm::fmat4 debugDeviceMatrix = debugProjMatrix;
                debugDeviceMatrix[2][0] *= zScale;
                debugDeviceMatrix[2][1] *= zScale;
                debugDeviceMatrix[2][2] = (debugDeviceMatrix[2][2] + debugDeviceMatrix[3][2] * zOffset) * zScale;
                debugDeviceMatrix[2][3] = debugDeviceMatrix[2][3] * zScale + debugDeviceMatrix[3][3] * zOffset;
                DebugDraw::instance().UpdateEyeProjection(side, glm::transpose(debugDeviceMatrix));
            }
        }
    }
    perspectiveProjection.deviceMatrix = newDeviceMatrix;

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;

    writeMemory(projectionOut, &perspectiveProjection);
    hCPU->gpr[3] = projectionOut;
}


void CemuHooks::hook_ModifyLightPrePassProjectionMatrix(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        return;
    }

    if (UseBlackBarsDuringEvents()) {
        return;
    }
    
    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }

    uint32_t projectionIn = hCPU->gpr[3];
    OpenXR::EyeSide side = hCPU->gpr[11] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionIn, &perspectiveProjection);

    auto currFovOpt = GetGameProjectionFOV(side, perspectiveProjection);
    if (!currFovOpt.has_value()) {
        return;
    }

    Log::print<RENDERING>("[{}] Modify light prepass projection", side);


    XrFovf currFOV = currFovOpt.value();
    auto newProjection = RenderUtils::CalculateFOVAndOffset(currFOV);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), currFOV);
    perspectiveProjection.matrix = newMatrix;

    // calculate device matrix
    glm::fmat4 newDeviceMatrix = newMatrix;

    float zScale = perspectiveProjection.deviceZScale.getLE();
    float zOffset = perspectiveProjection.deviceZOffset.getLE();

    newDeviceMatrix[2][0] *= zScale;
    newDeviceMatrix[2][1] *= zScale;
    newDeviceMatrix[2][2] = (newDeviceMatrix[2][2] + newDeviceMatrix[3][2] * zOffset) * zScale;
    newDeviceMatrix[2][3] = newDeviceMatrix[2][3] * zScale + newDeviceMatrix[3][3] * zOffset;

    perspectiveProjection.deviceMatrix = newDeviceMatrix;

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;

    writeMemory(projectionIn, &perspectiveProjection);
}

void CemuHooks::hook_OverwriteSeadPerspectiveProjectionSet(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
}

void CemuHooks::hook_ModifyProjectionUsingCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        return;
    }

    if (UseBlackBarsDuringEvents() || UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }

    uint32_t projectionPtr = hCPU->gpr[4];
    uint32_t cameraPtr = hCPU->gpr[7];
    OpenXR::EyeSide side = hCPU->gpr[5] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    // this is always true, since we currently only hook one caller
    if (hCPU->gpr[6] == 0x02C43454) {
        BESeadLookAtCamera camera = {};
        readMemory(cameraPtr, &camera);

        Log::print<RENDERING>("[{}] ModifyProjectionUsingCamera at {:08X}: {}", side, cameraPtr, camera);

        // the divine beast outside rendering actually use a separate camera that is at a different position
        // so don't use generic camera position, and instead recalculate it

        glm::mat4x3 viewMatrix = camera.mtx.getLEMatrix();
        glm::mat4 worldGame = glm::inverse(glm::mat4(viewMatrix));
        glm::vec3 basePos = glm::vec3(worldGame[3]);
        auto [newPos, newRot] = BuildCameraPoseFromBase(basePos, s_wsCameraRotation, side, true);

        glm::mat4 newWorldVR = glm::translate(glm::mat4(1.0f), newPos) * glm::mat4_cast(newRot);
        glm::mat4 newViewVR = glm::inverse(newWorldVR);
        camera.mtx.setLEMatrix(newViewVR);

        camera.pos = newPos;

        // Set look-at point by offsetting position in view direction
        glm::vec3 viewDir = -glm::vec3(newViewVR[2]); // Forward direction is -Z in view space
        camera.at = newPos + viewDir;

        // Transform world up vector by new rotation
        glm::vec3 upDir = glm::vec3(newViewVR[1]); // Up direction is +Y in view space
        camera.up = upDir;

        writeMemory(cameraPtr, &camera);
    }

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionPtr, &perspectiveProjection);

    auto currFovOpt = GetGameProjectionFOV(side, perspectiveProjection);
    if (!currFovOpt.has_value()) {
        return;
    }

    Log::print<RENDERING>("[{}] ModifyProjectionUsingCamera: {}", side, perspectiveProjection);

    XrFovf currFOV = currFovOpt.value();
    auto newProjection = RenderUtils::CalculateFOVAndOffset(currFOV);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), currFOV);
    perspectiveProjection.matrix = newMatrix;

    // calculate device matrix
    glm::fmat4 newDeviceMatrix = newMatrix;

    float zScale = perspectiveProjection.deviceZScale.getLE();
    float zOffset = perspectiveProjection.deviceZOffset.getLE();

    newDeviceMatrix[2][0] *= zScale;
    newDeviceMatrix[2][1] *= zScale;
    newDeviceMatrix[2][2] = (newDeviceMatrix[2][2] + newDeviceMatrix[3][2] * zOffset) * zScale;
    newDeviceMatrix[2][3] = newDeviceMatrix[2][3] * zScale + newDeviceMatrix[3][3] * zOffset;

    perspectiveProjection.deviceMatrix = newDeviceMatrix;

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;

    writeMemory(projectionPtr, &perspectiveProjection);
}

std::pair<glm::vec3, glm::fquat> CemuHooks::CalculateVRWorldPose(const BESeadLookAtCamera& camera, uint8_t side) {
    auto [gameplayPos, gameplayRot] = ResolveGameplayBasePose(camera);
    return BuildGameplayCameraPose(gameplayPos, gameplayRot, (OpenXR::EyeSide)side);
}

void CemuHooks::hook_CheckIfCameraCanSeePos(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        hCPU->gpr[3] = 0;
        return;
    }

    uint32_t camPtr = hCPU->gpr[3];
    uint32_t posPtr = hCPU->gpr[4];
    float radius = hCPU->fpr[1].fp0;
    float nearClip = hCPU->fpr[2].fp0;
    float farClip = hCPU->fpr[3].fp0;

    BESeadLookAtCamera camera = {};
    readMemory(camPtr, &camera);

    BEVec3 center;
    readMemory(posPtr, &center);

    Frustum frustum;
    bool visible = false;

    for (int i = 0; i < 2; ++i) {
        OpenXR::EyeSide side = (i == 0) ? EyeSide::LEFT : EyeSide::RIGHT;
        if (auto fovOpt = TryGetRenderFOV(side)) {
            auto [pos, rot] = CalculateVRWorldPose(camera, side);

            // pull the camera backwards a bit to account for it being a third-person game that encompassed a bigger area
            pos += rot * glm::vec3(0.0f, 0.0f, 1.0f);

            glm::mat4 view = glm::inverse(glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(rot));
            glm::mat4 proj = glm::transpose(RenderUtils::CalculateProjectionMatrix(nearClip, farClip, fovOpt.value()));
            glm::mat4 vp = proj * view;

            frustum.update(vp);
            if (frustum.checkSphere(center.getLE(), radius)) {
                visible = true;
                break;
            }
        }
    }

    //Log::print<PPC>("Checking visibility of {} (rad = {}, near = {}, far = {}): {}", center, radius, nearClip, farClip, visible ? "visible" : "invisible");

    hCPU->gpr[3] = visible ? 1 : 0;
}


void CemuHooks::hook_ModifyPixelUniformBlockData(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }

    auto currFovOpt = TryGetRenderFOV(EyeSide::RIGHT);
    if (!currFovOpt.has_value()) {
        return;
    }

    glm::fvec4 ubData = {};
    readMemory(hCPU->gpr[5], &ubData);

    XrFovf currFOV = currFovOpt.value();
    auto newProjection = RenderUtils::CalculateFOVAndOffset(currFOV);

    ubData.x = 0.5f + newProjection.offsetX.getLE();
    ubData.y = 0.5f + newProjection.offsetY.getLE();

    writeMemory(hCPU->gpr[5], &ubData);
}

void CemuHooks::hook_EndCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[3] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    // todo: sometimes this can deadlock apparently?
    if (VRManager::instance().XR->GetRenderer()->IsInitialized() && side == OpenXR::EyeSide::RIGHT) {
        m_heldWeaponsLastUpdate[0]++;
        m_heldWeaponsLastUpdate[1]++;
        if (m_heldWeaponsLastUpdate[0] >= 6) {
            m_heldWeapons[0] = 0;
            s_handWeaponTypes[0] = WeaponType::UnknownWeapon;
        }
        if (m_heldWeaponsLastUpdate[1] >= 6) {
            m_heldWeapons[1] = 0;
            s_handWeaponTypes[1] = WeaponType::UnknownWeapon;
            s_arrowNockedInRightHand = false;
        }
    }

    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("");
}

void CemuHooks::hook_UseCameraDistance(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsFirstPerson()) {
        hCPU->fpr[13].fp0 = 0.0f;
    }
    else if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON) {
        hCPU->fpr[13].fp0 = GetSettings().thirdPlayerDistance;
    }
    else {
        hCPU->fpr[13].fp0 = 0.5f; // use default distance when using the first-person camera
    }
}

void CemuHooks::hook_SetActorOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    float toBeSetOpacity = hCPU->fpr[1].fp0;
    uint32_t actorPtr = hCPU->gpr[3];

    ActorWiiU actor;
    readMemory(actorPtr, &actor);

    // normal behavior if it wasn't the player or a held weapon
    if (actor.modelOpacity.getLE() != toBeSetOpacity) {
        uint8_t opacityOrDoFlushOpacityToGPU = 1;
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, modelOpacity), &toBeSetOpacity);
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, opacityOrDoFlushOpacityToGPU), &opacityOrDoFlushOpacityToGPU);
    }
}

void CemuHooks::hook_CalculateModelOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!IsFirstPerson()) {
        return;
    }

    hCPU->fpr[1].fp0 = 1.0f;
}


struct CameraParamValueOffset {
    std::string name;
    uint32_t offsetInsideCamera;
    bool storedOriginalValue = false;
    float originalValue;
};

struct ActionFloatParamPointerOverride {
    std::string name;
    uint32_t destPointerAddress;
    bool storedOriginalPointer = false;
    uint32_t originalPointer = 0;
};

// key = vtable address, value = list of parameter names and their offsets inside the camera object
std::mutex storedCameraParametersLock;
std::unordered_map<uint32_t, std::vector<CameraParamValueOffset>> storedCameraParameters;
std::mutex storedActionFloatParamsLock;
std::unordered_map<uint32_t, std::vector<ActionFloatParamPointerOverride>> storedActionFloatParams;

static void ApplyStoredActionFloatParamOverrides() {
    constexpr uint32_t kPlayerLaunchZeroFloat = 0x101D3CC8;

    std::scoped_lock lock(storedActionFloatParamsLock);

    for (auto& entries : storedActionFloatParams | std::views::values) {
        for (auto& entry : entries) {
            uint32_t currentPointer = CemuHooks::getMemory<BEType<uint32_t>>(entry.destPointerAddress).getLE();
            if (!entry.storedOriginalPointer && currentPointer != 0 && currentPointer != kPlayerLaunchZeroFloat) {
                entry.originalPointer = currentPointer;
                entry.storedOriginalPointer = true;
            }

            if (!entry.storedOriginalPointer) {
                continue;
            }

            uint32_t targetPointer = (CemuHooks::IsFirstPerson() && GetSettings().ShouldPreventFirstPersonRagdoll()) ? kPlayerLaunchZeroFloat : entry.originalPointer;
            if (currentPointer != targetPointer) {
                CemuHooks::setMemory<uint32_t>(entry.destPointerAddress, targetPointer);
            }
        }
    }
}

void CemuHooks::hook_ReplaceCameraMode(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t currCameraInstance = hCPU->gpr[3];
    uint32_t cameraChaseInstance = hCPU->gpr[4]; // this is currently a pointer to the regular camera mode (CameraChase)
    uint32_t currentCameraVtbl = hCPU->gpr[5];


    // check if any patched parameters exist for this camera vtbl
    {
        std::scoped_lock(storedCameraParametersLock);

        auto it = storedCameraParameters.find(currCameraInstance);
        if (it != storedCameraParameters.end()) {
            for (auto& paramEntry : it->second) {
                uint32_t originalValuePtr = getMemory<BEType<uint32_t>>(paramEntry.offsetInsideCamera).getLE();
                BEType<float>* paramValueBE = (BEType<float>*)(s_memoryBaseAddress + originalValuePtr);

                // on first patch, store original value
                if (!paramEntry.storedOriginalValue) {
                    paramEntry.originalValue = paramValueBE->getLE();
                    paramEntry.storedOriginalValue = true;
                }

                if (IsFirstPerson()) {
                    // set to zero in first person
                    *paramValueBE = 0.0f;
                }
                else {
                    // restore original value in third person
                    *paramValueBE = paramEntry.originalValue;
                }
            }
        }
    }

    constexpr uint32_t kCameraChaseVtbl = 0x101B34F4;
    constexpr uint32_t kCameraTailVtbl = 0x101BC278;
    constexpr uint32_t kCameraMagneCatchVtbl = 0x101BAB4C;

    static uint32_t s_lastLoggedCameraVtbl = 0;
    static uint32_t s_lastLoggedCameraInstance = 0;
    static uint32_t s_lastLoggedCameraChaseInstance = 0;

    const bool isSnapTurnCamera = currentCameraVtbl == kCameraChaseVtbl || currentCameraVtbl == kCameraTailVtbl;

    if (currentCameraVtbl != s_lastLoggedCameraVtbl ||
        currCameraInstance != s_lastLoggedCameraInstance ||
        cameraChaseInstance != s_lastLoggedCameraChaseInstance) {
        Log::print<INFO>("Camera changed (vtbl={:#X}, camera={:#X}, chase={:#X}, snapTurnActive={})", currentCameraVtbl, currCameraInstance, cameraChaseInstance, isSnapTurnCamera);
        s_lastLoggedCameraVtbl = currentCameraVtbl;
        s_lastLoggedCameraInstance = currCameraInstance;
        s_lastLoggedCameraChaseInstance = cameraChaseInstance;
    }

    if (auto* xr = VRManager::instance().XR.get()) {
        xr->m_isSnapTurnCameraActive.store(isSnapTurnCamera, std::memory_order_relaxed);
    }

    //hCPU->gpr[3] = cameraChaseInstance;

    if (hCPU->gpr[5] == kCameraMagneCatchVtbl) {
        if (IsFirstPerson()) {
            //hCPU->gpr[3] = cameraChaseMode;
        }
    }

    if (hCPU->gpr[5] == kCameraTailVtbl) {
        //Log::print<RENDERING>("Current camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currentCameraMode, cameraTailMode, currentCameraVtbl);
        if (IsFirstPerson()) {
            // overwrite to tail mode
            //hCPU->gpr[3] = cameraTailMode;
        }
    }

    //Log::print<INFO>("Camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currCameraInstance, cameraChaseInstance, currentCameraVtbl);
}

void CemuHooks::UpdateFloatParamOverrides() {
    ApplyStoredActionFloatParamOverrides();
}

constexpr uint32_t orig_GetStaticParam_float_funcAddr = 0x030E9BE0;

static bool ShouldZeroFirstPersonDamageFloatParam(std::string_view paramName) {
    return paramName.find("Speed") != std::string_view::npos || paramName.starts_with("JumpHeight") || paramName.starts_with("AddLinearImpulse") || paramName.starts_with("AddRollImpulse") || paramName == "NoRagdollTime";
}

// hook for ksys::act::ai::ActionBase::getStaticParam<FLOAT> calls
void CemuHooks::hook_OverwriteFloatParam(PPCInterpreter_t* hCPU) {
    uint32_t actionPtr = hCPU->gpr[3];
    uint32_t destFloatPtr = hCPU->gpr[4];
    uint32_t paramNameArgPtr = hCPU->gpr[5];
    if (actionPtr == 0 || destFloatPtr == 0 || paramNameArgPtr == 0) {
        hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
        return;
    }

    uint32_t paramNamePtr = getMemory<uint32_t>(paramNameArgPtr).getLE();
    if (paramNamePtr == 0) {
        hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
        return;
    }

    const char* paramName = (const char*)(s_memoryBaseAddress + paramNamePtr);
    std::string_view paramNameStr = paramName;
    if (ShouldZeroFirstPersonDamageFloatParam(paramNameStr)) {
        std::string paramNameOwned = std::string(paramNameStr);

        {
            std::scoped_lock lock(storedActionFloatParamsLock);

            auto& paramList = storedActionFloatParams[actionPtr];
            auto it = std::ranges::find_if(paramList, [&paramNameOwned](const ActionFloatParamPointerOverride& entry) {
                return entry.name == paramNameOwned;
            });
            if (it == paramList.end()) {
                Log::print<PPC>("Storing float param '{}' offset {:08X} for action at {:08X}", paramNameOwned, destFloatPtr, actionPtr);
                paramList.push_back({ paramNameOwned, destFloatPtr });
            }
        }

        hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
        return;
    }

    {
        std::string paramNameOwned = std::string(paramNameStr);

        std::scoped_lock(storedCameraParametersLock);

        auto& paramList = storedCameraParameters[actionPtr];
        // store parameter offset if not already stored
        auto it = std::ranges::find_if(paramList, [&paramNameOwned](const CameraParamValueOffset& entry) {
            return entry.name == paramNameOwned;
        });
        if (it == paramList.end()) {
            hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;

            Log::print<PPC>("Storing float param '{}' offset {:08X} for action at {:08X}", paramNameOwned, destFloatPtr, actionPtr);
            paramList.push_back({ paramNameOwned, destFloatPtr });
        }
    }

    hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
}

void CemuHooks::hook_FixLadder(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto input = VRManager::instance().XR->m_input.load();

    if (input.shared.in_game && s_isLadderClimbing == 0) {
        return;
    }

    if (input.inGame.move.currentState.y >= -0.05) {
        //Log::print<INFO>("PLAYER LADDER MODE UP {}", input.inGame.move.currentState.y);
        hCPU->gpr[3] = 4; // allows pressing A to jump upwards, regardless of camera orientation
    }
    else {
        //Log::print<INFO>("PLAYER LADDER MODE DOWN {}", input.inGame.move.currentState.y);
        hCPU->gpr[3] = 1; // allows sliding down when holding move stick downwards
    }
}


void CemuHooks::hook_VisualizeRayCastHits(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!GetSettings().ShouldShowRaycastLines()) {
        return;
    }

    uint32_t rayCastResultPtr = hCPU->gpr[3];
    glm::fvec3 raycastHitPos = getMemory<BEVec3>(hCPU->gpr[4]).getLE();

    ksys::phys::RayCast rayCast = {};
    readMemory(rayCastResultPtr, &rayCast);

    glm::fvec3 rayStart = rayCast.from.getLE();
    glm::fvec3 rayEnd = rayCast.to.getLE();
    
    rayStart.y += 0.5f;
    rayEnd.y += 0.5f;
    raycastHitPos.y += 0.5f;

    DebugDraw::instance().Line(rayStart, raycastHitPos, IM_COL32(255, 0, 255, 255));
    DebugDraw::instance().Line(raycastHitPos, rayEnd, IM_COL32(128, 0, 128, 128));
}

static bool TryGetThrowDirection(glm::fvec3& outDirection) {
    if (CemuHooks::IsThirdPerson() || CemuHooks::GetFramesSinceLastCameraUpdate() > 4) {
        return false;
    }

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return false;
    }

    auto middlePoseOpt = renderer->GetMiddlePose();
    if (!middlePoseOpt.has_value()) {
        return false;
    }

    glm::mat4 cameraWorld = CemuHooks::s_lastCameraMtx * middlePoseOpt.value();
    // Throw seems to want the inverse of the camera's forward direction
    glm::fvec3 direction = glm::mat3(cameraWorld) * glm::fvec3(0.0f, 0.0f, -1.0f);
    if (!glm::all(glm::isfinite(direction)) || glm::dot(direction, direction) <= 1.0e-6f) {
        return false;
    }

    outDirection = glm::normalize(direction);
    return true;
}

static bool TryGetGuardDirection(glm::fvec3& outDirection) {
    if (CemuHooks::IsThirdPerson() || CemuHooks::GetFramesSinceLastCameraUpdate() > 4) {
        return false;
    }

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr) {
        return false;
    }

    auto middlePoseOpt = renderer->GetMiddlePose();
    if (!middlePoseOpt.has_value()) {
        return false;
    }

    glm::mat4 cameraWorld = CemuHooks::s_lastCameraMtx * middlePoseOpt.value();
    glm::fquat cameraRot = glm::quat_cast(cameraWorld);
    auto [_, yawOnly] = RenderUtils::swingTwistY(cameraRot);
    glm::fvec3 direction = yawOnly * glm::fvec3(0.0f, 0.0f, -1.0f);
    direction.y = 0.0f;
    if (!glm::all(glm::isfinite(direction)) || glm::dot(direction, direction) <= 1.0e-6f) {
        return false;
    }

    outDirection = glm::normalize(direction);
    return true;
}

void CemuHooks::hook_OverrideThrowDirection(PPCInterpreter_t* hCPU) {
    if (IsThirdPerson()) {
        float dirX = 0.0f;
        readMemoryBE(hCPU->gpr[31] + 0x1570, &dirX);
        hCPU->fpr[10].fp0 = dirX; // original instruction: lfs f10, 0x1570(r31)
        hCPU->instructionPointer = 0x02C9172C;
        return;
    }

    hCPU->instructionPointer = 0x02C91744;

    glm::fvec3 throwDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    if (!TryGetThrowDirection(throwDir)) {
        if (hCPU->gpr[31] != 0) {
            BEMatrix34 mtxCopy = {};
            readMemory(hCPU->gpr[31] + 0x1568, &mtxCopy);
            throwDir = glm::fvec3(mtxCopy.z_x.getLE(), mtxCopy.z_y.getLE(), mtxCopy.z_z.getLE());
        }

        if (!glm::all(glm::isfinite(throwDir)) || glm::dot(throwDir, throwDir) <= 1.0e-6f) {
            throwDir = glm::fvec3(0.0f, 0.0f, 1.0f);
        }
        else {
            throwDir = glm::normalize(throwDir);
        }
    }

    float dirX = throwDir.x;
    float dirY = throwDir.y;
    float dirZ = throwDir.z;
    const float dirYSq = dirY * dirY;
    const float dirXYSq = dirX * dirX + dirYSq;
    const uint32_t stackBase = hCPU->gpr[1];

    // Emulate the replaced preamble so ThrowSomething keeps using its original normalization path.
    hCPU->fpr[10].fp0 = dirX;
    hCPU->fpr[12].fp0 = dirZ;
    hCPU->fpr[13].fp0 = dirYSq;
    hCPU->fpr[0].fp0 = dirXYSq;

    float dirXStore = dirX;
    float dirYStore = dirY;
    writeMemoryBE(stackBase + 0x14, &dirXStore);
    writeMemoryBE(stackBase + 0x18, &dirYStore);
}

void CemuHooks::hook_OverrideGuardDirection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x02C16ECC;

    const uint32_t stackBase = hCPU->gpr[1];
    uint32_t actorPtr = 0;
    readMemoryBE(stackBase + 0x24, &actorPtr);

    glm::fvec3 guardDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    const bool shouldUseVrDirection = actorPtr != 0 && actorPtr == s_playerAddress && TryGetGuardDirection(guardDir);
    if (!shouldUseVrDirection && actorPtr != 0) {
        BEMatrix34 actorMtx = {};
        readMemory(actorPtr + offsetof(ActorWiiU, mtx), &actorMtx);
        guardDir = glm::fvec3(actorMtx.z_x.getLE(), actorMtx.z_y.getLE(), actorMtx.z_z.getLE());
    }

    if (!glm::all(glm::isfinite(guardDir)) || glm::dot(guardDir, guardDir) <= 1.0e-6f) {
        guardDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    }
    else if (shouldUseVrDirection) {
        guardDir = glm::normalize(guardDir);
    }

    float dirX = guardDir.x;
    float dirY = guardDir.y;
    float dirZ = guardDir.z;
    uint32_t dirXBits = 0;
    uint32_t dirYBits = 0;
    memcpy(&dirXBits, &dirX, sizeof(dirXBits));
    memcpy(&dirYBits, &dirY, sizeof(dirYBits));

    hCPU->gpr[12] = dirXBits;
    hCPU->gpr[0] = dirYBits;
    hCPU->fpr[12].fp0 = dirX;
    hCPU->fpr[6].fp0 = dirY;
    hCPU->fpr[8].fp0 = dirZ;

    float dirXStore = dirX;
    float dirYStore = dirY;
    float dirZStore = dirZ;
    writeMemoryBE(stackBase + 0xE0, &dirXStore);
    writeMemoryBE(stackBase + 0xE4, &dirYStore);
    writeMemoryBE(stackBase + 0xE8, &dirZStore);
}
