#include "pch.h"
#include "camera.h"
#include "stereo_view.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/player/player.h"
#include "game/botw/player/roomscale.h"
#include "utils/debug_draw.h"
#include "utils/render_utils.h"

static constexpr float hardcodedSwimOffset = 0.0f;
static constexpr float hardcodedSwimHeight = 1.73f;
static constexpr float hardcodedRidingOffset = 0.65f;
static constexpr float hardcodedCrouchOffset = 0.8f;
static constexpr float hardcodedClimbWallClearance = 0.35f;

static std::atomic_uint32_t s_framesSinceLastUpdate = 0;
static std::atomic<glm::mat4> s_referenceMtx = glm::mat4(1.0f);

static glm::fvec3 s_wsCameraPosition = glm::fvec3();
static glm::fquat s_wsCameraRotation = glm::identity<glm::fquat>();
static glm::fvec3 s_lastGameplayCameraTarget = glm::fvec3();
static bool s_hasGameplayCameraTarget = false;
static bool s_isSwimming = false;
static bool s_wasCrouching = false;
static float s_crouchOffset = 0.0f;
static std::chrono::steady_clock::time_point s_crouchStateChangeTime;

static float s_baseYawDegrees = 0.0f;
static bool s_hasBaseYaw = false;
static float s_pendingGameYawCorrection = 0.0f;
static float s_pendingGameStickYawDelta = 0.0f;

// the normal points out of the wall, towards Link
static glm::fvec3 s_climbWallPoint = glm::fvec3(0.0f);
static glm::fvec3 s_climbWallNormal = glm::fvec3(0.0f);
static uint64_t s_climbWallUpdateNs = 0;
static glm::fvec3 s_climbWallCameraOffset = glm::fvec3(0.0f);

void Camera::OnGameFrame() {
    ++s_framesSinceLastUpdate;
}

uint32_t Camera::GetFramesSinceLastUpdate() {
    return s_framesSinceLastUpdate.load();
}

glm::mat4 Camera::GetReferenceMtx() {
    return s_referenceMtx.load();
}

void Camera::SetReferenceMtx(const glm::mat4& referenceMtx) {
    s_referenceMtx.store(referenceMtx);
}

static std::atomic_bool s_appliesFirstPersonTurning = false;
struct SnapTurnRequest {
    int32_t direction = 0;
    uint64_t untilNs = 0;
};
static std::atomic<SnapTurnRequest> s_pendingSnapTurn;
static std::atomic<float> s_smoothTurnStickDeflection = 0.0f;
static std::atomic_uint64_t s_snapTurnFadeStartNs = 0;
static std::atomic_uint64_t s_snapTurnFadeUntilNs = 0;

bool Camera::AppliesFirstPersonTurning() {
    return s_appliesFirstPersonTurning;
}

void Camera::RequestSnapTurn(int direction) {
    constexpr uint64_t requestLifetimeNs = 120'000'000;
    s_pendingSnapTurn = SnapTurnRequest{ direction, GetTimeStamp() + requestLifetimeNs };
}

void Camera::CancelSnapTurn() {
    if (const SnapTurnRequest request = s_pendingSnapTurn.exchange({}); request.direction != 0)
        Log::print<INFO>("SnapTurn: cancelled pending request dir={}", request.direction);
}

void Camera::SetSmoothTurnInput(float stickDeflection) {
    s_smoothTurnStickDeflection = stickDeflection;
}

float Camera::GetSnapTurnFadeAmount() {
    const uint64_t startNs = s_snapTurnFadeStartNs;
    const uint64_t endNs = s_snapTurnFadeUntilNs;
    const uint64_t nowNs = GetTimeStamp();
    if (startNs == 0 || endNs <= startNs || nowNs >= endNs) {
        return 0.0f;
    }

    const double t = glm::clamp((double)(nowNs - startNs) / (double)(endNs - startNs), 0.0, 1.0);
    const double triangle = t <= 0.5 ? (t * 2.0) : ((1.0 - t) * 2.0);
    return (float)(triangle * triangle * (3.0 - (2.0 * triangle)));
}

static int TryConsumePendingSnapTurnDirection() {
    const SnapTurnRequest request = s_pendingSnapTurn.exchange({});
    return GetTimeStamp() <= request.untilNs ? request.direction : 0;
}

static float TryConsumePendingSmoothTurnDelta() {
    static uint64_t s_lastTimeNs = 0;
    const uint64_t nowNs = GetTimeStamp();
    const float dt = s_lastTimeNs == 0 ? 0.0f : (float)(nowNs - s_lastTimeNs) * 1.0e-9f;
    s_lastTimeNs = nowNs;

    const float stickDeflection = s_smoothTurnStickDeflection;
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
    constexpr uint64_t fadeDurationNs = 110'000'000;
    const uint64_t startNs = GetTimeStamp();
    s_snapTurnFadeStartNs = startNs;
    s_snapTurnFadeUntilNs = startNs + fadeDurationNs;
}

static float YawDegreesFromRotation(const glm::fquat& rot) {
    const glm::fquat twist = RenderUtils::swingTwistY(rot).second;
    return NormalizeDegrees(glm::degrees(2.0f * std::atan2(twist.y, twist.w)));
}

// arrival events after a warp ignore the camera, so the view keeps the facing it took instead of following the chase camera around
static bool IsEventTurningCamera() {
    const auto eventSettings = Events::GetFirstPersonSettings();
    return Events::IsActive() && !(eventSettings.has_value() && eventSettings->ignoreCameraRotation);
}

static bool ShouldAdoptGameCameraYaw() {
    return !s_hasBaseYaw || !GameState::IsFirstPerson() || IsEventTurningCamera();
}

// the head's yaw when the cutscene took over the camera
static std::optional<float> s_eventHeadYawDegrees;

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
    Log::print<CONTROLS>("SnapTurn: turning {} degrees", direction * snapAngle);
    return (float)direction * (float)snapAngle;
}

static float ConsumePendingGameYawCorrection() {
    return std::exchange(s_pendingGameYawCorrection, 0.0f);
}

static float ConsumeGameStickYawDelta() {
    return std::exchange(s_pendingGameStickYawDelta, 0.0f);
}

static bool ApplyYawDeltaToComputedCameraPivot(PPCInterpreter_t* hCPU, float yawDeltaDegrees) {
    if (yawDeltaDegrees == 0.0f || hCPU->gpr[4] == 0 || hCPU->gpr[5] == 0) {
        return false;
    }

    const glm::fvec3 cameraWorldPos = Guest::Get<BEVec3>(hCPU->gpr[4]).getLE();
    glm::fvec3 spherical = Guest::Get<BEVec3>(hCPU->gpr[5]).getLE();
    if (!glm::all(glm::isfinite(cameraWorldPos)) || !glm::all(glm::isfinite(spherical)) || spherical.x <= 1.0e-4f) {
        return false;
    }

    const glm::fvec3 targetWorldPos = cameraWorldPos + SphericalToCameraTargetVector(spherical);

    spherical.z = NormalizeDegrees(spherical.z - yawDeltaDegrees);
    const glm::fvec3 rotatedCameraWorldPos = targetWorldPos - SphericalToCameraTargetVector(spherical);
    if (!glm::all(glm::isfinite(rotatedCameraWorldPos))) {
        return false;
    }

    BEVec3 rotatedCameraWorldPosBE(rotatedCameraWorldPos.x, rotatedCameraWorldPos.y, rotatedCameraWorldPos.z);
    BEVec3 rotatedSphericalBE(spherical.x, spherical.y, spherical.z);
    Guest::Write(hCPU->gpr[4], &rotatedCameraWorldPosBE);
    Guest::Write(hCPU->gpr[5], &rotatedSphericalBE);
    return true;
}

static glm::fvec3 GetAppliedHeadsetOffset(glm::fvec3 eyePos) {
    if (GameState::IsFirstPerson()) {
        glm::fvec3 appliedHeadPos = Roomscale::GetAppliedHeadPosition();
        eyePos.x -= appliedHeadPos.x;
        eyePos.z -= appliedHeadPos.z;
    }
    return eyePos;
}

static glm::mat4 GetAppliedHeadsetPose(glm::mat4 pose) {
    pose[3] = glm::fvec4(GetAppliedHeadsetOffset(glm::fvec3(pose[3])), pose[3].w);
    return pose;
}

static std::pair<glm::fvec3, glm::fquat> GetAppliedEyePose(const Tracking::HeadViews& views, OpenXR::EyeSide side) {
    const XrPosef& eyePose = views.EyePose(side);
    return { GetAppliedHeadsetOffset(ToGLM(eyePose.position)), ToGLM(eyePose.orientation) };
}

static std::pair<glm::fvec3, glm::fquat> GetAppliedMiddlePose(const Tracking::HeadViews& views) {
    const glm::mat4 middlePose = views.MiddlePose();
    return { GetAppliedHeadsetOffset(glm::vec3(middlePose[3])), glm::quat_cast(middlePose) };
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

static float ResolvePlayerHeightAdjustment(HeightAdjustmentTarget target, const std::optional<Tracking::HeadViews>& views = std::nullopt) {
    const float heightOffset = GetSettings().GetPlayerHeightOffset();

    if (Player::IsRiding(true)) {
        return -hardcodedRidingOffset;
    }
    if (s_isSwimming) {
        if (target == HeightAdjustmentTarget::RenderedCamera && views.has_value()) {
            return hardcodedSwimHeight - views->MiddlePose()[3].y;
        }
        return heightOffset + hardcodedSwimOffset;
    }
    return heightOffset - s_crouchOffset;
}

static void UpdateCrouchOffset(bool isCrouching) {
    constexpr std::chrono::milliseconds crouchLerpDuration{ 150 };

    const auto now = std::chrono::steady_clock::now();
    if (isCrouching != s_wasCrouching) {
        s_crouchStateChangeTime = now;
        s_wasCrouching = isCrouching;
    }

    const float t = glm::clamp(std::chrono::duration<float>(now - s_crouchStateChangeTime).count() / std::chrono::duration<float>(crouchLerpDuration).count(), 0.0f, 1.0f);
    s_crouchOffset = isCrouching ? glm::mix(0.0f, hardcodedCrouchOffset, t) : glm::mix(hardcodedCrouchOffset, 0.0f, t);
}

static glm::fvec3 ResolveClimbWallCameraOffset(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& playerPos, const glm::fquat& cameraRotation) {
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

    if (!views.has_value()) {
        return glm::fvec3(0.0f);
    }

    const glm::fquat baseYaw = RenderUtils::swingTwistY(cameraRotation).second;
    const glm::fvec3 headPos = playerPos + baseYaw * GetAppliedMiddlePose(views.value()).first;

    const float clearance = glm::dot(headPos - s_climbWallPoint, s_climbWallNormal);
    if (!std::isfinite(clearance) || clearance >= hardcodedClimbWallClearance) {
        return glm::fvec3(0.0f);
    }

    return s_climbWallNormal * (hardcodedClimbWallClearance - clearance);
}

// call once per frame, not per eye, or the two eyes end up offset by different amounts
static void UpdateClimbWallCameraOffset(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& playerPos, const glm::fquat& cameraRotation) {
    constexpr float kClimbWallOffsetHalfLifeSeconds = 0.06f;

    static uint64_t s_lastTimeNs = 0;
    const uint64_t nowNs = GetTimeStamp();
    const float dt = s_lastTimeNs == 0 ? 0.0f : (float)(nowNs - s_lastTimeNs) * 1.0e-9f;
    s_lastTimeNs = nowNs;

    const glm::fvec3 target = ResolveClimbWallCameraOffset(views, playerPos, cameraRotation);
    const float blend = (dt <= 0.0f || dt > 0.5f) ? 1.0f : 1.0f - std::exp2(-dt / kClimbWallOffsetHalfLifeSeconds);
    s_climbWallCameraOffset = glm::mix(s_climbWallCameraOffset, target, blend);
}

static glm::fvec3 GetPlayerPosition() {
    return Player::GetMtxAddress() == 0 ? glm::fvec3(0.0f) : Guest::Get<BEMatrix34>(Player::GetMtxAddress()).getPos().getLE();
}

static glm::fvec3 ResolveGameplayAnchorPosition(const glm::fvec3& gameplayPos) {
    if (!GameState::IsFirstPerson() || Player::GetAddress() == 0) {
        return gameplayPos;
    }

    glm::fvec3 playerPos = GetPlayerPosition();
    playerPos.y += ResolvePlayerHeightAdjustment(HeightAdjustmentTarget::ReferenceAnchor);
    return playerPos + s_climbWallCameraOffset;
}

static glm::fquat ResolveBaseYaw(const glm::fquat& gameplayRot) {
    return RenderUtils::swingTwistY(gameplayRot).second;
}

std::pair<glm::fvec3, glm::fquat> Camera::BuildMiddlePose(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot) {
    if (!views.has_value()) {
        return { gameplayPos, gameplayRot };
    }

    const glm::fquat baseYaw = ResolveBaseYaw(gameplayRot);
    auto [middlePos, middleRot] = GetAppliedMiddlePose(views.value());
    return { ResolveGameplayAnchorPosition(gameplayPos) + (baseYaw * middlePos), baseYaw * middleRot };
}

std::pair<glm::fvec3, glm::fquat> Camera::BuildEyePose(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot, OpenXR::EyeSide side) {
    const glm::fvec3 basePos = ResolveGameplayAnchorPosition(gameplayPos);
    if (!views.has_value()) {
        return { basePos, gameplayRot };
    }

    const glm::fquat baseYaw = ResolveBaseYaw(gameplayRot);
    auto [eyePos, eyeRot] = GetAppliedEyePose(views.value(), side);
    return { basePos + (baseYaw * eyePos), baseYaw * eyeRot };
}

void Camera::UpdateDebugEyeViews(const std::optional<Tracking::HeadViews>& views, const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot) {
    for (uint32_t eyeIndex = 0; eyeIndex < 2; ++eyeIndex) {
        auto [eyePos, eyeRot] = BuildEyePose(views, gameplayPos, gameplayRot, (OpenXR::EyeSide)eyeIndex);
        glm::mat4 eyeWorld = glm::translate(glm::mat4(1.0f), eyePos) * glm::mat4_cast(eyeRot);
        DebugDraw::instance().UpdateEyeView(eyeIndex, glm::inverse(eyeWorld));
    }
}

void Camera::UpdateReferenceMtx(const glm::fvec3& gameplayPos, const glm::fquat& gameplayRot) {
    SetReferenceMtx(glm::translate(glm::identity<glm::fmat4>(), ResolveGameplayAnchorPosition(gameplayPos)) * glm::mat4(ResolveBaseYaw(gameplayRot)));
}

glm::mat4 Camera::GetFreshReferenceMtx() {
    glm::mat4 refMtx = GetReferenceMtx();
    refMtx[3] = glm::fvec4(ResolveGameplayAnchorPosition(glm::fvec3(refMtx[3])), 1.0f);
    return refMtx;
}

std::pair<glm::fvec3, glm::fquat> Camera::ResolveGameplayBasePose(const BESeadLookAtCamera& camera) {
    if (s_hasGameplayCameraTarget && GetFramesSinceLastUpdate() <= 4) {
        return { s_wsCameraPosition, s_wsCameraRotation };
    }

    glm::mat4 worldGame = glm::inverse(glm::mat4(camera.mtx.getLEMatrix()));
    return { glm::vec3(worldGame[3]), glm::quat_cast(worldGame) };
}

static void hook_UpdateCameraForGameplay(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::UseBlackBarsDuringEvents() || GameState::UseFlatCameraPresentation()) {
        return;
    }

    const uint32_t actCamPtr = hCPU->gpr[31];
    ActCamera actCam = {};
    Guest::Read(actCamPtr, &actCam);
    const std::optional<Tracking::HeadViews> views = Tracking::Views();

    glm::fvec3 oldCameraPosition = actCam.finalCamMtx.pos.getLE();
    glm::fvec3 oldCameraTarget = actCam.finalCamMtx.target.getLE();
    glm::fvec3 oldCameraUp = actCam.finalCamMtx.up.getLE();
    float oldCameraDistance = glm::distance(oldCameraPosition, oldCameraTarget);

    Log::print<RENDERING>("Getting gameplay camera (pos = {})", oldCameraPosition);

    const bool firstPerson = GameState::IsFirstPerson();
    if (firstPerson) {
        // remove verticality from the camera position to avoid pitch changes that aren't from the VR headset
        oldCameraPosition.y = oldCameraTarget.y;
    }

    glm::mat4 existingGameMtx = glm::lookAtRH(oldCameraPosition, oldCameraTarget, oldCameraUp);
    glm::fquat gameplayRotation = glm::quat_cast(glm::inverse(existingGameMtx));

    const float gameYawDegrees = YawDegreesFromRotation(gameplayRotation);
    const float gameStickYawDelta = ConsumeGameStickYawDelta();
    if (!IsEventTurningCamera()) {
        s_eventHeadYawDegrees.reset();
    }
    else if (!s_eventHeadYawDegrees.has_value() && views.has_value()) {
        s_eventHeadYawDegrees = YawDegreesFromRotation(glm::quat_cast(glm::fmat3(views->MiddlePose())));
    }
    if (ShouldAdoptGameCameraYaw()) {
        const glm::fquat eventFacing = glm::angleAxis(glm::radians(-s_eventHeadYawDegrees.value_or(0.0f)), glm::fvec3(0.0f, 1.0f, 0.0f));
        gameplayRotation = gameplayRotation * eventFacing;
        existingGameMtx = glm::mat4_cast(glm::inverse(eventFacing)) * existingGameMtx;
        s_baseYawDegrees = NormalizeDegrees(gameYawDegrees - s_eventHeadYawDegrees.value_or(0.0f));
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
    if (firstPerson && Player::GetAddress() != 0) {
        const PlayerMoveBitFlags moveBits = Player::GetMoveFlags();
        s_isSwimming = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_SWIMMING);
        UpdateCrouchOffset(HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_CROUCHING));

        glm::fvec3 playerPos = GetPlayerPosition();
        playerPos.y += ResolvePlayerHeightAdjustment(HeightAdjustmentTarget::RenderedCamera, views);

        UpdateClimbWallCameraOffset(views, playerPos, gameplayRotation);
        playerPos += s_climbWallCameraOffset;

        existingGameMtx = glm::inverse(glm::translate(glm::identity<glm::mat4>(), playerPos) * glm::mat4(gameplayRotation));
    }

    Camera::UpdateDebugEyeViews(views, s_wsCameraPosition, s_wsCameraRotation);

    if (!views.has_value()) {
        Log::print<ERROR>("hook_UpdateCameraForGameplay: No views available for the middle pose.");
        return;
    }
    glm::mat4 finalPose = glm::inverse(existingGameMtx) * GetAppliedHeadsetPose(views->MiddlePose());

    glm::fvec3 camPos = glm::fvec3(finalPose[3]);
    glm::fvec3 forward = -glm::normalize(glm::fvec3(finalPose[2]));
    glm::fvec3 up = glm::normalize(glm::fvec3(finalPose[1]));
    glm::fvec3 target = camPos + forward * oldCameraDistance;

    if (!TryUpdateGameplayCameraTarget(camPos, target, up)) {
        return;
    }

    Camera::UpdateReferenceMtx(s_wsCameraPosition, s_wsCameraRotation);

    actCam.finalCamMtx.pos = camPos;
    actCam.finalCamMtx.target = target;
    actCam.finalCamMtx.up = up;
    Guest::Write(actCamPtr, &actCam);
    s_framesSinceLastUpdate = 0;
}

// the gameplay camera is just a look-at camera that doesn't seem to have a pivot point at the point we hook it
// so this hook adjusts it afterwards to have properly working gameplay camera rotations
static void hook_AdjustGameplayCameraPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::IsFlatCameraModeActive() || GameState::IsThirdPerson() || !s_hasGameplayCameraTarget || Camera::GetFramesSinceLastUpdate() > 4 || GameState::UseBlackBarsDuringEvents()) {
        return;
    }

    uint32_t pivotPtr = hCPU->gpr[4];
    if (pivotPtr == 0) {
        return;
    }

    const glm::fvec3 currentPivot = Guest::Get<BEVec3>(pivotPtr).getLE();
    const glm::fvec3 pivotDelta = s_lastGameplayCameraTarget - currentPivot;
    if (!glm::all(glm::isfinite(pivotDelta)) || glm::dot(pivotDelta, pivotDelta) <= 1.0e-6f) {
        return;
    }

    BEVec3 pivot(s_lastGameplayCameraTarget.x, s_lastGameplayCameraTarget.y, s_lastGameplayCameraTarget.z);
    Guest::Write(pivotPtr, &pivot);
}

// only the VR controller's stick is withheld from the game, so this is in practice the rotation a
// real controller's right stick asked for
static void hook_AddGameCameraStickYaw(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::IsFlatCameraModeActive()) {
        return;
    }

    const float yawDelta = (float)hCPU->fpr[1].fp0;
    if (yawDelta == 0.0f || !std::isfinite(yawDelta) || ShouldAdoptGameCameraYaw()) {
        return;
    }

    s_pendingGameStickYawDelta = NormalizeDegrees(s_pendingGameStickYawDelta + yawDelta);
}

static void hook_CaptureClimbWallSurface(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    constexpr uint32_t kClimbWallNormalOffset = 0x54;
    constexpr uint32_t kClimbWallPointOffset = 0x60;

    const uint32_t wallStatePtr = hCPU->gpr[3];
    if (wallStatePtr == 0) {
        return;
    }

    const glm::fvec3 normal = Guest::Get<BEVec3>(wallStatePtr + kClimbWallNormalOffset).getLE();
    const glm::fvec3 point = Guest::Get<BEVec3>(wallStatePtr + kClimbWallPointOffset).getLE();
    // the game leaves the normal at zero until the sweep has actually hit something
    if (!IsAllFinite(normal) || !IsAllFinite(point) || glm::dot(normal, normal) < 0.5f) {
        return;
    }

    s_climbWallNormal = glm::normalize(normal);
    s_climbWallPoint = point;
    s_climbWallUpdateNs = GetTimeStamp();
}

static bool ShouldApplySnapTurnToGameCamera() {
    return !GameState::IsFlatCameraModeActive() && GameState::IsFirstPerson() && !Events::IsActive();
}

static void hook_SnapTurnCameraPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (ShouldApplySnapTurnToGameCamera()) {
        ApplyYawDeltaToComputedCameraPivot(hCPU, ConsumePendingGameYawCorrection());
    }
}

static bool ApplyYawDeltaToCameraTailPivot(PPCInterpreter_t* hCPU, float yawDeltaDegrees) {
    if (yawDeltaDegrees == 0.0f || hCPU->gpr[3] == 0 || hCPU->gpr[4] == 0) {
        return false;
    }

    constexpr uint32_t kCameraTailFloat98Offset = 0x98;
    constexpr uint32_t kSphericalYawOffset = 8;

    uint32_t sphericalYawAddr = hCPU->gpr[4] + kSphericalYawOffset;
    float sphericalYaw = Guest::Get<float>(sphericalYawAddr).getLE();
    if (!std::isfinite(sphericalYaw)) {
        return false;
    }
    Guest::Set<float>(sphericalYawAddr, NormalizeDegrees(sphericalYaw - yawDeltaDegrees));

    uint32_t float98Addr = hCPU->gpr[3] + kCameraTailFloat98Offset;
    float float98 = Guest::Get<float>(float98Addr).getLE();
    if (std::isfinite(float98)) {
        Guest::Set<float>(float98Addr, NormalizeDegrees(float98 - yawDeltaDegrees));
    }

    return true;
}

static void hook_SnapTurnCameraTailPivot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (ShouldApplySnapTurnToGameCamera()) {
        ApplyYawDeltaToCameraTailPivot(hCPU, ConsumePendingGameYawCorrection());
    }
}

static void hook_ShouldSkipEventCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto eventSettings = Events::GetFirstPersonSettings();
    hCPU->gpr[3] = GameState::IsFirstPerson() && eventSettings.has_value() && eventSettings->ignoreCameraRotation ? 1 : 0;
}

static void hook_UseCameraDistance(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::IsFirstPerson()) {
        hCPU->fpr[13].fp0 = 0.0f;
    }
    else if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON) {
        hCPU->fpr[13].fp0 = GetSettings().thirdPlayerDistance;
    }
    else {
        hCPU->fpr[13].fp0 = 0.5f; // use default distance when using the first-person camera
    }
}

// Decides both camera collision passes: the early-out in sub_2E61FC0 and the sub_2E622FC predicate.
// only needed in third person and in cutscenes with black bars
static void hook_SkipCameraCollision(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const bool skipCollision = Tracking::IsSessionRunning()
        && GameState::IsInGame()
        && !GameState::IsThirdPerson()
        && !GameState::UseBlackBarsDuringEvents();
    hCPU->gpr[3] = skipCollision ? 1u : 0u;

    static std::atomic_int s_lastDecision = -1;
    if (s_lastDecision.exchange(skipCollision ? 1 : 0) != (skipCollision ? 1 : 0)) {
        Log::print<VERBOSE>("[CameraCollision] {} camera collision (thirdPerson={}, cutscene={})", skipCollision ? "skipping" : "running", GameState::IsThirdPerson(), Events::IsActive());
    }
}

struct CameraParamValueOffset {
    std::string name;
    uint32_t offsetInsideCamera;
    bool storedOriginalValue = false;
    float originalValue;
};

// key = camera instance, value = list of parameter names and their offsets inside the camera object
static std::mutex s_cameraParametersLock;
static std::unordered_map<uint32_t, std::vector<CameraParamValueOffset>> s_cameraParameters;

static void ApplyCameraParameterOverrides(uint32_t cameraInstance, uint32_t cameraVtbl, uint32_t cameraChaseVtbl) {
    std::scoped_lock lock(s_cameraParametersLock);

    auto it = s_cameraParameters.find(cameraInstance);
    if (it == s_cameraParameters.end()) {
        return;
    }
    if (cameraVtbl != cameraChaseVtbl) {
        // entries were stored for a CameraChase that used to live at this address, so they describe a freed object
        s_cameraParameters.erase(it);
        return;
    }

    std::erase_if(it->second, [](CameraParamValueOffset& paramEntry) {
        uint32_t originalValuePtr = Guest::Get<uint32_t>(paramEntry.offsetInsideCamera).getLE();
        if (originalValuePtr < 0x10000000 || originalValuePtr >= 0x50000000 || (originalValuePtr & 3) != 0) {
            return true;
        }
        BEType<float>* paramValueBE = Guest::HostPointer<BEType<float>>(originalValuePtr);

        if (!paramEntry.storedOriginalValue) {
            paramEntry.originalValue = paramValueBE->getLE();
            paramEntry.storedOriginalValue = true;
        }

        *paramValueBE = GameState::IsFirstPerson() ? 0.0f : paramEntry.originalValue;
        return false;
    });
}

static void hook_ReplaceCameraMode(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t currCameraInstance = hCPU->gpr[3];
    uint32_t cameraChaseInstance = hCPU->gpr[4]; // this is currently a pointer to the regular camera mode (CameraChase)
    uint32_t currentCameraVtbl = hCPU->gpr[5];

    constexpr uint32_t kCameraChaseVtbl = 0x101B34F4;
    constexpr uint32_t kCameraTailVtbl = 0x101BC278;

    ApplyCameraParameterOverrides(currCameraInstance, currentCameraVtbl, kCameraChaseVtbl);

    static uint32_t s_lastLoggedCameraVtbl = 0;
    static uint32_t s_lastLoggedCameraInstance = 0;
    static uint32_t s_lastLoggedCameraChaseInstance = 0;
    static bool s_wasFlatCameraMode = false;
    static uint32_t s_lastCameraChaseInstance = 0;

    const bool isFlatCameraMode = GameState::IsFlatCameraModeActive();
    const bool isSnapTurnCamera = currentCameraVtbl == kCameraChaseVtbl || currentCameraVtbl == kCameraTailVtbl;
    // a new CameraChase means a load or warp, so take the game's facing again
    if (s_wasFlatCameraMode != isFlatCameraMode || s_lastCameraChaseInstance != cameraChaseInstance) {
        s_hasBaseYaw = false;
        s_pendingGameYawCorrection = 0.0f;
        s_pendingGameStickYawDelta = 0.0f;
        s_lastCameraChaseInstance = cameraChaseInstance;
    }
    if (s_wasFlatCameraMode != isFlatCameraMode) {
        if (isFlatCameraMode) {
            StereoView::ResetFlatCameraFov();
        }
        Log::print<INFO>("Flat camera mode {} (scope={}, CameraFinder={})", isFlatCameraMode ? "started" : "ended", GameState::IsScopeModeActive(), GameState::IsCameraFinderModeActive());
        s_wasFlatCameraMode = isFlatCameraMode;
    }

    if (currentCameraVtbl != s_lastLoggedCameraVtbl || currCameraInstance != s_lastLoggedCameraInstance || cameraChaseInstance != s_lastLoggedCameraChaseInstance) {
        Log::print<VERBOSE>("Camera changed (vtbl={:#X}, camera={:#X}, chase={:#X}, snapTurnActive={}, flatCameraActive={})", currentCameraVtbl, currCameraInstance, cameraChaseInstance, isSnapTurnCamera, isFlatCameraMode);
        s_lastLoggedCameraVtbl = currentCameraVtbl;
        s_lastLoggedCameraInstance = currCameraInstance;
        s_lastLoggedCameraChaseInstance = cameraChaseInstance;
    }

    s_appliesFirstPersonTurning = isSnapTurnCamera;
}

constexpr uint32_t orig_GetStaticParam_float_funcAddr = 0x030E9BE0;

// hook for ksys::act::ai::ActionBase::getStaticParam<FLOAT> calls
static void hook_OverwriteFloatParam(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;

    uint32_t actionPtr = hCPU->gpr[3];
    uint32_t destFloatPtr = hCPU->gpr[4];
    uint32_t paramNameArgPtr = hCPU->gpr[5];
    if (actionPtr == 0 || destFloatPtr == 0 || paramNameArgPtr == 0) {
        return;
    }

    uint32_t paramNamePtr = Guest::Get<uint32_t>(paramNameArgPtr).getLE();
    if (paramNamePtr == 0) {
        return;
    }

    const std::string_view paramName = Guest::CString(paramNamePtr);
    if (Player::TrackDamageFloatParam(actionPtr, destFloatPtr, paramName)) {
        return;
    }

    std::scoped_lock lock(s_cameraParametersLock);
    auto& paramList = s_cameraParameters[actionPtr];
    if (std::ranges::none_of(paramList, [paramName](const CameraParamValueOffset& entry) { return entry.name == paramName; })) {
        Log::print<PPC>("Storing float param '{}' offset {:08X} for action at {:08X}", paramName, destFloatPtr, actionPtr);
        paramList.push_back({ std::string(paramName), destFloatPtr });
    }
}

void Camera::RegisterHooks() {
    CemuHost::RegisterHook("hook_UpdateCameraForGameplay", &hook_UpdateCameraForGameplay);
    CemuHost::RegisterHook("hook_AdjustGameplayCameraPivot", &hook_AdjustGameplayCameraPivot);
    CemuHost::RegisterHook("hook_AddGameCameraStickYaw", &hook_AddGameCameraStickYaw);
    CemuHost::RegisterHook("hook_CaptureClimbWallSurface", &hook_CaptureClimbWallSurface);
    CemuHost::RegisterHook("hook_SnapTurnCameraPivot", &hook_SnapTurnCameraPivot);
    CemuHost::RegisterHook("hook_SnapTurnCameraTailPivot", &hook_SnapTurnCameraTailPivot);
    CemuHost::RegisterHook("hook_ShouldSkipEventCamera", &hook_ShouldSkipEventCamera);
    CemuHost::RegisterHook("hook_UseCameraDistance", &hook_UseCameraDistance);
    CemuHost::RegisterHook("hook_SkipCameraCollision", &hook_SkipCameraCollision);
    CemuHost::RegisterHook("hook_ReplaceCameraMode", &hook_ReplaceCameraMode);
    CemuHost::RegisterHook("hook_OverwriteFloatParam", &hook_OverwriteFloatParam);
}
