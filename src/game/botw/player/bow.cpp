#include "pch.h"
#include "bow.h"
#include "hand_aim.h"
#include "player.h"
#include "skeleton.h"
#include "weapon.h"
#include "roomscale.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/camera.h"
#include "rendering/tracking.h"
#include "utils/debug_draw.h"

enum class BowParamActionKind : uint8_t {
    Move,
    Homing,
    MoveWithStickOffset,
    MoveForLargeObject,
    Sky,
    Unknown,
};

enum class BowVisualizationOriginSource : uint8_t {
    Unknown,
    WeaponAttackPos,
    RightHandPose,
    LeftHandPose,
    CameraPose,
};

enum class BowVisualizationDirectionSource : uint8_t {
    Unknown,
    TargetPos,
    BowEntityForward,
    RightHandAimPose,
    LeftHandAimPose,
    RightHandForward,
    LeftHandForward,
    CameraAim,
    CameraForward,
};

struct BowVisualizationState {
    bool valid = false;
    bool hasLaunchOrigin = false;
    bool hasLaunchDirection = false;
    bool hasTargetPos = false;
    bool hasRelativeVel = false;
    bool hasAtRange = false;
    bool hasFallSpeedRatioByRange = false;
    bool usedSyntheticTargetPos = false;
    uint32_t weaponPtr = 0;
    float drawAmount = 0.0f;
    float launchSpeed = 0.0f;
    float accel = 0.0f;
    float aimSpeed = 0.0f;
    float fallAccel = 0.0f;
    float fallAimSpeed = 0.0f;
    float gravity = 0.0f;
    float atRange = 0.0f;
    float fallSpeedRatioByRange = 0.0f;
    BowParamActionKind actionKind = BowParamActionKind::Unknown;
    BowVisualizationOriginSource originSource = BowVisualizationOriginSource::Unknown;
    BowVisualizationDirectionSource directionSource = BowVisualizationDirectionSource::Unknown;
    glm::vec3 launchOrigin = glm::vec3(0.0f);
    glm::vec3 launchDirection = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 targetPos = glm::vec3(0.0f);
    glm::vec3 firstSpeed = glm::vec3(0.0f);
    glm::vec3 relativeVel = glm::vec3(0.0f);
};

struct BowParamSnapshot {
    bool valid = false;
    bool hasIsPlayerShot = false;
    bool isPlayerShot = false;
    uint64_t timestampMs = 0;
    uint32_t previewAiPtr = 0;
};

static const char* GetBowParamActionKindName(BowParamActionKind actionKind);

static constexpr const char* kParamFirstSpeed = "FirstSpeed";
static constexpr const char* kParamTargetPos = "TargetPos";
static constexpr const char* kParamHomingTargetPos = "HomingTargetPos";
static constexpr const char* kParamIsShootByPlayer = "IsShootByPlayer";

static constexpr uint32_t kArrowShotVisualizationCallsiteStart = 0x02705644;
static constexpr uint32_t kArrowShotVisualizationCallsiteEnd = 0x02706584;
static constexpr uint32_t kArrowShootMoveLoadParamsCallsiteStart = 0x020E29AC;
static constexpr uint32_t kArrowShootMoveLoadParamsCallsiteEnd = 0x020E3A4C;
static constexpr uint32_t kArrowShootHomingLoadParamsCallsiteStart = 0x020E1538;
static constexpr uint32_t kArrowShootHomingLoadParamsCallsiteEnd = 0x020E1808;
static constexpr uint32_t kArrowShootMoveWithStickOffsetLoadParamsCallsiteStart = 0x020E8DEC;
static constexpr uint32_t kArrowShootMoveWithStickOffsetLoadParamsCallsiteEnd = 0x020E8E40;
static constexpr uint32_t kArrowHandleStateShootHomingDecisionCallsite = 0x0270640C;
static constexpr uint32_t kArrowHandleStateShootSkyDecisionCallsite = 0x027064D8;
static constexpr uint32_t kArrowHandleStateShootMoveDecisionCallsite = 0x02706518;
static constexpr uint32_t kAiChangeChildAddress = 0x037B8284;
static constexpr uint64_t kArrowShootDecisionLifetimeMs = 1500;

static std::mutex s_captureMutex;
static BowParamSnapshot s_previewSnapshot = {};
static BowParamActionKind s_lastArrowShootDecisionKind = BowParamActionKind::Unknown;
static uint64_t s_lastArrowShootDecisionTimestampMs = 0;
static std::atomic<float> s_arrowVfrScale{1.0f};

static BowParamSnapshot& EnsurePreviewSnapshotLocked();
static BowParamActionKind ResolveArrowShootDecisionKind(uint32_t callsiteAddress);
static BowParamActionKind GetFreshArrowShootDecisionKindLocked();

static bool IsFiniteVec3(const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

static bool IsReturnAddressInRange(uint32_t returnAddress, uint32_t start, uint32_t end) {
    return returnAddress >= start && returnAddress < end;
}

static bool IsPreviewVisualReturnAddress(uint32_t returnAddress) {
    return IsReturnAddressInRange(returnAddress, kArrowShotVisualizationCallsiteStart, kArrowShotVisualizationCallsiteEnd);
}

static bool IsShotLoadReturnAddress(uint32_t returnAddress) {
    return IsReturnAddressInRange(returnAddress, kArrowShootMoveLoadParamsCallsiteStart, kArrowShootMoveLoadParamsCallsiteEnd) || IsReturnAddressInRange(returnAddress, kArrowShootHomingLoadParamsCallsiteStart, kArrowShootHomingLoadParamsCallsiteEnd) || IsReturnAddressInRange(returnAddress, kArrowShootMoveWithStickOffsetLoadParamsCallsiteStart, kArrowShootMoveWithStickOffsetLoadParamsCallsiteEnd);
}

static const char* ResolveParamName(uint32_t keyPtr) {
    if (keyPtr == 0) {
        return nullptr;
    }

    return Guest::CString(keyPtr);
}

static bool ParamMatches(const char* paramName, const char* expected) {
    return paramName != nullptr && std::strcmp(paramName, expected) == 0;
}

static uint32_t ReadInlineParamPackAddVec3CallerSavedR30(PPCInterpreter_t* hCPU) {
    uint32_t savedCallerR30 = 0;
    // addVec3 saves the caller's r30 at sp+0x10 before reusing r30 for the SafeString* argument
    Guest::ReadBE(hCPU->gpr[1] + 0x10, &savedCallerR30);
    return savedCallerR30;
}

static void CapturePreviewHookContext(PPCInterpreter_t* hCPU, uint32_t sourceReturnAddress) {
    if (!IsPreviewVisualReturnAddress(sourceReturnAddress)) {
        return;
    }

    std::scoped_lock lock(s_captureMutex);
    BowParamSnapshot& snapshot = EnsurePreviewSnapshotLocked();
    snapshot.previewAiPtr = ReadInlineParamPackAddVec3CallerSavedR30(hCPU);
}

static void RecordArrowShootDecision(uint32_t arrowAiPtr, uint32_t callsiteAddress) {
    const BowParamActionKind actionKind = ResolveArrowShootDecisionKind(callsiteAddress);
    if (actionKind == BowParamActionKind::Unknown) {
        return;
    }

    std::scoped_lock lock(s_captureMutex);
    BowParamSnapshot& snapshot = EnsurePreviewSnapshotLocked();
    snapshot.previewAiPtr = arrowAiPtr;
    if (snapshot.hasIsPlayerShot && snapshot.isPlayerShot) {
        s_lastArrowShootDecisionKind = actionKind;
        s_lastArrowShootDecisionTimestampMs = GetTimeStampMs();
    }
}

static BowParamActionKind ResolveArrowShootDecisionKind(uint32_t callsiteAddress) {
    switch (callsiteAddress) {
        case kArrowHandleStateShootMoveDecisionCallsite:
            return BowParamActionKind::Move;
        case kArrowHandleStateShootHomingDecisionCallsite:
            return BowParamActionKind::Homing;
        case kArrowHandleStateShootSkyDecisionCallsite:
            return BowParamActionKind::Sky;
        default:
            return BowParamActionKind::Unknown;
    }
}

static BowParamActionKind GetFreshArrowShootDecisionKindLocked() {
    if (s_lastArrowShootDecisionKind == BowParamActionKind::Unknown) {
        return BowParamActionKind::Unknown;
    }

    const uint64_t now = GetTimeStampMs();
    if (now < s_lastArrowShootDecisionTimestampMs || now - s_lastArrowShootDecisionTimestampMs > kArrowShootDecisionLifetimeMs) {
        return BowParamActionKind::Unknown;
    }

    return s_lastArrowShootDecisionKind;
}

static BowParamSnapshot& BeginPreviewSnapshotLocked() {
    s_previewSnapshot = {};
    s_previewSnapshot.valid = true;
    s_previewSnapshot.timestampMs = GetTimeStampMs();
    return s_previewSnapshot;
}

static BowParamSnapshot& EnsurePreviewSnapshotLocked() {
    if (!s_previewSnapshot.valid) {
        return BeginPreviewSnapshotLocked();
    }

    s_previewSnapshot.timestampMs = GetTimeStampMs();
    return s_previewSnapshot;
}

static uint32_t ReadInlineParamPackAddVec3CallerReturnAddress(PPCInterpreter_t* hCPU) {
    uint32_t returnAddress = 0;
    Guest::ReadBE(hCPU->gpr[1] + 0x1C, &returnAddress);
    return returnAddress;
}

static void RecordBool(const InlineParamBool& boolParam, uint32_t sourceReturnAddress) {
    if (!IsPreviewVisualReturnAddress(sourceReturnAddress)) {
        return;
    }

    const char* paramName = ResolveParamName(boolParam.keyPtr.getLE());
    if (!ParamMatches(paramName, kParamIsShootByPlayer)) {
        return;
    }

    std::scoped_lock lock(s_captureMutex);
    BowParamSnapshot& snapshot = EnsurePreviewSnapshotLocked();
    snapshot.hasIsPlayerShot = true;
    snapshot.isPlayerShot = boolParam.value.getLE() != 0;
}

static constexpr float kPreviewOriginForwardOffset = 0.05f;
static constexpr float kPreviewArcControllerOriginForwardOffset = 0.18f * 5.0;
static constexpr float kPreviewOriginMaxSnapDistanceSq = 1.0f;
static constexpr float kPreviewAimDistance = 100.0f;
static constexpr float kLaunchTargetDistance = 100.0f;
static constexpr float kMinimumArrowSpeed = 0.01f;
static constexpr float kMinimumBowDrawAmount = 0.001f;
static constexpr uint64_t kActiveSnapshotLifetimeMs = 1500;
static constexpr uint64_t kPendingShotOverrideLifetimeMs = 250;
static constexpr uint32_t kPresetDefaultMaxFrames = 48;
static constexpr float kPreviewFirstFrameStep = 0.01f;
static constexpr float kPreviewSpeedEpsilon = 0.00000011920929f;
static constexpr float kPreviewFallSpeedRatioRangeBase = 20.0f;
static constexpr float kDefaultFallSpeedRatioByRange = 1.0f;
static constexpr float kDefaultVfrScale = 1.0f;
static constexpr float kPreviewImmediateFallRange = 10.0f;
static constexpr uint32_t kActorParamPtrOffset = 0x39C;
static constexpr uint32_t kActorParamGParamListOffset = 0x78;
static constexpr uint32_t kWeaponAttackPosOffset = 0x770;
static constexpr uint32_t kWeaponTargetPosOffset = 0x77C;
static constexpr uint32_t kWeaponAttackPosValidOffset = 0x7A0;
static constexpr uint32_t kWeaponDrawAmountOffset = 0x94C;
static constexpr uint32_t kWeaponActorMtxOffset = 0x1F8;
static constexpr uint32_t kGParamListNumObjectsOffset = 0x240;
static constexpr uint32_t kGParamListObjectsPtrOffset = 0x244;
static constexpr uint32_t kAttackObjectSlotOffset = 0x20;
static constexpr uint32_t kBowObjectSlotOffset = 0x44;
static constexpr uint32_t kAttackParamAtRangeOffset = 0x5C;
static constexpr uint32_t kBowParamFirstSpeedOffset = 0x104;
static constexpr uint32_t kBowParamAccelerationOffset = 0x114;
static constexpr uint32_t kBowParamStabilitySpeedOffset = 0x124;
static constexpr uint32_t kBowParamFallAccelerationOffset = 0x134;
static constexpr uint32_t kBowParamFallStabilitySpeedOffset = 0x144;
static constexpr uint32_t kBowParamGravityOffset = 0x154;
static constexpr uint32_t kWeaponOriginalScaleZOffset = 0x998;
static constexpr uint32_t kInlineParamPackEntryCountOffset = 0x680;
static constexpr uint32_t kInlineParamPackMaxEntries = 32;

struct LiveBowState {
    bool valid = false;
    bool hasOrigin = false;
    bool hasTargetPos = false;
    uint32_t weaponPtr = 0;
    float drawAmount = 0.0f;
    float launchSpeed = 0.0f;
    float accel = 0.0f;
    float aimSpeed = 0.0f;
    float fallAccel = 0.0f;
    float fallAimSpeed = 0.0f;
    float gravity = 0.0f;
    float atRange = 0.0f;
    glm::vec3 origin = glm::vec3(0.0f);
    glm::vec3 targetPos = glm::vec3(0.0f);
};

struct PoseFallbackState {
    bool valid = false;
    bool hasTargetPos = false;
    glm::vec3 origin = glm::vec3(0.0f);
    glm::vec3 direction = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 targetPos = glm::vec3(0.0f);
    BowVisualizationOriginSource originSource = BowVisualizationOriginSource::Unknown;
    BowVisualizationDirectionSource directionSource = BowVisualizationDirectionSource::Unknown;
};

using PreviewPoint = glm::vec3;

struct PendingShotOverride {
    bool valid = false;
    bool loadParamsConfirmed = false;
    uint64_t timestampMs = 0;
    float launchSpeed = 0.0f;
    glm::vec3 origin = glm::vec3(0.0f);
    glm::vec3 direction = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 targetPos = glm::vec3(0.0f);
};

static std::mutex s_pendingShotOverrideMutex;
static PendingShotOverride s_pendingShotOverride = {};

static bool HasFiniteDirection(const glm::vec3& value) {
    return IsFiniteVec3(value) && glm::length2(value) > 0.000001f;
}

static bool IsNearZero(float value) {
    return value >= -kPreviewSpeedEpsilon && value <= kPreviewSpeedEpsilon;
}

static glm::vec3 NormalizeOrFallback(const glm::vec3& value, const glm::vec3& fallback) {
    if (HasFiniteDirection(value)) {
        return glm::normalize(value);
    }

    return fallback;
}

static glm::vec3 ScaleVectorToLengthOrScaleRaw(const glm::vec3& value, float targetLength) {
    if (!IsFiniteVec3(value) || !std::isfinite(targetLength)) {
        return glm::vec3(0.0f);
    }

    const float magnitudeSq = glm::length2(value);
    if (magnitudeSq > 0.000001f) {
        return glm::normalize(value) * targetLength;
    }

    return value * targetLength;
}

static bool IsSnapshotFresh(const BowParamSnapshot& snapshot, uint64_t maxAgeMs) {
    if (!snapshot.valid) {
        return false;
    }

    const uint64_t now = GetTimeStampMs();
    if (now < snapshot.timestampMs) {
        return false;
    }

    return now - snapshot.timestampMs <= maxAgeMs;
}

static BowParamActionKind GetPreviewActionKind() {
    std::scoped_lock lock(s_captureMutex);
    const BowParamActionKind actionKind = GetFreshArrowShootDecisionKindLocked();
    return actionKind == BowParamActionKind::Unknown ? BowParamActionKind::Move : actionKind;
}

static bool TryGetMatchingPlayerPreviewSnapshot(uint32_t previewAiPtr) {
    if (previewAiPtr == 0) {
        return false;
    }

    std::scoped_lock lock(s_captureMutex);
    const BowParamSnapshot& snapshot = s_previewSnapshot;
    if (!snapshot.valid || snapshot.previewAiPtr != previewAiPtr || !snapshot.hasIsPlayerShot || !snapshot.isPlayerShot || !IsSnapshotFresh(snapshot, kActiveSnapshotLifetimeMs)) {
        return false;
    }

    return true;
}

static uint32_t GetHeldLeftBowWeaponPtr() {
    const uint32_t weaponPtr = Weapons::GetHeldActor(OpenXR::EyeSide::LEFT);
    if (weaponPtr == 0) {
        return 0;
    }

    Weapon weapon = {};
    Guest::Read(weaponPtr, &weapon);
    return weapon.type.getLE() == WeaponType::Bow ? weaponPtr : 0;
}

static bool TryReadNativeWeaponAttackOrigin(uint32_t weaponPtr, glm::vec3& outOrigin) {
    outOrigin = glm::vec3(0.0f);
    if (weaponPtr == 0) {
        return false;
    }

    uint8_t attackPosValid = 0;
    Guest::Read(weaponPtr + kWeaponAttackPosValidOffset, &attackPosValid);
    if (attackPosValid == 0) {
        return false;
    }

    BEVec3 beOrigin = {};
    Guest::Read(weaponPtr + kWeaponAttackPosOffset, &beOrigin);
    outOrigin = beOrigin.getLE();
    return IsFiniteVec3(outOrigin);
}

static bool TryReadWeaponActorPosition(uint32_t weaponPtr, glm::vec3& outPosition) {
    outPosition = glm::vec3(0.0f);
    if (weaponPtr == 0) {
        return false;
    }

    BEMatrix34 actorMtx = {};
    Guest::Read(weaponPtr + kWeaponActorMtxOffset, &actorMtx);
    outPosition = actorMtx.getPos().getLE();
    return IsFiniteVec3(outPosition);
}

static bool TryReadWeaponAttackOrigin(uint32_t weaponPtr, glm::vec3& outOrigin) {
    if (TryReadNativeWeaponAttackOrigin(weaponPtr, outOrigin)) {
        return true;
    }
    return TryReadWeaponActorPosition(weaponPtr, outOrigin);
}

static bool TryReadWeaponTargetPos(uint32_t weaponPtr, glm::vec3& outTargetPos) {
    outTargetPos = glm::vec3(0.0f);
    if (weaponPtr == 0) {
        return false;
    }

    BEVec3 beTargetPos = {};
    Guest::Read(weaponPtr + kWeaponTargetPosOffset, &beTargetPos);
    outTargetPos = beTargetPos.getLE();
    return IsFiniteVec3(outTargetPos);
}

static bool TryGetBowEntityWorldForward(glm::vec3& outForward) {
    const uint32_t bowWeaponPtr = GetHeldLeftBowWeaponPtr();
    if (bowWeaponPtr == 0) {
        return false;
    }

    BEMatrix34 bowMtx = {};
    Guest::Read(bowWeaponPtr + kWeaponActorMtxOffset, &bowMtx);
    const glm::mat4x3 m = bowMtx.getLEMatrix();
    const glm::vec3 forward = -glm::vec3(m[0]);

    if (!HasFiniteDirection(forward)) {
        return false;
    }

    outForward = glm::normalize(forward);
    return true;
}

static bool TryReadLiveBowState(LiveBowState& outState) {
    outState = {};

    const uint32_t weaponPtr = GetHeldLeftBowWeaponPtr();
    if (weaponPtr == 0) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: no bow equipped in left hand");
        return false;
    }

    float drawAmount = 0.0f;
    Guest::ReadBE(weaponPtr + kWeaponDrawAmountOffset, &drawAmount);
    drawAmount = glm::clamp(drawAmount, 0.0f, 1.5f);

    if (GameState::IsFirstPerson() && drawAmount <= kMinimumBowDrawAmount && Skeleton::IsArrowNearBowStart()) {
        drawAmount = 1.0f;
    }

    uint32_t actorParamPtr = 0;
    Guest::ReadBE(weaponPtr + kActorParamPtrOffset, &actorParamPtr);
    if (actorParamPtr == 0) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: actorParamPtr is 0 (weapon=0x{:08X}, draw={:.3f})", weaponPtr, drawAmount);
        return false;
    }

    uint32_t gParamListPtr = 0;
    Guest::ReadBE(actorParamPtr + kActorParamGParamListOffset, &gParamListPtr);
    if (gParamListPtr == 0) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: gParamListPtr is 0");
        return false;
    }

    uint32_t numObjects = 0;
    uint32_t objectsPtr = 0;
    Guest::ReadBE(gParamListPtr + kGParamListNumObjectsOffset, &numObjects);
    Guest::ReadBE(gParamListPtr + kGParamListObjectsPtrOffset, &objectsPtr);
    if (numObjects <= (kBowObjectSlotOffset / sizeof(uint32_t)) || objectsPtr == 0) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: bad GParamList (numObjects={}, objectsPtr=0x{:08X})", numObjects, objectsPtr);
        return false;
    }

    uint32_t bowParamPtr = 0;
    Guest::ReadBE(objectsPtr + kBowObjectSlotOffset, &bowParamPtr);
    if (bowParamPtr == 0) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: bowParamPtr is 0");
        return false;
    }

    float firstSpeedRaw = 0.0f;
    float accelerationRaw = 0.0f;
    float stableSpeedRaw = 0.0f;
    float fallAccelerationRaw = 0.0f;
    float fallStableSpeedRaw = 0.0f;
    float gravityRaw = 0.0f;
    float attackAtRangeRaw = 0.0f;
    float originalScaleZ = 1.0f;
    glm::vec3 targetPos = glm::vec3(0.0f);
    glm::vec3 origin = glm::vec3(0.0f);

    Guest::ReadBE(bowParamPtr + kBowParamFirstSpeedOffset, &firstSpeedRaw);
    Guest::ReadBE(bowParamPtr + kBowParamAccelerationOffset, &accelerationRaw);
    Guest::ReadBE(bowParamPtr + kBowParamStabilitySpeedOffset, &stableSpeedRaw);
    Guest::ReadBE(bowParamPtr + kBowParamFallAccelerationOffset, &fallAccelerationRaw);
    Guest::ReadBE(bowParamPtr + kBowParamFallStabilitySpeedOffset, &fallStableSpeedRaw);
    Guest::ReadBE(bowParamPtr + kBowParamGravityOffset, &gravityRaw);
    if (numObjects > (kAttackObjectSlotOffset / sizeof(uint32_t))) {
        uint32_t attackParamPtr = 0;
        Guest::ReadBE(objectsPtr + kAttackObjectSlotOffset, &attackParamPtr);
        if (attackParamPtr != 0) {
            Guest::ReadBE(attackParamPtr + kAttackParamAtRangeOffset, &attackAtRangeRaw);
        }
    }

    Guest::ReadBE(weaponPtr + kWeaponOriginalScaleZOffset, &originalScaleZ);

    if (!std::isfinite(firstSpeedRaw) || !std::isfinite(accelerationRaw) || !std::isfinite(stableSpeedRaw) || !std::isfinite(fallAccelerationRaw) || !std::isfinite(fallStableSpeedRaw) || !std::isfinite(gravityRaw) || !std::isfinite(attackAtRangeRaw) || !std::isfinite(originalScaleZ)) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryReadLiveBowState FAILED: non-finite bow params (firstSpeed={}, accel={}, gravity={})", firstSpeedRaw, accelerationRaw, gravityRaw);
        return false;
    }

    outState.valid = true;
    outState.weaponPtr = weaponPtr;
    outState.drawAmount = drawAmount;
    outState.launchSpeed = glm::max(firstSpeedRaw * drawAmount * drawAmount * Skeleton::GetBowPullPower(), 0.0f);
    outState.accel = accelerationRaw;
    outState.aimSpeed = stableSpeedRaw;
    outState.fallAccel = std::abs(fallAccelerationRaw);
    outState.fallAimSpeed = fallStableSpeedRaw;
    outState.gravity = gravityRaw / 900.0f;
    outState.atRange = attackAtRangeRaw * originalScaleZ;
    outState.hasOrigin = TryReadWeaponAttackOrigin(weaponPtr, origin);
    outState.origin = origin;
    outState.hasTargetPos = TryReadWeaponTargetPos(weaponPtr, targetPos) && glm::length2(targetPos) > 0.000001f;
    outState.targetPos = targetPos;
    return true;
}

static bool TryGetGameplayCameraCenterPose(glm::vec3& outPosition, glm::vec3& outForward, long frameIdx = -1) {
    std::optional<glm::fmat4> middlePoseOpt = Tracking::HeadPoseOfCapturedFrame(frameIdx);
    if (!middlePoseOpt.has_value()) {
        return false;
    }

    glm::mat4 middlePose = middlePoseOpt.value();
    middlePose[3] = glm::vec4(HandAim::RemoveHeadsetHorizontalOffset(glm::vec3(middlePose[3])), 1.0f);

    const glm::mat4 cameraWorld = HandAim::ResolveCameraReferenceMtx(frameIdx) * middlePose;
    const glm::vec3 worldForward = glm::mat3(cameraWorld) * glm::vec3(0.0f, 0.0f, -1.0f);
    outPosition = glm::vec3(cameraWorld[3]);
    outForward = glm::length2(worldForward) > 0.000001f ? glm::normalize(worldForward) : glm::vec3(0.0f, 0.0f, -1.0f);
    return IsFiniteVec3(outPosition) && IsFiniteVec3(outForward);
}

static bool TryResolveDirectionToTarget(const glm::vec3& origin, const glm::vec3& targetPos, glm::vec3& outDirection) {
    const glm::vec3 aimVector = targetPos - origin;
    if (!HasFiniteDirection(aimVector)) {
        return false;
    }

    outDirection = glm::normalize(aimVector);
    return true;
}

static bool TryBuildFallbackLaunchPose(const LiveBowState& liveState, bool hasTargetPos, const glm::vec3& targetPos, bool preferPoseAim, PoseFallbackState& outPose, long cameraFrameIdx = -1) {
    outPose = {};
    const OpenXR::InputState inputs = Tracking::Hands();

    glm::vec3 cameraOrigin = glm::vec3(0.0f);
    glm::vec3 cameraForward = glm::vec3(0.0f, 0.0f, -1.0f);
    const bool hasCameraPose = TryGetGameplayCameraCenterPose(cameraOrigin, cameraForward, cameraFrameIdx);

    if (!GameState::IsFirstPerson()) {
        if (!hasCameraPose) {
            return false;
        }

        outPose.valid = true;
        outPose.origin = cameraOrigin;
        outPose.originSource = BowVisualizationOriginSource::CameraPose;
        if (!preferPoseAim && hasTargetPos && TryResolveDirectionToTarget(outPose.origin, targetPos, outPose.direction)) {
            outPose.directionSource = BowVisualizationDirectionSource::TargetPos;
        }
        else {
            outPose.direction = cameraForward;
            outPose.directionSource = BowVisualizationDirectionSource::CameraForward;
        }
        return true;
    }

    glm::vec3 rightWorldPos = glm::vec3(0.0f);
    glm::vec3 rightWorldForward = glm::vec3(0.0f, 0.0f, -1.0f);
    const bool hasRightHandPose = HandAim::TryGetControllerWorldPose(inputs, OpenXR::EyeSide::RIGHT, false, rightWorldPos, rightWorldForward, cameraFrameIdx);

    glm::vec3 leftWorldPos = glm::vec3(0.0f);
    glm::vec3 leftWorldForward = glm::vec3(0.0f, 0.0f, -1.0f);
    const bool hasLeftHandPose = HandAim::TryGetControllerWorldPose(inputs, OpenXR::EyeSide::LEFT, false, leftWorldPos, leftWorldForward, cameraFrameIdx);

    glm::vec3 rightAimWorldPos = glm::vec3(0.0f);
    glm::vec3 rightAimWorldForward = glm::vec3(0.0f, 0.0f, -1.0f);
    const bool hasRightAimPose = HandAim::TryGetControllerWorldPose(inputs, OpenXR::EyeSide::RIGHT, true, rightAimWorldPos, rightAimWorldForward, cameraFrameIdx);

    glm::vec3 leftAimWorldPos = glm::vec3(0.0f);
    glm::vec3 leftAimWorldForward = glm::vec3(0.0f, 0.0f, -1.0f);
    const bool hasLeftAimPose = HandAim::TryGetControllerWorldPose(inputs, OpenXR::EyeSide::LEFT, true, leftAimWorldPos, leftAimWorldForward, cameraFrameIdx);

    auto resolveDirectionAndTarget = [&](const glm::vec3& origin, glm::vec3& outDirection, BowVisualizationDirectionSource& outSource, glm::vec3& outTargetPos) -> bool {
        auto tryAimPoint = [&](const glm::vec3& aimPoint, BowVisualizationDirectionSource source) -> bool {
            if (!TryResolveDirectionToTarget(origin, aimPoint, outDirection)) {
                return false;
            }

            outTargetPos = aimPoint;
            outSource = source;
            return true;
        };

        if (preferPoseAim) {
            if (hasLeftAimPose && tryAimPoint(leftAimWorldPos + leftAimWorldForward * kPreviewAimDistance, BowVisualizationDirectionSource::LeftHandAimPose)) {
                return true;
            }
            if (hasRightAimPose && tryAimPoint(rightAimWorldPos + rightAimWorldForward * kPreviewAimDistance, BowVisualizationDirectionSource::RightHandAimPose)) {
                return true;
            }
            if (hasCameraPose && tryAimPoint(cameraOrigin + cameraForward * kPreviewAimDistance, BowVisualizationDirectionSource::CameraAim)) {
                return true;
            }
        }

        if (hasTargetPos && tryAimPoint(targetPos, BowVisualizationDirectionSource::TargetPos)) {
            return true;
        }

        if (!preferPoseAim) {
            if (hasLeftAimPose && tryAimPoint(leftAimWorldPos + leftAimWorldForward * kPreviewAimDistance, BowVisualizationDirectionSource::LeftHandAimPose)) {
                return true;
            }
            if (hasRightAimPose && tryAimPoint(rightAimWorldPos + rightAimWorldForward * kPreviewAimDistance, BowVisualizationDirectionSource::RightHandAimPose)) {
                return true;
            }
        }

        if (hasCameraPose) {
            return tryAimPoint(cameraOrigin + cameraForward * kPreviewAimDistance, BowVisualizationDirectionSource::CameraAim);
        }

        return false;
    };

    auto trySnapToWeaponAttackOrigin = [&](const glm::vec3& referencePos) {
        if (!liveState.hasOrigin || glm::distance2(liveState.origin, referencePos) > kPreviewOriginMaxSnapDistanceSq) {
            return;
        }

        outPose.origin = liveState.origin;
        outPose.originSource = BowVisualizationOriginSource::WeaponAttackPos;
    };

    auto tryFinalizePose = [&](const glm::vec3& fallbackForward, BowVisualizationDirectionSource fallbackSource) -> bool {
        glm::vec3 bowEntityForward = glm::vec3(0.0f);
        if (TryGetBowEntityWorldForward(bowEntityForward)) {
            outPose.direction = bowEntityForward;
            outPose.directionSource = BowVisualizationDirectionSource::BowEntityForward;
            outPose.targetPos = outPose.origin + glm::normalize(bowEntityForward) * kPreviewAimDistance;
            outPose.hasTargetPos = true;
        }
        else if (resolveDirectionAndTarget(outPose.origin, outPose.direction, outPose.directionSource, outPose.targetPos)) {
            outPose.hasTargetPos = true;
        }
        else {
            outPose.direction = fallbackForward;
            outPose.directionSource = fallbackSource;
            if (HasFiniteDirection(outPose.direction)) {
                outPose.targetPos = outPose.origin + glm::normalize(outPose.direction) * kPreviewAimDistance;
                outPose.hasTargetPos = true;
            }
        }

        outPose.valid = IsFiniteVec3(outPose.origin) && HasFiniteDirection(outPose.direction);
        return outPose.valid;
    };

    if (hasLeftHandPose) {
        outPose.origin = leftWorldPos + leftWorldForward * kPreviewOriginForwardOffset;
        outPose.originSource = BowVisualizationOriginSource::LeftHandPose;
        trySnapToWeaponAttackOrigin(leftWorldPos);
        if (hasLeftAimPose) {
            trySnapToWeaponAttackOrigin(leftAimWorldPos);
        }

        const glm::vec3 fallbackForward = hasLeftAimPose ? leftAimWorldForward : leftWorldForward;
        const BowVisualizationDirectionSource fallbackSource = hasLeftAimPose ? BowVisualizationDirectionSource::LeftHandAimPose : BowVisualizationDirectionSource::LeftHandForward;
        return tryFinalizePose(fallbackForward, fallbackSource);
    }

    if (liveState.hasOrigin) {
        outPose.origin = liveState.origin;
        outPose.originSource = BowVisualizationOriginSource::WeaponAttackPos;
        const glm::vec3 fallbackForward = hasLeftAimPose ? leftAimWorldForward : (hasLeftHandPose ? leftWorldForward : (hasRightAimPose ? rightAimWorldForward : glm::vec3(0.0f, 0.0f, -1.0f)));
        const BowVisualizationDirectionSource fallbackSource = hasLeftAimPose ? BowVisualizationDirectionSource::LeftHandAimPose : (hasLeftHandPose ? BowVisualizationDirectionSource::LeftHandForward : (hasRightAimPose ? BowVisualizationDirectionSource::RightHandAimPose : BowVisualizationDirectionSource::CameraForward));
        return tryFinalizePose(fallbackForward, fallbackSource);
    }

    if (hasRightHandPose) {
        outPose.origin = rightWorldPos + rightWorldForward * kPreviewOriginForwardOffset;
        outPose.originSource = BowVisualizationOriginSource::RightHandPose;
        const glm::vec3 fallbackForward = hasLeftAimPose ? leftAimWorldForward : rightWorldForward;
        const BowVisualizationDirectionSource fallbackSource = hasLeftAimPose ? BowVisualizationDirectionSource::LeftHandAimPose : BowVisualizationDirectionSource::RightHandForward;
        return tryFinalizePose(fallbackForward, fallbackSource);
    }

    if (hasLeftAimPose) {
        outPose.origin = leftAimWorldPos;
        outPose.originSource = BowVisualizationOriginSource::LeftHandPose;
        return tryFinalizePose(leftAimWorldForward, BowVisualizationDirectionSource::LeftHandAimPose);
    }

    if (!hasCameraPose) {
        return false;
    }

    outPose.valid = true;
    outPose.origin = cameraOrigin;
    outPose.originSource = BowVisualizationOriginSource::CameraPose;
    if (resolveDirectionAndTarget(outPose.origin, outPose.direction, outPose.directionSource, outPose.targetPos)) {
        outPose.hasTargetPos = true;
        return true;
    }

    outPose.direction = cameraForward;
    outPose.directionSource = BowVisualizationDirectionSource::CameraForward;
    outPose.targetPos = outPose.origin + outPose.direction * kPreviewAimDistance;
    outPose.hasTargetPos = true;
    return true;
}

static void SetTransformFromLaunchPose(BEMatrix34& transform, const glm::vec3& origin, const glm::vec3& direction) {
    const glm::mat4x3 currentMatrix = transform.getLEMatrix();
    const glm::vec3 forward = HasFiniteDirection(direction) ? glm::normalize(direction) : NormalizeOrFallback(-glm::vec3(currentMatrix[2]), glm::vec3(0.0f, 0.0f, -1.0f));
    const glm::vec3 zAxis = -forward;

    glm::vec3 upHint = NormalizeOrFallback(glm::vec3(currentMatrix[1]), glm::vec3(0.0f, 1.0f, 0.0f));
    if (std::abs(glm::dot(upHint, zAxis)) > 0.999f) {
        upHint = std::abs(zAxis.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
    }

    glm::vec3 xAxis = glm::cross(upHint, zAxis);
    if (!HasFiniteDirection(xAxis)) {
        const glm::vec3 fallbackUp = std::abs(zAxis.y) < 0.999f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        xAxis = glm::cross(fallbackUp, zAxis);
    }

    xAxis = NormalizeOrFallback(xAxis, glm::vec3(1.0f, 0.0f, 0.0f));
    const glm::vec3 yAxis = NormalizeOrFallback(glm::cross(zAxis, xAxis), glm::vec3(0.0f, 1.0f, 0.0f));
    transform.setLEMatrix(glm::mat4x3(xAxis, yAxis, zAxis, origin));
}

static void StorePendingShotOverride(const BowVisualizationState& launchState) {
    if (!launchState.hasLaunchOrigin || !launchState.hasLaunchDirection) {
        return;
    }

    PendingShotOverride nextOverride = {};
    nextOverride.valid = true;
    nextOverride.timestampMs = GetTimeStampMs();
    nextOverride.launchSpeed = launchState.launchSpeed;
    nextOverride.origin = launchState.launchOrigin;
    nextOverride.direction = glm::normalize(launchState.launchDirection);
    nextOverride.targetPos = launchState.hasTargetPos ? launchState.targetPos : (launchState.launchOrigin + nextOverride.direction * kLaunchTargetDistance);

    std::scoped_lock lock(s_pendingShotOverrideMutex);
    s_pendingShotOverride = nextOverride;

    Log::print<ARROW_SHOT_CAPTURE>(
        "[BowShot] Prepared override origin={} direction={} targetPos={} speed={:.3f} action={}",
        nextOverride.origin,
        nextOverride.direction,
        nextOverride.targetPos,
        nextOverride.launchSpeed,
        GetBowParamActionKindName(launchState.actionKind)
    );
}

static void ClearPendingShotOverride() {
    std::scoped_lock lock(s_pendingShotOverrideMutex);
    s_pendingShotOverride = {};
}

static bool TryGetPendingShotOverride(PendingShotOverride& outOverride, bool requireLoadParamsConfirmed) {
    std::scoped_lock lock(s_pendingShotOverrideMutex);
    if (!s_pendingShotOverride.valid) {
        return false;
    }

    const uint64_t nowMs = GetTimeStampMs();
    if (nowMs < s_pendingShotOverride.timestampMs || nowMs - s_pendingShotOverride.timestampMs > kPendingShotOverrideLifetimeMs) {
        s_pendingShotOverride = {};
        return false;
    }

    if (requireLoadParamsConfirmed && !s_pendingShotOverride.loadParamsConfirmed) {
        return false;
    }

    outOverride = s_pendingShotOverride;
    return true;
}

static void ConfirmPendingShotOverrideFromBool(const InlineParamBool& boolParam, uint32_t sourceReturnAddress) {
    if (!IsShotLoadReturnAddress(sourceReturnAddress)) {
        return;
    }

    const char* paramName = ResolveParamName(boolParam.keyPtr.getLE());
    if (!ParamMatches(paramName, kParamIsShootByPlayer)) {
        return;
    }

    std::scoped_lock lock(s_pendingShotOverrideMutex);
    if (!s_pendingShotOverride.valid) {
        return;
    }

    const uint64_t nowMs = GetTimeStampMs();
    if (nowMs < s_pendingShotOverride.timestampMs || nowMs - s_pendingShotOverride.timestampMs > kPendingShotOverrideLifetimeMs) {
        s_pendingShotOverride = {};
        return;
    }

    if (boolParam.value.getLE() == 0) {
        s_pendingShotOverride = {};
        return;
    }

    s_pendingShotOverride.loadParamsConfirmed = true;
    Log::print<ARROW_SHOT_CAPTURE>("[BowShot] Confirmed load-param override for player shot");
}

static bool TryOverrideShotLoadParamVec3(InlineParamVec3& vec3Param, uint32_t sourceReturnAddress) {
    if (!IsShotLoadReturnAddress(sourceReturnAddress)) {
        return false;
    }

    PendingShotOverride pendingShotOverride = {};
    if (!TryGetPendingShotOverride(pendingShotOverride, true)) {
        return false;
    }

    const char* paramName = ResolveParamName(vec3Param.keyPtr.getLE());
    const glm::vec3 originalValue = vec3Param.value.getLE();
    if (ParamMatches(paramName, kParamFirstSpeed)) {
        float launchSpeed = glm::length(vec3Param.value.getLE());
        if ((!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) && std::isfinite(pendingShotOverride.launchSpeed) && pendingShotOverride.launchSpeed > kMinimumArrowSpeed) {
            launchSpeed = pendingShotOverride.launchSpeed;
        }
        if (!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) {
            return false;
        }

        vec3Param.value = pendingShotOverride.direction * launchSpeed * Skeleton::ConsumeReleasedBowPullPower();
        Log::print<ARROW_SHOT_CAPTURE>(
            "[BowShot] Override {} {} -> {} (return=0x{:08X})",
            paramName != nullptr ? paramName : "<unnamed>",
            originalValue,
            vec3Param.value.getLE(),
            sourceReturnAddress
        );
        return true;
    }

    if (ParamMatches(paramName, kParamTargetPos) || ParamMatches(paramName, kParamHomingTargetPos)) {
        vec3Param.value = pendingShotOverride.targetPos;
        Log::print<ARROW_SHOT_CAPTURE>(
            "[BowShot] Override {} {} -> {} (return=0x{:08X})",
            paramName != nullptr ? paramName : "<unnamed>",
            originalValue,
            vec3Param.value.getLE(),
            sourceReturnAddress
        );
        return true;
    }

    return false;
}

static bool TryOverridePendingChildParamPackVec3(uint32_t paramPackPtr, const BowVisualizationState& launchState) {
    if (paramPackPtr == 0 || !launchState.hasLaunchOrigin || !launchState.hasLaunchDirection) {
        return false;
    }

    uint32_t entryCount = 0;
    Guest::ReadBE(paramPackPtr + kInlineParamPackEntryCountOffset, &entryCount);
    entryCount = std::min(entryCount, kInlineParamPackMaxEntries);
    if (entryCount == 0) {
        return false;
    }

    bool isPlayerShot = false;
    for (uint32_t entryIndex = 0; entryIndex < entryCount; entryIndex++) {
        const uint32_t entryPtr = paramPackPtr + (entryIndex * sizeof(InlineParamVec3));
        uint32_t keyPtr = 0;
        Guest::ReadBE(entryPtr + offsetof(InlineParamBool, keyPtr), &keyPtr);
        const char* paramName = ResolveParamName(keyPtr);
        if (!ParamMatches(paramName, kParamIsShootByPlayer)) {
            continue;
        }

        uint8_t boolValue = 0;
        Guest::Read(entryPtr + offsetof(InlineParamBool, value), &boolValue);
        isPlayerShot = boolValue != 0;
        break;
    }

    if (!isPlayerShot) {
        return false;
    }

    bool modifiedAny = false;
    for (uint32_t entryIndex = 0; entryIndex < entryCount; entryIndex++) {
        const uint32_t entryPtr = paramPackPtr + (entryIndex * sizeof(InlineParamVec3));
        uint32_t keyPtr = 0;
        Guest::ReadBE(entryPtr + offsetof(InlineParamVec3, keyPtr), &keyPtr);
        const char* paramName = ResolveParamName(keyPtr);
        if (paramName == nullptr) {
            continue;
        }

        if (!ParamMatches(paramName, kParamFirstSpeed) && !ParamMatches(paramName, kParamTargetPos) && !ParamMatches(paramName, kParamHomingTargetPos)) {
            continue;
        }

        InlineParamVec3 entry = {};
        Guest::Read(entryPtr, &entry);
        const glm::vec3 originalValue = entry.value.getLE();

        glm::vec3 replacementValue = originalValue;
        if (ParamMatches(paramName, kParamFirstSpeed)) {
            float launchSpeed = glm::length(originalValue);
            if ((!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) && std::isfinite(launchState.launchSpeed) && launchState.launchSpeed > kMinimumArrowSpeed) {
                launchSpeed = launchState.launchSpeed;
            }
            if (!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) {
                continue;
            }

            replacementValue = glm::normalize(launchState.launchDirection) * launchSpeed * Skeleton::ConsumeReleasedBowPullPower();
        }
        else {
            replacementValue = launchState.hasTargetPos ? launchState.targetPos : (launchState.launchOrigin + glm::normalize(launchState.launchDirection) * kLaunchTargetDistance);
        }

        if (!IsFiniteVec3(replacementValue)) {
            continue;
        }

        entry.value = replacementValue;
        Guest::Write(entryPtr, &entry);
        Log::print<ARROW_SHOT_CAPTURE>(
            "[BowShot] Override child param {} {} -> {} (pack=0x{:08X}, entry={})",
            paramName,
            originalValue,
            replacementValue,
            paramPackPtr,
            entryIndex
        );
        modifiedAny = true;
    }

    return modifiedAny;
}

static float IntegrateSpeed(float currentSpeed, float accel, float targetSpeed, float vfrScale) {
    const float nextSpeed = currentSpeed + (accel * vfrScale);
    if (accel < -kPreviewSpeedEpsilon) {
        return glm::max(nextSpeed, targetSpeed);
    }
    if (accel > kPreviewSpeedEpsilon) {
        return glm::min(nextSpeed, targetSpeed);
    }

    return currentSpeed;
}

static float ComputeFallTargetSpeed(const BowVisualizationState& launchState, float currentSpeed) {
    const float atRange = launchState.hasAtRange ? launchState.atRange : 0.0f;
    const float fallSpeedRatioByRange = launchState.hasFallSpeedRatioByRange ? launchState.fallSpeedRatioByRange : kDefaultFallSpeedRatioByRange;

    float fallTargetSpeed = launchState.fallAimSpeed + (((atRange / kPreviewFallSpeedRatioRangeBase) - 1.0f) * fallSpeedRatioByRange);
    fallTargetSpeed = glm::clamp(fallTargetSpeed, 0.0f, launchState.aimSpeed);
    if (IsNearZero(launchState.fallAccel)) {
        fallTargetSpeed = currentSpeed;
        if (IsNearZero(fallTargetSpeed)) {
            fallTargetSpeed = std::abs(launchState.gravity);
        }
    }

    return fallTargetSpeed;
}

static bool BuildPreviewTrajectory(const BowVisualizationState& launchState, std::array<PreviewPoint, kPresetDefaultMaxFrames + 2>& outPoints, uint32_t& outPointCount, int32_t& outFirstFallIndex) {
    outPointCount = 0;
    outFirstFallIndex = -1;
    if (!launchState.hasLaunchOrigin || !launchState.hasLaunchDirection) {
        return false;
    }

    float launchSpeed = launchState.launchSpeed;
    if (!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) {
        launchSpeed = glm::length(launchState.firstSpeed);
    }
    if (!std::isfinite(launchSpeed) || launchSpeed <= kMinimumArrowSpeed) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] BuildPreviewTrajectory FAILED: launchSpeed invalid ({:.6f})", launchSpeed);
        return false;
    }

    const glm::vec3 relativeVel = launchState.hasRelativeVel && IsFiniteVec3(launchState.relativeVel) ? launchState.relativeVel : glm::vec3(0.0f);
    const bool hasRelativeVelocity = glm::length2(relativeVel) > 0.000001f;
    const float vfrScale = s_arrowVfrScale.load(std::memory_order_relaxed);
    const float atRange = launchState.hasAtRange && std::isfinite(launchState.atRange) ? launchState.atRange : 0.0f;

    glm::vec3 position = launchState.launchOrigin;
    glm::vec3 direction = glm::normalize(launchState.launchDirection);
    glm::vec3 currentVelocity = glm::vec3(0.0f);
    glm::vec3 gravityBias = glm::vec3(0.0f);
    float currentSpeed = launchSpeed;
    float currentAccel = launchState.accel;
    float targetSpeed = launchState.aimSpeed;
    bool isFirstFrame = true;
    bool isFalling = false;

    outPoints[outPointCount++] = position;

    for (uint32_t frame = 1; frame <= kPresetDefaultMaxFrames && outPointCount < outPoints.size(); frame++) {
        if (isFirstFrame) {
            currentSpeed = IntegrateSpeed(currentSpeed, currentAccel, targetSpeed, vfrScale);
            const glm::vec3 baseVelocity = direction * currentSpeed;
            const glm::vec3 combinedVelocity = hasRelativeVelocity ? (baseVelocity + relativeVel) : baseVelocity;
            currentVelocity = ScaleVectorToLengthOrScaleRaw(combinedVelocity, kPreviewFirstFrameStep);
            position += currentVelocity * vfrScale;
            outPoints[outPointCount++] = position;
            isFirstFrame = false;
            continue;
        }

        if (!isFalling) {
            const float distanceFromOrigin = glm::distance(position, launchState.launchOrigin);
            const bool enterFallNextFrame = atRange <= kPreviewImmediateFallRange || distanceFromOrigin >= atRange;
            currentSpeed = IntegrateSpeed(currentSpeed, currentAccel, targetSpeed, vfrScale);
            const glm::vec3 baseVelocity = direction * currentSpeed;
            currentVelocity = hasRelativeVelocity ? (baseVelocity + relativeVel) : baseVelocity;
            position += currentVelocity * vfrScale;
            outPoints[outPointCount++] = position;
            if (enterFallNextFrame) {
                isFalling = true;
                currentAccel = std::abs(launchState.fallAccel);
                targetSpeed = ComputeFallTargetSpeed(launchState, currentSpeed);
                gravityBias = glm::vec3(0.0f, launchState.gravity, 0.0f);
                outFirstFallIndex = (int32_t)outPointCount;
            }
            continue;
        }

        currentSpeed = IntegrateSpeed(currentSpeed, currentAccel, targetSpeed, vfrScale);
        glm::vec3 localVelocity = currentVelocity;
        if (hasRelativeVelocity) {
            localVelocity -= relativeVel;
        }

        const glm::vec3 scaledLocalVelocity = ScaleVectorToLengthOrScaleRaw(localVelocity, currentSpeed);
        const glm::vec3 combinedVelocity = scaledLocalVelocity + gravityBias;
        currentVelocity = hasRelativeVelocity ? (combinedVelocity + relativeVel) : combinedVelocity;
        position += currentVelocity * vfrScale;
        outPoints[outPointCount++] = position;
    }

    return outPointCount >= 2;
}

static const char* GetBowParamActionKindName(BowParamActionKind actionKind) {
    switch (actionKind) {
        case BowParamActionKind::Move:
            return "move";
        case BowParamActionKind::Homing:
            return "homing";
        case BowParamActionKind::MoveWithStickOffset:
            return "stick_offset";
        case BowParamActionKind::MoveForLargeObject:
            return "large_object";
        case BowParamActionKind::Sky:
            return "sky";
        case BowParamActionKind::Unknown:
        default:
            return "unknown";
    }
}

static bool TryBuildBowVisualizationState(BowVisualizationState& outState, long cameraFrameIdx = -1) {
    outState = {};

    LiveBowState liveBowState = {};
    if (!TryReadLiveBowState(liveBowState)) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryBuildBowVisualizationState FAILED: TryReadLiveBowState returned false (see prior messages)");
        return false;
    }

    outState.valid = true;
    outState.weaponPtr = liveBowState.weaponPtr;
    outState.drawAmount = liveBowState.drawAmount;
    outState.actionKind = GetPreviewActionKind();

    const glm::vec3 candidateTargetPos = liveBowState.targetPos;
    const bool hasCandidateTargetPos = liveBowState.hasTargetPos;
    if (GameState::IsThirdPerson()) {
        if (GetSettings().ShouldAimThirdPersonBowFromCamera()) {
            glm::vec3 cameraPosition = glm::vec3(0.0f);
            glm::vec3 cameraForward = glm::vec3(0.0f, 0.0f, -1.0f);
            if (!TryGetGameplayCameraCenterPose(cameraPosition, cameraForward, cameraFrameIdx)) {
                Log::print<ARROW_SHOT_CAPTURE>("[BowViz] Third-person camera pose unavailable");
                return false;
            }

            outState.launchOrigin = cameraPosition;
            outState.originSource = BowVisualizationOriginSource::CameraPose;
            outState.launchDirection = cameraForward;
            outState.directionSource = BowVisualizationDirectionSource::CameraForward;
        }
        else {
            glm::vec3 bowPosition = glm::vec3(0.0f);
            glm::vec3 bowForward = glm::vec3(0.0f);
            const bool hasBowPosition = TryReadWeaponActorPosition(liveBowState.weaponPtr, bowPosition);
            const bool hasBowForward = TryGetBowEntityWorldForward(bowForward);
            if (!hasBowPosition || !hasBowForward) {
                Log::print<ARROW_SHOT_CAPTURE>("[BowViz] Third-person bow pose unavailable (hasPosition={}, hasForward={})", hasBowPosition, hasBowForward);
                return false;
            }

            outState.launchOrigin = bowPosition;
            outState.originSource = BowVisualizationOriginSource::WeaponAttackPos;
            outState.launchDirection = bowForward;
            outState.directionSource = BowVisualizationDirectionSource::BowEntityForward;
        }

        outState.hasLaunchOrigin = true;
        outState.hasLaunchDirection = true;
        outState.targetPos = outState.launchOrigin + outState.launchDirection * kLaunchTargetDistance;
        outState.hasTargetPos = true;
        outState.usedSyntheticTargetPos = true;
    }
    else {
        PoseFallbackState fallbackPose = {};
        TryBuildFallbackLaunchPose(liveBowState, hasCandidateTargetPos, candidateTargetPos, true, fallbackPose, cameraFrameIdx);

        if (fallbackPose.valid) {
            outState.launchOrigin = fallbackPose.origin;
            outState.originSource = fallbackPose.originSource;
            outState.hasLaunchOrigin = true;
        }
        else if (liveBowState.hasOrigin) {
            outState.launchOrigin = liveBowState.origin;
            outState.originSource = BowVisualizationOriginSource::WeaponAttackPos;
            outState.hasLaunchOrigin = true;
        }

        if (fallbackPose.valid && fallbackPose.hasTargetPos) {
            outState.targetPos = fallbackPose.targetPos;
            outState.hasTargetPos = true;
        }
        else if (liveBowState.hasTargetPos) {
            outState.targetPos = liveBowState.targetPos;
            outState.hasTargetPos = true;
        }

        if (fallbackPose.valid && HasFiniteDirection(fallbackPose.direction)) {
            outState.launchDirection = glm::normalize(fallbackPose.direction);
            outState.directionSource = fallbackPose.directionSource;
            outState.hasLaunchDirection = true;
        }
        else if (outState.hasTargetPos && outState.hasLaunchOrigin && TryResolveDirectionToTarget(outState.launchOrigin, outState.targetPos, outState.launchDirection)) {
            outState.directionSource = BowVisualizationDirectionSource::TargetPos;
            outState.hasLaunchDirection = true;
        }

        if (!outState.hasTargetPos && outState.hasLaunchOrigin && outState.hasLaunchDirection) {
            outState.targetPos = outState.launchOrigin + outState.launchDirection * kLaunchTargetDistance;
            outState.hasTargetPos = true;
            outState.usedSyntheticTargetPos = true;
        }
    }

    outState.launchSpeed = liveBowState.launchSpeed;
    outState.firstSpeed = outState.hasLaunchDirection ? outState.launchDirection * outState.launchSpeed : glm::vec3(0.0f, 0.0f, -outState.launchSpeed);
    outState.relativeVel = glm::vec3(0.0f);
    outState.hasRelativeVel = false;
    outState.accel = liveBowState.accel;
    outState.aimSpeed = liveBowState.aimSpeed;
    outState.fallAccel = liveBowState.fallAccel;
    outState.fallAimSpeed = liveBowState.fallAimSpeed;
    outState.gravity = liveBowState.gravity;
    outState.hasAtRange = true;
    outState.atRange = liveBowState.atRange;
    outState.hasFallSpeedRatioByRange = true;
    outState.fallSpeedRatioByRange = kDefaultFallSpeedRatioByRange;

    if (!outState.hasLaunchOrigin || !outState.hasLaunchDirection) {
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] TryBuildBowVisualizationState FAILED: hasOrigin={}, hasDir={}, weapon=0x{:08X}, draw={:.3f}, action={}", outState.hasLaunchOrigin, outState.hasLaunchDirection, outState.weaponPtr, outState.drawAmount, GetBowParamActionKindName(outState.actionKind));
    }
    return outState.hasLaunchOrigin && outState.hasLaunchDirection;
}

struct AimingArcCache {
    bool valid = false;
    std::array<PreviewPoint, kPresetDefaultMaxFrames + 2> points = {};
    uint32_t pointCount = 0;
    int32_t firstFallIndex = -1;
    bool hasTargetPos = false;
    glm::vec3 targetPos = glm::vec3(0.0f);
};

static AimingArcCache s_aimingArcCache = {};

static uint32_t LerpColor(uint32_t a, uint32_t b, float t) {
    const uint8_t r = (uint8_t)glm::mix((float)(a & 0xFF), (float)(b & 0xFF), t);
    const uint8_t g = (uint8_t)glm::mix((float)((a >> 8) & 0xFF), (float)((b >> 8) & 0xFF), t);
    const uint8_t bl = (uint8_t)glm::mix((float)((a >> 16) & 0xFF), (float)((b >> 16) & 0xFF), t);
    const uint8_t al = (uint8_t)glm::mix((float)((a >> 24) & 0xFF), (float)((b >> 24) & 0xFF), t);
    return (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)bl << 16) | ((uint32_t)al << 24);
}

static void BuildArcDistanceFractions(const std::array<PreviewPoint, kPresetDefaultMaxFrames + 2>& points, uint32_t pointCount, std::array<float, kPresetDefaultMaxFrames + 2>& outFractions) {
    outFractions.fill(0.0f);
    if (pointCount < 2) {
        return;
    }

    float totalDistance = 0.0f;
    for (uint32_t i = 1; i < pointCount; ++i) {
        totalDistance += glm::distance(points[i - 1], points[i]);
        outFractions[i] = totalDistance;
    }

    if (totalDistance <= 0.000001f) {
        return;
    }

    const float invTotalDistance = 1.0f / totalDistance;
    for (uint32_t i = 1; i < pointCount; ++i) {
        outFractions[i] = glm::clamp(outFractions[i] * invTotalDistance, 0.0f, 1.0f);
    }
}

static uint32_t ArcGradientColor(float t, float fallFraction) {
    const uint32_t blue = DebugDrawColor(60, 176, 255, 245);
    const uint32_t red = DebugDrawColor(255, 52, 36, 245);
    const uint32_t deepRed = DebugDrawColor(200, 22, 10, 245);

    const float fadeIn = glm::smoothstep(0.0f, 0.008f, t);

    uint32_t color;
    if (fallFraction <= 0.0f || t < fallFraction) {
        color = blue;
    }
    else if (t > fallFraction + 0.08f) {
        const float redProgress = (t - fallFraction - 0.08f) / (1.0f - fallFraction - 0.08f);
        color = LerpColor(red, deepRed, glm::smoothstep(0.0f, 0.7f, redProgress));
    }
    else {
        const float trans = glm::smoothstep(fallFraction, fallFraction + 0.08f, t);
        color = LerpColor(blue, red, trans);
    }

    const uint8_t a = (uint8_t)((float)((color >> 24) & 0xFF) * fadeIn);
    return (color & 0x00FFFFFF) | ((uint32_t)a << 24);
}

static uint32_t ScaleColorAlpha(uint32_t color, float scale) {
    const uint8_t a = (uint8_t)glm::clamp((int)std::lround((float)((color >> 24) & 0xFF) * scale), 0, 255);
    return (color & 0x00FFFFFF) | ((uint32_t)a << 24);
}

static void DrawAimingArc(const AimingArcCache& cache) {
    const uint32_t targetColor = DebugDrawColor(100, 180, 255, 100);
    const uint32_t fallMarkerColor = DebugDrawColor(255, 52, 36, 200);
    constexpr float kThickness = 1.5f;
    constexpr bool kPreviewXray = false;

    const float alphaScale = GetSettings().GetBowArcOpacity();

    if (cache.pointCount >= 2) {
        std::array<uint32_t, kPresetDefaultMaxFrames + 2> colors = {};
        std::array<float, kPresetDefaultMaxFrames + 2> distanceFractions = {};
        BuildArcDistanceFractions(cache.points, cache.pointCount, distanceFractions);
        const float fallFraction = cache.firstFallIndex >= 0 ? distanceFractions[std::min((uint32_t)cache.firstFallIndex, cache.pointCount - 1)] : -1.0f;
        for (uint32_t i = 0; i < cache.pointCount; ++i) {
            colors[i] = ScaleColorAlpha(ArcGradientColor(distanceFractions[i], fallFraction), alphaScale);
        }
        DebugDraw::instance().Polyline(
            std::span<const glm::vec3>(cache.points.data(), cache.pointCount),
            std::span<const uint32_t>(colors.data(), cache.pointCount),
            kThickness,
            kPreviewXray
        );

        if (cache.firstFallIndex >= 0 && (uint32_t)cache.firstFallIndex < cache.pointCount) {
            DebugDraw::instance().Sphere(cache.points[cache.firstFallIndex], 0.12f, ScaleColorAlpha(fallMarkerColor, alphaScale), 12, kPreviewXray);
        }
    }

    if (cache.hasTargetPos) {
        DebugDraw::instance().Sphere(cache.targetPos, 0.16f, ScaleColorAlpha(targetColor, alphaScale), 12, kPreviewXray);
    }
}

void Bow::QueueAimingArcPreview(long frameIdx) {
    if (GetHeldLeftBowWeaponPtr() == 0) {
        s_aimingArcCache = {};
        return;
    }

    if (GameState::IsFirstPerson() && !Skeleton::IsBowDrawEngaged() && !Skeleton::IsArrowNearBowStart()) {
        s_aimingArcCache = {};
        return;
    }

    BowVisualizationState launchState = {};
    if (!TryBuildBowVisualizationState(launchState, frameIdx) || launchState.drawAmount <= kMinimumBowDrawAmount) {
        s_aimingArcCache = {};
        return;
    }

    if (launchState.hasLaunchOrigin && launchState.hasLaunchDirection) {
        AimingArcCache cache = {};
        if (BuildPreviewTrajectory(launchState, cache.points, cache.pointCount, cache.firstFallIndex)) {
            cache.valid = true;
            cache.hasTargetPos = launchState.hasTargetPos && !launchState.usedSyntheticTargetPos;
            cache.targetPos = launchState.targetPos;
            s_aimingArcCache = cache;
            Log::print<ARROW_SHOT_CAPTURE>("[BowViz] Rendering {} arc points (origin={}, direction={}, firstFall={}, draw={:.3f}, atRange={:.1f})", cache.pointCount, launchState.launchOrigin, launchState.launchDirection, cache.firstFallIndex, launchState.drawAmount, launchState.atRange);
        }
        else {
            s_aimingArcCache = {};
            Log::print<ARROW_SHOT_CAPTURE>("[BowViz] QueueBowAimingArcPreview FAILED: BuildPreviewTrajectory (launchSpeed={:.3f})", launchState.launchSpeed);
        }
    }
    else {
        s_aimingArcCache = {};
        Log::print<ARROW_SHOT_CAPTURE>("[BowViz] QueueBowAimingArcPreview FAILED: TryBuildBowVisualizationState or pose missing (valid={}, hasOrigin={}, hasDir={})", launchState.valid, launchState.hasLaunchOrigin, launchState.hasLaunchDirection);
    }

    if (s_aimingArcCache.valid) {
        DrawAimingArc(s_aimingArcCache);
    }
}

static void hook_LoadDynamicBool(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    hCPU->sprNew.LR = hCPU->gpr[0];

    const uint32_t sourceReturnAddress = hCPU->gpr[0];
    InlineParamBool boolParam = {};
    Guest::Read(hCPU->gpr[30], &boolParam);
    ConfirmPendingShotOverrideFromBool(boolParam, sourceReturnAddress);
    RecordBool(boolParam, sourceReturnAddress);
}

static void hook_LoadDynamicVec3(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x030EC844;
    const uint32_t sourceReturnAddress = ReadInlineParamPackAddVec3CallerReturnAddress(hCPU);

    InlineParamVec3 vec3Param = {};
    Guest::Read(hCPU->gpr[31], &vec3Param);
    vec3Param.value.z = hCPU->fpr[0].fp0;
    TryOverrideShotLoadParamVec3(vec3Param, sourceReturnAddress);
    Guest::Write(hCPU->gpr[31], &vec3Param);
    CapturePreviewHookContext(hCPU, sourceReturnAddress);
}

static void hook_CaptureArrowShootDecision(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = kAiChangeChildAddress;

    const uint32_t returnAddress = hCPU->sprNew.LR;
    const uint32_t callsiteAddress = returnAddress >= 4 ? returnAddress - 4 : 0;
    RecordArrowShootDecision(hCPU->gpr[3], callsiteAddress);

    BowVisualizationState launchState = {};
    if (TryBuildBowVisualizationState(launchState)) {
        TryOverridePendingChildParamPackVec3(hCPU->gpr[5], launchState);
    }
}

static void hook_OverrideArrowShotTransform(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x03486CD4;

    ClearPendingShotOverride();
    BEMatrix34 transform = {};
    Guest::Read(hCPU->gpr[4], &transform);

    if (TryGetMatchingPlayerPreviewSnapshot(hCPU->gpr[30])) {
        BowVisualizationState launchState = {};
        if (TryBuildBowVisualizationState(launchState) && launchState.hasLaunchOrigin && launchState.hasLaunchDirection) {
            StorePendingShotOverride(launchState);
            SetTransformFromLaunchPose(transform, launchState.launchOrigin, launchState.launchDirection);
            Log::print<ARROW_SHOT_CAPTURE>(
                "[BowShot] Override transform origin={} direction={} previewAi=0x{:08X}",
                launchState.launchOrigin,
                launchState.launchDirection,
                hCPU->gpr[30]
            );
            Guest::Write(hCPU->gpr[4], &transform);
        }
    }
}

static void hook_ArrowFpsScale(PPCInterpreter_t* hCPU) {
    const float fpr0 = hCPU->fpr[0].fp0;

    float storeValue = fpr0;
    Guest::WriteBE(hCPU->gpr[30] + 0x1C, &storeValue);

    hCPU->instructionPointer = 0x03793388;

    if (std::isfinite(fpr0) && fpr0 > 0.0f && fpr0 <= 2.0f) {
        s_arrowVfrScale.store(fpr0, std::memory_order_relaxed);
    }
}

static constexpr uint32_t kActorGsysModelOffset = 0x330;
static constexpr uint32_t kGsysModelRootMtxOffset = 0x34;
static constexpr uint32_t kGsysModelUnitArrayOffset = 0x24;
static constexpr uint32_t kModelUnitVtableOffset = 0x74;
static constexpr uint32_t kModelUnitVtableAddress = 0x1034AB98;
static constexpr uint32_t kModelUnitStateFlagsOffset = 0xC4;
static constexpr uint8_t kModelUnitWorldUpdatedFlags = 0x22;
static constexpr uint32_t kModelUnitSkeletonOffset = 0x9C;
static constexpr uint32_t kSkeletonResOffset = 0x00;
static constexpr uint32_t kSkeletonFlagsOffset = 0x04;
static constexpr uint16_t kSkeletonWorldUpdatedFlag = 0x10;
static constexpr uint32_t kSkeletonBoneArrayOffset = 0x08;
static constexpr uint32_t kSkeletonWorldMtxArrayOffset = 0x10;
static constexpr uint32_t kSkeletonBoneCountOffset = 0x2C;
static constexpr uint32_t kResBoneStride = 0x40;
static constexpr uint32_t kWorldMtxStride = 0x30;
static constexpr uint16_t kMaxBowBoneCount = 64;
static constexpr int32_t kMaxResNameRelativeOffset = 0x04000000;
// Weapon_Arrow_001 runs along +Z with its nock notch just behind the fletching
static constexpr glm::vec3 kArrowNockLocalPos = { 0.0f, 0.0f, -0.086f };
// the bow animation pulls the string to 1.42x, so this leaves room to pull further
static constexpr float kMaxStringStretchPerHalfSpan = 2.0f;
static constexpr float kStringFollowRampSeconds = 0.1f;
static constexpr uint32_t kPlayerBowStateOffset = 0x19B4;
static constexpr uint32_t kPlayerBowStateCharging = 2;
static constexpr uint32_t kPlayerBowStateCharged = 3;
// from Weapon_Bow_Attack_Draw at rest and fully drawn, the right side mirrors the left
static constexpr float kLimbBaseEulerX = 1.5707964f;
static constexpr float kLimbBaseRestEulerY = 1.3614f;
static constexpr float kLimbBaseDrawnEulerY = 1.0969f;
static constexpr float kLimbTipRestEulerZ = 0.1579f;
static constexpr float kLimbTipDrawnEulerZ = 0.6322f;
// the same animation pulls the nock 0.639 from the rest string, whose half-span is 0.6065
static constexpr float kFullDrawPullPerHalfSpan = 1.054f;
static constexpr std::array<std::array<std::string_view, 4>, 2> kBowSideBoneNames = { {
    { "Bow_L_1", "Bow_L_2", "String_L_1", "String_L_2" },
    { "Bow_R_1", "Bow_R_2", "String_R_1", "String_R_2" },
} };

struct BowStringBones {
    uint32_t resSkeleton = 0;
    bool valid = false;
    uint16_t root = 0;
    std::array<uint16_t, 2> limbBase = {};
    std::array<uint16_t, 2> limbTip = {};
    std::array<uint16_t, 2> stringTip = {};
    std::array<uint16_t, 2> nock = {};
};

struct BowLimbPose {
    glm::mat4 limbBase;
    glm::mat4 limbTip;
    glm::mat4 stringTip;
};

static std::atomic_uint32_t s_bowStringSlotAddress = 0;
static BowStringBones s_bowStringBones = {};
static float s_bowStringFollowWeight = 0.0f;
static std::chrono::steady_clock::time_point s_lastBowStringPoseTime = {};

static uint32_t ResolveFirstModelUnit(uint32_t actorPtr) {
    const uint32_t gsysModelPtr = Guest::Get<uint32_t>(actorPtr + kActorGsysModelOffset).getLE();
    if (gsysModelPtr == 0)
        return 0;
    const uint32_t unitArray = Guest::Get<uint32_t>(gsysModelPtr + kGsysModelUnitArrayOffset).getLE();
    if (unitArray == 0)
        return 0;
    const uint32_t unitHolder = Guest::Get<uint32_t>(unitArray).getLE();
    if (unitHolder == 0)
        return 0;
    const uint32_t unitPtr = Guest::Get<uint32_t>(unitHolder).getLE();
    if (unitPtr == 0 || Guest::Get<uint32_t>(unitPtr + kModelUnitVtableOffset).getLE() != kModelUnitVtableAddress)
        return 0;
    return unitPtr;
}

static std::string_view GetResBoneName(uint32_t boneArray, uint16_t boneIdx) {
    const uint32_t bonePtr = boneArray + boneIdx * kResBoneStride;
    const int32_t nameOffset = (int32_t)Guest::Get<uint32_t>(bonePtr).getLE();
    if (nameOffset == 0 || nameOffset > kMaxResNameRelativeOffset || nameOffset < -kMaxResNameRelativeOffset)
        return {};
    const char* name = Guest::CString(bonePtr + nameOffset);
    return std::string_view(name, strnlen(name, 32));
}

static const BowStringBones& ResolveBowStringBones(uint32_t skeletonPtr) {
    const uint32_t resSkeleton = Guest::Get<uint32_t>(skeletonPtr + kSkeletonResOffset).getLE();
    if (resSkeleton == s_bowStringBones.resSkeleton)
        return s_bowStringBones;

    s_bowStringBones = { resSkeleton };
    const uint32_t boneArray = Guest::Get<uint32_t>(skeletonPtr + kSkeletonBoneArrayOffset).getLE();
    const uint16_t boneCount = Guest::Get<uint16_t>(skeletonPtr + kSkeletonBoneCountOffset).getLE();
    if (resSkeleton == 0 || boneArray == 0 || boneCount > kMaxBowBoneCount)
        return s_bowStringBones;

    const std::array<std::array<uint16_t*, 4>, 2> sideSlots = { {
        { &s_bowStringBones.limbBase[0], &s_bowStringBones.limbTip[0], &s_bowStringBones.stringTip[0], &s_bowStringBones.nock[0] },
        { &s_bowStringBones.limbBase[1], &s_bowStringBones.limbTip[1], &s_bowStringBones.stringTip[1], &s_bowStringBones.nock[1] },
    } };
    uint32_t foundMask = 0;
    for (uint16_t boneIdx = 0; boneIdx < boneCount; boneIdx++) {
        const std::string_view name = GetResBoneName(boneArray, boneIdx);
        if (name == "Bow_Root") {
            s_bowStringBones.root = boneIdx;
            foundMask |= 1;
            continue;
        }
        for (uint32_t side = 0; side < 2; side++) {
            for (uint32_t slot = 0; slot < 4; slot++) {
                if (name == kBowSideBoneNames[side][slot]) {
                    *sideSlots[side][slot] = boneIdx;
                    foundMask |= 2u << (side * 4 + slot);
                }
            }
        }
    }
    s_bowStringBones.valid = foundMask == 0x1FF;
    Log::print<ARROW_SHOT_CAPTURE>("[BowString] Skeleton 0x{:08X} with {} bones: string bones {}", resSkeleton, boneCount, s_bowStringBones.valid ? "found" : "missing");
    return s_bowStringBones;
}

static glm::mat4 ReadWorldBoneMtx(uint32_t worldArray, uint16_t boneIdx) {
    BEMatrix34 mtx = {};
    Guest::Read(worldArray + boneIdx * kWorldMtxStride, &mtx);
    return glm::mat4(mtx.getLEMatrix());
}

static void WriteWorldBoneMtx(uint32_t worldArray, uint16_t boneIdx, const glm::mat4& mtx) {
    BEMatrix34 beMtx = {};
    beMtx.setLEMatrix(glm::mat4x3(mtx));
    Guest::Write(worldArray + boneIdx * kWorldMtxStride, &beMtx);
}

static glm::mat4 EulerXYZToMtx(float x, float y, float z) {
    return glm::rotate(glm::rotate(glm::rotate(glm::identity<glm::mat4>(), z, glm::vec3(0.0f, 0.0f, 1.0f)), y, glm::vec3(0.0f, 1.0f, 0.0f)), x, glm::vec3(1.0f, 0.0f, 0.0f));
}

static BowLimbPose PoseBowLimb(const glm::mat4& rootWorld, const glm::vec3& limbBaseOffset, const glm::vec3& limbTipOffset, const glm::mat4& stringTipLocal, float sideSign, float bend) {
    BowLimbPose pose = {};
    const float baseEulerY = glm::mix(kLimbBaseRestEulerY, kLimbBaseDrawnEulerY, bend);
    const float tipEulerZ = glm::mix(kLimbTipRestEulerZ, kLimbTipDrawnEulerZ, bend);
    pose.limbBase = rootWorld * glm::translate(glm::identity<glm::mat4>(), limbBaseOffset) * EulerXYZToMtx(sideSign * kLimbBaseEulerX, sideSign * baseEulerY, 0.0f);
    pose.limbTip = pose.limbBase * glm::translate(glm::identity<glm::mat4>(), limbTipOffset) * EulerXYZToMtx(0.0f, 0.0f, tipEulerZ);
    pose.stringTip = pose.limbTip * stringTipLocal;
    return pose;
}

static bool TryResolveArrowNockWorldPos(uint32_t arrowPtr, glm::vec3& outPos) {
    const uint32_t gsysModelPtr = Guest::Get<uint32_t>(arrowPtr + kActorGsysModelOffset).getLE();
    if (gsysModelPtr == 0)
        return false;
    BEMatrix34 rootMtx = {};
    Guest::Read(gsysModelPtr + kGsysModelRootMtxOffset, &rootMtx);
    outPos = rootMtx.getLEMatrix() * glm::vec4(kArrowNockLocalPos, 1.0f);
    return IsFiniteVec3(outPos);
}

static float AdvanceBowStringFollowWeight(bool following) {
    const auto now = std::chrono::steady_clock::now();
    const float dt = glm::clamp(std::chrono::duration<float>(now - s_lastBowStringPoseTime).count(), 0.0f, 0.1f);
    s_lastBowStringPoseTime = now;
    s_bowStringFollowWeight = following ? glm::min(s_bowStringFollowWeight + dt / kStringFollowRampSeconds, 1.0f) : 0.0f;
    return s_bowStringFollowWeight;
}

static bool IsPlayerDrawingBow() {
    const uint32_t playerPtr = Player::GetAddress();
    if (playerPtr == 0)
        return false;
    const uint32_t bowState = Guest::Get<uint32_t>(playerPtr + kPlayerBowStateOffset).getLE();
    return bowState == kPlayerBowStateCharging || bowState == kPlayerBowStateCharged;
}

static void hook_RegisterBowStringSlot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    s_bowStringSlotAddress = hCPU->gpr[3];
    Guest::Set<uint32_t>(hCPU->gpr[3], 0);
}

// r3 = the held bow's gsys::ModelNW, at the entry of m_78_postCalcWorld
static void hook_PoseBowString(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t unitPtr = hCPU->gpr[3];
    const bool following = GameState::IsFirstPerson() && Weapons::IsArrowNockedInRightHand() && IsPlayerDrawingBow();
    const float weight = AdvanceBowStringFollowWeight(following);
    if (weight <= 0.0f)
        return;

    const uint32_t bowPtr = GetHeldLeftBowWeaponPtr();
    const uint32_t arrowPtr = Weapons::GetHeldActor(OpenXR::EyeSide::RIGHT);
    if (bowPtr == 0 || arrowPtr == 0 || ResolveFirstModelUnit(bowPtr) != unitPtr)
        return;

    const uint32_t skeletonPtr = Guest::Get<uint32_t>(unitPtr + kModelUnitSkeletonOffset).getLE();
    if (skeletonPtr == 0)
        return;
    const BowStringBones& bones = ResolveBowStringBones(skeletonPtr);
    const uint32_t worldArray = Guest::Get<uint32_t>(skeletonPtr + kSkeletonWorldMtxArrayOffset).getLE();
    if (!bones.valid || worldArray == 0)
        return;

    glm::vec3 arrowNock = glm::vec3(0.0f);
    if (!TryResolveArrowNockWorldPos(arrowPtr, arrowNock))
        return;

    const glm::mat4 rootWorld = ReadWorldBoneMtx(worldArray, bones.root);
    const glm::mat4 rootWorldInverse = glm::inverse(rootWorld);
    std::array<glm::vec3, 2> limbBaseOffsets = {};
    std::array<glm::vec3, 2> limbTipOffsets = {};
    std::array<glm::mat4, 2> stringTipLocals = {};
    std::array<glm::vec3, 2> animatedNocks = {};
    for (uint32_t side = 0; side < 2; side++) {
        const glm::mat4 limbBaseWorld = ReadWorldBoneMtx(worldArray, bones.limbBase[side]);
        const glm::mat4 limbTipWorld = ReadWorldBoneMtx(worldArray, bones.limbTip[side]);
        limbBaseOffsets[side] = glm::vec3((rootWorldInverse * limbBaseWorld)[3]);
        limbTipOffsets[side] = glm::vec3((glm::inverse(limbBaseWorld) * limbTipWorld)[3]);
        stringTipLocals[side] = glm::inverse(limbTipWorld) * ReadWorldBoneMtx(worldArray, bones.stringTip[side]);
        animatedNocks[side] = glm::vec3(ReadWorldBoneMtx(worldArray, bones.nock[side])[3]);
    }

    auto poseLimbs = [&](float bend) {
        return std::array<BowLimbPose, 2>{
            PoseBowLimb(rootWorld, limbBaseOffsets[0], limbTipOffsets[0], stringTipLocals[0], 1.0f, bend),
            PoseBowLimb(rootWorld, limbBaseOffsets[1], limbTipOffsets[1], stringTipLocals[1], -1.0f, bend),
        };
    };

    const std::array<BowLimbPose, 2> restLimbs = poseLimbs(0.0f);
    const glm::vec3 restTipL = glm::vec3(restLimbs[0].stringTip[3]);
    const glm::vec3 restTipR = glm::vec3(restLimbs[1].stringTip[3]);
    const glm::vec3 restTipMid = 0.5f * (restTipL + restTipR);
    const float restHalfSpan = 0.5f * glm::distance(restTipL, restTipR);
    if (!std::isfinite(restHalfSpan) || restHalfSpan < 0.001f)
        return;
    const float bend = glm::clamp(glm::distance(arrowNock, restTipMid) / (kFullDrawPullPerHalfSpan * restHalfSpan), 0.0f, 1.0f) * weight;

    const std::array<BowLimbPose, 2> limbs = poseLimbs(bend);
    const glm::vec3 tipL = glm::vec3(limbs[0].stringTip[3]);
    const glm::vec3 tipR = glm::vec3(limbs[1].stringTip[3]);
    const glm::vec3 tipMid = 0.5f * (tipL + tipR);
    const float halfSpan = 0.5f * glm::distance(tipL, tipR);

    glm::vec3 offset = arrowNock - tipMid;
    const float maxStretch = kMaxStringStretchPerHalfSpan * halfSpan;
    const float stretch = glm::length(offset);
    if (stretch > maxStretch)
        offset *= maxStretch / stretch;
    const glm::vec3 target = tipMid + offset;

    for (uint32_t side = 0; side < 2; side++) {
        WriteWorldBoneMtx(worldArray, bones.limbBase[side], limbs[side].limbBase);
        WriteWorldBoneMtx(worldArray, bones.limbTip[side], limbs[side].limbTip);
        WriteWorldBoneMtx(worldArray, bones.stringTip[side], limbs[side].stringTip);
        glm::mat4 nockWorld = limbs[side].stringTip;
        nockWorld[3] = glm::vec4(glm::mix(animatedNocks[side], target, weight), 1.0f);
        WriteWorldBoneMtx(worldArray, bones.nock[side], nockWorld);
    }

    const uint16_t skeletonFlags = Guest::Get<uint16_t>(skeletonPtr + kSkeletonFlagsOffset).getLE();
    Guest::Set<uint16_t>(skeletonPtr + kSkeletonFlagsOffset, skeletonFlags | kSkeletonWorldUpdatedFlag);
    const uint8_t unitFlags = Guest::Get<uint8_t>(unitPtr + kModelUnitStateFlagsOffset).getLE();
    Guest::Set<uint8_t>(unitPtr + kModelUnitStateFlagsOffset, unitFlags | kModelUnitWorldUpdatedFlags);
}

void Bow::OnGameFrame() {
    const uint32_t slotAddress = s_bowStringSlotAddress;
    if (slotAddress == 0)
        return;

    const uint32_t bowPtr = Tracking::IsSessionRunning() && GameState::IsFirstPerson() ? GetHeldLeftBowWeaponPtr() : 0;
    Guest::Set<uint32_t>(slotAddress, bowPtr != 0 ? ResolveFirstModelUnit(bowPtr) : 0);
}

void Bow::RegisterHooks() {
    CemuHost::RegisterHook("hook_RegisterBowStringSlot", &hook_RegisterBowStringSlot);
    CemuHost::RegisterHook("hook_PoseBowString", &hook_PoseBowString);
    CemuHost::RegisterHook("hook_LoadDynamicBool", &hook_LoadDynamicBool);
    CemuHost::RegisterHook("hook_LoadDynamicVec3", &hook_LoadDynamicVec3);
    CemuHost::RegisterHook("hook_CaptureArrowShootDecision", &hook_CaptureArrowShootDecision);
    CemuHost::RegisterHook("hook_OverrideArrowShotTransform", &hook_OverrideArrowShotTransform);
    CemuHost::RegisterHook("hook_ArrowFpsScale", &hook_ArrowFpsScale);
}
