#include "pch.h"
#include "roomscale.h"
#include "player.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/game_utils.h"
#include "game/botw/camera/camera.h"
#include "rendering/tracking.h"
#include "utils/debug_draw.h"

enum RoomscaleHookResult : uint32_t {
    RoomscaleHookResult_Skip = 0,
    RoomscaleHookResult_Cast = 1,
    RoomscaleHookResult_Warp = 2,
};

enum RoomscaleQueryType : uint32_t {
    RoomscaleQueryType_Sweep = 0,
    RoomscaleQueryType_GroundProbe = 1,
    RoomscaleQueryType_SupportProbe = 2,
};

enum RoomscaleResolvePhase : uint32_t {
    RoomscaleResolvePhase_ForwardSweep = 0,
    RoomscaleResolvePhase_DownSupportProbe = 1,
    RoomscaleResolvePhase_StepUpSweep = 2,
    RoomscaleResolvePhase_StepForwardSweep = 3,
    RoomscaleResolvePhase_StepDownSupportProbe = 4,
};

struct RoomscaleRaycastScratch {
    BEVec3 currentPos = {};
    BEVec3 castFrom = {};
    BEVec3 castDelta = {};
    BEVec3 hitPos = {};
    BEType<uint32_t> groundHit = 0;
    BEType<uint32_t> queryType = RoomscaleQueryType_Sweep;
    BEType<float> sweepRadius = 0.35f;
    BEVec3 hitNormal = {};
};

static_assert(offsetof(RoomscaleRaycastScratch, currentPos) == 0x00, "Roomscale scratch current position offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, castFrom) == 0x0C, "Roomscale scratch cast from offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, castDelta) == 0x18, "Roomscale scratch cast delta offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, hitPos) == 0x24, "Roomscale scratch hit offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, groundHit) == 0x30, "Roomscale scratch ground-hit offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, queryType) == 0x34, "Roomscale scratch query-type offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, sweepRadius) == 0x38, "Roomscale scratch sweep-radius offset mismatch");
static_assert(offsetof(RoomscaleRaycastScratch, hitNormal) == 0x3C, "Roomscale scratch hit-normal offset mismatch");
static_assert(sizeof(RoomscaleRaycastScratch) == 0x48, "Roomscale scratch size mismatch");

struct RoomscaleSupportState {
    bool valid = false;
    glm::fvec3 point = {};
    glm::fvec3 normal = {};
    glm::fvec3 bodyPos = {};
};

struct RoomscaleState {
    bool hasPreviousHeadPos = false;
    bool hasAppliedHeadPos = false;
    bool isResolving = false;
    bool isProbeOnly = false;
    bool isBlocked = false;
    bool isBlockedByCollision = false;
    bool hasActiveStep = false;
    bool currentStepBlockedByCollision = false;
    glm::fvec3 previousHeadPos = {};
    glm::fvec3 appliedHeadPos = {};
    glm::fvec3 trackedHeadPos = {};
    glm::fvec3 heldResidual = {};
    glm::fvec3 remainingRawDelta = {};
    glm::fvec3 remainingWorldDelta = {};
    glm::fvec3 currentStepRawDelta = {};
    glm::fvec3 currentStepWorldDelta = {};
    glm::fvec3 currentStepResolvedWorldDelta = {};
    glm::fvec3 currentStepCandidateWorldPos = {};
    RoomscaleSupportState support = {};
    RoomscaleSupportState currentStepSupport = {};
    uint32_t attemptIndex = 0;
    uint32_t maxAttempts = 0;
    RoomscaleResolvePhase resolvePhase = RoomscaleResolvePhase_ForwardSweep;
};

enum class RoomscaleBodyDriveMode : uint8_t {
    DisabledNotFirstPerson,
    DisabledNotInGame,
    DisabledNoRenderer,
    DisabledNoPositional,
    DisabledNoPlayer,
    DisabledSpecialMovement,
    Enabled,
};

static RoomscaleState s_roomscaleState;

// the body warp lands in the player matrix later in the frame, so keep the old head offset until it does
struct RoomscalePendingBodyMove {
    bool active = false;
    glm::fvec3 playerPos = {};
    glm::fvec3 appliedHeadPos = {};
};
static RoomscalePendingBodyMove s_pendingBodyMove;
static RoomscaleBodyDriveMode s_roomscaleBodyDriveMode = RoomscaleBodyDriveMode::DisabledNotInGame;

constexpr float kMaxRoomscaleJumpDistanceSq = 0.25f;
constexpr float kMinRoomscaleMoveDistanceSq = 0.000001f;
constexpr float kRoomscaleStepDistance = 0.0625f;
constexpr float kRoomscaleGroundSnapMaxRise = 0.10f;
constexpr float kRoomscaleGroundSnapJitterTolerance = 0.01f;
constexpr float kRoomscaleSupportRetainNormalDot = 0.98f;
constexpr float kRoomscaleStepUpHeight = 0.30f;
constexpr float kRoomscaleStandableSweepNormalY = 0.55f;
constexpr float kRoomscaleBlockedReturnTangentialAllowance = 0.5f;
constexpr float kRoomscaleDebugFallbackBodyRadius = 0.35f;
constexpr float kRoomscaleDebugFallbackBodyHalfHeight = 0.9f;
constexpr uint32_t kMaxRoomscaleRaycastAttempts = 8;
constexpr uint32_t kRoomscaleGroundHit = 15;

struct RoomscaleDebugBody {
    glm::fvec3 centerOffset = glm::fvec3(0.0f, kRoomscaleDebugFallbackBodyHalfHeight, 0.0f);
    float radius = kRoomscaleDebugFallbackBodyRadius;
    float halfHeight = kRoomscaleDebugFallbackBodyHalfHeight;
    float sweepRadius = kRoomscaleDebugFallbackBodyRadius;
    float groundProbeUp = 0.2f;
    float groundProbeDown = 0.75f;
};

static RoomscaleDebugBody s_roomscaleResolveBody = {};
static bool s_roomscaleShouldShowPhysicsDebug = false;

struct RoomscaleDebugPalette {
    uint32_t pathColor = 0;
    uint32_t pathSecondaryColor = 0;
    uint32_t bodyColor = 0;
    uint32_t floorColor = 0;
    uint32_t markerColor = 0;
    uint32_t successColor = 0;
    uint32_t blockedColor = 0;
    uint32_t xrayBlockedColor = 0;
};

static glm::fmat3 GetRoomscaleWorldToRoomRotation();
static glm::fmat3 GetRoomscaleRoomToWorldRotation();
static glm::fvec3 GetRoomscaleResidual();
static void ClearRoomscaleResolveState();

static uint32_t ScaleRoomscaleDebugColor(uint32_t color, float rgbScale, float alphaScale = 1.0f) {
    auto scaleChannel = [&](int shift, float scale) {
        uint8_t value = (uint8_t)((color >> shift) & 0xFFu);
        return (uint8_t)glm::clamp((int)std::lround((float)value * scale), 0, 255);
    };

    return DebugDrawColor(scaleChannel(0, rgbScale), scaleChannel(8, rgbScale), scaleChannel(16, rgbScale), scaleChannel(24, alphaScale));
}

static RoomscaleDebugPalette GetRoomscaleDebugPalette(bool isGroundProbe, bool isProbeOnly) {
    if (isGroundProbe) {
        return {
            .pathColor = DebugDrawColor(196, 104, 255, 255),
            .pathSecondaryColor = DebugDrawColor(220, 160, 255, 168),
            .bodyColor = DebugDrawColor(188, 112, 255, 118),
            .floorColor = DebugDrawColor(212, 144, 255, 156),
            .markerColor = DebugDrawColor(255, 84, 220, 255),
            .successColor = DebugDrawColor(172, 132, 255, 232),
            .blockedColor = DebugDrawColor(255, 72, 220, 210),
            .xrayBlockedColor = DebugDrawColor(255, 72, 220, 128),
        };
    }

    if (isProbeOnly) {
        return {
            .pathColor = DebugDrawColor(96, 255, 136, 255),
            .pathSecondaryColor = DebugDrawColor(156, 255, 184, 180),
            .bodyColor = DebugDrawColor(96, 255, 136, 156),
            .floorColor = DebugDrawColor(120, 255, 164, 148),
            .markerColor = DebugDrawColor(72, 255, 128, 255),
            .successColor = DebugDrawColor(188, 255, 168, 228),
            .blockedColor = DebugDrawColor(72, 220, 104, 220),
            .xrayBlockedColor = DebugDrawColor(72, 220, 104, 136),
        };
    }

    return {
        .pathColor = DebugDrawColor(88, 212, 255, 255),
        .pathSecondaryColor = DebugDrawColor(144, 232, 255, 180),
        .bodyColor = DebugDrawColor(88, 212, 255, 144),
        .floorColor = DebugDrawColor(128, 224, 255, 144),
        .markerColor = DebugDrawColor(120, 255, 255, 255),
        .successColor = DebugDrawColor(96, 255, 136, 228),
        .blockedColor = DebugDrawColor(255, 88, 88, 224),
        .xrayBlockedColor = DebugDrawColor(255, 88, 88, 132),
    };
}

static RoomscaleDebugBody BuildRoomscaleDebugBody(const PlayerBase& player) {
    RoomscaleDebugBody body = {};

    const glm::fvec3 localMin = player.aabb.min.getLE();
    const glm::fvec3 localMax = player.aabb.max.getLE();
    const glm::fvec3 localHalfExtents = (localMax - localMin) * 0.5f;
    const bool hasFiniteBounds = std::isfinite(localMin.x) && std::isfinite(localMin.y) && std::isfinite(localMin.z) && std::isfinite(localMax.x) && std::isfinite(localMax.y) && std::isfinite(localMax.z);
    if (!hasFiniteBounds || glm::any(glm::lessThanEqual(localHalfExtents, glm::fvec3(0.0f)))) {
        return body;
    }

    body.centerOffset = (localMin + localMax) * 0.5f;
    body.radius = std::max(localHalfExtents.x, localHalfExtents.z);
    body.halfHeight = std::max(localHalfExtents.y, body.radius * 0.5f);
    body.sweepRadius = glm::clamp(body.radius, 0.2f, 0.45f);

    float footOffset = std::max(-localMin.y, 0.0f);
    body.groundProbeUp = std::max(footOffset + 0.05f, 0.2f);
    body.groundProbeDown = std::max(footOffset + 0.25f, 0.5f);
    return body;
}

static void CacheRoomscaleResolveContext(const PlayerBase& player) {
    s_roomscaleResolveBody = BuildRoomscaleDebugBody(player);
    s_roomscaleShouldShowPhysicsDebug = GetSettings().ShouldShowRoomPhysics();
}

static bool IsRoomscaleSweepPhase(RoomscaleResolvePhase phase) {
    return phase == RoomscaleResolvePhase_ForwardSweep || phase == RoomscaleResolvePhase_StepUpSweep || phase == RoomscaleResolvePhase_StepForwardSweep;
}

static bool IsRoomscaleSupportProbePhase(RoomscaleResolvePhase phase) {
    return phase == RoomscaleResolvePhase_DownSupportProbe || phase == RoomscaleResolvePhase_StepDownSupportProbe;
}

static bool HasRoomscaleValidSweepNormal(const glm::fvec3& normal) {
    return glm::all(glm::isfinite(normal)) && glm::dot(normal, normal) > 0.000001f;
}

static bool HasRoomscaleValidSupportState(const RoomscaleSupportState& support) {
    return support.valid && glm::all(glm::isfinite(support.point)) && glm::all(glm::isfinite(support.normal)) && glm::all(glm::isfinite(support.bodyPos)) && HasRoomscaleValidSweepNormal(support.normal);
}

static glm::fvec3 NormalizeRoomscaleSweepHitNormal(const RoomscaleRaycastScratch& scratch) {
    glm::fvec3 hitNormal = scratch.hitNormal.getLE();
    if (!HasRoomscaleValidSweepNormal(hitNormal)) {
        return glm::fvec3(0.0f);
    }

    hitNormal = glm::normalize(hitNormal);
    glm::fvec3 castDelta = scratch.castDelta.getLE();
    if (glm::dot(castDelta, castDelta) > 0.000001f && glm::dot(hitNormal, castDelta) > 0.0f) {
        hitNormal = -hitNormal;
    }

    return hitNormal;
}

static bool IsRoomscaleSweepNormalFloorLike(const glm::fvec3& normal) {
    return HasRoomscaleValidSweepNormal(normal) && normal.y >= kRoomscaleStandableSweepNormalY;
}

static glm::fvec3 GetRoomscaleGroundProbeDelta(const RoomscaleDebugBody& body) {
    return glm::fvec3(0.0f, -(body.groundProbeUp + body.groundProbeDown), 0.0f);
}

static glm::fvec3 GetRoomscaleBodyCapsuleCenter(const RoomscaleDebugBody& body, const glm::fvec3& bodyPos) {
    return bodyPos + body.centerOffset;
}

static void PrepareRoomscaleSupportProbe(RoomscaleRaycastScratch& scratch, const RoomscaleDebugBody& body, const glm::fvec3& bodyPos) {
    scratch.queryType = RoomscaleQueryType_SupportProbe;
    scratch.sweepRadius = body.radius;
    scratch.castFrom = GetRoomscaleBodyCapsuleCenter(body, bodyPos) + glm::fvec3(0.0f, body.groundProbeUp, 0.0f);
    scratch.castDelta = GetRoomscaleGroundProbeDelta(body);
    scratch.hitPos = scratch.castFrom.getLE() + scratch.castDelta.getLE();
    scratch.hitNormal = glm::fvec3(0.0f);
}

static bool TryBuildRoomscaleSupportState(const RoomscaleDebugBody& body, const glm::fvec3& referenceBodyPos, float maxRise, const RoomscaleRaycastScratch& scratch, RoomscaleSupportState& outSupport) {
    outSupport = {};
    glm::fvec3 supportNormal = NormalizeRoomscaleSweepHitNormal(scratch);
    if (HasRoomscaleValidSweepNormal(supportNormal) && supportNormal.y < 0.0f) {
        supportNormal = -supportNormal;
    }

    if (!IsRoomscaleSweepNormalFloorLike(supportNormal)) {
        return false;
    }

    glm::fvec3 castDelta = scratch.castDelta.getLE();
    float castLengthSq = glm::dot(castDelta, castDelta);
    if (castLengthSq < 0.000001f) {
        return false;
    }

    const float cylinderHalfHeight = std::max(body.halfHeight - body.radius, 0.0f);
    const glm::fvec3 hitPos = scratch.hitPos.getLE();
    const glm::fvec3 bottomSphereCenter = hitPos + (supportNormal * body.radius);
    const glm::fvec3 capsuleCenter = bottomSphereCenter + glm::fvec3(0.0f, cylinderHalfHeight, 0.0f);
    float alongCast = glm::dot(capsuleCenter - scratch.castFrom.getLE(), castDelta) / castLengthSq;
    if (alongCast < 0.0f || alongCast > 1.0f) {
        return false;
    }

    glm::fvec3 bodyPos = capsuleCenter - body.centerOffset;
    if (!glm::all(glm::isfinite(bodyPos)) || (bodyPos.y - referenceBodyPos.y) > maxRise) {
        return false;
    }

    outSupport.valid = true;
    outSupport.point = hitPos;
    outSupport.normal = supportNormal;
    outSupport.bodyPos = bodyPos;
    return true;
}

static RoomscaleSupportState StabilizeRoomscaleSupportState(const RoomscaleSupportState& previousSupport, const RoomscaleSupportState& nextSupport) {
    if (!HasRoomscaleValidSupportState(nextSupport)) {
        return {};
    }

    if (!HasRoomscaleValidSupportState(previousSupport)) {
        return nextSupport;
    }

    RoomscaleSupportState resolvedSupport = nextSupport;
    if (std::abs(nextSupport.bodyPos.y - previousSupport.bodyPos.y) <= kRoomscaleGroundSnapJitterTolerance && glm::dot(nextSupport.normal, previousSupport.normal) >= kRoomscaleSupportRetainNormalDot) {
        resolvedSupport.bodyPos.y = previousSupport.bodyPos.y;
        resolvedSupport.point.y = previousSupport.point.y;
        resolvedSupport.normal = previousSupport.normal;
    }

    return resolvedSupport;
}

static glm::fvec3 ProjectRoomscaleDeltaOntoSupport(const glm::fvec3& worldDelta, const RoomscaleSupportState& support) {
    if (!HasRoomscaleValidSupportState(support)) {
        return worldDelta;
    }

    glm::fvec3 supportNormal = glm::normalize(support.normal);
    if (std::abs(supportNormal.y) < 0.000001f) {
        return worldDelta;
    }

    glm::fvec3 projectedDelta = worldDelta;
    projectedDelta.y = -(supportNormal.x * worldDelta.x + supportNormal.z * worldDelta.z) / supportNormal.y;
    return projectedDelta;
}

static void BeginRoomscaleResolvePhase(RoomscaleResolvePhase phase) {
    s_roomscaleState.resolvePhase = phase;
}

static bool WouldRoomscaleResidualImproveWithAppliedDelta(glm::fvec3 appliedHeadDelta) {
    if (!s_roomscaleState.isProbeOnly) {
        return true;
    }

    appliedHeadDelta.y = 0.0f;
    glm::fvec3 oldResidual = GetRoomscaleResidual();
    glm::fvec3 newResidual = oldResidual - appliedHeadDelta;
    newResidual.y = 0.0f;
    return glm::dot(newResidual, newResidual) + kMinRoomscaleMoveDistanceSq < glm::dot(oldResidual, oldResidual);
}

static std::optional<glm::fvec3> TryGetPlayerMtxPos() {
    const uint32_t mtxAddress = Player::GetMtxAddress();
    if (mtxAddress == 0) {
        return std::nullopt;
    }
    return Guest::Get<BEMatrix34>(mtxAddress).getPos().getLE();
}

static bool CommitRoomscaleBodyMove(uint32_t scratchPtr, RoomscaleRaycastScratch& scratch, const glm::fvec3& worldPos, glm::fvec3 appliedHeadDelta) {
    appliedHeadDelta.y = 0.0f;
    if (!WouldRoomscaleResidualImproveWithAppliedDelta(appliedHeadDelta)) {
        return false;
    }

    if (!s_pendingBodyMove.active) {
        if (std::optional<glm::fvec3> playerPos = TryGetPlayerMtxPos()) {
            s_pendingBodyMove.playerPos = playerPos.value();
            s_pendingBodyMove.appliedHeadPos = s_roomscaleState.appliedHeadPos;
            s_pendingBodyMove.active = true;
        }
    }

    scratch.currentPos = worldPos;
    s_roomscaleState.appliedHeadPos += appliedHeadDelta;
    Guest::Write(scratchPtr, &scratch);
    return true;
}

static bool CommitRoomscaleBodyMoveWithSupport(uint32_t scratchPtr, RoomscaleRaycastScratch& scratch, const RoomscaleSupportState& support, glm::fvec3 appliedHeadDelta) {
    if (!HasRoomscaleValidSupportState(support)) {
        return false;
    }

    if (!CommitRoomscaleBodyMove(scratchPtr, scratch, support.bodyPos, appliedHeadDelta)) {
        return false;
    }

    s_roomscaleState.support = support;
    return true;
}

static void AdvanceRoomscaleCommittedStep(glm::fvec3 appliedHeadDelta) {
    appliedHeadDelta.y = 0.0f;
    s_roomscaleState.remainingRawDelta -= appliedHeadDelta;
    s_roomscaleState.remainingWorldDelta = GetRoomscaleRoomToWorldRotation() * s_roomscaleState.remainingRawDelta;
    s_roomscaleState.remainingWorldDelta.y = 0.0f;
    s_roomscaleState.attemptIndex++;
    s_roomscaleState.hasActiveStep = false;
    BeginRoomscaleResolvePhase(RoomscaleResolvePhase_ForwardSweep);
}

static void FinishRoomscaleResolveWarp(PPCInterpreter_t* hCPU, bool blocked, bool blockedByCollision) {
    s_roomscaleState.isBlocked = blocked;
    s_roomscaleState.isBlockedByCollision = blocked && blockedByCollision;
    ClearRoomscaleResolveState();
    hCPU->gpr[3] = RoomscaleHookResult_Warp;
}

static void DrawRoomscaleDebugPositionMarker(const glm::fvec3& pos, float radius, uint32_t color, bool xray = false) {
    DebugDraw::instance().Dot(pos, radius, color, xray);
}

static void DrawRoomscaleDebugBodyCapsule(const RoomscaleDebugBody& body, const glm::fvec3& bodyPos, uint32_t color, bool xray = false) {
    DebugDraw::instance().Capsule(GetRoomscaleBodyCapsuleCenter(body, bodyPos), body.radius, body.halfHeight, color, 1.0f, 0, xray);
}

static bool IsRoomscaleGroundProbe(const RoomscaleRaycastScratch& scratch) {
    return scratch.queryType.getLE() != RoomscaleQueryType_Sweep;
}

static void DrawRoomscaleDebugCast(const RoomscaleRaycastScratch& scratch, const RoomscaleDebugBody& body, bool isProbeOnly, bool shouldShowRoomPhysics) {
    if (!shouldShowRoomPhysics || IsRoomscaleGroundProbe(scratch)) {
        return;
    }

    const RoomscaleDebugPalette palette = GetRoomscaleDebugPalette(false, isProbeOnly);
    const glm::fvec3 castFrom = scratch.castFrom.getLE();
    const glm::fvec3 castTo = castFrom + scratch.castDelta.getLE();

    DebugDraw::instance().Line(castFrom, castTo, palette.pathColor, 1.0f, true);
    DrawRoomscaleDebugBodyCapsule(body, castFrom - body.centerOffset, ScaleRoomscaleDebugColor(palette.bodyColor, 0.85f, 0.28f), true);
    DrawRoomscaleDebugBodyCapsule(body, castTo - body.centerOffset, ScaleRoomscaleDebugColor(palette.bodyColor, 0.75f, 0.22f), true);
    DrawRoomscaleDebugPositionMarker(castFrom, 0.04f, palette.markerColor, true);
    DrawRoomscaleDebugPositionMarker(castTo, 0.04f, palette.pathSecondaryColor, true);
}

static void DrawRoomscaleDebugCastResult(const RoomscaleRaycastScratch& scratch, bool isProbeOnly, bool hadHit, bool acceptedHit, bool shouldShowRoomPhysics) {
    if (!shouldShowRoomPhysics) {
        return;
    }

    const bool isGroundProbe = IsRoomscaleGroundProbe(scratch);
    const RoomscaleDebugPalette palette = GetRoomscaleDebugPalette(isGroundProbe, isProbeOnly);
    const glm::fvec3 castFrom = scratch.castFrom.getLE();
    const glm::fvec3 castTo = castFrom + scratch.castDelta.getLE();
    if (!hadHit) {
        return;
    }

    const glm::fvec3 hitPos = scratch.hitPos.getLE();
    glm::fvec3 hitNormal = NormalizeRoomscaleSweepHitNormal(scratch);
    if (isGroundProbe) {
        uint32_t hitColor = acceptedHit ? palette.successColor : palette.blockedColor;
        DebugDraw::instance().Line(castFrom, hitPos, hitColor, 1.0f, true);
        if (glm::dot(hitPos - castTo, hitPos - castTo) > 0.000001f) {
            DebugDraw::instance().Line(hitPos, castTo, ScaleRoomscaleDebugColor(hitColor, 0.45f, 0.35f), 1.0f, true);
        }
        if (HasRoomscaleValidSweepNormal(hitNormal)) {
            DebugDraw::instance().Line(hitPos, hitPos + (hitNormal * 0.18f), palette.floorColor, 1.0f, true);
        }
        DrawRoomscaleDebugPositionMarker(hitPos, 0.08f, hitColor, true);
        return;
    }

    const float sweepRadius = std::max(scratch.sweepRadius.getLE(), 0.01f);
    DebugDraw::instance().Line(castFrom, hitPos, palette.blockedColor, 1.0f, true);
    if (glm::dot(hitPos - castTo, hitPos - castTo) > 0.000001f) {
        DebugDraw::instance().Line(hitPos, castTo, ScaleRoomscaleDebugColor(palette.blockedColor, 0.45f, 0.35f), 1.0f, true);
    }
    if (HasRoomscaleValidSweepNormal(hitNormal)) {
        DebugDraw::instance().Line(hitPos, hitPos + (hitNormal * 0.18f), palette.markerColor, 1.0f, true);
    }
    DebugDraw::instance().Sphere(hitPos, sweepRadius, ScaleRoomscaleDebugColor(palette.blockedColor, 0.85f, 0.28f), 0, true);
    DrawRoomscaleDebugPositionMarker(hitPos, 0.08f, palette.blockedColor, true);
}

static void ResetRoomscaleActiveStep() {
    s_roomscaleState.hasActiveStep = false;
    s_roomscaleState.currentStepBlockedByCollision = false;
    s_roomscaleState.currentStepRawDelta = glm::fvec3(0.0f);
    s_roomscaleState.currentStepWorldDelta = glm::fvec3(0.0f);
    s_roomscaleState.currentStepResolvedWorldDelta = glm::fvec3(0.0f);
    s_roomscaleState.currentStepCandidateWorldPos = glm::fvec3(0.0f);
    s_roomscaleState.currentStepSupport = {};
    BeginRoomscaleResolvePhase(RoomscaleResolvePhase_ForwardSweep);
}

static void BeginRoomscaleActiveStep(uint32_t remainingSteps) {
    s_roomscaleState.hasActiveStep = true;
    s_roomscaleState.currentStepBlockedByCollision = false;
    s_roomscaleState.currentStepRawDelta = s_roomscaleState.remainingRawDelta / (float)remainingSteps;
    s_roomscaleState.currentStepWorldDelta = s_roomscaleState.remainingWorldDelta / (float)remainingSteps;
    s_roomscaleState.currentStepResolvedWorldDelta = ProjectRoomscaleDeltaOntoSupport(s_roomscaleState.currentStepWorldDelta, s_roomscaleState.support);
    s_roomscaleState.currentStepCandidateWorldPos = glm::fvec3(0.0f);
    s_roomscaleState.currentStepSupport = {};
    BeginRoomscaleResolvePhase(RoomscaleResolvePhase_ForwardSweep);
}

static glm::fmat3 GetRoomscaleRoomToWorldRotation() {
    return glm::fmat3(Camera::GetReferenceMtx());
}

static glm::fmat3 GetRoomscaleWorldToRoomRotation() {
    return glm::transpose(GetRoomscaleRoomToWorldRotation());
}

static void ClearRoomscaleResolveState() {
    BetterVRProfiler::EndSpan(BetterVRProfiler::Section::RoomscaleResolve);

    s_roomscaleState.isResolving = false;
    s_roomscaleState.isProbeOnly = false;
    s_roomscaleState.remainingRawDelta = glm::fvec3(0.0f);
    s_roomscaleState.remainingWorldDelta = glm::fvec3(0.0f);
    s_roomscaleState.attemptIndex = 0;
    s_roomscaleState.maxAttempts = 0;
    s_roomscaleResolveBody = {};
    s_roomscaleShouldShowPhysicsDebug = false;
    ResetRoomscaleActiveStep();
}

static void ResetRoomscaleState() {
    s_pendingBodyMove.active = false;
    s_roomscaleState.hasPreviousHeadPos = false;
    s_roomscaleState.hasAppliedHeadPos = false;
    s_roomscaleState.isBlocked = false;
    s_roomscaleState.isBlockedByCollision = false;
    s_roomscaleState.support = {};
    s_roomscaleState.previousHeadPos = glm::fvec3(0.0f);
    s_roomscaleState.appliedHeadPos = glm::fvec3(0.0f);
    s_roomscaleState.trackedHeadPos = glm::fvec3(0.0f);
    ClearRoomscaleResolveState();
}

static glm::fvec3 GetRoomscaleResidual() {
    if (!s_roomscaleState.hasAppliedHeadPos) {
        return glm::fvec3(0.0f);
    }

    glm::fvec3 residual = s_roomscaleState.trackedHeadPos - s_roomscaleState.appliedHeadPos;
    residual.y = 0.0f;
    return residual;
}

static glm::fvec3 RemoveResolvedRoomscaleDelta(glm::fvec3 roomDelta, glm::fvec3 residual) {
    residual.y = 0.0f;
    float residualDistance = glm::length(residual);
    if (residualDistance < 0.000001f) {
        return roomDelta;
    }

    glm::fvec3 residualDir = residual / residualDistance;
    float alongResidual = glm::dot(roomDelta, residualDir);
    if (alongResidual >= 0.0f) {
        return roomDelta;
    }

    glm::fvec3 tangentialDelta = roomDelta - (residualDir * alongResidual);
    if (s_roomscaleState.isBlockedByCollision) {
        float towardBodyDistance = -alongResidual;
        float tangentialDistance = glm::length(tangentialDelta);
        if (towardBodyDistance >= (tangentialDistance * kRoomscaleBlockedReturnTangentialAllowance)) {
            return glm::fvec3(0.0f);
        }
    }

    return tangentialDelta;
}

static std::optional<glm::fvec3> TryGetCurrentHeadPos() {
    std::optional<glm::fmat4> middlePose = Tracking::HeadPose();
    if (!middlePose.has_value()) {
        return std::nullopt;
    }

    glm::fvec3 headPos = glm::fvec3(middlePose.value()[3]);
    if (!glm::all(glm::isfinite(headPos))) {
        return std::nullopt;
    }

    return headPos;
}

static const char* GetRoomscaleBodyDriveModeName(RoomscaleBodyDriveMode mode) {
    switch (mode) {
        case RoomscaleBodyDriveMode::DisabledNotFirstPerson:
            return "disabled (not first-person)";
        case RoomscaleBodyDriveMode::DisabledNotInGame:
            return "disabled (not in game)";
        case RoomscaleBodyDriveMode::DisabledNoRenderer:
            return "disabled (renderer unavailable)";
        case RoomscaleBodyDriveMode::DisabledNoPositional:
            return "disabled (no positional tracking)";
        case RoomscaleBodyDriveMode::DisabledNoPlayer:
            return "disabled (player unavailable)";
        case RoomscaleBodyDriveMode::DisabledSpecialMovement:
            return "disabled (special movement state)";
        case RoomscaleBodyDriveMode::Enabled:
            return "enabled";
    }

    return "disabled (unknown)";
}

static void SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode mode) {
    if (s_roomscaleBodyDriveMode == mode) {
        return;
    }

    s_roomscaleBodyDriveMode = mode;
    Log::print<PPC>("Roomscale body drive mode switched: {}", GetRoomscaleBodyDriveModeName(mode));
}

static void PrimeRoomscaleBaseline(const glm::fvec3& headPos) {
    s_pendingBodyMove.active = false;
    s_roomscaleState.previousHeadPos = headPos;
    s_roomscaleState.appliedHeadPos = headPos;
    s_roomscaleState.trackedHeadPos = headPos;
    s_roomscaleState.hasPreviousHeadPos = true;
    s_roomscaleState.hasAppliedHeadPos = true;
    s_roomscaleState.isBlocked = false;
    s_roomscaleState.isBlockedByCollision = false;
    s_roomscaleState.support = {};
    ClearRoomscaleResolveState();
}

// special movement doesn't move the body, so the camera keeps the head-to-body gap it had when it started
static void HoldRoomscaleResidual(const glm::fvec3& headPos) {
    if (s_roomscaleBodyDriveMode != RoomscaleBodyDriveMode::DisabledSpecialMovement) {
        const bool hasResidual = s_roomscaleState.hasPreviousHeadPos && s_roomscaleState.hasAppliedHeadPos;
        s_roomscaleState.heldResidual = hasResidual ? s_roomscaleState.previousHeadPos - s_roomscaleState.appliedHeadPos : glm::fvec3(0.0f);
        s_roomscaleState.heldResidual.y = 0.0f;
    }
    PrimeRoomscaleBaseline(headPos);
    s_roomscaleState.appliedHeadPos = headPos - s_roomscaleState.heldResidual;
}

static bool ShouldDrivePlayerBodyWithVR(const glm::fvec3& currentHeadPos, const PlayerBase& player) {
    if (!GameState::IsFirstPerson()) {
        SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode::DisabledNotFirstPerson);
        PrimeRoomscaleBaseline(currentHeadPos);
        return false;
    }

    if (!GameState::IsInGame()) {
        SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode::DisabledNotInGame);
        PrimeRoomscaleBaseline(currentHeadPos);
        return false;
    }

    if (!Tracking::SupportsPositionalTracking()) {
        SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode::DisabledNoPositional);
        PrimeRoomscaleBaseline(currentHeadPos);
        return false;
    }

    PlayerMoveBitFlags moveBits = player.moveBitFlags.getLE();
    bool shouldDisablePlayerBodyDrive = Player::IsClimbing() || Player::IsGliding() || Player::IsRiding() || HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_SWIMMING);
    if (shouldDisablePlayerBodyDrive) {
        HoldRoomscaleResidual(currentHeadPos);
        SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode::DisabledSpecialMovement);
        return false;
    }

    SetRoomscaleBodyDriveMode(RoomscaleBodyDriveMode::Enabled);
    return true;
}

static bool TryGetRoomscaleMoveDelta(const PlayerBase& player, glm::fvec3& currentHeadPos, glm::fvec3& roomDelta, glm::fvec3& worldDelta) {
    currentHeadPos = glm::fvec3(0.0f);
    roomDelta = glm::fvec3(0.0f);
    worldDelta = glm::fvec3(0.0f);

    std::optional<glm::fvec3> currentHeadPosOpt = TryGetCurrentHeadPos();
    if (!currentHeadPosOpt.has_value()) {
        ResetRoomscaleState();
        return false;
    }

    currentHeadPos = currentHeadPosOpt.value();
    s_roomscaleState.trackedHeadPos = currentHeadPos;

    if (!ShouldDrivePlayerBodyWithVR(currentHeadPos, player)) {
        return false;
    }

    if (!s_roomscaleState.hasPreviousHeadPos) {
        PrimeRoomscaleBaseline(currentHeadPos);
        return false;
    }

    glm::fvec3 previousResidual = s_roomscaleState.previousHeadPos - s_roomscaleState.appliedHeadPos;
    previousResidual.y = 0.0f;
    roomDelta = currentHeadPos - s_roomscaleState.previousHeadPos;
    s_roomscaleState.previousHeadPos = currentHeadPos;

    roomDelta.y = 0.0f;
    if (glm::dot(roomDelta, roomDelta) < kMinRoomscaleMoveDistanceSq) {
        return false;
    }

    if (glm::dot(roomDelta, roomDelta) > kMaxRoomscaleJumpDistanceSq) {
        PrimeRoomscaleBaseline(currentHeadPos);
        return false;
    }

    const glm::fmat3 roomToWorld = GetRoomscaleRoomToWorldRotation();
    worldDelta = roomToWorld * roomDelta;
    worldDelta.y = 0.0f;
    glm::fvec3 previousResidualWorld = roomToWorld * previousResidual;
    previousResidualWorld.y = 0.0f;
    worldDelta = RemoveResolvedRoomscaleDelta(worldDelta, previousResidualWorld);
    if (glm::dot(worldDelta, worldDelta) < kMinRoomscaleMoveDistanceSq) {
        return false;
    }

    roomDelta = GetRoomscaleWorldToRoomRotation() * worldDelta;
    roomDelta.y = 0.0f;

    return true;
}

glm::fvec3 Roomscale::GetAppliedHeadPosition() {
    if (s_roomscaleBodyDriveMode == RoomscaleBodyDriveMode::DisabledSpecialMovement) {
        if (std::optional<glm::fvec3> currentHeadPos = TryGetCurrentHeadPos()) {
            return currentHeadPos.value() - s_roomscaleState.heldResidual;
        }
    }
    if (s_roomscaleState.hasAppliedHeadPos) {
        if (s_pendingBodyMove.active && TryGetPlayerMtxPos() == s_pendingBodyMove.playerPos) {
            return s_pendingBodyMove.appliedHeadPos;
        }
        return s_roomscaleState.appliedHeadPos;
    }

    std::optional<glm::fvec3> currentHeadPos = TryGetCurrentHeadPos();
    if (currentHeadPos.has_value()) {
        return currentHeadPos.value();
    }

    return glm::fvec3(0.0f);
}


static void hook_BeginRoomscaleMovement(PPCInterpreter_t* hCPU) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::RoomscaleBeginHook);

    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t controller = hCPU->gpr[3];
    hCPU->gpr[3] = RoomscaleHookResult_Skip;
    ClearRoomscaleResolveState();

    PlayerBase player = {};
    if (!Player::TryRead(player)) {
        return;
    }

    uint32_t playerController = GameUtils::ResolveActorPhysicsController(player);
    if (playerController == 0 || controller != playerController) {
        return;
    }

    s_pendingBodyMove.active = false;

    glm::fvec3 currentHeadPos = {};
    glm::fvec3 roomDelta = {};
    glm::fvec3 worldDelta = {};
    if (!TryGetRoomscaleMoveDelta(player, currentHeadPos, roomDelta, worldDelta)) {
        if (s_roomscaleBodyDriveMode == RoomscaleBodyDriveMode::DisabledSpecialMovement) {
            return;
        }
        glm::fvec3 residual = GetRoomscaleResidual();
        glm::fvec3 residualWorldDelta = GetRoomscaleRoomToWorldRotation() * residual;
        residualWorldDelta.y = 0.0f;
        if (glm::dot(residualWorldDelta, residualWorldDelta) < kMinRoomscaleMoveDistanceSq) {
            s_roomscaleState.isBlocked = false;
            s_roomscaleState.isBlockedByCollision = false;
            return;
        }

        float residualMoveDistance = glm::length(residualWorldDelta);
        uint32_t maxAttempts = std::min<uint32_t>(std::max<uint32_t>((uint32_t)std::ceil(residualMoveDistance / kRoomscaleStepDistance), 1u), kMaxRoomscaleRaycastAttempts);

        s_roomscaleState.remainingRawDelta = residual;
        s_roomscaleState.remainingWorldDelta = residualWorldDelta;
        s_roomscaleState.attemptIndex = 0;
        s_roomscaleState.maxAttempts = maxAttempts;
        s_roomscaleState.isResolving = true;
        s_roomscaleState.isProbeOnly = true;
        CacheRoomscaleResolveContext(player);
        hCPU->gpr[3] = RoomscaleHookResult_Cast;
        BetterVRProfiler::BeginSpan(BetterVRProfiler::Section::RoomscaleResolve);
        return;
    }
    (void)currentHeadPos;

    float moveDistance = glm::length(worldDelta);
    uint32_t maxAttempts = std::min<uint32_t>(std::max<uint32_t>((uint32_t)std::ceil(moveDistance / kRoomscaleStepDistance), 1u), kMaxRoomscaleRaycastAttempts);

    s_roomscaleState.remainingRawDelta = roomDelta;
    s_roomscaleState.remainingWorldDelta = worldDelta;
    s_roomscaleState.attemptIndex = 0;
    s_roomscaleState.maxAttempts = maxAttempts;
    s_roomscaleState.isResolving = true;
    s_roomscaleState.isProbeOnly = false;
    CacheRoomscaleResolveContext(player);
    hCPU->gpr[3] = RoomscaleHookResult_Cast;
    BetterVRProfiler::BeginSpan(BetterVRProfiler::Section::RoomscaleResolve);
}

static void hook_PrepareRoomscaleRaycast(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t scratchPtr = hCPU->gpr[3];
    hCPU->gpr[3] = RoomscaleHookResult_Skip;

    if (!s_roomscaleState.isResolving) {
        return;
    }

    if (s_roomscaleState.attemptIndex >= s_roomscaleState.maxAttempts || glm::dot(s_roomscaleState.remainingWorldDelta, s_roomscaleState.remainingWorldDelta) < kMinRoomscaleMoveDistanceSq) {
        ClearRoomscaleResolveState();
        hCPU->gpr[3] = RoomscaleHookResult_Warp;
        return;
    }

    RoomscaleRaycastScratch scratch = {};
    Guest::Read(scratchPtr, &scratch);

    uint32_t remainingSteps = s_roomscaleState.maxAttempts - s_roomscaleState.attemptIndex;
    if (remainingSteps == 0) {
        ClearRoomscaleResolveState();
        hCPU->gpr[3] = RoomscaleHookResult_Warp;
        return;
    }

    if (!s_roomscaleState.hasActiveStep) {
        BeginRoomscaleActiveStep(remainingSteps);
    }

    if (glm::dot(s_roomscaleState.currentStepWorldDelta, s_roomscaleState.currentStepWorldDelta) < kMinRoomscaleMoveDistanceSq) {
        ClearRoomscaleResolveState();
        hCPU->gpr[3] = RoomscaleHookResult_Warp;
        return;
    }

    const RoomscaleDebugBody& body = s_roomscaleResolveBody;
    const glm::fvec3 currentWorldPos = scratch.currentPos.getLE();
    const RoomscaleResolvePhase phase = s_roomscaleState.resolvePhase;

    scratch.groundHit = kRoomscaleGroundHit;
    scratch.sweepRadius = body.sweepRadius;
    scratch.hitNormal = glm::fvec3(0.0f);
    if (phase == RoomscaleResolvePhase_ForwardSweep) {
        scratch.queryType = RoomscaleQueryType_Sweep;
        scratch.castFrom = GetRoomscaleBodyCapsuleCenter(body, currentWorldPos);
        scratch.castDelta = s_roomscaleState.currentStepResolvedWorldDelta;
        scratch.hitPos = scratch.castFrom.getLE() + scratch.castDelta.getLE();
        s_roomscaleState.currentStepCandidateWorldPos = currentWorldPos + s_roomscaleState.currentStepResolvedWorldDelta;
    }
    else if (phase == RoomscaleResolvePhase_StepUpSweep) {
        const glm::fvec3 stepUpDelta = glm::fvec3(0.0f, kRoomscaleStepUpHeight, 0.0f);
        scratch.queryType = RoomscaleQueryType_Sweep;
        scratch.castFrom = GetRoomscaleBodyCapsuleCenter(body, currentWorldPos);
        scratch.castDelta = stepUpDelta;
        scratch.hitPos = scratch.castFrom.getLE() + stepUpDelta;
        s_roomscaleState.currentStepCandidateWorldPos = currentWorldPos + stepUpDelta;
    }
    else if (phase == RoomscaleResolvePhase_StepForwardSweep) {
        const glm::fvec3 steppedUpWorldPos = s_roomscaleState.currentStepCandidateWorldPos;
        scratch.queryType = RoomscaleQueryType_Sweep;
        scratch.castFrom = GetRoomscaleBodyCapsuleCenter(body, steppedUpWorldPos);
        scratch.castDelta = s_roomscaleState.currentStepResolvedWorldDelta;
        scratch.hitPos = scratch.castFrom.getLE() + scratch.castDelta.getLE();
        s_roomscaleState.currentStepCandidateWorldPos = steppedUpWorldPos + s_roomscaleState.currentStepResolvedWorldDelta;
    }
    else if (IsRoomscaleSupportProbePhase(phase)) {
        glm::fvec3 probeBodyPos = s_roomscaleState.currentStepCandidateWorldPos;
        PrepareRoomscaleSupportProbe(scratch, body, probeBodyPos);
    }

    DrawRoomscaleDebugCast(scratch, body, s_roomscaleState.isProbeOnly, s_roomscaleShouldShowPhysicsDebug);
    Guest::Write(scratchPtr, &scratch);
    hCPU->gpr[3] = RoomscaleHookResult_Cast;
}

static void hook_BuildRoomscaleWarpTransform(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t currentPosPtr = hCPU->gpr[3];
    uint32_t transformPtr = hCPU->gpr[4];

    BEVec3 currentPos = {};
    BEMatrix34 transform = {};
    Guest::Read(currentPosPtr, &currentPos);
    Guest::Read(transformPtr, &transform);

    transform.setPos(currentPos.getLE());
    Guest::Write(transformPtr, &transform);
}

static void hook_ConsumeRoomscaleRaycast(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t scratchPtr = hCPU->gpr[3];
    bool hadHit = hCPU->gpr[4] != 0;
    hCPU->gpr[3] = RoomscaleHookResult_Skip;

    if (!s_roomscaleState.isResolving) {
        return;
    }

    uint32_t remainingSteps = s_roomscaleState.maxAttempts - s_roomscaleState.attemptIndex;
    if (remainingSteps == 0) {
        ClearRoomscaleResolveState();
        hCPU->gpr[3] = RoomscaleHookResult_Warp;
        return;
    }

    if (IsRoomscaleSweepPhase(s_roomscaleState.resolvePhase)) {
        if (s_roomscaleShouldShowPhysicsDebug) {
            RoomscaleRaycastScratch scratch = {};
            Guest::Read(scratchPtr, &scratch);
            DrawRoomscaleDebugCastResult(scratch, s_roomscaleState.isProbeOnly, hadHit, false, s_roomscaleShouldShowPhysicsDebug);
        }

        if (hadHit) {
            s_roomscaleState.currentStepBlockedByCollision = true;
            if (s_roomscaleState.resolvePhase == RoomscaleResolvePhase_ForwardSweep) {
                BeginRoomscaleResolvePhase(RoomscaleResolvePhase_StepUpSweep);
                hCPU->gpr[3] = RoomscaleHookResult_Cast;
                return;
            }

            FinishRoomscaleResolveWarp(hCPU, true, true);
            return;
        }

        if (s_roomscaleState.resolvePhase == RoomscaleResolvePhase_ForwardSweep) {
            BeginRoomscaleResolvePhase(RoomscaleResolvePhase_DownSupportProbe);
        }
        else if (s_roomscaleState.resolvePhase == RoomscaleResolvePhase_StepUpSweep) {
            BeginRoomscaleResolvePhase(RoomscaleResolvePhase_StepForwardSweep);
        }
        else {
            BeginRoomscaleResolvePhase(RoomscaleResolvePhase_StepDownSupportProbe);
        }

        hCPU->gpr[3] = RoomscaleHookResult_Cast;
        return;
    }

    RoomscaleRaycastScratch scratch = {};
    Guest::Read(scratchPtr, &scratch);
    const RoomscaleDebugBody& body = s_roomscaleResolveBody;
    const bool isStepProbe = s_roomscaleState.resolvePhase == RoomscaleResolvePhase_StepDownSupportProbe;
    const float maxGroundRise = isStepProbe ? (kRoomscaleStepUpHeight + kRoomscaleGroundSnapMaxRise) : kRoomscaleGroundSnapMaxRise;
    RoomscaleSupportState support = {};
    const bool hasSupport = hadHit && TryBuildRoomscaleSupportState(body, scratch.currentPos.getLE(), maxGroundRise, scratch, support);
    DrawRoomscaleDebugCastResult(scratch, s_roomscaleState.isProbeOnly, hadHit, hasSupport, s_roomscaleShouldShowPhysicsDebug);

    if (!hasSupport) {
        if (!isStepProbe) {
            glm::fvec3 appliedHeadDelta = s_roomscaleState.currentStepRawDelta;
            if (!CommitRoomscaleBodyMove(scratchPtr, scratch, s_roomscaleState.currentStepCandidateWorldPos, appliedHeadDelta)) {
                FinishRoomscaleResolveWarp(hCPU, true, s_roomscaleState.currentStepBlockedByCollision);
                return;
            }

            s_roomscaleState.support = {};
            AdvanceRoomscaleCommittedStep(appliedHeadDelta);
            if (glm::dot(s_roomscaleState.remainingWorldDelta, s_roomscaleState.remainingWorldDelta) < kMinRoomscaleMoveDistanceSq) {
                FinishRoomscaleResolveWarp(hCPU, false, false);
                return;
            }

            if (s_roomscaleState.attemptIndex >= s_roomscaleState.maxAttempts) {
                FinishRoomscaleResolveWarp(hCPU, true, false);
                return;
            }

            hCPU->gpr[3] = RoomscaleHookResult_Cast;
            return;
        }

        FinishRoomscaleResolveWarp(hCPU, true, s_roomscaleState.currentStepBlockedByCollision);
        return;
    }

    s_roomscaleState.currentStepSupport = StabilizeRoomscaleSupportState(s_roomscaleState.support, support);
    glm::fvec3 appliedHeadDelta = s_roomscaleState.currentStepRawDelta;
    if (!CommitRoomscaleBodyMoveWithSupport(scratchPtr, scratch, s_roomscaleState.currentStepSupport, appliedHeadDelta)) {
        FinishRoomscaleResolveWarp(hCPU, true, s_roomscaleState.currentStepBlockedByCollision);
        return;
    }

    AdvanceRoomscaleCommittedStep(appliedHeadDelta);
    if (glm::dot(s_roomscaleState.remainingWorldDelta, s_roomscaleState.remainingWorldDelta) < kMinRoomscaleMoveDistanceSq) {
        FinishRoomscaleResolveWarp(hCPU, false, false);
        return;
    }

    if (s_roomscaleState.attemptIndex >= s_roomscaleState.maxAttempts) {
        FinishRoomscaleResolveWarp(hCPU, true, false);
        return;
    }

    hCPU->gpr[3] = RoomscaleHookResult_Cast;
}

void Roomscale::RegisterHooks() {
    CemuHost::RegisterHook("hook_BeginRoomscaleMovement", &hook_BeginRoomscaleMovement);
    CemuHost::RegisterHook("hook_PrepareRoomscaleRaycast", &hook_PrepareRoomscaleRaycast);
    CemuHost::RegisterHook("hook_BuildRoomscaleWarpTransform", &hook_BuildRoomscaleWarpTransform);
    CemuHost::RegisterHook("hook_ConsumeRoomscaleRaycast", &hook_ConsumeRoomscaleRaycast);
}
