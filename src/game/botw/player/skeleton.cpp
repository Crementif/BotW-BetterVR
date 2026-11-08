#include "pch.h"
#include "skeleton.h"
#include "head_visibility.h"
#include "link_skeleton.h"
#include "weapon.h"
#include "player.h"
#include "roomscale.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/camera.h"
#include "rendering/tracking.h"
#include "utils/debug_draw.h"
#include "utils/render_utils.h"

static BoneHierarchy s_skeleton;
static bool s_skeletonParsed = false;
static glm::vec3 s_restEyeOffset = glm::vec3(0.0f);
static glm::vec3 s_manualBodyOffset = glm::vec3(0.0f, 0.0f, -0.075f);
static constexpr glm::vec3 NeckToEyeOffset = glm::vec3(0.0f, 0.10f, -0.09f);
static constexpr float MaximumGameHandSeparationSq = 4.0f;
static constexpr std::array<std::array<const char*, 5>, 2> GameAnimArmChainBoneNames = { { { "Clavicle_L", "Arm_1_L", "Arm_2_L", "Wrist_L", "Weapon_L" },
                                                                                          { "Clavicle_R", "Arm_1_R", "Arm_2_R", "Wrist_R", "Weapon_R" } } };
static constexpr uint32_t GameAnimArmChainRequiredMask = 0b01110;
static std::array<std::array<glm::mat4, 5>, 2> s_gameAnimArmChainLocals = {};
static std::array<uint32_t, 2> s_gameAnimArmChainSeenMasks = { 0, 0 };
static glm::mat4 s_gameLeftToRightWeaponMtx = glm::identity<glm::mat4>();
static bool s_gameLeftToRightWeaponMtxValid = false;
// cutscenes marked HND_OFF animate Link's arms themselves
static bool s_passAnimatedHands = false;

static void UpdateGameLeftToRightWeaponTransform() {
    if (!s_skeletonParsed)
        return;

    std::array<glm::mat4, 2> gameWeaponMtxs = { glm::identity<glm::mat4>(), glm::identity<glm::mat4>() };
    for (uint32_t sideIdx = 0; sideIdx < 2; sideIdx++) {
        if ((s_gameAnimArmChainSeenMasks[sideIdx] & GameAnimArmChainRequiredMask) != GameAnimArmChainRequiredMask)
            return;

        for (uint32_t chainIdx = 0; chainIdx < GameAnimArmChainBoneNames[sideIdx].size(); chainIdx++) {
            if (s_gameAnimArmChainSeenMasks[sideIdx] & (1u << chainIdx)) {
                gameWeaponMtxs[sideIdx] = gameWeaponMtxs[sideIdx] * s_gameAnimArmChainLocals[sideIdx][chainIdx];
            }
            else {
                Bone* restBone = s_skeleton.GetBone(GameAnimArmChainBoneNames[sideIdx][chainIdx]);
                if (restBone == nullptr)
                    return;
                gameWeaponMtxs[sideIdx] = gameWeaponMtxs[sideIdx] * restBone->localMatrix;
            }
        }
    }

    // Preserve the game's right-hand placement in the left attachment bone's coordinate frame.
    const glm::mat4 relativeMtx = glm::inverse(gameWeaponMtxs[OpenXR::EyeSide::LEFT]) * gameWeaponMtxs[OpenXR::EyeSide::RIGHT];
    if (!IsAllFinite(relativeMtx) || glm::length2(glm::vec3(relativeMtx[3])) > MaximumGameHandSeparationSq)
        return;

    s_gameLeftToRightWeaponMtx = relativeMtx;
    s_gameLeftToRightWeaponMtxValid = true;
}

static constexpr float BowFullDrawPullDistance = 0.30f;
static constexpr float BowSnapMaxEngageDistance = 0.35f;
static constexpr float BowSnapRampSeconds = 0.15f;
static constexpr float BowSnapMinDrawLength = 0.05f;
static constexpr float BowSnapMaxDrawLength = 0.9f;
static constexpr float BowMinPullPower = 0.35f;
static constexpr uint64_t BowReleasedPullLifetimeMs = 500;
static bool s_bowSnapTriggerHeld = false;
static bool s_bowSnapEngaged = false;
static bool s_arrowNearBowStart = false;

struct BowPullState {
    bool valid = false;
    float engageSeparation = 0.0f;
    float ratio = 0.0f;
};
static BowPullState s_bowPull;
static std::atomic<float> s_liveBowPullPower = 1.0f;
static std::atomic<float> s_releasedBowPullPower = 1.0f;
static std::atomic<uint64_t> s_bowReleasedAtMs = 0;

bool Skeleton::IsBowDrawEngaged() {
    return s_bowSnapEngaged;
}

bool Skeleton::IsArrowNearBowStart() {
    return GameState::IsFirstPerson() && s_arrowNearBowStart;
}

enum class ArmPoseState : uint8_t {
    ControllerFollow,
    TwoHandGrip,
    BowDrawSnap,
    CarryPin,
};

struct ArmPoseFrame {
    std::array<ArmPoseState, 2> handState = { ArmPoseState::ControllerFollow, ArmPoseState::ControllerFollow };
    float bowSnapWeight = 0.0f;
    float carryWeight = 0.0f;
};
static ArmPoseFrame s_armPoseFrame;

struct GameAnimSnapshot {
    bool carryingThrowable = false;
};
static GameAnimSnapshot s_lastGameAnimSnapshot;

static GameAnimSnapshot ReadGameAnimSnapshot() {
    GameAnimSnapshot snapshot;
    snapshot.carryingThrowable = Weapons::IsThrowableCarriedInBothHands();
    return snapshot;
}

static float SmoothStep01(float t) {
    return t * t * (3.0f - 2.0f * t);
}

struct TimedRamp {
    float value = 0.0f;
    std::chrono::steady_clock::time_point lastUpdate{};

    float Advance(bool active, float rampSeconds) {
        const auto now = std::chrono::steady_clock::now();
        const float dt = glm::clamp(std::chrono::duration<float>(now - lastUpdate).count(), 0.0f, 0.1f);
        lastUpdate = now;
        value = glm::clamp(value + (active ? dt : -dt) / rampSeconds, 0.0f, 1.0f);
        return value;
    }
};

static constexpr float CarryPinRampSeconds = 0.3f;
static TimedRamp s_carryRamp;

static TimedRamp s_bowSnapRamp;

static float GetBowDrawSnapWeight() {
    return SmoothStep01(s_bowSnapRamp.Advance(s_bowSnapEngaged && s_bowPull.valid, BowSnapRampSeconds));
}

static float CalcBowPullPower() {
    if (!s_bowSnapEngaged || !s_bowPull.valid)
        return 1.0f;
    return glm::mix(BowMinPullPower, 1.0f, glm::clamp(s_bowPull.ratio, 0.0f, 1.0f));
}

float Skeleton::GetBowPullPower() {
    return s_liveBowPullPower.load(std::memory_order_relaxed);
}

float Skeleton::ConsumeReleasedBowPullPower() {
    const uint64_t releasedAtMs = s_bowReleasedAtMs.exchange(0);
    if (releasedAtMs == 0 || GetTimeStampMs() - releasedAtMs > BowReleasedPullLifetimeMs)
        return 1.0f;
    return s_releasedBowPullPower.load(std::memory_order_relaxed);
}

static glm::mat4 BlendTransforms(const glm::mat4& fromMtx, const glm::mat4& toMtx, float weight) {
    const glm::fquat fromRot = glm::quat_cast(glm::mat3(fromMtx));
    const glm::fquat toRot = glm::quat_cast(glm::mat3(toMtx));
    const glm::vec3 blendedPos = glm::mix(glm::vec3(fromMtx[3]), glm::vec3(toMtx[3]), weight);
    return glm::translate(glm::identity<glm::mat4>(), blendedPos) * glm::mat4_cast(glm::slerp(fromRot, toRot, weight));
}

static glm::fquat ExtractPlayerBodyYaw(const glm::fmat4& sourceMtx) {
    glm::fquat sourceRot = glm::quat_cast(sourceMtx);
    float yProj = glm::dot(glm::fvec3(sourceRot.x, sourceRot.y, sourceRot.z), glm::fvec3(0.0f, 1.0f, 0.0f));
    glm::fquat yawRot(sourceRot.w, 0.0f, yProj, 0.0f);
    float lenSq = glm::dot(yawRot, yawRot);
    yawRot = (lenSq > 0.000001f) ? yawRot * (1.0f / sqrtf(lenSq)) : glm::identity<glm::fquat>();

    // fix body inversion
    return yawRot * glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}

static constexpr uint32_t TWO_HAND_WEAPON_ACTOR_MTX_OFFSET = 0x1F8;
static constexpr float TWO_HAND_MIN_HAND_SEPARATION = 0.05f; // avoids dividing by zero, how far the other hand can reach is limited by the weapon's bounding box
static constexpr float TWO_HAND_BLADE_AXIS_SMOOTHING = 0.15f;
static constexpr float TWO_HAND_GRAB_AABB_MARGIN = 0.04f;

struct TwoHandGripState {
    bool engaged = false;
    glm::vec3 rearOriginWorld = glm::vec3(0.0f);
    glm::vec3 leftPinnedWorld = glm::vec3(0.0f);
    glm::fquat rightGripRotWorld = glm::identity<glm::fquat>();
    glm::fquat leftGripRotWorld = glm::identity<glm::fquat>();
};
static TwoHandGripState s_twoHandGrip;
static bool s_twoHandGripActive = false;
static bool s_twoHandGripConsumesGrabInput = false;

bool Skeleton::IsTwoHandGripActive() {
    return s_twoHandGripActive;
}

bool Skeleton::TwoHandGripConsumesGrabInput() {
    return s_twoHandGripConsumesGrabInput;
}

void Skeleton::ReleaseTwoHandGripGrabInput() {
    s_twoHandGripConsumesGrabInput = false;
}

static glm::fquat s_lastAppliedRightWristRot = glm::identity<glm::fquat>();
static bool s_lastAppliedRightWristRotValid = false;
static uint32_t s_bladeAxisWeaponPtr = 0;
static glm::fvec3 s_bladeAxisInHand = glm::fvec3(0.0f);
static bool s_bladeAxisValid = false;
static glm::fvec3 s_lastGripAxisWorld = glm::fvec3(0.0f);

static bool s_twoHandGripButtonHeld = false;
static bool s_twoHandGripLatched = false;
static glm::fvec3 s_latchedGripOffsetInRightHand = glm::fvec3(0.0f);
static float s_latchedBladeDirectionSign = 1.0f;

static glm::fquat MinimalArcQuat(const glm::fvec3& from, const glm::fvec3& to) {
    if (!IsAllFinite(from) || !IsAllFinite(to) || glm::length2(from) < 0.000001f || glm::length2(to) < 0.000001f)
        return glm::identity<glm::fquat>();
    const glm::fvec3 fromDir = glm::normalize(from);
    const glm::fvec3 toDir = glm::normalize(to);
    const float alignment = glm::clamp(glm::dot(fromDir, toDir), -1.0f, 1.0f);
    if (alignment > 0.99999f)
        return glm::identity<glm::fquat>();

    if (alignment < -0.99999f) {
        // pick perpendicular axis for anti-parallel directions
        glm::fvec3 axis = glm::cross(fromDir, glm::fvec3(0.0f, 1.0f, 0.0f));
        if (glm::length2(axis) < 0.000001f)
            axis = glm::cross(fromDir, glm::fvec3(1.0f, 0.0f, 0.0f));
        return glm::angleAxis(glm::pi<float>(), glm::normalize(axis));
    }

    return glm::angleAxis(std::acos(alignment), glm::normalize(glm::cross(fromDir, toDir)));
}

static bool IsArmBone(int boneIndex) {
    for (int index = boneIndex; index >= 0; index = s_skeleton.GetBone(index)->parentIndex) {
        const std::string& name = s_skeleton.GetBone(index)->name;
        if (name == "Clavicle_L" || name == "Clavicle_R")
            return true;
    }
    return false;
}

static void EnsureSkeletonInitialized() {
    if (s_skeletonParsed)
        return;

    s_skeleton.Parse(LinkSkeleton::RestPose);
    s_skeletonParsed = true;
    Bone* eyeL = s_skeleton.GetBone("Eyeball_L");
    Bone* eyeR = s_skeleton.GetBone("Eyeball_R");
    Bone* sklRoot = s_skeleton.GetBone("Skl_Root");
    if (eyeL && eyeR && sklRoot) {
        const glm::vec3 eyePos = (glm::vec3(eyeL->worldMatrix[3]) + glm::vec3(eyeR->worldMatrix[3])) * 0.5f;
        s_restEyeOffset = eyePos - glm::vec3(sklRoot->worldMatrix[3]);
    }
}

static void DebugDrawWeaponAxes(uint32_t weaponPtr) {
    if (weaponPtr == 0)
        return;

    BEMatrix34 weaponMtxBE = {};
    Guest::Read(weaponPtr + TWO_HAND_WEAPON_ACTOR_MTX_OFFSET, &weaponMtxBE);

    const glm::mat4x3 weaponMtx = weaponMtxBE.getLEMatrix();
    const glm::vec3 origin = glm::vec3(weaponMtx[3]);
    if (!IsAllFinite(origin))
        return;

    constexpr float axisLength = 0.9f;
    auto drawAxis = [&](const glm::vec3& axis, uint32_t color) {
        if (glm::length2(axis) < 0.000001f)
            return;
        DebugDraw::instance().Line(origin, origin + glm::normalize(axis) * axisLength, color, 3.0f, true);
    };

    drawAxis(glm::vec3(weaponMtx[0]), DebugDrawColor(255, 60, 60));
    drawAxis(glm::vec3(weaponMtx[1]), DebugDrawColor(60, 255, 60));
    drawAxis(glm::vec3(weaponMtx[2]), DebugDrawColor(80, 120, 255));
    DebugDraw::instance().Sphere(origin, 0.04f, DebugDrawColor(255, 255, 0), 10, true);
}

static bool IsPointInsideWeaponAABB(uint32_t weaponPtr, const glm::vec3& worldPoint) {
    if (weaponPtr == 0)
        return false;

    BEMatrix34 weaponMtxBE = {};
    Guest::Read(weaponPtr + offsetof(ActorWiiU, mtx), &weaponMtxBE);
    const glm::vec3 weaponPos = glm::vec3(weaponMtxBE.getLEMatrix()[3]);
    const glm::fquat weaponRot = weaponMtxBE.getRotLE();

    const glm::vec3 aabbMin = Guest::Get<BEVec3>(weaponPtr + offsetof(ActorWiiU, aabb.min)).getLE();
    const glm::vec3 aabbMax = Guest::Get<BEVec3>(weaponPtr + offsetof(ActorWiiU, aabb.max)).getLE();
    if (!IsAllFinite(aabbMin) || !IsAllFinite(aabbMax) || !IsAllFinite(weaponPos))
        return false;

    if (glm::all(glm::lessThanEqual(aabbMax - aabbMin, glm::vec3(0.0f))))
        return false;

    const glm::vec3 localPoint = glm::inverse(weaponRot) * (worldPoint - weaponPos);
    const glm::vec3 margin = glm::vec3(TWO_HAND_GRAB_AABB_MARGIN);
    return glm::all(glm::greaterThanEqual(localPoint, aabbMin - margin)) && glm::all(glm::lessThanEqual(localPoint, aabbMax + margin));
}

struct BoneFrameContext {
    const OpenXR::InputState& inputs;
    glm::mat4 playerMtx;
    glm::mat4 stageToWorld;
};

static glm::mat4 FirstPersonStageToWorld(const glm::mat4& cameraMtx) {
    const glm::fvec3 appliedHeadPos = Roomscale::GetAppliedHeadPosition();
    return cameraMtx * glm::translate(glm::identity<glm::mat4>(), glm::vec3(-appliedHeadPos.x, 0.0f, -appliedHeadPos.z));
}

static glm::mat4 CalcControllerTargetModel(const BoneFrameContext& ctx, OpenXR::EyeSide arm) {
    const auto& handPose = ctx.inputs.shared.poseLocation[arm];
    glm::fvec3 controllerPos = glm::fvec3();
    glm::fquat controllerRot = glm::identity<glm::fquat>();
    if (handPose.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
        controllerPos = ToGLM(handPose.pose.position);
    if (handPose.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
        controllerRot = ToGLM(handPose.pose.orientation);

    glm::mat4 handCorrectionMtx = glm::mat4_cast(LinkSkeleton::HandCorrection(arm));
    glm::mat4 controllerMat = glm::translate(glm::identity<glm::mat4>(), controllerPos) * glm::mat4_cast(controllerRot) * handCorrectionMtx;
    glm::mat4 targetWorld = ctx.stageToWorld * controllerMat;

    const char* weaponName = arm == OpenXR::EyeSide::LEFT ? "Weapon_L" : "Weapon_R";
    if (Bone* weapon = s_skeleton.GetBone(weaponName)) {
        glm::vec3 weaponOffset = glm::vec3(weapon->localMatrix[3]);
        targetWorld = targetWorld * glm::translate(glm::identity<glm::mat4>(), -weaponOffset);
    }

    return glm::inverse(ctx.playerMtx) * targetWorld;
}

static glm::mat4 CalcTwoHandGripTargetModel(const BoneFrameContext& ctx, OpenXR::EyeSide arm) {
    const bool targetIsLeft = arm == OpenXR::EyeSide::LEFT;
    const glm::vec3 gripOrigin = targetIsLeft ? s_twoHandGrip.leftPinnedWorld : s_twoHandGrip.rearOriginWorld;
    const glm::fquat gripRot = targetIsLeft ? s_twoHandGrip.leftGripRotWorld : s_twoHandGrip.rightGripRotWorld;
    glm::mat4 targetWorld = glm::translate(glm::identity<glm::mat4>(), gripOrigin) * glm::mat4_cast(gripRot);

    const char* weaponName = arm == OpenXR::EyeSide::LEFT ? "Weapon_L" : "Weapon_R";
    if (Bone* weapon = s_skeleton.GetBone(weaponName)) {
        glm::vec3 weaponOffset = glm::vec3(weapon->localMatrix[3]);
        targetWorld = targetWorld * glm::translate(glm::identity<glm::mat4>(), -weaponOffset);
    }

    return glm::inverse(ctx.playerMtx) * targetWorld;
}

// where the game's animation puts the right hand's wrist relative to the left hand's
static std::optional<glm::mat4> CalcGameLeftToRightHandWristMtx() {
    Bone* leftWeapon = s_skeleton.GetBone("Weapon_L");
    Bone* rightWeapon = s_skeleton.GetBone("Weapon_R");
    if (!s_gameLeftToRightWeaponMtxValid || leftWeapon == nullptr || rightWeapon == nullptr)
        return std::nullopt;

    return leftWeapon->localMatrix * s_gameLeftToRightWeaponMtx * glm::inverse(rightWeapon->localMatrix);
}

static std::optional<glm::mat4> CalcBowDrawTargetModel(const BoneFrameContext& ctx) {
    if (!Weapons::IsArrowNockedInRightHand() || Weapons::GetHeldType(OpenXR::EyeSide::LEFT) != WeaponType::Bow || !ctx.inputs.shared.pose[OpenXR::EyeSide::LEFT].isActive)
        return std::nullopt;

    const std::optional<glm::mat4> leftToRightWristMtx = CalcGameLeftToRightHandWristMtx();
    if (!leftToRightWristMtx.has_value())
        return std::nullopt;

    // keep the game's hand to hand offset, from the VR bow hand
    return CalcControllerTargetModel(ctx, OpenXR::EyeSide::LEFT) * leftToRightWristMtx.value();
}

static std::optional<glm::mat4> CalcCarryLeftWristTargetModel(const BoneFrameContext& ctx) {
    const float carryWeight = s_armPoseFrame.carryWeight;
    if (carryWeight <= 0.0f)
        return std::nullopt;

    constexpr XrSpaceLocationFlags requiredFlags = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    if ((ctx.inputs.shared.poseLocation[OpenXR::EyeSide::RIGHT].locationFlags & requiredFlags) != requiredFlags)
        return std::nullopt;

    const std::optional<glm::mat4> leftToRightWristMtx = CalcGameLeftToRightHandWristMtx();
    if (!leftToRightWristMtx.has_value())
        return std::nullopt;

    // pin the left hand to the carried object using the carry animation's live left-to-right hand relationship
    const glm::mat4 rightWristTargetMtx = CalcControllerTargetModel(ctx, OpenXR::EyeSide::RIGHT);
    const glm::mat4 leftWristTargetMtx = rightWristTargetMtx * glm::inverse(leftToRightWristMtx.value());

    if (carryWeight >= 1.0f)
        return leftWristTargetMtx;
    return BlendTransforms(CalcControllerTargetModel(ctx, OpenXR::EyeSide::LEFT), leftWristTargetMtx, carryWeight);
}

static glm::mat4 ResolveArmTargetModel(const BoneFrameContext& ctx, OpenXR::EyeSide arm) {
    if (arm == OpenXR::EyeSide::LEFT && s_armPoseFrame.handState[arm] == ArmPoseState::CarryPin) {
        if (const std::optional<glm::mat4> carryTargetMtx = CalcCarryLeftWristTargetModel(ctx); carryTargetMtx.has_value())
            return carryTargetMtx.value();
    }
    if (arm == OpenXR::EyeSide::RIGHT && s_armPoseFrame.handState[arm] == ArmPoseState::BowDrawSnap) {
        if (const std::optional<glm::mat4> bowTargetMtx = CalcBowDrawTargetModel(ctx); bowTargetMtx.has_value()) {
            // keep the game's draw grip so the arrow lies on the bow, only pulled back along it as far as the controller is
            const glm::mat4 controllerMtx = CalcControllerTargetModel(ctx, arm);
            const glm::vec3 bowHandPos = glm::vec3(CalcControllerTargetModel(ctx, OpenXR::EyeSide::LEFT)[3]);
            const glm::vec3 gameDrawDir = glm::vec3(bowTargetMtx.value()[3]) - bowHandPos;
            glm::vec3 drawPos = glm::vec3(bowTargetMtx.value()[3]);
            if (glm::length2(gameDrawDir) > 1.0e-4f) {
                const glm::vec3 drawAxis = glm::normalize(gameDrawDir);
                drawPos = bowHandPos + drawAxis * glm::clamp(glm::dot(glm::vec3(controllerMtx[3]) - bowHandPos, drawAxis), BowSnapMinDrawLength, BowSnapMaxDrawLength);
            }
            const glm::mat4 drawMtx = glm::translate(glm::identity<glm::mat4>(), drawPos) * glm::mat4(glm::mat3(bowTargetMtx.value()));
            if (s_armPoseFrame.bowSnapWeight >= 1.0f)
                return drawMtx;
            return BlendTransforms(controllerMtx, drawMtx, s_armPoseFrame.bowSnapWeight);
        }
    }
    if (s_twoHandGrip.engaged)
        return CalcTwoHandGripTargetModel(ctx, arm);
    return CalcControllerTargetModel(ctx, arm);
}

// taken once on the root bone so all bones use the same controller sample
static OpenXR::InputState s_skeletonPassInputs = {};

static void hook_ModifyBoneMatrix(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    const uint32_t modelUnitPtr = hCPU->gpr[7];
    hCPU->gpr[7] = 0;
    hCPU->gpr[8] = 0;
    hCPU->gpr[9] = 0;

    const uint32_t gsysModelPtr = hCPU->gpr[3];
    const uint32_t matrixPtr = hCPU->gpr[4];
    const uint32_t scalePtr = hCPU->gpr[5];
    const uint32_t boneNamePtr = hCPU->gpr[6];
    if (!gsysModelPtr || !matrixPtr || !scalePtr || !boneNamePtr) return;

    std::string boneName(Guest::CString(boneNamePtr));
    const auto modelName = Guest::Get<sead::FixedSafeString100>(gsysModelPtr + 0x128);
    EnsureSkeletonInitialized();
    const uint32_t playerActor = Player::GetAddress();
    if (modelName.getLE() != "GameROMPlayer" || playerActor == 0 || Guest::Get<uint32_t>(playerActor + 0x330).getLE() != gsysModelPtr)
        return;

    const bool isLeft = boneName.ends_with("_L");
    const OpenXR::EyeSide side = isLeft ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;
    // capture the game's animated arm pose before it gets overwritten below
    for (uint32_t chainIdx = 0; chainIdx < GameAnimArmChainBoneNames[side].size(); chainIdx++) {
        if (boneName == GameAnimArmChainBoneNames[side][chainIdx]) {
            s_gameAnimArmChainLocals[side][chainIdx] = glm::mat4(Guest::Get<BEMatrix34>(matrixPtr).getLEMatrix());
            s_gameAnimArmChainSeenMasks[side] |= 1u << chainIdx;
            break;
        }
    }

    // reset grip state on root to prevent stranding on tracking loss
    if (boneName == "Skl_Root") {
        s_skeletonPassInputs = Tracking::Hands();
        s_passAnimatedHands = Events::UsesAnimatedHands();
        UpdateGameLeftToRightWeaponTransform();
        s_twoHandGrip = {};
        s_armPoseFrame = {};
        s_twoHandGripActive = false;
        HeadVisibility::OnSkeletonRoot();
    }

    auto writeBoneMatrix = [&](const glm::mat4x3& mtx, const glm::fvec3& scale) {
        BEMatrix34 m{ mtx };
        Guest::Set(matrixPtr, m);
        Guest::Set(scalePtr, scale);
    };
    auto writeBoneQuat = [&](const glm::vec3& pos, const glm::quat& rot, const glm::fvec3& scale) {
        BEMatrix34 m{ pos, rot };
        Guest::Set(matrixPtr, m);
        Guest::Set(scalePtr, scale);
    };

    const bool isThirdPerson = GameState::IsThirdPerson() || !GameState::IsInGame();

    // one material per bone, so a whole head model converges well within a single frame
    HeadVisibility::SubmitNextMaterial(hCPU, !isThirdPerson);

    if (isThirdPerson) {
        // wrist is game-animated in third person
        s_lastAppliedRightWristRotValid = false;
        if (HeadVisibility::IsFaceBone(boneName)) {
            Guest::Set(scalePtr, glm::fvec3(1.0f));
        }
        return;
    }

    if (HeadVisibility::IsFaceBone(boneName)) {
        return;
    }

    glm::fvec3 boneScale = Guest::Get<BEVec3>(scalePtr).getLE();
    const glm::fmat4 playerMtx4 = glm::fmat4(Guest::Get<BEMatrix34>(Player::GetMtxAddress()).getLEMatrix());
    const glm::mat4 cameraMtx = Camera::GetFreshReferenceMtx();

    const OpenXR::InputState& inputs = s_skeletonPassInputs;
    if (!inputs.shared.pose[side].isActive) {
        if (boneName == "Skl_Root")
            s_lastAppliedRightWristRotValid = false;
        return;
    }

    EnsureSkeletonInitialized();

    int boneIndex = s_skeleton.GetBoneIndex(boneName);
    if (boneIndex == -1)
        return;

    const BoneFrameContext ctx{ inputs, playerMtx4, FirstPersonStageToWorld(cameraMtx) };

    glm::mat4 calculatedLocalMat = s_skeleton.GetBone(boneIndex)->localMatrix;

    if (boneName == "Skl_Root") {
        auto headsetPose = Tracking::HeadPose();
        glm::mat4 headsetMtx = headsetPose.value_or(ToMat4(glm::fvec3(0)));

        // headset in model space
        glm::mat4 headsetModel = glm::inverse(playerMtx4) * cameraMtx * headsetMtx;
        headsetModel[3].x = 0.0f;
        headsetModel[3].z = 0.0f;

        const glm::mat4 animRootLocal = glm::mat4(Guest::Get<BEMatrix34>(matrixPtr).getLEMatrix());
        // animated hands reach for what the cutscene put in front of Link, so his body keeps the cutscene's facing
        glm::fquat yawRot = s_passAnimatedHands ? RenderUtils::swingTwistY(glm::quat_cast(glm::mat3(animRootLocal))).second : ExtractPlayerBodyYaw(headsetModel);

        // position the root so that yawRot * s_restEyeOffset lands at the headset position
        glm::vec3 targetPos = glm::vec3(headsetModel[3]) - (yawRot * s_restEyeOffset) + (yawRot * s_manualBodyOffset);

        // the eyes swing around the neck when tilting the head, so keep the body under the neck instead
        const glm::fquat headRot = glm::quat_cast(glm::mat3(headsetModel));
        glm::vec3 neckToEyeShift = (headRot * NeckToEyeOffset) - (RenderUtils::swingTwistY(headRot).second * NeckToEyeOffset);
        neckToEyeShift.y = 0.0f;
        targetPos -= neckToEyeShift;

        // update skeleton for children (hands)
        if (Bone* rootBone = s_skeleton.GetBone("Skl_Root")) {
            rootBone->localMatrix = glm::translate(glm::identity<glm::mat4>(), targetPos) * glm::mat4_cast(yawRot);
            s_skeleton.UpdateWorldMatrices();
        }

        auto calcHandWorldMat = [&](OpenXR::EyeSide handSide) {
            const auto& handPoseLocation = inputs.shared.poseLocation[handSide];
            glm::fvec3 handPos = glm::fvec3();
            glm::fquat handRot = glm::identity<glm::fquat>();
            if (handPoseLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
                handPos = ToGLM(handPoseLocation.pose.position);
            if (handPoseLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
                handRot = ToGLM(handPoseLocation.pose.orientation);
            const glm::mat4 handCorrectionMtx = glm::mat4_cast(LinkSkeleton::HandCorrection(handSide));
            return ctx.stageToWorld * glm::translate(glm::identity<glm::mat4>(), handPos) * glm::mat4_cast(handRot) * handCorrectionMtx;
        };

        const glm::mat4 rightHandWorld = calcHandWorldMat(OpenXR::EyeSide::RIGHT);
        const glm::fquat rightHandRotWorld = glm::quat_cast(glm::mat3(rightHandWorld));

        if (GetSettings().ShouldShowWeaponAxes()) {
            DebugDrawWeaponAxes(Weapons::GetHeldActor(OpenXR::EyeSide::LEFT));
            DebugDrawWeaponAxes(Weapons::GetHeldActor(OpenXR::EyeSide::RIGHT));

            const glm::vec3 rightGripPos = glm::vec3(rightHandWorld[3]);
            const glm::vec3 leftGripPos = glm::vec3(calcHandWorldMat(OpenXR::EyeSide::LEFT)[3]);
            DebugDraw::instance().Line(rightGripPos, leftGripPos, DebugDrawColor(0, 220, 255), 2.0f, true);
        }

        const uint32_t rightWeaponPtr = Weapons::GetHeldActor(OpenXR::EyeSide::RIGHT);
        if (Weapons::TwoHandGripEnabled && rightWeaponPtr != 0 && s_lastAppliedRightWristRotValid) {
            BEMatrix34 weaponMtxBE = {};
            Guest::Read(rightWeaponPtr + TWO_HAND_WEAPON_ACTOR_MTX_OFFSET, &weaponMtxBE);

            glm::vec3 bladeWorld = glm::vec3(weaponMtxBE.getLEMatrix()[2]); // Z column = blade axis
            if (s_bladeAxisWeaponPtr != rightWeaponPtr) {
                s_bladeAxisWeaponPtr = rightWeaponPtr;
                s_bladeAxisValid = false;
                s_twoHandGripLatched = false;
            }

            if (glm::length2(bladeWorld) > 0.000001f && IsAllFinite(bladeWorld)) {
                bladeWorld = glm::normalize(bladeWorld);
                const glm::fvec3 measuredBladeInHand = glm::normalize(glm::inverse(s_lastAppliedRightWristRot) * bladeWorld);
                s_bladeAxisInHand = s_bladeAxisValid ? glm::normalize(glm::mix(s_bladeAxisInHand, measuredBladeInHand, TWO_HAND_BLADE_AXIS_SMOOTHING)) : measuredBladeInHand;
                s_bladeAxisValid = true;
            }
        }

        const glm::mat4 leftHandWorld = calcHandWorldMat(EyeSide::LEFT);
        const glm::vec3 rearPos = glm::vec3(rightHandWorld[3]);
        const glm::vec3 leftPos = glm::vec3(leftHandWorld[3]);

        const bool leftGripHeld = Weapons::TwoHandGripEnabled && Weapons::CanEngageTwoHandGrip() && inputs.inGame.grab[EyeSide::LEFT].currentState > Weapons::TwoHandGripButtonThreshold;
        if (leftGripHeld && !s_twoHandGripButtonHeld && s_bladeAxisValid && IsPointInsideWeaponAABB(rightWeaponPtr, leftPos)) {
            const glm::vec3 grabOffsetWorld = leftPos - rearPos;
            const float grabSeparation = glm::length(grabOffsetWorld);
            if (grabSeparation > TWO_HAND_MIN_HAND_SEPARATION) {
                s_twoHandGripLatched = true;
                s_twoHandGripConsumesGrabInput = true;
                s_latchedGripOffsetInRightHand = glm::inverse(rightHandRotWorld) * grabOffsetWorld;
                s_latchedBladeDirectionSign = glm::dot(rightHandRotWorld * s_bladeAxisInHand, grabOffsetWorld) >= 0.0f ? 1.0f : -1.0f;
            }
        }
        if (!leftGripHeld) {
            s_twoHandGripLatched = false;
        }
        s_twoHandGripButtonHeld = leftGripHeld;

        if (s_twoHandGripLatched && s_bladeAxisValid) {
            const glm::vec3 rearToLeft = leftPos - rearPos;
            const float handSeparation = glm::length(rearToLeft);
            if (handSeparation > TWO_HAND_MIN_HAND_SEPARATION) {
                s_lastGripAxisWorld = rearToLeft / handSeparation; // calculate double-handed steering direction
            }

            const glm::vec3 bladeDir = s_lastGripAxisWorld;
            if (glm::length2(bladeDir) > 0.5f) {
                const glm::fquat leftHandRotWorld = glm::quat_cast(glm::mat3(leftHandWorld));

                const glm::vec3 currentBladeWorld = rightHandRotWorld * s_bladeAxisInHand;
                const glm::vec3 targetBladeDir = bladeDir * s_latchedBladeDirectionSign;

                const glm::fquat rightToLeftCorrectedHandRot = glm::inverse(LinkSkeleton::HandCorrection(EyeSide::LEFT)) * LinkSkeleton::HandCorrection(EyeSide::RIGHT);
                const glm::fvec3 leftBladeInHand = rightToLeftCorrectedHandRot * s_bladeAxisInHand;
                const glm::fquat solvedRightGripRotWorld = MinimalArcQuat(currentBladeWorld, targetBladeDir) * rightHandRotWorld;
                s_twoHandGrip.engaged = true;
                s_twoHandGrip.rearOriginWorld = rearPos;
                s_twoHandGrip.leftPinnedWorld = rearPos + solvedRightGripRotWorld * s_latchedGripOffsetInRightHand;
                s_twoHandGrip.rightGripRotWorld = solvedRightGripRotWorld;
                s_twoHandGrip.leftGripRotWorld = MinimalArcQuat(leftHandRotWorld * leftBladeInHand, targetBladeDir) * leftHandRotWorld;
            }
        }

        s_twoHandGripActive = s_twoHandGrip.engaged;

        // the trigger starts the draw, pulling the arrow hand away from the bow hand charges it
        const bool bowTriggerHeld = inputs.inGame.useRightItem.currentState;
        const bool bowInLeftHand = Weapons::GetHeldType(EyeSide::LEFT) == WeaponType::Bow;
        const float bowHandSeparation = glm::distance(rearPos, leftPos);

        const std::optional<glm::mat4> bowStartTargetMtx = CalcBowDrawTargetModel(ctx);
        const glm::vec3 controllerHandPos = glm::vec3(CalcControllerTargetModel(ctx, EyeSide::RIGHT)[3]);
        s_arrowNearBowStart = bowStartTargetMtx.has_value() &&
                              (inputs.shared.poseLocation[OpenXR::EyeSide::RIGHT].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
                              glm::distance(glm::vec3(bowStartTargetMtx.value()[3]), controllerHandPos) <= BowSnapMaxEngageDistance;

        if (!bowTriggerHeld && s_bowSnapTriggerHeld) {
            s_releasedBowPullPower = CalcBowPullPower();
            s_bowReleasedAtMs = GetTimeStampMs();
        }

        if (bowTriggerHeld && !s_bowSnapTriggerHeld) {
            s_bowSnapEngaged = s_arrowNearBowStart;
            s_bowPull = { bowInLeftHand, bowHandSeparation, 0.0f };
        }
        else if (!bowTriggerHeld) {
            s_bowSnapEngaged = false;
        }
        else if (s_bowPull.valid) {
            // an arrow hand brought to the string later still snaps onto it, and the pull counts from there
            if (!s_bowSnapEngaged && s_arrowNearBowStart) {
                s_bowSnapEngaged = true;
                s_bowPull.engageSeparation = bowHandSeparation;
            }
            s_bowPull.ratio = glm::clamp((bowHandSeparation - s_bowPull.engageSeparation) / BowFullDrawPullDistance, 0.0f, 1.0f);
        }

        if (!bowInLeftHand) {
            s_bowPull.valid = false;
            s_arrowNearBowStart = false;
        }

        s_bowSnapTriggerHeld = bowTriggerHeld;
        s_liveBowPullPower = CalcBowPullPower();

        const GameAnimSnapshot animSnapshot = ReadGameAnimSnapshot();
        if (animSnapshot.carryingThrowable != s_lastGameAnimSnapshot.carryingThrowable)
            Log::print<CONTROLS>("GameAnim: carrying throwable {}", animSnapshot.carryingThrowable ? "started" : "ended");
        s_lastGameAnimSnapshot = animSnapshot;

        s_armPoseFrame.carryWeight = SmoothStep01(s_carryRamp.Advance(Weapons::CarryHandPinEnabled && animSnapshot.carryingThrowable, CarryPinRampSeconds));

        s_armPoseFrame.bowSnapWeight = GetBowDrawSnapWeight();
        if (s_armPoseFrame.bowSnapWeight > 0.0f)
            s_armPoseFrame.handState[OpenXR::EyeSide::RIGHT] = ArmPoseState::BowDrawSnap;
        else if (s_twoHandGrip.engaged)
            s_armPoseFrame.handState[OpenXR::EyeSide::RIGHT] = ArmPoseState::TwoHandGrip;
        if (s_armPoseFrame.carryWeight > 0.0f)
            s_armPoseFrame.handState[OpenXR::EyeSide::LEFT] = ArmPoseState::CarryPin;
        else if (s_twoHandGrip.engaged)
            s_armPoseFrame.handState[OpenXR::EyeSide::LEFT] = ArmPoseState::TwoHandGrip;

        s_lastAppliedRightWristRot = s_twoHandGrip.engaged ? s_twoHandGrip.rightGripRotWorld : rightHandRotWorld;
        s_lastAppliedRightWristRotValid = !s_passAnimatedHands;

        writeBoneQuat(targetPos, yawRot, boneScale);
        return;
    }

    if (s_passAnimatedHands && IsArmBone(boneIndex))
        return;

    if (boneName == "Arm_1_L" || boneName == "Arm_1_R" || boneName == "Elbow_L" || boneName == "Elbow_R" || boneName == "Wrist_Assist_L" || boneName == "Wrist_Assist_R") {

        int arm1Index = s_skeleton.GetBoneIndex(isLeft ? "Arm_1_L" : "Arm_1_R");
        int arm2Index = s_skeleton.GetBoneIndex(isLeft ? "Arm_2_L" : "Arm_2_R");
        int wristIndex = s_skeleton.GetBoneIndex(isLeft ? "Wrist_L" : "Wrist_R");

        if (arm1Index != -1 && arm2Index != -1 && wristIndex != -1) {
            glm::vec3 targetPos = glm::vec3(ResolveArmTargetModel(ctx, side)[3]);

            // pole vector (elbow hint): left-down-back / right-down-back, rotated by body yaw
            glm::vec3 poleDir = isLeft ? glm::vec3(1.0f, -1.0f, -0.5f) : glm::vec3(-1.0f, -1.0f, -0.5f);
            if (Bone* rootBone = s_skeleton.GetBone("Skl_Root"))
                poleDir = glm::quat_cast(rootBone->localMatrix) * poleDir;

            s_skeleton.SolveTwoBoneIK(arm1Index, arm2Index, wristIndex, targetPos, poleDir, isLeft ? 1.0f : -1.0f);
            calculatedLocalMat = s_skeleton.GetBone(boneIndex)->localMatrix;
        }
    }

    if (boneName == "Wrist_L" || boneName == "Wrist_R" || boneName == "Wrist_Assist_L" || boneName == "Wrist_Assist_R") {
        calculatedLocalMat = s_skeleton.CalculateLocalMatrixFromWorld(boneIndex, ResolveArmTargetModel(ctx, side));
    }

    writeBoneMatrix(glm::mat4x3(calculatedLocalMat), boneScale);
}

void Skeleton::RegisterHooks() {
    CemuHost::RegisterHook("hook_ModifyBoneMatrix", &hook_ModifyBoneMatrix);
}
