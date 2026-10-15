#include "pch.h"
#include "magnesis.h"
#include "hand_aim.h"
#include "cemu/cemu_host.h"
#include "utils/debug_draw.h"

static constexpr uint32_t MagneManagerInstanceAddress = 0x10467BE0;

static constexpr uint32_t MagneActionCancelAngle = 0x40;
static constexpr uint32_t MagneActionCancelHeight = 0x44;
static constexpr uint32_t MagneActionCancelAngleFixed = 0x48;
static constexpr uint32_t MagneActionCancelHeightFixed = 0x4C;
static constexpr uint32_t MagneActionPlayerPos = 0x80;
static constexpr uint32_t MagneActionDistance = 0xA0;
static constexpr uint32_t MagneActionHeight = 0xA4;
static constexpr uint32_t MagneActionYaw = 0xA8;
static constexpr uint32_t MagneActionBaseDirection = 0xC4;

static constexpr uint32_t MagneManagerObjectPos = 0xD4;
static constexpr uint32_t MagneManagerFixedObject = 0x2C3;

// fraction of the game's own release thresholds the aim is allowed to reach
static constexpr float MagneCancelSafetyMargin = 0.6f;
static constexpr float MagneFallbackCancelAngle = 0.45f;
static constexpr float MagneFallbackCancelHeight = 2.0f;

// the hand's distance to the object scales with the reach ratio to this power, so a short pull brings a far object close
static constexpr float MagneReachGain = 2.0f;
static constexpr float MagneMinReach = 0.15f;
static constexpr float MagneGripThreshold = 0.51f;

static bool s_aimGripHeld = false;
static float s_gripReach = 0.0f;
static glm::vec3 s_gripHandForward = glm::vec3(0.0f, 0.0f, -1.0f);
static glm::vec3 s_gripHandToObject = glm::vec3(0.0f);

// angular velocity per radian the object is behind the hand, and its cap in radians per second
static constexpr float MagneSpinGain = 15.0f;
static constexpr float MagneMaxSpin = 20.0f;

static glm::quat s_grabHandRotation = glm::identity<glm::quat>();
static glm::quat s_grabBodyRotation = glm::identity<glm::quat>();
static bool s_spinGripHeld = false;

static bool IsPlausiblePPCPointer(uint32_t pointer) {
    return pointer >= 0x10000000 && pointer < 0xF0000000 && (pointer & 3) == 0;
}

static float ReadMagneStaticFloatParam(uint32_t action, uint32_t fieldOffset, float fallback) {
    const uint32_t valuePointer = Guest::Get<BEType<uint32_t>>(action + fieldOffset).getLE();
    if (!IsPlausiblePPCPointer(valuePointer)) {
        return fallback;
    }

    const float value = Guest::Get<BEType<float>>(valuePointer).getLE();
    return (std::isfinite(value) && value > 0.0f) ? value : fallback;
}

static glm::vec3 FlattenHorizontally(glm::vec3 value) {
    value.y = 0.0f;
    return value;
}

static glm::vec3 ClampDirectionToCone(const glm::vec3& desired, const glm::vec3& reference, float maxAngle) {
    const float alignment = glm::clamp(glm::dot(desired, reference), -1.0f, 1.0f);
    if (alignment >= std::cos(maxAngle)) {
        return desired;
    }

    glm::vec3 perpendicular = desired - reference * alignment;
    if (glm::length2(perpendicular) < 0.000001f) {
        return desired;
    }

    perpendicular = glm::normalize(perpendicular);
    return glm::normalize(reference * std::cos(maxAngle) + perpendicular * std::sin(maxAngle));
}

static bool DriveMagnesisAimFromController(uint32_t action) {
    const uint32_t manager = Guest::Get<BEType<uint32_t>>(MagneManagerInstanceAddress).getLE();
    if (!IsPlausiblePPCPointer(manager)) {
        return false;
    }

    const float gameDistance = Guest::Get<BEType<float>>(action + MagneActionDistance).getLE();
    if (!std::isfinite(gameDistance) || gameDistance < 0.01f) {
        return false;
    }

    BEVec3 playerPosBE = {};
    BEVec3 objectPosBE = {};
    Guest::Read(action + MagneActionPlayerPos, &playerPosBE);
    Guest::Read(manager + MagneManagerObjectPos, &objectPosBE);
    const glm::vec3 playerPos = playerPosBE.getLE();
    const glm::vec3 objectPos = objectPosBE.getLE();
    if (!IsAllFinite(playerPos) || !IsAllFinite(objectPos)) {
        return false;
    }

    const glm::vec3 objectOffset = FlattenHorizontally(objectPos - playerPos);
    if (glm::length2(objectOffset) < 0.01f) {
        return false;
    }

    const OpenXR::InputState inputs = Tracking::Hands();
    glm::vec3 handPos = glm::vec3(0.0f);
    glm::vec3 handForward = glm::vec3(0.0f, 0.0f, -1.0f);
    if (!HandAim::TryGetControllerWorldPose(inputs, OpenXR::EyeSide::RIGHT, false, handPos, handForward)) {
        return false;
    }

    const auto headPose = Tracking::HeadPose();
    if (!headPose.has_value()) {
        return false;
    }

    // only follows the hand while the grip is held
    const bool gripHeld = inputs.inGame.grab[OpenXR::EyeSide::RIGHT].currentState > MagneGripThreshold;
    const bool gripStarted = gripHeld && !s_aimGripHeld;
    s_aimGripHeld = gripHeld;
    if (!gripHeld) {
        return false;
    }

    // holding the hand further out pushes the object further away
    const glm::vec3 headToHand = FlattenHorizontally(ToGLM(inputs.shared.poseLocation[OpenXR::EyeSide::RIGHT].pose.position) - glm::vec3(headPose.value()[3]));
    const float reach = std::max(glm::length(headToHand), MagneMinReach);

    if (gripStarted) {
        s_gripReach = reach;
        s_gripHandForward = handForward;
        s_gripHandToObject = objectPos - handPos;
    }

    // the object hangs on a stick from the hand that swings with where the hand points
    const glm::vec3 handToObject = glm::rotation(s_gripHandForward, handForward) * s_gripHandToObject * std::pow(reach / s_gripReach, MagneReachGain);
    const glm::vec3 targetOffset = handPos + handToObject - playerPos;
    const glm::vec3 aimOffset = FlattenHorizontally(targetOffset);
    const float distance = glm::length(aimOffset);
    if (!IsAllFinite(targetOffset) || distance < 0.001f) {
        return false;
    }

    uint8_t fixedObjectFlag = 0;
    Guest::Read(manager + MagneManagerFixedObject, &fixedObjectFlag);
    const bool isFixedObject = fixedObjectFlag != 0;
    const float cancelAngle = ReadMagneStaticFloatParam(action, isFixedObject ? MagneActionCancelAngleFixed : MagneActionCancelAngle, MagneFallbackCancelAngle);
    const float cancelHeight = ReadMagneStaticFloatParam(action, isFixedObject ? MagneActionCancelHeightFixed : MagneActionCancelHeight, MagneFallbackCancelHeight);

    const glm::vec3 objectDirection = glm::normalize(objectOffset);
    const glm::vec3 baseDirection = ClampDirectionToCone(glm::normalize(aimOffset), objectDirection, cancelAngle * MagneCancelSafetyMargin);

    const float heightLimit = cancelHeight * MagneCancelSafetyMargin;
    const float objectHeight = objectPos.y - playerPos.y;
    const float height = glm::clamp(targetOffset.y, objectHeight - heightLimit, objectHeight + heightLimit);
    if (!IsAllFinite(baseDirection) || !std::isfinite(height)) {
        return false;
    }

    // AppearMagneForce rotates +0xC4 by the yaw at +0xA8 and never writes +0xC4 itself
    BEVec3 baseDirectionBE = { baseDirection.x, 0.0f, baseDirection.z };
    Guest::Write(action + MagneActionBaseDirection, &baseDirectionBE);
    Guest::Set<float>(action + MagneActionYaw, 0.0f);
    // the game clamps this to its own magnesis range when it integrates the stick input
    Guest::Set<float>(action + MagneActionDistance, distance);
    Guest::Set<float>(action + MagneActionHeight, height);

    if (GetSettings().ShouldShowRaycastLines()) {
        DebugDraw::instance().Line(playerPos + baseDirection * distance + glm::vec3(0.0f, height, 0.0f), objectPos, DebugDrawColor(51, 204, 255));
    }

    return true;
}

static void hook_DriveMagnesisAim(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x020D72D0;

    static bool s_wasDriving = false;
    const bool driving = DriveMagnesisAimFromController(hCPU->gpr[3]);
    if (driving != s_wasDriving) {
        Log::print<CONTROLS>("[Magnesis] hand aim {}", driving ? "on" : "off");
        s_wasDriving = driving;
    }
}

// r3 = sead::Matrix34 of the held body, r4 = Vec3 that receives the angular velocity, returns whether to apply it
static void hook_GetMagnesisHeldBodySpin(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t bodyMtxAddress = hCPU->gpr[3];
    const uint32_t spinAddress = hCPU->gpr[4];
    hCPU->gpr[3] = 0;

    const uint32_t manager = Guest::Get<BEType<uint32_t>>(MagneManagerInstanceAddress).getLE();
    if (!IsPlausiblePPCPointer(manager)) {
        return;
    }

    // doors and sliding blocks hang on constraints that a forced spin would fight
    uint8_t fixedObjectFlag = 0;
    Guest::Read(manager + MagneManagerFixedObject, &fixedObjectFlag);
    if (fixedObjectFlag != 0) {
        return;
    }

    const OpenXR::InputState inputs = Tracking::Hands();
    const XrSpaceLocation& handLocation = inputs.shared.poseLocation[OpenXR::EyeSide::RIGHT];
    if (!inputs.shared.in_game || (handLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) == 0) {
        return;
    }

    glm::mat3 referenceBasis = glm::mat3(HandAim::ResolveCameraReferenceMtx(-1));
    referenceBasis[0] = glm::normalize(referenceBasis[0]);
    referenceBasis[1] = glm::normalize(referenceBasis[1]);
    referenceBasis[2] = glm::normalize(referenceBasis[2]);
    const glm::quat handRotation = glm::normalize(glm::quat_cast(referenceBasis) * ToGLM(handLocation.pose.orientation));

    BEMatrix34 bodyMtx = {};
    Guest::Read(bodyMtxAddress, &bodyMtx);
    glm::mat3 bodyBasis = glm::mat3(bodyMtx.getLEMatrix());
    bodyBasis[0] = glm::normalize(bodyBasis[0]);
    bodyBasis[1] = glm::normalize(bodyBasis[1]);
    bodyBasis[2] = glm::normalize(bodyBasis[2]);
    const glm::quat bodyRotation = glm::normalize(glm::quat_cast(bodyBasis));
    if (!IsAllFinite(glm::vec3(handRotation.x, handRotation.y, handRotation.z)) || !IsAllFinite(glm::vec3(bodyRotation.x, bodyRotation.y, bodyRotation.z))) {
        return;
    }

    const bool gripHeld = inputs.inGame.grab[OpenXR::EyeSide::RIGHT].currentState > MagneGripThreshold;
    const bool gripStarted = gripHeld && !s_spinGripHeld;
    s_spinGripHeld = gripHeld;
    if (!gripHeld) {
        return;
    }
    if (gripStarted) {
        s_grabHandRotation = handRotation;
        s_grabBodyRotation = bodyRotation;
    }

    // the object turns by however much the hand has turned since its grip was squeezed
    const glm::quat targetRotation = handRotation * glm::inverse(s_grabHandRotation) * s_grabBodyRotation;
    glm::quat error = glm::normalize(targetRotation * glm::inverse(bodyRotation));
    if (error.w < 0.0f) {
        error = -error;
    }

    const glm::vec3 errorAxis = glm::vec3(error.x, error.y, error.z);
    const float errorAxisLength = glm::length(errorAxis);
    glm::vec3 spin = glm::vec3(0.0f);
    if (errorAxisLength > 0.0001f) {
        const float errorAngle = 2.0f * std::atan2(errorAxisLength, error.w);
        spin = errorAxis / errorAxisLength * std::min(errorAngle * MagneSpinGain, MagneMaxSpin);
    }

    BEVec3 spinBE = { spin.x, spin.y, spin.z };
    Guest::Write(spinAddress, &spinBE);
    hCPU->gpr[3] = 1;
}

void Magnesis::RegisterHooks() {
    CemuHost::RegisterHook("hook_GetMagnesisHeldBodySpin", &hook_GetMagnesisHeldBodySpin);
    CemuHost::RegisterHook("hook_DriveMagnesisAim", &hook_DriveMagnesisAim);
}
