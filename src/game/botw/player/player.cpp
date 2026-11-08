#include "pch.h"
#include "player.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/camera.h"
#include "game/botw/debug/entity_debugger.h"
#include "rendering/tracking.h"
#include "utils/render_utils.h"

static uint32_t s_playerAddress = 0;
static bool s_playerSeenThisPass = false;
static uint32_t s_gameFramesSinceLadderClimb = UINT32_MAX;

uint32_t Player::GetAddress() {
    return s_playerAddress;
}

uint32_t Player::GetMtxAddress() {
    return s_playerAddress == 0 ? 0 : s_playerAddress + offsetof(ActorWiiU, mtx);
}

bool Player::TryRead(PlayerBase& player) {
    if (s_playerAddress == 0)
        return false;

    Guest::Read(s_playerAddress, &player);
    return true;
}

PlayerMoveBitFlags Player::GetMoveFlags() {
    if (s_playerAddress == 0)
        return PlayerMoveBitFlags{};
    return Guest::Get<PlayerMoveBitFlags>(s_playerAddress + offsetof(PlayerBase, moveBitFlags)).getLE();
}

bool Player::IsGliding() {
    return HAS_FLAG(GetMoveFlags(), PlayerMoveBitFlags::IS_GLIDER_ACTIVE);
}

bool Player::IsRiding(bool ignoreSandSeal) {
    if (s_playerAddress == 0)
        return false;

    // Player::getPlayerRideInfo (0x02D7A660), Player::isRiding (0x02D32E50)
    constexpr uint32_t RideFlagsOffset = 0x1F74 + 0x18;
    const bool riding = (Guest::Get<uint16_t>(s_playerAddress + RideFlagsOffset).getLE() & 1u) != 0;
    // act::acc::PlayerBase::isRidingSandSeal (0x02D3E344)
    return riding || (!ignoreSandSeal && HAS_FLAG(GetMoveFlags(), PlayerMoveBitFlags::IS_RIDING_SAND_SEAL));
}

bool Player::IsClimbingLadder() {
    return s_gameFramesSinceLadderClimb <= 2;
}

bool Player::IsClimbing() {
    return HAS_FLAG(GetMoveFlags(), PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_CLIMBING_WALL) || IsClimbingLadder();
}

// runs over the whole actor list every time an actor gets created
static void hook_UpdateActorList(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t listIndex = hCPU->gpr[5];
    const uint32_t actorPtr = hCPU->gpr[6];
    const uint32_t listSize = hCPU->gpr[7];
    if (listIndex == 0) {
        s_playerSeenThisPass = false;
    }

    const uint32_t actorNamePtr = Guest::Get<uint32_t>(actorPtr + offsetof(ActorWiiU, name) + offsetof(sead::FixedSafeString40, c_str)).getLE();
    if (actorNamePtr != 0) {
        const char* actorName = Guest::CString(actorNamePtr);
        EntityDebugger::TrackActor(listIndex, actorPtr, actorName);

        if (strcmp(actorName, "GameROMPlayer") == 0) {
            s_playerAddress = actorPtr;
            s_playerSeenThisPass = true;
        }
    }

    if (listIndex + 1 >= listSize && !s_playerSeenThisPass) {
        s_playerAddress = 0;
    }
}

constexpr uint32_t orig_PlayerNormalChangeStateInnerChangeChildFuncAddr = 0x037B8284;

static bool ShouldBlockFirstPersonPlayerNormalState(uint32_t stateNamePtr) {
    // the damage reaction states, the get-up state (0x101E0234/0x101E0C1C) has to stay since it's the only way out of a ragdoll
    switch (stateNamePtr) {
    case 0x101E0910:
    case 0x101E0930:
    case 0x101E0940:
        return true;
    default:
        return false;
    }
}

static void hook_PlayerNormalChangeState(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = orig_PlayerNormalChangeStateInnerChangeChildFuncAddr;

    const uint32_t stateNamePtr = hCPU->gpr[4];
    if (stateNamePtr == 0) {
        return;
    }

    if (GameState::IsFirstPerson() && GetSettings().ShouldPreventFirstPersonRagdoll() && ShouldBlockFirstPersonPlayerNormalState(stateNamePtr)) {
        Log::print<PPC>("Suppressing PlayerNormal state '{}' in first person", Guest::CString(stateNamePtr));
        hCPU->instructionPointer = hCPU->sprNew.LR;
        return;
    }

    static std::string s_currentPlayerNormalState;
    if (s_currentPlayerNormalState != Guest::CString(stateNamePtr)) {
        s_currentPlayerNormalState = Guest::CString(stateNamePtr);
        Log::print<PPC>("PlayerNormal state changed to '{}'", s_currentPlayerNormalState);
    }
}

struct ActionFloatParamPointerOverride {
    std::string name;
    uint32_t destPointerAddress;
    bool storedOriginalPointer = false;
    uint32_t originalPointer = 0;
};

static std::mutex s_actionFloatParamsLock;
static std::unordered_map<uint32_t, std::vector<ActionFloatParamPointerOverride>> s_actionFloatParams;

bool Player::TrackDamageFloatParam(uint32_t actionPtr, uint32_t destFloatPtr, std::string_view paramName) {
    const bool isDamageParam = paramName.find("Speed") != std::string_view::npos || paramName.starts_with("JumpHeight") || paramName.starts_with("AddLinearImpulse") || paramName.starts_with("AddRollImpulse");
    if (!isDamageParam) {
        return false;
    }

    std::scoped_lock lock(s_actionFloatParamsLock);
    auto& paramList = s_actionFloatParams[actionPtr];
    if (std::ranges::none_of(paramList, [paramName](const ActionFloatParamPointerOverride& entry) { return entry.name == paramName; })) {
        Log::print<PPC>("Storing float param '{}' offset {:08X} for action at {:08X}", paramName, destFloatPtr, actionPtr);
        paramList.push_back({ std::string(paramName), destFloatPtr });
    }
    return true;
}

static void ApplyStoredActionFloatParamOverrides() {
    constexpr uint32_t kPlayerLaunchZeroFloat = 0x101D3CC8;

    std::scoped_lock lock(s_actionFloatParamsLock);

    for (auto it = s_actionFloatParams.begin(); it != s_actionFloatParams.end();) {
        std::erase_if(it->second, [](ActionFloatParamPointerOverride& entry) {
            uint32_t currentPointer = Guest::Get<uint32_t>(entry.destPointerAddress).getLE();
            if (!entry.storedOriginalPointer) {
                if (currentPointer != 0 && currentPointer != kPlayerLaunchZeroFloat) {
                    entry.originalPointer = currentPointer;
                    entry.storedOriginalPointer = true;
                }
                return false;
            }

            // the slot no longer holds a pointer this override has put or seen there, so the action object was freed (player recreation) and this memory must not be touched again
            if (currentPointer != entry.originalPointer && currentPointer != kPlayerLaunchZeroFloat) {
                return true;
            }

            uint32_t targetPointer = (GameState::IsFirstPerson() && GetSettings().ShouldPreventFirstPersonRagdoll()) ? kPlayerLaunchZeroFloat : entry.originalPointer;
            if (currentPointer != targetPointer) {
                Guest::Set<uint32_t>(entry.destPointerAddress, targetPointer);
            }
            return false;
        });
        it = it->second.empty() ? s_actionFloatParams.erase(it) : std::next(it);
    }
}

void Player::OnGameFrame() {
    if (s_gameFramesSinceLadderClimb != UINT32_MAX) {
        s_gameFramesSinceLadderClimb++;
    }
    ApplyStoredActionFloatParamOverrides();
}

// PlayerLadder::calc, so this runs every game frame Link is on a ladder
static void hook_StoreLadderState(PPCInterpreter_t* hCPU) {
    hCPU->gpr[0] = hCPU->sprNew.LR;
    hCPU->instructionPointer = 0x02D07CEC;

    s_gameFramesSinceLadderClimb = 0;
}

static void hook_FixLadder(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // PlayerLadder::calc classifies the raw stick angle, so its answer is already camera independent
    if (hCPU->sprNew.LR == 0x02D07E28) {
        return;
    }

    auto input = Tracking::Hands();

    if (input.shared.in_game && !(GameState::IsFirstPerson() && Player::IsClimbingLadder())) {
        return;
    }

    if (input.inGame.move.currentState.y >= -0.05) {
        hCPU->gpr[3] = 4; // allows pressing A to jump upwards, regardless of camera orientation
    }
    else {
        hCPU->gpr[3] = 1; // allows sliding down when holding move stick downwards
    }
}

static bool IsStickPushed(uint32_t playerPtr) {
    return Guest::Get<float>(playerPtr + 0x1AC4).getLE() > 0.0f;
}

// with the stick let go the target yaw is just the camera's, and snapping to it would turn Link and his slide around
static void hook_ShouldSnapToStickDirection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    hCPU->gpr[3] = GameState::IsFirstPerson() && IsStickPushed(hCPU->gpr[3]) ? 1 : 0;
}

static void hook_SetActorOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    float toBeSetOpacity = hCPU->fpr[1].fp0;
    uint32_t actorPtr = hCPU->gpr[3];

    if (Guest::Get<float>(actorPtr + offsetof(ActorWiiU, modelOpacity)).getLE() != toBeSetOpacity) {
        uint8_t opacityOrDoFlushOpacityToGPU = 1;
        Guest::WriteBE(actorPtr + offsetof(ActorWiiU, modelOpacity), &toBeSetOpacity);
        Guest::WriteBE(actorPtr + offsetof(ActorWiiU, opacityOrDoFlushOpacityToGPU), &opacityOrDoFlushOpacityToGPU);
    }
}

static void hook_CalculateModelOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!GameState::IsFirstPerson()) {
        return;
    }

    hCPU->fpr[1].fp0 = 1.0f;
}

static std::optional<glm::mat4> TryGetHeadsetWorldMtx() {
    if (GameState::IsThirdPerson() || Camera::GetFramesSinceLastUpdate() > 4) {
        return std::nullopt;
    }

    auto middlePoseOpt = Tracking::HeadPose();
    if (!middlePoseOpt.has_value()) {
        return std::nullopt;
    }

    return Camera::GetReferenceMtx() * middlePoseOpt.value();
}

static bool TryNormalizeDirection(glm::fvec3 direction, glm::fvec3& outDirection) {
    if (!glm::all(glm::isfinite(direction)) || glm::dot(direction, direction) <= 1.0e-6f) {
        return false;
    }

    outDirection = glm::normalize(direction);
    return true;
}

static bool TryGetThrowDirection(glm::fvec3& outDirection) {
    auto headsetWorld = TryGetHeadsetWorldMtx();
    // Throw seems to want the inverse of the camera's forward direction
    return headsetWorld.has_value() && TryNormalizeDirection(glm::mat3(headsetWorld.value()) * glm::fvec3(0.0f, 0.0f, -1.0f), outDirection);
}

static bool TryGetGuardDirection(glm::fvec3& outDirection) {
    auto headsetWorld = TryGetHeadsetWorldMtx();
    if (!headsetWorld.has_value()) {
        return false;
    }

    auto [_, yawOnly] = RenderUtils::swingTwistY(glm::quat_cast(headsetWorld.value()));
    glm::fvec3 direction = yawOnly * glm::fvec3(0.0f, 0.0f, -1.0f);
    direction.y = 0.0f;
    return TryNormalizeDirection(direction, outDirection);
}

static void hook_OverrideThrowDirection(PPCInterpreter_t* hCPU) {
    if (GameState::IsThirdPerson()) {
        float dirX = 0.0f;
        Guest::ReadBE(hCPU->gpr[31] + 0x1570, &dirX);
        hCPU->fpr[10].fp0 = dirX; // original instruction: lfs f10, 0x1570(r31)
        hCPU->instructionPointer = 0x02C9172C;
        return;
    }

    hCPU->instructionPointer = 0x02C91744;

    glm::fvec3 throwDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    if (!TryGetThrowDirection(throwDir)) {
        glm::fvec3 playerForward = glm::fvec3(0.0f, 0.0f, 1.0f);
        if (hCPU->gpr[31] != 0) {
            BEMatrix34 mtxCopy = {};
            Guest::Read(hCPU->gpr[31] + 0x1568, &mtxCopy);
            playerForward = glm::fvec3(mtxCopy.z_x.getLE(), mtxCopy.z_y.getLE(), mtxCopy.z_z.getLE());
        }
        if (!TryNormalizeDirection(playerForward, throwDir)) {
            throwDir = glm::fvec3(0.0f, 0.0f, 1.0f);
        }
    }

    const float dirYSq = throwDir.y * throwDir.y;

    // emulate the replaced instructions so ThrowSomething still normalizes it
    hCPU->fpr[10].fp0 = throwDir.x;
    hCPU->fpr[12].fp0 = throwDir.z;
    hCPU->fpr[13].fp0 = dirYSq;
    hCPU->fpr[0].fp0 = throwDir.x * throwDir.x + dirYSq;

    const uint32_t stackBase = hCPU->gpr[1];
    Guest::WriteBE(stackBase + 0x14, &throwDir.x);
    Guest::WriteBE(stackBase + 0x18, &throwDir.y);
}

struct HandThrowRequest {
    glm::fvec3 velocity = {};
    uint64_t expiresAtNs = 0;
};
static std::atomic<HandThrowRequest> s_handThrowRequest;
static std::optional<glm::fvec3> s_handThrowVelocity;
static uint64_t s_skipAnimationThrowUntilNs = 0;

void Player::RequestHandThrow(const glm::fvec3& worldVelocity) {
    constexpr uint64_t requestLifetimeNs = 500'000'000;
    s_handThrowRequest = HandThrowRequest{ worldVelocity, GetTimeStamp() + requestLifetimeNs };
}

// PlayerGrabThrow::calc_ (0x02C91AB4) asking whether its animation reached the throw frame
static void hook_TimeGrabThrowRelease(PPCInterpreter_t* hCPU) {
    constexpr uint64_t skipAnimationThrowNs = 1'500'000'000;
    const bool animationThrows = hCPU->gpr[3] != 0; // original instruction: cmpwi r3, 0
    const uint64_t nowNs = GetTimeStamp();

    bool throwNow = animationThrows;
    if (const HandThrowRequest request = s_handThrowRequest.exchange({}); request.expiresAtNs > nowNs) {
        s_handThrowVelocity = request.velocity;
        s_skipAnimationThrowUntilNs = nowNs + skipAnimationThrowNs;
        throwNow = true;
    }
    else if (animationThrows && nowNs < s_skipAnimationThrowUntilNs) {
        s_skipAnimationThrowUntilNs = 0;
        throwNow = false;
    }
    hCPU->instructionPointer = throwNow ? 0x02C91B2C : 0x02C91B44;
}

// ThrowSomething (0x02C916E4) with the throw velocity built at sp+0x8, its Y still in f0
static void hook_OverrideThrowVelocity(PPCInterpreter_t* hCPU) {
    hCPU->gpr[3] = hCPU->gpr[31]; // original instruction: mr r3, r31
    hCPU->instructionPointer = 0x02C9182C;

    const uint32_t velocityPtr = hCPU->gpr[1] + 0x8;
    const std::optional<glm::fvec3> handVelocity = std::exchange(s_handThrowVelocity, std::nullopt);
    float gameVelocityX = 0.0f;
    float gameVelocityZ = 0.0f;
    Guest::ReadBE(velocityPtr, &gameVelocityX);
    Guest::ReadBE(velocityPtr + 0x8, &gameVelocityZ);
    const glm::fvec3 gameVelocity = glm::fvec3(gameVelocityX, (float)hCPU->fpr[0].fp0, gameVelocityZ);
    if (!handVelocity.has_value()) {
        Log::print<CONTROLS>("[Throw] game velocity {}", gameVelocity);
        return;
    }

    // keep hand throws within reach of the game's own throw distance
    constexpr float MaxSpeedOverGame = 1.2f;
    glm::fvec3 velocity = handVelocity.value();
    const float maxSpeed = glm::length(gameVelocity) * MaxSpeedOverGame;
    if (maxSpeed > 0.0f && glm::length(velocity) > maxSpeed)
        velocity *= maxSpeed / glm::length(velocity);

    Log::print<CONTROLS>("[Throw] hand velocity {} instead of {}", velocity, gameVelocity);
    Guest::WriteBE(velocityPtr, &velocity.x);
    Guest::WriteBE(velocityPtr + 0x8, &velocity.z);
    hCPU->fpr[0].fp0 = velocity.y;
}

static void hook_OverrideGuardDirection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x02C16ECC;

    const uint32_t stackBase = hCPU->gpr[1];
    uint32_t actorPtr = 0;
    Guest::ReadBE(stackBase + 0x24, &actorPtr);

    glm::fvec3 guardDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    const bool shouldUseVrDirection = actorPtr != 0 && actorPtr == s_playerAddress && TryGetGuardDirection(guardDir);
    if (!shouldUseVrDirection && actorPtr != 0) {
        BEMatrix34 actorMtx = {};
        Guest::Read(actorPtr + offsetof(ActorWiiU, mtx), &actorMtx);
        guardDir = glm::fvec3(actorMtx.z_x.getLE(), actorMtx.z_y.getLE(), actorMtx.z_z.getLE());
    }

    if (!glm::all(glm::isfinite(guardDir)) || glm::dot(guardDir, guardDir) <= 1.0e-6f) {
        guardDir = glm::fvec3(0.0f, 0.0f, 1.0f);
    }

    uint32_t dirXBits = 0;
    uint32_t dirYBits = 0;
    memcpy(&dirXBits, &guardDir.x, sizeof(dirXBits));
    memcpy(&dirYBits, &guardDir.y, sizeof(dirYBits));

    hCPU->gpr[12] = dirXBits;
    hCPU->gpr[0] = dirYBits;
    hCPU->fpr[12].fp0 = guardDir.x;
    hCPU->fpr[6].fp0 = guardDir.y;
    hCPU->fpr[8].fp0 = guardDir.z;

    Guest::WriteBE(stackBase + 0xE0, &guardDir.x);
    Guest::WriteBE(stackBase + 0xE4, &guardDir.y);
    Guest::WriteBE(stackBase + 0xE8, &guardDir.z);
}

void Player::RegisterHooks() {
    CemuHost::RegisterHook("hook_UpdateActorList", &hook_UpdateActorList);
    CemuHost::RegisterHook("hook_PlayerNormalChangeState", &hook_PlayerNormalChangeState);
    CemuHost::RegisterHook("hook_StoreLadderState", &hook_StoreLadderState);
    CemuHost::RegisterHook("hook_FixLadder", &hook_FixLadder);
    CemuHost::RegisterHook("hook_ShouldSnapToStickDirection", &hook_ShouldSnapToStickDirection);
    CemuHost::RegisterHook("hook_SetActorOpacity", &hook_SetActorOpacity);
    CemuHost::RegisterHook("hook_CalculateModelOpacity", &hook_CalculateModelOpacity);
    CemuHost::RegisterHook("hook_OverrideThrowDirection", &hook_OverrideThrowDirection);
    CemuHost::RegisterHook("hook_TimeGrabThrowRelease", &hook_TimeGrabThrowRelease);
    CemuHost::RegisterHook("hook_OverrideThrowVelocity", &hook_OverrideThrowVelocity);
    CemuHost::RegisterHook("hook_OverrideGuardDirection", &hook_OverrideGuardDirection);
}
