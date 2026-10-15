#include "pch.h"
#include "weapon.h"
#include "skeleton.h"
#include "player.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "rendering/haptics.h"
#include "rendering/tracking.h"

// in game frames, hook_ChangeWeaponMtx rebinds held weapons every frame
static constexpr uint32_t HeldWeaponStaleFrames = 6;

static std::array<WeaponMotionAnalyser, 2> s_motionAnalysers = {};
static std::array<std::atomic_uint32_t, 2> s_heldActors = { 0, 0 };
static std::array<uint32_t, 2> s_framesSinceHeldActorBound = { 0, 0 };
static std::array<std::atomic<WeaponType>, 2> s_heldTypes = { WeaponType::UnknownWeapon, WeaponType::UnknownWeapon };
static std::array<std::atomic_bool, 2> s_heldItemAims = { false, false };
static std::array<XrTime, 2> s_lastAnalysedInputTime = { 0, 0 };
static bool s_arrowNockedInRightHand = false;

static std::array<std::atomic<EquipType>, 2> s_observedEquipTypes = { EquipType::None, EquipType::None };
static std::array<std::atomic_bool, 2> s_observedOccupied = { false, false };
static std::atomic_bool s_observedThrowable = false;

uint32_t Weapons::GetHeldActor(uint32_t side) {
    return s_heldActors[side];
}

WeaponType Weapons::GetHeldType(uint32_t side) {
    return s_heldTypes[side];
}

bool Weapons::IsArrowNockedInRightHand() {
    return s_arrowNockedInRightHand;
}

bool Weapons::IsAimingItemHeld() {
    return s_heldItemAims[0] || s_heldItemAims[1];
}

WeaponMotionAnalyser& Weapons::GetMotionAnalyser(uint32_t side) {
    return s_motionAnalysers[side];
}

Weapons::HandObservations Weapons::ReadHandObservations() {
    return { { s_observedEquipTypes[0], s_observedEquipTypes[1] }, { s_observedOccupied[0], s_observedOccupied[1] }, s_observedThrowable };
}

void Weapons::ResetHandObservations() {
    for (uint32_t side = 0; side < 2; side++) {
        s_observedEquipTypes[side] = EquipType::None;
        s_observedOccupied[side] = false;
    }
    s_observedThrowable = false;
}

bool Weapons::IsThrowableHeld() {
    return s_observedThrowable;
}

struct PendingWeaponDrop {
    uint32_t actor = 0;
    uint32_t actorId = 0;
    int64_t expiresAt = 0;
};
static std::array<std::atomic<PendingWeaponDrop>, 2> s_pendingDrops;

static int64_t DropRequestTime() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Weapons::RequestDrop(uint32_t side) {
    if (side >= s_heldActors.size())
        return;
    const uint32_t actor = s_heldActors[side];
    if (actor != 0)
        s_pendingDrops[side].store({ actor, Guest::Get<uint32_t>(actor + offsetof(BaseProc, id)).getLE(), DropRequestTime() + 400'000'000 });
}

void Weapons::CancelDrops() {
    for (auto& request : s_pendingDrops)
        request.store({});
}


// has to run once per game frame like hook_ChangeWeaponMtx, or held weapons get dropped
void Weapons::OnGameFrame() {
    if (!Tracking::IsSessionRunning()) {
        return;
    }

    for (uint32_t side = 0; side < 2; side++) {
        if (++s_framesSinceHeldActorBound[side] < HeldWeaponStaleFrames)
            continue;
        s_heldActors[side] = 0;
        s_heldTypes[side] = WeaponType::UnknownWeapon;
        s_heldItemAims[side] = false;
        if (side == OpenXR::EyeSide::RIGHT)
            s_arrowNockedInRightHand = false;
    }
}

static float ComputeAttackDamageMultiplier(float swingPower) {
    const float normalizedPower = glm::clamp((swingPower - 0.15f) / 0.85f, 0.0f, 1.0f);
    const float rampedPower = normalizedPower * normalizedPower * (3.0f - 2.0f * normalizedPower);

    return 0.20f + 1.15f * rampedPower;
}

static bool IsSwingableWeaponType(WeaponType weaponType) {
    return weaponType == WeaponType::SmallSword
        || weaponType == WeaponType::LargeSword
        || weaponType == WeaponType::Spear;
}

static constexpr float SINGLE_HAND_DAMAGE_MULTIPLIER = 0.85f;
static constexpr float TWO_HAND_GRIP_DAMAGE_MULTIPLIER = 1.20f;
static constexpr float MULTI_HIT_DAMAGE_MULTIPLIER = 0.50f;

bool Weapons::CanEngageTwoHandGrip() {
    if (!Weapons::TwoHandGripEnabled)
        return false;

    if (!GameState::IsFirstPerson())
        return false;

    const WeaponType rightHandWeaponType = s_heldTypes[OpenXR::EyeSide::RIGHT];
    const bool isTwoHandedWeapon = rightHandWeaponType == WeaponType::LargeSword || rightHandWeaponType == WeaponType::Spear;
    if (!isTwoHandedWeapon || s_heldActors[OpenXR::EyeSide::RIGHT] == 0)
        return false;

    if (s_observedOccupied[OpenXR::EyeSide::LEFT])
        return false;

    const auto inputs = Tracking::Hands();
    for (uint32_t side = 0; side < 2; side++) {
        if (!inputs.shared.pose[side].isActive)
            return false;

        const XrSpaceLocationFlags locationFlags = inputs.shared.poseLocation[side].locationFlags;
        if (!(locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) || !(locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))
            return false;
    }

    return true;
}

static bool isDroppable(std::string_view actorName) {
    static const std::string_view nonDroppableItems[] = {
        "AncientArrow",
        "Animal_Insect_A",
        "Animal_Insect_B",
        "Animal_Insect_F",
        "Animal_Insect_H",
        "Animal_Insect_M",
        "Animal_Insect_S",
        "Animal_Insect_X",
        "Armor_Default_Extra_00",
        "Armor_Default_Extra_01",
        "bj_SupportApp_Wind",
        "BombArrow_A",
        "BrightArrow",
        "BrightArrowTP",
        "CarryBox",
        "Dm_Npc_Gerudo_HeroSoul_Kago",
        "Dm_Npc_Goron_HeroSoul_Kago",
        "Dm_Npc_RevivalFairy",
        "Dm_Npc_Rito_HeroSoul_Kago",
        "Dm_Npc_Zora_HeroSoul_Kago",
        "ElectricArrow",
        "Explode",
        "FireArrow",
        "FireRodLv1Fire",
        "FireRodLv2Fire",
        "FireRodLv2FireChild",
        "GameRomHorseReins_01",
        "GameRomHorseReins_02",
        "GameRomHorseReins_03",
        "GameRomHorseReins_04",
        "GameRomHorseReins_05",
        "GameRomHorseReins_10",
        "GameRomHorseSaddle_01",
        "GameRomHorseSaddle_02",
        "GameRomHorseSaddle_03",
        "GameRomHorseSaddle_04",
        "GameRomHorseSaddle_05",
        "GameRomHorseSaddle_10",
        "GameROMPlayer",
        "Get_TwnObj_DLC_MemorialPicture_A_01",
        "IceArrow",
        "IceRodLv1Ice",
        "IceRodLv2Ice",
        "Item_Conductor",
        "Item_CookSet",
        "Item_Magnetglove",
        "Item_Material_01",
        "Item_Material_03",
        "Item_Material_07",
        "Item_Ore_F",
        "KeySmall",
        "NormalArrow",
        "Obj_Armor_115_Head",
        "Obj_DLC_HeroSeal_Gerudo",
        "Obj_DLC_HeroSeal_Goron",
        "Obj_DLC_HeroSeal_Rito",
        "Obj_DLC_HeroSeal_Zora",
        "Obj_DLC_HeroSoul_Gerudo",
        "Obj_DLC_HeroSoul_Goron",
        "Obj_DLC_HeroSoul_Rito",
        "Obj_DLC_HeroSoul_Zora",
        "Obj_DRStone_A_01",
        "Obj_DRStone_Get",
        "Obj_DungeonClearSeal",
        "Obj_HeartUtuwa_A_01",
        "Obj_HeroSoul_Gerudo",
        "Obj_HeroSoul_Goron",
        "Obj_HeroSoul_Rito",
        "Obj_HeroSoul_Zora",
        "Obj_IceMakerBlock",
        "Obj_KorokNuts",
        "Obj_Maracas",
        "Obj_ProofBook",
        "Obj_ProofGiantKiller",
        "Obj_ProofGolemKiller",
        "Obj_ProofKorok",
        "Obj_ProofSandwormKiller",
        "Obj_StaminaUtuwa_A_01",
        "Obj_WarpDLC",
        "PlayerStole2",
        "PlayerStole2_Vagrant",
        "Weapon_Bow_071",
        "Weapon_Sword_056",
        "Weapon_Sword_070",
        "Weapon_Sword_080",
        "Weapon_Sword_081",
        "Weapon_Sword_502"
    };

    for (const auto& item : nonDroppableItems) {
        if (actorName == item) {
            return false;
        }
    }

    // prevent dropping arrows
    if (actorName.contains("Arrow")) {
        return false;
    }

    return true;
}

constexpr uint32_t ACTOR_WEAPONS_SLOT_COUNT = 6;
constexpr uint32_t BASE_PROC_LINK_SIZE = 0x10;
constexpr uint32_t NO_EQUIP_SLOT = 0xFFFFFFFF;

constexpr uint32_t FLAG_THROWABLE = 0x00004000;
bool ObjectCanBeThrown(uint32_t flags)
{
    return (flags & FLAG_THROWABLE);
}

static void hook_ChangeWeaponMtx(PPCInterpreter_t* hCPU) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::WeaponHandHook);
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // r3 = holder, r4 = bone name, r5 = the bone's world matrix it binds from, r8 = attached actor; r11/r13 return a drop request.
    hCPU->gpr[11] = 0;
    hCPU->gpr[13] = 0;
    const uint32_t holder = hCPU->gpr[3];
    const uint32_t boneNamePtr = hCPU->gpr[4];
    const uint32_t actor = hCPU->gpr[8];
    if (holder == 0 || holder != Player::GetAddress() || boneNamePtr == 0 || actor == 0)
        return;

    const auto holderName = Guest::Get<sead::FixedSafeString40>(holder + offsetof(ActorWiiU, name));
    if (std::string_view(holderName.data, strnlen(holderName.data, sizeof(holderName.data))) != "GameROMPlayer")
        return;
    const char* boneName = Guest::CString(boneNamePtr);
    const bool left = strcmp(boneName, "Weapon_L") == 0;
    if (!left && strcmp(boneName, "Weapon_R") != 0)
        return;
    const uint32_t side = left ? 0 : 1;

    const auto name = Guest::Get<sead::FixedSafeString40>(actor + offsetof(ActorWiiU, name));
    const std::string_view actorName(name.data, strnlen(name.data, sizeof(name.data)));
    const WeaponType type = Guest::Get<WeaponType>(actor + offsetof(Weapon, type)).getLE();
    s_heldActors[side] = actor;
    s_framesSinceHeldActorBound[side] = 0;
    s_heldTypes[side] = type;

    s_heldItemAims[side] = left ? (type == WeaponType::Bow || actorName == "Item_Conductor") : actorName == "Item_Magnetglove";
    if (!left)
        s_arrowNockedInRightHand = actorName.contains("Arrow");
    if (GameState::IsThirdPerson()) {
        s_pendingDrops[side].store({});
        return;
    }

    EquipType equipType = EquipType::None;
    switch (type) {
    case WeaponType::SmallSword:
    case WeaponType::LargeSword:
    case WeaponType::Spear: equipType = EquipType::Melee; break;
    case WeaponType::Bow: equipType = EquipType::Bow; break;
    case WeaponType::Shield: equipType = EquipType::Shield; break;
    }
    if (left) {
        if (actorName == "Item_Conductor")
            equipType = EquipType::SheikahSlate;
    }
    else {
        if (actorName == "Item_Magnetglove")
            equipType = EquipType::MagnetGlove;
        else if (s_arrowNockedInRightHand)
            equipType = EquipType::Arrow;
        s_observedThrowable = ObjectCanBeThrown(Guest::Get<uint32_t>(actor + offsetof(Weapon, flags2)).getLE());
    }
    s_observedEquipTypes[side] = equipType;
    s_observedOccupied[side] = true;

    const PendingWeaponDrop drop = s_pendingDrops[side].exchange({});
    if (drop.actor == actor && drop.actorId == Guest::Get<uint32_t>(actor + offsetof(BaseProc, id)).getLE()
        && DropRequestTime() <= drop.expiresAt && isDroppable(actorName)) {
        hCPU->gpr[11] = 1;
        hCPU->gpr[13] = actor;
    }
}

static void hook_EnableWeaponAttackSensor(PPCInterpreter_t* hCPU) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::WeaponAttackHook);

    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON)
        return;

    uint32_t weaponPtr = hCPU->gpr[3];
    uint32_t heldIndex = hCPU->gpr[5]; // this is either 0 or 1 depending on which hand the weapon is in
    bool isHeldByPlayer = hCPU->gpr[6] == 0;

    Weapon weapon = {};
    Guest::Read(weaponPtr, &weapon);

    WeaponType weaponType = weapon.type.getLE();
    if (!IsSwingableWeaponType(weaponType)) {
        return;
    }

    if (heldIndex >= s_motionAnalysers.size()) {
        Log::print<CONTROLS>("Invalid heldIndex: {}. Skipping motion analysis.", heldIndex);
        return;
    }

    // Game's hand indexing is inverted relative to OpenXR's convention
    uint32_t motionIndex = heldIndex == 0 ? 1 : 0;

    // Weapons not currently bound to a VR hand (e.g. sheathed on back) must never
    // consume that hand's motion analysis slot, otherwise the real held weapon can
    // lose its swing detection for the frame.
    bool isHandBoundWeapon = weaponPtr == s_heldActors[motionIndex];
    auto& motionAnalyser = s_motionAnalysers[motionIndex];
    const bool canUseWeaponMotion = isHandBoundWeapon && isHeldByPlayer;

    if (!canUseWeaponMotion) {
        if (s_heldActors[motionIndex] == 0 || isHandBoundWeapon) {
            motionAnalyser.Reset();
        }

        motionAnalyser.SetHitboxEnabled(false);
        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 1; // deactivate attack sensor
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        Guest::Write(weaponPtr, &weapon);
        return;
    }

    auto inputs = Tracking::Hands();
    if (inputs.shared.inputTime == s_lastAnalysedInputTime[motionIndex]) {
        return;
    }

    auto headset = Tracking::HeadPose();
    if (!headset.has_value()) {
        return;
    }
    s_lastAnalysedInputTime[motionIndex] = inputs.shared.inputTime;

    motionAnalyser.ResetIfWeaponTypeChanged(weaponType);
    motionAnalyser.ApplyProfile(GetSettings().GetSwingSensitivity());
    motionAnalyser.Update(inputs.shared.poseLocation[motionIndex], inputs.shared.poseVelocity[motionIndex], headset.value(), inputs.shared.inputTime);

    const bool isTwoHandGrip = motionIndex == OpenXR::EyeSide::RIGHT && Skeleton::IsTwoHandGripActive();

    // Use the analysed motion to determine whether the weapon is swinging or stabbing
    if (canUseWeaponMotion && motionAnalyser.IsAttacking()) {
        // Re-arming bumps the attack id in AttackSensor+0x3C, which wipes the game's per-attack hit list.
        const bool oneHitPerSwing = GetSettings().UseOneHitPerSwing();
        const bool armSensor = !oneHitPerSwing || !motionAnalyser.IsHitboxEnabled();

        motionAnalyser.SetHitboxEnabled(true);
        weapon.setupAttackSensor.mode = 2;
        weapon.setupAttackSensor.shieldBreakPower = 2; // damageType 2 required for chopping trees
        if (armSensor) {
            weapon.setupAttackSensor.resetAttack = 1;
            weapon.setupAttackSensor.isContactLayerInitialized = 0;
        }

        const float power = motionAnalyser.GetSwingPower();
        const bool isDualGrippableWeapon = weaponType == WeaponType::LargeSword || weaponType == WeaponType::Spear;
        const float gripDamageMultiplier = isTwoHandGrip ? TWO_HAND_GRIP_DAMAGE_MULTIPLIER : (isDualGrippableWeapon ? SINGLE_HAND_DAMAGE_MULTIPLIER : 1.0f);
        // A multi-hit swing lands on several frames, so each hit is worth less.
        const float repeatDamageMultiplier = oneHitPerSwing ? 1.0f : MULTI_HIT_DAMAGE_MULTIPLIER;
        const float damageMultiplier = ComputeAttackDamageMultiplier(power) * gripDamageMultiplier * repeatDamageMultiplier;

        // Only the arming frame copies setupAttackSensor into the finalized block.
        weapon.setupAttackSensor.multiplier = damageMultiplier;
        weapon.finalizedAttackSensor.multiplier = damageMultiplier;

        Guest::Write(weaponPtr, &weapon);
    }
    else if (!canUseWeaponMotion || motionAnalyser.IsHitboxEnabled()) {
        motionAnalyser.SetHitboxEnabled(false);
        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 1; // deactivate attack sensor
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        Guest::Write(weaponPtr, &weapon);
    }

    // Haptic feedback
    if (canUseWeaponMotion && motionAnalyser.IsAttacking()) {
        // A thrust rarely beats the hand-speed threshold, so swing power provides a floor.
        const float speedRumble = std::max(0.0f, motionAnalyser.GetHandSpeed() - WeaponMotionAnalyser::HAND_VELOCITY_LENGTH_THRESHOLD);
        const float powerRumble = motionAnalyser.GetSwingPower() * 1.4f;
        float rumbleVelocity = std::max(speedRumble, powerRumble);

        const int rumbleHands[2] = { 1, 0 };
        const int rumbleHandCount = isTwoHandGrip ? 2 : 1;
        for (int i = 0; i < rumbleHandCount; i++) {
            const RumbleParameters rumbleParams = {
                isTwoHandGrip,
                rumbleHands[i],
                RumbleType::Fixed,
                0.0f,
                false,
                0.001,
                0.5f * rumbleVelocity,
                0.7f * rumbleVelocity
            };

            Haptics::Play(rumbleParams);
        }
    }
}

// r3 = ActorWeapons*, r4 = the Weapon* to drop. Returns the equip slot in r3, or -1 if not equipped.
// ActorWeapons::getEquippedWeapon only works on the actor job threads, and ModelBindInfo::Calc doesn't run on those
static void hook_FindWeaponEquipSlot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t actorWeaponsPtr = hCPU->gpr[3];
    const uint32_t weaponPtr = hCPU->gpr[4];

    hCPU->gpr[3] = NO_EQUIP_SLOT;
    if (actorWeaponsPtr == 0 || weaponPtr == 0)
        return;

    // each link is { procSlot*, id, engagedByte, _ }, resolving to the actor in procSlot+0x40
    for (uint32_t slot = 0; slot < ACTOR_WEAPONS_SLOT_COUNT; slot++) {
        const uint32_t linkPtr = actorWeaponsPtr + slot * BASE_PROC_LINK_SIZE;
        const uint32_t procSlotPtr = Guest::Get<uint32_t>(linkPtr).getLE();
        const uint32_t linkId = Guest::Get<uint32_t>(linkPtr + 4).getLE();
        if (procSlotPtr == 0 || linkId == 0xFFFFFFFF)
            continue;
        if (Guest::Get<uint32_t>(procSlotPtr + 0x3C).getLE() != linkId)
            continue; // stale link, that proc slot has been reused since
        if (Guest::Get<uint32_t>(procSlotPtr + 0x40).getLE() != weaponPtr)
            continue;

        hCPU->gpr[3] = slot;
        break;
    }

    if (hCPU->gpr[3] == NO_EQUIP_SLOT)
        Log::print<WARNING>("Not dropping weapon {:08X}: it isn't in any equip slot of ActorWeapons {:08X}", weaponPtr, actorWeaponsPtr);
}

void Weapons::RegisterHooks() {
    CemuHost::RegisterHook("hook_ChangeWeaponMtx", &hook_ChangeWeaponMtx);
    CemuHost::RegisterHook("hook_EnableWeaponAttackSensor", &hook_EnableWeaponAttackSensor);
    CemuHost::RegisterHook("hook_FindWeaponEquipSlot", &hook_FindWeaponEquipSlot);
}
