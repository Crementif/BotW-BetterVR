#include "pch.h"

#include "instance.h"
#include "cemu_hooks.h"
#include "weapon.h"


std::array<WeaponMotionAnalyser, 2> CemuHooks::m_motionAnalyzers = {};
std::array<uint32_t, 2> CemuHooks::m_heldWeapons = { 0, 0 };
std::array<uint32_t, 2> CemuHooks::m_heldWeaponsLastUpdate = { 0, 0 };
std::array<WeaponType, 2> CemuHooks::s_handWeaponTypes = { WeaponType::UnknownWeapon, WeaponType::UnknownWeapon };
bool CemuHooks::s_arrowNockedInRightHand = false;
bool CemuHooks::s_twoHandGripActive = false;
bool CemuHooks::s_twoHandGripConsumesGrabInput = false;

std::array s_cameraRotations = {
    glm::identity<glm::fquat>(),
    glm::identity<glm::fquat>()
};
std::array s_cameraPositions = {
    glm::fvec3(0.0f),
    glm::fvec3(0.0f)
};

// Must be driven once per game frame, since hook_ChangeWeaponMtx rebinds a held weapon at that same
// rate. Ticking it on a rendering hook instead evicts a live weapon within the frame that bound it.
void CemuHooks::UpdateHeldWeaponStaleness() {
    RND_Renderer* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr || !renderer->IsInitialized()) {
        return;
    }

    m_heldWeaponsLastUpdate[0]++;
    m_heldWeaponsLastUpdate[1]++;

    if (m_heldWeaponsLastUpdate[0] >= HeldWeaponStaleFrames) {
        m_heldWeapons[0] = 0;
        s_handWeaponTypes[0] = WeaponType::UnknownWeapon;
    }
    if (m_heldWeaponsLastUpdate[1] >= HeldWeaponStaleFrames) {
        m_heldWeapons[1] = 0;
        s_handWeaponTypes[1] = WeaponType::UnknownWeapon;
        s_arrowNockedInRightHand = false;
    }
}

static float ComputeAttackDamageMultiplier(float swingPower) {
    const float normalizedPower = glm::clamp((swingPower - 0.15f) / 0.85f, 0.0f, 1.0f);
    const float rampedPower = normalizedPower * normalizedPower * (3.0f - 2.0f * normalizedPower);

    // Keep incidental motion weak, then ramp into full damage once the swing is committed.
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

bool CemuHooks::IsTwoHandGripEngaged() {
    if (!s_twoHandGripEnabled)
        return false;

    if (!IsFirstPerson())
        return false;

    const WeaponType rightHandWeaponType = s_handWeaponTypes[OpenXR::EyeSide::RIGHT];
    const bool isTwoHandedWeapon = rightHandWeaponType == WeaponType::LargeSword || rightHandWeaponType == WeaponType::Spear;
    if (!isTwoHandedWeapon || m_heldWeapons[OpenXR::EyeSide::RIGHT] == 0)
        return false;

    const auto gameState = VRManager::instance().XR->m_gameState.load();
    if (gameState.has_something_in_left_hand)
        return false;

    const auto inputs = VRManager::instance().XR->m_input.load();
    for (uint32_t side = 0; side < 2; side++) {
        if (!inputs.shared.pose[side].isActive)
            return false;

        const XrSpaceLocationFlags locationFlags = inputs.shared.poseLocation[side].locationFlags;
        if (!(locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) || !(locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT))
            return false;
    }

    return true;
}

static bool isDroppable(std::string actorName) {
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

void CemuHooks::hook_ChangeWeaponMtx(PPCInterpreter_t* hCPU) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::WeaponHandHook);

    hCPU->instructionPointer = hCPU->sprNew.LR;

    // r3 holds the source actor pointer
    // r4 holds the bone name
    // r5 holds the matrix that is to be set
    // r6 holds the extra matrix that is to be set
    // r7 holds the ModelBindInfo->mtx
    // r8 holds the target actor pointer
    // r9 returns 1 if the weapon is modified
    // r10 holds the camera pointer
    // r11 returns 1 when the weapon should be dropped
    // r13 returns the weapon actor to drop, whose equip slot the patch resolves itself

    hCPU->gpr[9] = 0; // this is used to indicate whether the weapon was modified
    hCPU->gpr[11] = 0; // this is used to drop the weapon if the grip button is pressed

    uint32_t actorPtr = hCPU->gpr[3]; // holder of weapon
    uint32_t boneNamePtr = hCPU->gpr[4];
    uint32_t weaponMtxPtr = hCPU->gpr[5];
    uint32_t playerMtxPtr = hCPU->gpr[6];
    uint32_t modelBindInfoMtxPtr = hCPU->gpr[7];
    uint32_t targetActorPtr = hCPU->gpr[8]; // weapon that's being held
    uint32_t cameraPtr = hCPU->gpr[10];

    if (actorPtr != 0 && boneNamePtr != 0) {
        sead::FixedSafeString40 actorName = getMemory<sead::FixedSafeString40>(actorPtr + offsetof(ActorWiiU, name));
        char* boneName = (char*)s_memoryBaseAddress + boneNamePtr;
        if (actorName.getLE() == "GameROMPlayer" && (strcmp(boneName, "Weapon_L") == 0 || strcmp(boneName, "Weapon_R") == 0)) {
            const OpenXR::EyeSide side = strcmp(boneName, "Weapon_L") == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;
            m_heldWeapons[side] = targetActorPtr;
            m_heldWeaponsLastUpdate[side] = 0;

            Weapon targetActor = {};
            readMemory(targetActorPtr, &targetActor);

            if (strcmp(boneName, "Weapon_L") == 0) {
                const bool isBow = targetActor.type.getLE() == WeaponType::Bow;
                const bool isSlateRune = targetActor.name.getLE() == "Item_Conductor";
                RND_Renderer::Layer2D::SetBowAimingActive(isBow || isSlateRune);
            }
            else {
                const std::string targetActorName = targetActor.name.getLE();
                s_arrowNockedInRightHand = targetActorName.contains("Arrow");
                if (targetActorName == "Item_Magnetglove")
                    RND_Renderer::Layer2D::SetBowAimingActive(true);
            }
        }
    }

    if (IsThirdPerson()) {
        return;
    }

    sead::FixedSafeString40 actorName = getMemory<sead::FixedSafeString40>(actorPtr + offsetof(ActorWiiU, name));

    // todo: remove this?
    BESeadLookAtCamera camera = {};
    readMemory(cameraPtr, &camera);
    glm::fvec3 cameraPos = camera.pos.getLE();
    glm::fvec3 cameraAt = camera.at.getLE();
    glm::fquat lookAtQuat = glm::quatLookAtRH(glm::normalize(cameraAt - cameraPos), { 0.0, 1.0, 0.0 });
    glm::fvec3 lookAtPos = cameraPos;

    // read bone name
    if (boneNamePtr == 0)
        return;
    char* boneName = (char*)s_memoryBaseAddress + boneNamePtr;

    // real logic
    bool isHeldByPlayer = actorName.getLE() == "GameROMPlayer";
    bool isLeftHandWeapon = strcmp(boneName, "Weapon_L") == 0;
    bool isRightHandWeapon = strcmp(boneName, "Weapon_R") == 0;

    if (!actorName.getLE().empty() && boneName[0] != '\0' && isHeldByPlayer && (isLeftHandWeapon || isRightHandWeapon)) {
        OpenXR::EyeSide side = isLeftHandWeapon ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

        m_heldWeapons[side] = targetActorPtr;
        m_heldWeaponsLastUpdate[side] = 0;

        BEMatrix34 weaponMtx = {};
        readMemory(weaponMtxPtr, &weaponMtx);

        BEMatrix34 modelBindInfoMtx = {};
        readMemory(modelBindInfoMtxPtr, &modelBindInfoMtx);

        s_cameraPositions[side] = lookAtPos;
        s_cameraRotations[side] = lookAtQuat;

        Weapon targetActor = {};
        readMemory(targetActorPtr, &targetActor);

        s_handWeaponTypes[side] = targetActor.type.getLE();

        //Fetch data for inputs handling
        auto gameState = VRManager::instance().XR->m_gameState.load();
        gameState.is_throwable_object_held = ObjectCanBeThrown(targetActor.flags2.getLE());
        auto equipType = EquipType::None;
        switch (targetActor.type.getLE()) {
            case WeaponType::SmallSword:
            case WeaponType::LargeSword:
            case WeaponType::Spear:
                equipType = EquipType::Melee;
                break;
            case WeaponType::Bow:
                equipType = EquipType::Bow;
                break;
            case WeaponType::Shield:
                equipType = EquipType::Shield;
                break;
            default:
                equipType = EquipType::None;
                break;
        }

        if (isRightHandWeapon) {
            gameState.has_something_in_right_hand = true;
            
            if (targetActor.name.getLE() == "Item_Magnetglove")
                equipType = EquipType::MagnetGlove;

            if (gameState.left_hand_current_equip_type == EquipType::Bow)
                equipType = EquipType::Arrow;

            gameState.right_hand_current_equip_type = equipType;
        }
        else {
            gameState.has_something_in_left_hand = true;
            if (targetActor.name.getLE() == "Item_Conductor")
                equipType = EquipType::SheikahSlate;

            gameState.left_hand_current_equip_type = equipType;
        }
        VRManager::instance().XR->m_gameState.store(gameState);

        // check if weapon is held and if a drop should be triggered
        auto input = VRManager::instance().XR->m_input.load();
        auto dropSide = input.inGame.drop_weapon[side];

        if (input.shared.in_game && dropSide && isDroppable(targetActor.name.getLE())) {
            Log::print<INFO>("Dropping weapon {} with type of {} from the {} hand due to long press on right waist body slot", targetActor.name.getLE().c_str(), (uint32_t)targetActor.type.getLE(), isLeftHandWeapon ? "left" : "right");
            hCPU->gpr[11] = 1;
            hCPU->gpr[9] = 1;
            hCPU->gpr[13] = targetActorPtr; // the patch matches this against every equip slot to find the one to drop
            return;
        }

        hCPU->gpr[9] = 1;
    }
}

// todo: this function does nothing, we use ChangeWeaponMtx instead
void CemuHooks::hook_DropEquipment(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson() || HasActiveCutscene()) {
        return;
    }

    uint32_t weaponPtr = hCPU->gpr[3];
    uint32_t parentActorPtr = hCPU->gpr[4];
    uint32_t heldIndex = hCPU->gpr[5]; // this is either 0 or 1 depending on which hand the weapon is in
    bool isHeldByPlayer = hCPU->gpr[6] == 0;

    Weapon weapon = {};
    readMemory(weaponPtr, &weapon);
}


void CemuHooks::hook_GetContactLayerOfAttack(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON) {
        return;
    }

    uint32_t player = hCPU->gpr[25];

    ActorWiiU actor = {};
    readMemory(player, &actor);

    uint32_t originalContactLayerPtr = hCPU->gpr[5];
    uint32_t originalContactLayer = getMemory<uint32_t>(originalContactLayerPtr).getLE();
    const char* originalContactLayerStr = (const char*)(s_memoryBaseAddress + originalContactLayer);

    uint32_t contactLayerValue = hCPU->gpr[3];

    hCPU->gpr[3] = contactLayerValue;
}


void CemuHooks::hook_EnableWeaponAttackSensor(PPCInterpreter_t* hCPU) {
    BetterVRProfiler::Scope profile(BetterVRProfiler::Section::WeaponAttackHook);

    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON)
        return;

    uint32_t weaponPtr = hCPU->gpr[3];
    uint32_t heldIndex = hCPU->gpr[5]; // this is either 0 or 1 depending on which hand the weapon is in
    bool isHeldByPlayer = hCPU->gpr[6] == 0;
    uint32_t frameCounter = hCPU->gpr[7];

    Weapon weapon = {};
    readMemory(weaponPtr, &weapon);

    WeaponType weaponType = weapon.type.getLE();
    if (!IsSwingableWeaponType(weaponType)) {
        return;
    }

    if (heldIndex >= m_motionAnalyzers.size()) {
        Log::print<CONTROLS>("Invalid heldIndex: {}. Skipping motion analysis.", heldIndex);
        return;
    }

    // Game's hand indexing is inverted relative to OpenXR's convention
    uint32_t motionIndex = heldIndex == 0 ? 1 : 0;

    // Weapons not currently bound to a VR hand (e.g. sheathed on back) must never
    // consume that hand's motion analysis slot, otherwise the real held weapon can
    // lose its swing detection for the frame.
    bool isHandBoundWeapon = weaponPtr == m_heldWeapons[motionIndex];
    auto& motionAnalyser = m_motionAnalyzers[motionIndex];
    const bool canUseWeaponMotion = isHandBoundWeapon && isHeldByPlayer;

    if (!canUseWeaponMotion) {
        if (m_heldWeapons[motionIndex] == 0 || isHandBoundWeapon) {
            motionAnalyser.Reset();
        }

        motionAnalyser.SetHitboxEnabled(false);
        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 1; // deactivate attack sensor
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        writeMemory(weaponPtr, &weapon);
        return;
    }

    auto* motionRenderer = VRManager::instance().XR->GetRenderer();
    if (motionRenderer == nullptr) {
        // the renderer is absent while a session is being rebuilt
        return;
    }

    auto& currFrame = motionRenderer->GetFrame(frameCounter);
    if (currFrame.ranMotionAnalysis[heldIndex]) {
        Log::print<CONTROLS>("Skipping motion analysis for {}: already ran this frame", heldIndex);
        return;
    }
    currFrame.ranMotionAnalysis[heldIndex] = true;

    auto inputs = VRManager::instance().XR->m_input.load();
    auto headset = VRManager::instance().XR->GetMiddlePose();
    if (!headset.has_value()) {
        return;
    }

    motionAnalyser.ResetIfWeaponTypeChanged(weaponType);
    motionAnalyser.ApplyProfile(GetSettings().GetSwingSensitivity());
    motionAnalyser.Update(inputs.shared.poseLocation[motionIndex], inputs.shared.poseVelocity[motionIndex], headset.value(), inputs.shared.inputTime);

    const bool isTwoHandGrip = motionIndex == OpenXR::EyeSide::RIGHT && s_twoHandGripActive;

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

        // Weak motions stay light; proper swings ramp up closer to full damage.
        const float power = motionAnalyser.GetSwingPower();
        const bool isDualGrippableWeapon = weaponType == WeaponType::LargeSword || weaponType == WeaponType::Spear;
        const float gripDamageMultiplier = isTwoHandGrip ? TWO_HAND_GRIP_DAMAGE_MULTIPLIER : (isDualGrippableWeapon ? SINGLE_HAND_DAMAGE_MULTIPLIER : 1.0f);
        // A multi-hit swing lands on several frames, so each hit is worth less.
        const float repeatDamageMultiplier = oneHitPerSwing ? 1.0f : MULTI_HIT_DAMAGE_MULTIPLIER;
        const float damageMultiplier = ComputeAttackDamageMultiplier(power) * gripDamageMultiplier * repeatDamageMultiplier;

        // Only the arming frame copies setupAttackSensor into the finalized block.
        weapon.setupAttackSensor.multiplier = damageMultiplier;
        weapon.finalizedAttackSensor.multiplier = damageMultiplier;

        writeMemory(weaponPtr, &weapon);
    }
    else if (!canUseWeaponMotion || motionAnalyser.IsHitboxEnabled()) {
        motionAnalyser.SetHitboxEnabled(false);
        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 1; // deactivate attack sensor
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        writeMemory(weaponPtr, &weapon);
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

            VRManager::instance().XR->GetRumbleManager()->enqueueInputsRumbleCommand(rumbleParams);
        }
    }
}

void CemuHooks::hook_SetPlayerWeaponScale(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson()) {
        return;
    }
}

void CemuHooks::hook_EquipWeapon(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto input = VRManager::instance().XR->m_input.load();
    // Check both hands for a short press to pick up weapon
    for (int side = 0; side < 2; ++side) {
        auto& grabState = input.inGame.grabState[side];
    }
    // Default behavior if no short press
}


void CemuHooks::hook_DropWeaponLogging(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t actorPtr = hCPU->gpr[3];

    PlayerOrEnemy player;
    readMemory(actorPtr, &player);

    uint32_t actorLinkPtr = actorPtr + offsetof(ActorWiiU, name) + offsetof(sead::FixedSafeString40, c_str);
    uint32_t actorNamePtr = 0;
    readMemoryBE(actorLinkPtr, &actorNamePtr);
    if (actorNamePtr == 0)
        return;
    char* actorName = (char*)s_memoryBaseAddress + actorNamePtr;

    uint32_t weaponIdx = hCPU->gpr[4];
    BEVec3 position;
    readMemory(hCPU->gpr[5], &position);
    uint32_t a4 = hCPU->gpr[6];
    uint32_t a5 = hCPU->gpr[7];
    uint32_t a6 = hCPU->gpr[8];
    uint32_t a7 = hCPU->gpr[9];

    Log::print<CONTROLS>("{} ({:08X}) is dropping weapon with idx={}, position={}, a4={}, a5={}, a6={}, a7={}", actorName, actorPtr, weaponIdx, position, a4, a5, a6, a7);
}

// r3 = ActorWeapons*, r4 = the Weapon* to drop. Returns the equip slot in r3, or -1 if not equipped.
//
// ActorWeapons::getEquippedWeapon would answer this, but it only returns a proc when
// BaseProcMgr::isHighPriorityThread() is true, and the drop is requested from ModelBindInfo::Calc,
// which is not one of the actor job threads, so every slot comes back empty there.
void CemuHooks::hook_FindWeaponEquipSlot(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t actorWeaponsPtr = hCPU->gpr[3];
    const uint32_t weaponPtr = hCPU->gpr[4];

    hCPU->gpr[3] = NO_EQUIP_SLOT;
    if (actorWeaponsPtr == 0 || weaponPtr == 0)
        return;

    // each link is { procSlot*, id, engagedByte, _ }, resolving to the actor in procSlot+0x40
    for (uint32_t slot = 0; slot < ACTOR_WEAPONS_SLOT_COUNT; slot++) {
        const uint32_t linkPtr = actorWeaponsPtr + slot * BASE_PROC_LINK_SIZE;
        const uint32_t procSlotPtr = getMemory<uint32_t>(linkPtr).getLE();
        const uint32_t linkId = getMemory<uint32_t>(linkPtr + 4).getLE();
        if (procSlotPtr == 0 || linkId == 0xFFFFFFFF)
            continue;
        if (getMemory<uint32_t>(procSlotPtr + 0x3C).getLE() != linkId)
            continue; // stale link, that proc slot has been reused since
        if (getMemory<uint32_t>(procSlotPtr + 0x40).getLE() != weaponPtr)
            continue;

        hCPU->gpr[3] = slot;
        break;
    }

    if (hCPU->gpr[3] == NO_EQUIP_SLOT)
        Log::print<WARNING>("Not dropping weapon {:08X}: it isn't in any equip slot of ActorWeapons {:08X}", weaponPtr, actorWeaponsPtr);
}

void CemuHooks::hook_ModifyHandModelAccessSearch(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

#ifdef _DEBUG
    if (hCPU->gpr[3] == 0) {
        return;
    }
    // r3 holds the address of the string to search for
    const char* actorName = (const char*)(s_memoryBaseAddress + hCPU->gpr[3]);

    if (actorName != nullptr) {
        // Weapon_R is presumably his right hand bone name
        Log::print<CONTROLS>("Searching for model handle using {}", actorName);
    }
#endif
}

