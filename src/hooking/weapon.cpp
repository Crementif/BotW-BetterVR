#include "instance.h"
#include "cemu_hooks.h"
#include "weapon.h"


std::array<WeaponMotionAnalyser, 2> CemuHooks::m_motionAnalyzers = {};
std::array<uint32_t, 2> CemuHooks::m_heldWeapons = { 0, 0 };
std::array<uint32_t, 2> CemuHooks::m_heldWeaponsLastUpdate = { 0, 0 };

std::array s_cameraRotations = {
    glm::identity<glm::fquat>(),
    glm::identity<glm::fquat>()
};
std::array s_cameraPositions = {
    glm::fvec3(0.0f),
    glm::fvec3(0.0f)
};

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

void CemuHooks::hook_ChangeWeaponMtx(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // r3 holds the source actor pointer
    // r4 holds the bone name
    // r5 holds the matrix that is to be set
    // r6 holds the extra matrix that is to be set
    // r7 holds the ModelBindInfo->mtx
    // r8 holds the target actor pointer
    // r9 returns 1 if the weapon is modified
    // r10 holds the camera pointer

    hCPU->gpr[9] = 0; // this is used to indicate whether the weapon was modified
    hCPU->gpr[11] = 0; // this is used to drop the weapon if the grip button is pressed

    if (IsThirdPerson()) {
        return;
    }

    uint32_t actorPtr = hCPU->gpr[3]; // holder of weapon
    uint32_t boneNamePtr = hCPU->gpr[4];
    uint32_t weaponMtxPtr = hCPU->gpr[5];
    uint32_t playerMtxPtr = hCPU->gpr[6];
    uint32_t modelBindInfoMtxPtr = hCPU->gpr[7];
    uint32_t targetActorPtr = hCPU->gpr[8]; // weapon that's being held
    uint32_t cameraPtr = hCPU->gpr[10];

    sead::FixedSafeString40 actorName = getMemory<sead::FixedSafeString40>(actorPtr + offsetof(ActorWiiU, name));

    // todo: remove this?
    BESeadLookAtCamera camera = {};
    readMemory(cameraPtr, &camera);
    glm::fvec3 cameraPos = camera.pos.getLE();
    glm::fvec3 cameraAt = camera.at.getLE();
    glm::fquat lookAtQuat = glm::quatLookAtRH(glm::normalize(cameraAt - cameraPos), { 0.0, 1.0, 0.0 });
    glm::fvec3 lookAtPos = cameraPos;
    //lookAtPos.y += GetSettings().playerHeightSetting.getLE();

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

        //ModifyWeaponMtxToVRPose(side, weaponMtx, lookAtQuat, lookAtPos);

        s_cameraPositions[side] = lookAtPos;
        s_cameraRotations[side] = lookAtQuat;


        //// prevent weapon transparency
        //BEType<float> modelOpacity = 1.0f;
        //BEType<float> negativeOpacity = 0.0f;
        //writeMemory(targetActorPtr + offsetof(ActorWiiU, modelOpacity), &modelOpacity);
        //writeMemory(targetActorPtr + offsetof(ActorWiiU, startModelOpacity), &modelOpacity);
        //writeMemory(targetActorPtr + offsetof(ActorWiiU, modelOpacityRelated), &negativeOpacity);
        //uint8_t opacityOrDoFlushOpacityToGPU = 1;
        //writeMemory(targetActorPtr + offsetof(ActorWiiU, opacityOrDoFlushOpacityToGPU), &opacityOrDoFlushOpacityToGPU);

        //writeMemory(weaponMtxPtr, &weaponMtx);
        //writeMemory(modelBindInfoMtxPtr, &modelBindInfoMtx);

        Weapon targetActor = {};
        readMemory(targetActorPtr, &targetActor);

        //Update for inputs handling
        auto gameState = VRManager::instance().XR->m_gameState.load();
        gameState.is_weapon_or_object_held = true;
        if (isRightHandWeapon)
            gameState.right_weapon_type = targetActor.type.getLE();
        else
            gameState.left_weapon_type = targetActor.type.getLE();
        VRManager::instance().XR->m_gameState.store(gameState);

        // check if weapon is held and if the grip button is held, drop it
        auto input = VRManager::instance().XR->m_input.load();
        auto dropSide = input.inGame.drop_weapon[side];

        if (input.inGame.in_game && dropSide && isDroppable(targetActor.name.getLE())) {
            Log::print<INFO>("Dropping weapon {} with type of {} due to double press on grab button", targetActor.name.getLE().c_str(), (uint32_t)targetActor.type.getLE());
            hCPU->gpr[11] = 1;
            hCPU->gpr[9] = 1;
            hCPU->gpr[13] = isLeftHandWeapon ? 1 : 0; // set the hand index to 0 for left hand, 1 for right hand
            return;
        }
        // Support for long press (placeholder)
        //if (input.inGame.in_game && grabState.lastEvent == ButtonState::Event::LongPress) {
        //    Log::print<CONTROLS>("Long press detected for {} (side {})", targetActor.name.getLE().c_str(), (int)side);
        //    // TODO: Implement long press action (e.g., temporarily bind item)
        //    //grabState.longPress = false;
        //}
        //// Support for short press (placeholder)
        //if (input.inGame.in_game && grabState.lastEvent == ButtonState::Event::ShortPress) {
        //    Log::print<CONTROLS>("Short press detected for {} (side {})", targetActor.name.getLE().c_str(), (int)side);
        //    // TODO: Implement short press action (e.g., cycle weapon)
        //}

        hCPU->gpr[9] = 1;
    }
}

// todo: this only runs when it's shown for the first time!
void CemuHooks::hook_CreateNewScreen(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const char* screenName = (const char*)(s_memoryBaseAddress + hCPU->gpr[7]);
    ScreenId screenId = (ScreenId)hCPU->gpr[5];
    Log::print<CONTROLS>("Creating new screen \"{}\" with ID {:08X}...", screenName, std::to_underlying(screenId));

    // todo: When a pickup screen is shown, we should track if the user does a short grip press, and if it was the left and right hand.
    if (screenId == ScreenId::PickUp_00) {
        Log::print<CONTROLS>("PickUp screen detected, waiting for grip button press to bind item to hand...");
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

    //// check if weapon is held and if the grip button is held, drop it
    //auto input = VRManager::instance().XR->m_input.load();
    //if (input.inGame.in_game && isHeldByPlayer && input.inGame.grab[heldIndex].currentState) {
    //    // if the weapon is held by the player and the grip button is pressed, drop it
    //    //Log::print("!! Dropping weapon {} because grip button is pressed", weapon.name.getLE());
    //    hCPU->gpr[7] = 1;
    //    return;
    //}
}


void CemuHooks::hook_GetContactLayerOfAttack(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    if (GetSettings().IsThirdPersonMode()) {
        return;
    }

    uint32_t player = hCPU->gpr[25];

    ActorWiiU actor = {};
    readMemory(player, &actor);

    uint32_t originalContactLayerPtr = hCPU->gpr[5];
    uint32_t originalContactLayer = getMemory<uint32_t>(originalContactLayerPtr).getLE();
    const char* originalContactLayerStr = (const char*)(s_memoryBaseAddress + originalContactLayer);

    uint32_t contactLayerValue = hCPU->gpr[3];

    // hardcoded 0!!!
    //if (m_motionAnalyzers[0].IsAttacking()) {
    //    contactLayerValue = (uint32_t)ContactLayer::SensorAttackPlayer;
    //}
    //else {
    //    //contactLayerValue = (uint32_t)ContactLayer::SensorChemical;
    //}


    //for (uint32_t i = 0; i < contactLayerNames.size(); i++) {
    //    std::string& layerName = contactLayerNames[i];
    //    if (layerName == "SensorNoHit") {
    //        //Log::print<INFO>("Layer {} is {}", layerName, i);
    //        contactLayerValue = i;
    //    }
    //}

    hCPU->gpr[3] = contactLayerValue;

    //Log::print<INFO>("GetContactLayerOfAttack called by {} ({:08X}) with contact layer {} which is a value of {}", actor.name.getLE(), player, originalContactLayerStr, contactLayerValue);
}


void CemuHooks::hook_EnableWeaponAttackSensor(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;


    if (GetSettings().IsThirdPersonMode()) {
        return;
    }

    uint32_t weaponPtr = hCPU->gpr[3];
    uint32_t heldIndex = hCPU->gpr[5]; // this is either 0 or 1 depending on which hand the weapon is in
    bool isHeldByPlayer = hCPU->gpr[6] == 0;
    uint32_t frameCounter = hCPU->gpr[7];

    Weapon weapon = {};
    readMemory(weaponPtr, &weapon);

    WeaponType weaponType = weapon.type.getLE();
    if (weaponType == WeaponType::Bow || weaponType == WeaponType::Shield) {
        //Log::print<INFO>("Skipping motion analysis for Bow/Shield (type: {}): {}", (int)weaponType, weapon.name.getLE());
        return;
    }

    if (heldIndex >= m_motionAnalyzers.size()) {
        Log::print<CONTROLS>("Invalid heldIndex: {}. Skipping motion analysis.", heldIndex);
        return;
    }

    auto& currFrame = VRManager::instance().XR->GetRenderer()->GetFrame(frameCounter);
    if (currFrame.ranMotionAnalysis[heldIndex]) {
        Log::print<CONTROLS>("Skipping motion analysis for {}: already ran this frame", heldIndex);
        return;
    }
    currFrame.ranMotionAnalysis[heldIndex] = true;

    heldIndex = heldIndex == 0 ? 1 : 0;


    //Log::print("!! Running weapon analysis for {}", heldIndex);

    auto state = VRManager::instance().XR->m_input.load();
    auto headset = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (!headset.has_value()) {
        return;
    }

    m_motionAnalyzers[heldIndex].ResetIfWeaponTypeChanged(weaponType);
    m_motionAnalyzers[heldIndex].Update(state.inGame.poseLocation[heldIndex], state.inGame.poseVelocity[heldIndex], headset.value(), state.inGame.inputTime);

    // Use the analysed motion to determine whether the weapon is swinging or stabbing, and whether the attackSensor should be active this frame
    bool CHEAT_alwaysEnableWeaponCollision = false;
    if (isHeldByPlayer && (m_motionAnalyzers[heldIndex].IsAttacking() || CHEAT_alwaysEnableWeaponCollision)) {
        m_motionAnalyzers[heldIndex].SetHitboxEnabled(true);
        //Log::print("!! Activate sensor for {}: isHeldByPlayer={}, weaponType={}", heldIndex, isHeldByPlayer, (int)weaponType);
        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 2;
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        //weapon.setupAttackSensor.overrideImpact = 1;
        //weapon.setupAttackSensor.impact = 2312;
        //weapon.setupAttackSensor.multiplier = 20.0f;
        //weapon.setupAttackSensor.overrideImpact = 1;
        //weapon.setupAttackSensor.multiplier = analyzer->GetDamage();
        //weapon.setupAttackSensor.impact = analyzer->GetImpulse();

        writeMemory(weaponPtr, &weapon);
    }
    else if (m_motionAnalyzers[heldIndex].IsHitboxEnabled()) {
        m_motionAnalyzers[heldIndex].SetHitboxEnabled(false);
        //Log::print("!! Deactivate sensor for {}: isHeldByPlayer={}, weaponType={}", heldIndex, isHeldByPlayer, (int)weaponType);

        weapon.setupAttackSensor.resetAttack = 1;
        weapon.setupAttackSensor.mode = 1; // deactivate attack sensor
        weapon.setupAttackSensor.isContactLayerInitialized = 0;
        writeMemory(weaponPtr, &weapon);
    }

    // rumbles
    if (m_motionAnalyzers[heldIndex].IsAttacking()) {
        float rumbleVelocity = m_motionAnalyzers[heldIndex].handVelocityLength - WeaponMotionAnalyser::HAND_VELOCITY_LENGTH_THRESHOLD;
        if (rumbleVelocity <= 0.0f) {
            rumbleVelocity = 0.0f;
        }
        VRManager::instance().XR->GetRumbleManager()->startSimpleRumble(!heldIndex, 0.1f, 0.5f * rumbleVelocity, 0.7f * rumbleVelocity);
    }
}

void CemuHooks::hook_SetPlayerWeaponScale(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson()) {
        return;
    }

    //uint32_t weaponPtr = hCPU->gpr[31];
    //Weapon weapon = {};
    //readMemory(weaponPtr, &weapon);
    //WeaponType weaponType = weapon.type.getLE();

    ////Log::print<INFO>("Setting weapon scale to {} for weapon {} of type {}", weapon.originalScale.getLE(), weapon.name.getLE().c_str(), std::to_underlying(weaponType));
    //
    ////weapon.originalScale = glm::fvec3(0.9f, 0.9f, 0.9f);

    //writeMemory(weaponPtr, &weapon);
}

void CemuHooks::hook_EquipWeapon(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto input = VRManager::instance().XR->m_input.load();
    // Check both hands for a short press to pick up weapon
    for (int side = 0; side < 2; ++side) {
        auto& grabState = input.inGame.grabState[side];
        // todo: Make sword smaller while its equipped. I think this might be a member value, but otherwise we can just scale the weapon matrix.

        //if (input.inGame.in_game && grabState.shortPress) {
        //    // Set the slot to equip based on which hand was pressed
        //    hCPU->gpr[25] = side; // 0 = LEFT, 1 = RIGHT
        //    Log::print("!! Short grip press detected on side {}: equipping weapon", side);
        //    grabState.shortPress = false; // Reset after use
        //    return;
        //}
    }
    // Default behavior if no short press
    // (leave as is, or add fallback logic if needed)
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

void CemuHooks::hook_ModifyHandModelAccessSearch(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (hCPU->gpr[3] == 0) {
        return;
    }
#ifdef _DEBUG
    // r3 holds the address of the string to search for
    const char* actorName = (const char*)(s_memoryBaseAddress + hCPU->gpr[3]);

    if (actorName != nullptr) {
        // Weapon_R is presumably his right hand bone name
        Log::print<CONTROLS>("Searching for model handle using {}", actorName);
    }
#endif
}

void CemuHooks::DrawDebugOverlays() {
    if (ImGui::Begin("Weapon Motion Debugger")) {
        for (auto it = m_motionAnalyzers.rbegin(); it != m_motionAnalyzers.rend(); ++it) {
            ImGui::PushID(&(*it));
            ImGui::BeginGroup();
            it->DrawDebugOverlay();
            ImGui::EndGroup();
            ImGui::PopID();
        }
    }
    ImGui::End();
}
