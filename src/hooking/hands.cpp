#include "cemu_hooks.h"

struct ActorWiiU {
    uint32_t vtable;
    BEType<uint32_t> baseProcPtr;
    uint8_t unk_08[0x1F8 - 0x08];
    Matrix34 mtx;
    uint32_t physicsMtxPtr;
    Matrix34 homeMtx;
    Vec3 velocity;
    Vec3 angularVelocity;
    Vec3 scale;
};

std::vector<ActorWiiU> s_actors;
std::vector<uint32_t> s_actorPtrs;

void CemuHooks::hook_UpdateActorList(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // r7 holds actor list size
    // r5 holds current actor index
    // r6 holds current actor* list entry

    // clear actor list when reiterating actor list again
    if (hCPU->gpr[5] == 0) {
        s_actors.clear();
    }

    uint32_t actorPtr = hCPU->gpr[6] + offsetof(ActorWiiU, baseProcPtr);
    uint32_t actorPtrUnderscore = 0;
    readMemoryBE(actorPtr, &actorPtrUnderscore);
    if (actorPtrUnderscore == 0) {
        Log::print("Updating actor list [{}/{}] {:08x} - No Name", hCPU->gpr[5], hCPU->gpr[7], hCPU->gpr[6]);
        return;
    }

    char* actorName = (char*)s_memoryBaseAddress + actorPtrUnderscore;

    if (strcmp(actorName, "FldObj_PushRock_A_M_01") == 0) {
        Log::print("Updating actor list [{}/{}] {:08x} - {}", hCPU->gpr[5], hCPU->gpr[7], hCPU->gpr[6], actorName);
        // float velocityY = 0.0f;
        // readMemoryBE(hCPU->gpr[6] + offsetof(ActorWiiU, velocity.y), &velocityY);
        // velocityY = velocityY * 1.5f;
        // writeMemoryBE(hCPU->gpr[6] + offsetof(ActorWiiU, velocity.y), &velocityY);
        s_actorPtrs.emplace_back(hCPU->gpr[6]);
    }
}

void CemuHooks::updateFrames() {
    for (uint32_t actorPtr : s_actorPtrs) {
        float velocityAngY = 0.0f;
        readMemoryBE(actorPtr + offsetof(ActorWiiU, angularVelocity.y), &velocityAngY);
        velocityAngY = 300.0f;
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, angularVelocity.y), &velocityAngY);

        float velocityY = 0.0f;
        readMemoryBE(actorPtr + offsetof(ActorWiiU, velocity.y), &velocityY);
        velocityY = 300.0f;
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, velocity.y), &velocityY);
    }
}

/*
Updating actor list [0/582] 41bc010c - No Name
Updating actor list [1/582] 449e571c - TipsSystemActor
Updating actor list [2/582] 4301d1b8 - Dm_Npc_Gerudo_HeroSoul_Kago
Updating actor list [3/582] 4316cab8 - Explode
Updating actor list [4/582] 4316c56c - Obj_SupportApp_Wind
Updating actor list [5/582] 4301d994 - GameROMPlayer
Updating actor list [6/582] 43171bc4 - Animal_Insect_S
Updating actor list [7/582] 43187000 - Animal_Insect_M
Updating actor list [8/582] 4319d458 - Animal_Insect_A
Updating actor list [9/582] 4319c5bc - Animal_Insect_B
Updating actor list [10/582] 431ca234 - Animal_Insect_X
Updating actor list [11/582] 431cb0d0 - Item_Conductor
Updating actor list [12/582] 431e9cf4 - Armor_Default_Extra_01
Updating actor list [13/582] 431efbb4 - Armor_Default_Extra_00
Updating actor list [14/582] 431f0214 - WakeBoardRope
Updating actor list [15/582] 43205268 - PlayerStole2
Updating actor list [16/582] 4320cc68 - Dm_Npc_RevivalFairy
Updating actor list [17/582] 4320d564 - Dm_Npc_Zora_HeroSoul_Kago
Updating actor list [18/582] 43246e18 - Dm_Npc_Rito_HeroSoul_Kago
Updating actor list [19/582] 43269bf4 - Dm_Npc_Goron_HeroSoul_Kago
Updating actor list [20/582] 4329a1f0 - EventSystemActor
Updating actor list [21/582] 4329a73c - EventSystemActor
Updating actor list [22/582] 43299ca4 - EventSystemActor
Updating actor list [23/582] 3fa085d4 - EventControllerRumble
Updating actor list [24/582] 3fa0a080 - Fader
Updating actor list [25/582] 3fa0a5cc - EventMessageTransmitter1
Updating actor list [26/582] 3fa0d504 - Fader
Updating actor list [27/582] 3fa0f040 - EventMessageTransmitter1
Updating actor list [28/582] 3fa10988 - EventMessageTransmitter1
Updating actor list [29/582] 3fa122d0 - EventMessageTransmitter1
Updating actor list [30/582] 3fa13c18 - EventMessageTransmitter1
Updating actor list [31/582] 3fa15560 - TerrainCalcCenterTag
Updating actor list [32/582] 3fa16ed8 - DemoXLinkActor
Updating actor list [33/582] 3fa18fe4 - DemoXLinkActor
Updating actor list [34/582] 3fa1b0f0 - DemoXLinkActor
Updating actor list [35/582] 3fa1d1fc - DemoXLinkActor
Updating actor list [36/582] 3fa1f308 - Fader
Updating actor list [37/582] 3fa20e44 - EventControllerRumble
Updating actor list [38/582] 3fa228f0 - EventControllerRumble
Updating actor list [39/582] 3fa2439c - EventControllerRumble
Updating actor list [40/582] 3fa25e48 - DemoXLinkActor
Updating actor list [41/582] 3fa27f54 - DemoXLinkActor
Updating actor list [42/582] 3fa2a060 - DemoXLinkActor
Updating actor list [43/582] 3fa2c16c - DemoXLinkActor
Updating actor list [44/582] 3fa2e280 - EventControllerRumble
Updating actor list [45/582] 3fa2fd2c - EventControllerRumble
Updating actor list [46/582] 3fa317d8 - EventCameraRumble
Updating actor list [47/582] 3fa33280 - EventControllerRumble
Updating actor list [48/582] 3fa34d2c - EventCameraRumble
Updating actor list [49/582] 3fa367d4 - DemoXLinkActor
Updating actor list [50/582] 3fa388e0 - DemoXLinkActor
Updating actor list [51/582] 432b7398 - SoundTriggerTag
Updating actor list [52/582] 432b78e4 - SoundTriggerTag
Updating actor list [53/582] 432ba350 - SoundTriggerTag
Updating actor list [54/582] 432bbb2c - SoundTriggerTag
Updating actor list [55/582] 432bd308 - SceneSoundCtrlTag
Updating actor list [56/582] 432beac4 - SceneSoundCtrlTag
Updating actor list [57/582] 432c0280 - SceneSoundCtrlTag
Updating actor list [58/582] 432c1a3c - SceneSoundCtrlTag
Updating actor list [59/582] 432c31f8 - EventSystemActor
Updating actor list [60/582] 432ccef4 - EventSystemActor
Updating actor list [61/582] 432d6bf0 - EventSystemActor
Updating actor list [62/582] 432e08ec - EventSystemActor
Updating actor list [63/582] 44ae2e30 - Chemical
Updating actor list [64/582] 44ae2398 - ReactionHit
Updating actor list [65/582] 44af2524 - GetItemSound
Updating actor list [66/582] 44ae7de4 - BoxWaterSound
Updating actor list [67/582] 44af82d4 - EnvSoundWater
Updating actor list [68/582] 44ae28e4 - ReactionField
Updating actor list [69/582] 44af9164 - EnvSoundGrass
Updating actor list [70/582] 44afe100 - EnvSoundTree
Updating actor list [71/582] 44afd294 - PlayerFootStep
Updating actor list [72/582] 44afef30 - LocatorEffect
Updating actor list [73/582] 44b00388 - GameRomHorse05
Updating actor list [74/582] 44b54104 - Horse_Link_Mane_Reduction
Updating actor list [75/582] 44b61ff4 - Horse_Link_Mane
Updating actor list [76/582] 44affd3c - Horse_Link_Mane_Grabbed
Updating actor list [77/582] 44b4f39c - Weapon_Sword_018
Updating actor list [78/582] 44b4ff08 - Weapon_Bow_071
Updating actor list [79/582] 44b50a74 - Armor_140_Head
Updating actor list [80/582] 44b510d4 - Armor_151_Upper
Updating actor list [81/582] 44b51734 - Armor_015_Lower
Updating actor list [82/582] 44b51d94 - Weapon_Sheath_016
Updating actor list [83/582] 44b52408 - NPC_GodVoice
Updating actor list [84/582] 44b52d04 - NPC_PublicVoice
Updating actor list [85/582] 44b53600 - NPC_CaptionVoice
Updating actor list [86/582] 44b88c70 - PauseMenuPlayer
Updating actor list [87/582] 44b894d4 - Dm_Npc_RevivalFairy
Updating actor list [88/582] 44b02b14 - Obj_Sun_A_01
Updating actor list [89/582] 44b01cfc - Obj_Moon_A_01
Updating actor list [90/582] 44e53cb0 - GameRomCamera
Updating actor list [91/582] 44e64a68 - EditCamera
Updating actor list [92/582] 44fde71c - RemainsWind_Far
Updating actor list [93/582] 44fdf30c - FldObj_MapTower_A_01_Far
Updating actor list [94/582] 44fdfdc8 - FldObj_MapTower_A_01_Far
Updating actor list [95/582] 44feb924 - Obj_SweepCollision
Updating actor list [96/582] 44febea4 - Obj_SweepCollision
Updating actor list [97/582] 44ff97bc - Obj_SweepCollision
Updating actor list [98/582] 4524f0d8 - Obj_SweepCollision
Updating actor list [99/582] 4515bb30 - Area
Updating actor list [100/582] 45251370 - Obj_TreeBroadleaf_A_L
Updating actor list [101/582] 4526a154 - Area
Updating actor list [102/582] 45250cc0 - EnvSeEmitPointBirdTemperate
Updating actor list [103/582] 4526bf08 - EnemyFortressMgrTag
Updating actor list [104/582] 4534e130 - EnvSeEmitPointInsectTemperate
Updating actor list [105/582] 4535b03c - EnemyLookTag
Updating actor list [106/582] 453598d0 - LookTag
Updating actor list [107/582] 453e6078 - BarrelBomb
Updating actor list [108/582] 453ffc0c - BarrelBomb
Updating actor list [109/582] 45411d40 - BarrelBomb
Updating actor list [110/582] 453fec6c - Rope
Updating actor list [111/582] 4544ed58 - Obj_Oil
Updating actor list [112/582] 45439d8c - Obj_Oil
Updating actor list [113/582] 454394f4 - Rope
Updating actor list [114/582] 4540a96c - FldObj_HangedLamp_A
Updating actor list [115/582] 4541ca6c - FldObj_HangedLamp_A
Updating actor list [116/582] 4543d71c - BarrelBomb
Updating actor list [117/582] 4544ddd8 - Area
Updating actor list [118/582] 454c93d8 - PlayerOnScaffoldCheckTag
Updating actor list [119/582] 45508b6c - Enemy_Bokoblin_Junior
Updating actor list [120/582] 4553eeb4 - Enemy_Bokoblin_Middle
Updating actor list [121/582] 4553e018 - Enemy_Bokoblin_Guard_Junior
Updating actor list [122/582] 456a895c - Weapon_Sword_004
Updating actor list [123/582] 455c25e4 - Weapon_Sheath_004
Updating actor list [124/582] 45705bb0 - Weapon_Bow_004
Updating actor list [125/582] 45716abc - Enemy_Bokoblin_Junior
Updating actor list [126/582] 457b7064 - Weapon_Bow_004
Updating actor list [127/582] 45705044 - Weapon_Sword_005
Updating actor list [128/582] 457b1cb4 - BokoblinPipe
Updating actor list [129/582] 457a3be4 - Weapon_Quiver_001
Updating actor list [130/582] 456a57e4 - Weapon_Sheath_004
Updating actor list [131/582] 456a5e58 - Weapon_Quiver_001
Updating actor list [132/582] 456a64cc - BattleBgmRequestFinishTag
Updating actor list [133/582] 4572dccc - EventTag
Updating actor list [134/582] 4572d160 - Weapon_Shield_005
Updating actor list [135/582] 45863fe4 - SwitchTimeLag
Updating actor list [136/582] 456a6a18 - TBox_Field_Enemy
Updating actor list [137/582] 45635228 - Area
Updating actor list [138/582] 45634650 - BarrelBomb
Updating actor list [139/582] 456340e4 - GrassCutTag
Updating actor list [140/582] 4587dd80 - Enemy_Bokoblin_Guard_Junior
Updating actor list [141/582] 4563694c - EnemyChatFortMgrTag
Updating actor list [142/582] 4598f92c - Weapon_Bow_004
Updating actor list [143/582] 45889b7c - Enemy_Bokoblin_Guard_Junior
Updating actor list [144/582] 459065e4 - Weapon_Quiver_001
Updating actor list [145/582] 45908c28 - Area
Updating actor list [146/582] 45906c58 - Weapon_Bow_004
Updating actor list [147/582] 459091ac - ChangeWeatherTag
Updating actor list [148/582] 45979738 - BarrelBomb
Updating actor list [149/582] 45977de4 - Weapon_Quiver_001
Updating actor list [150/582] 45979f14 - BarrelBomb
Updating actor list [151/582] 4597a6f0 - BarrelBomb
Updating actor list [152/582] 459b4044 - Item_Meat_01
Updating actor list [153/582] 459c2d1c - FldObj_PushRock_A_M_01
Updating actor list [154/582] 459d90c4 - EventTag
Updating actor list [155/582] 459d9664 - DgnObj_WarpPointSP
Updating actor list [156/582] 45ac28c0 - AirWallForE3
Updating actor list [157/582] 45a43e80 - Enemy_Bokoblin_Guard_Junior
Updating actor list [158/582] 45ac7800 - AirWallForE3
Updating actor list [159/582] 459c34f8 - FldObj_DownloadTerminal_A_01
Updating actor list [160/582] 45ad15a0 - AirWallForE3
Updating actor list [161/582] 45a2d6ac - AirWallForE3
Updating actor list [162/582] 45a2dc34 - Area
Updating actor list [163/582] 45ad5c80 - EventTag
Updating actor list [164/582] 45ae987c - Area
Updating actor list [165/582] 45a2e1b8 - SignalFlowchart
Updating actor list [166/582] 45ad1bb0 - Weapon_Sword_001
Updating actor list [167/582] 45b1a364 - Weapon_Shield_004
Updating actor list [168/582] 45aeb06c - Npc_King_Parasail002
Updating actor list [169/582] 45ab32e4 - Weapon_Sheath_001
Updating actor list [170/582] 45aed2a0 - Npc_King_Vagrant006
Updating actor list [171/582] 45ab3958 - Weapon_Sword_500
Updating actor list [172/582] 45bdd1bc - FldObj_PushRock_A_M_01
Updating actor list [173/582] 45b7d71c - Enemy_Bokoblin_Junior
Updating actor list [174/582] 45c31dfc - EnvSeEmitPointInsectTemperate
Updating actor list [175/582] 45cc8264 - Weapon_Sword_004
Updating actor list [176/582] 45c32d84 - Weapon_Sword_500
Updating actor list [177/582] 45cc7bfc - EnvSeEmitPointBirdTemperate
Updating actor list [178/582] 45cacde4 - Weapon_Sheath_004
Updating actor list [179/582] 45bd4a1c - EnvSeEmitPointBirdSubarctic
Updating actor list [180/582] 45cdb684 - EnvSeEmitPointBirdTemperate
Updating actor list [181/582] 45ce14bc - EnvSeEmitPointInsectTemperate
Updating actor list [182/582] 45cf58c8 - Enemy_Bokoblin_Junior
Updating actor list [183/582] 45e0448c - Item_Meat_01
Updating actor list [184/582] 45d91c44 - Enemy_Bokoblin_Junior
Updating actor list [185/582] 45e1c350 - FldObj_PushRock_A_M_01
Updating actor list [186/582] 45e0e1c8 - Weapon_Sword_004
Updating actor list [187/582] 45e27afc - EnvSeEmitPointBirdTemperate
Updating actor list [188/582] 45e4f464 - Weapon_Sword_004
Updating actor list [189/582] 45d65658 - Weapon_Sword_044
Updating actor list [190/582] 45e01958 - EnvSeEmitPointBirdSubarctic
Updating actor list [191/582] 45e032a4 - Weapon_Lsheath_056
Updating actor list [192/582] 45e97abc - EnvSeEmitPointBirdSubarctic
Updating actor list [193/582] 45e02c3c - EnvSeEmitPointInsectTemperate
Updating actor list [194/582] 45efa760 - EnvSeEmitPointBirdTemperate
Updating actor list [195/582] 45efadc8 - Enemy_Bokoblin_Junior
Updating actor list [196/582] 45f8a780 - Enemy_Bokoblin_Junior
Updating actor list [197/582] 4603f2c4 - Item_Meat_01
Updating actor list [198/582] 46056eac - Weapon_Sword_004
Updating actor list [199/582] 46049000 - Weapon_Sword_044
Updating actor list [200/582] 460480c4 - EnvSeEmitPointBirdSubarctic
Updating actor list [201/582] 45f6a4e4 - Weapon_Lsheath_056
Updating actor list [202/582] 460849c0 - EnvSeEmitPointBirdTemperate
Updating actor list [203/582] 4608a7bc - Weapon_Sword_044
Updating actor list [204/582] 46090dbc - EnvSeEmitPointBirdTemperate
Updating actor list [205/582] 460c0370 - Weapon_Lsheath_056
Updating actor list [206/582] 460c09e4 - Area
Updating actor list [207/582] 460c249c - Area
Updating actor list [208/582] 461018bc - Obj_LiftRockWhite_A_01
Updating actor list [209/582] 4613b830 - Area
Updating actor list [210/582] 46128ad0 - Npc_HiddenKorokGround
Updating actor list [211/582] 461603b0 - SwitchTimeLag
Updating actor list [212/582] 460fb41c - Npc_HiddenKorokFly
Updating actor list [213/582] 46161908 - Enemy_Chuchu_Junior
Updating actor list [214/582] 4617498c - Enemy_Chuchu_Junior
Updating actor list [215/582] 461f0aa0 - Area
Updating actor list [216/582] 461c1bc4 - EnemyChatFortMgrTag
Updating actor list [217/582] 461c0bf8 - Enemy_Chuchu_Junior
Updating actor list [218/582] 4627ae34 - Enemy_Bokoblin_Junior
Updating actor list [219/582] 462179e8 - UMii_Korogu_PlantL_C_002
Updating actor list [220/582] 46278a94 - Weapon_Sword_004
Updating actor list [221/582] 46126130 - EnemyLookTag
Updating actor list [222/582] 46126b2c - GrassCutTag
Updating actor list [223/582] 4636d418 - Area
Updating actor list [224/582] 4637bbbc - EnvSeEmitPointBirdTemperate
Updating actor list [225/582] 46277bf8 - Enemy_Bokoblin_Junior
Updating actor list [226/582] 46380a84 - Weapon_Sword_004
Updating actor list [227/582] 4635f258 - EnvSeEmitPointInsectTemperate
Updating actor list [228/582] 4637ff18 - Weapon_Sword_043
Updating actor list [229/582] 463608c4 - EnvSeEmitPointBirdTemperate
Updating actor list [230/582] 46360e2c - Weapon_Lsheath_056
Updating actor list [231/582] 463614a0 - EnvSeEmitPointBirdTemperate
Updating actor list [232/582] 4637c124 - Item_Mushroom_J
Updating actor list [233/582] 4638c360 - Weapon_Sword_043
Updating actor list [234/582] 463a777c - EnvSeEmitPointBirdTemperate
Updating actor list [235/582] 463a7ce4 - Weapon_Lsheath_056
Updating actor list [236/582] 463ecebc - EnvSeEmitPointBirdTemperate
Updating actor list [237/582] 4638cecc - Item_CookSet
Updating actor list [238/582] 463ed424 - EnvSeEmitPointInsectTemperate
Updating actor list [239/582] 463e5434 - EnvSeEmitPointInsectTemperate
Updating actor list [240/582] 463e5af0 - Area
Updating actor list [241/582] 463e70f0 - AirWallHorse_HiddnKorok
Updating actor list [242/582] 463fbe04 - SwitchTimeLag
Updating actor list [243/582] 463fa698 - ActorObserverTag
Updating actor list [244/582] 463fe544 - KorokAnswerResponce
Updating actor list [245/582] 464276f8 - Npc_HiddenKorokFly
Updating actor list [246/582] 4644bfdc - Weapon_Bow_001
Updating actor list [247/582] 46485b74 - Area
Updating actor list [248/582] 464874bc - UMii_Korogu_PlantL_C_002
Updating actor list [249/582] 464594b4 - Weapon_Quiver_001
Updating actor list [250/582] 4650f940 - Obj_LiftRockWhite_A_01
Updating actor list [251/582] 4644b060 - AirWallHorse_HiddnKorok
Updating actor list [252/582] 46449630 - Npc_HiddenKorokGround
Updating actor list [253/582] 46519988 - Area
Updating actor list [254/582] 46540ff0 - Obj_Plant_IvyBurn_A_01
Updating actor list [255/582] 46519f0c - Obj_Plant_IvyBurn_A_01
Updating actor list [256/582] 465c2c5c - Obj_Plant_IvyBurn_A_01
Updating actor list [257/582] 465c3ba0 - Obj_Plant_IvyBurn_A_01
Updating actor list [258/582] 465bad5c - Obj_Plant_IvyBurn_A_01
Updating actor list [259/582] 465d325c - Obj_Plant_IvyBurn_A_01
Updating actor list [260/582] 465cb35c - Obj_Plant_IvyBurn_A_01
Updating actor list [261/582] 465e2f5c - Obj_LiftRockWhite_A_01
Updating actor list [262/582] 465db15c - Npc_HiddenKorokGround
Updating actor list [263/582] 46642e5c - Obj_Plant_IvyBurn_A_01
Updating actor list [264/582] 466772a8 - Obj_Plant_IvyBurn_A_01
Updating actor list [265/582] 46643638 - SpotBgmTag
Updating actor list [266/582] 466b8b5c - Obj_HeartUtuwa_A_01
Updating actor list [267/582] 46717794 - LocationTag
Updating actor list [268/582] 466f5c78 - Obj_StaminaUtuwa_A_01
Updating actor list [269/582] 46717ce0 - Area
Updating actor list [270/582] 466b0f04 - TwnObj_GoddesStatue_A_01
Updating actor list [271/582] 46734b0c - Obj_TreeBroadleaf_A_L_Trunk
Updating actor list [272/582] 4672e004 - FireWoodDie
Updating actor list [273/582] 467254e8 - Obj_TreeBroadleaf_A_L_Trunk
Updating actor list [274/582] 46726054 - SpotBgmTag
Updating actor list [275/582] 46785c3c - Obj_KorokIronRock_A_01
Updating actor list [276/582] 46724d0c - FldObj_PushRockIron_A_M_01
Updating actor list [277/582] 467d5878 - FldObj_Chain_A
Updating actor list [278/582] 467d09a4 - Npc_King_Vagrant005
Updating actor list [279/582] 467c7efc - FireWoodDie
Updating actor list [280/582] 467f8e04 - Weapon_Bow_001
Updating actor list [281/582] 467f8614 - Item_Ore_I
Updating actor list [282/582] 468c8d44 - ChangeWeatherTag
Updating actor list [283/582] 4685bf1c - Weapon_Quiver_001
Updating actor list [284/582] 4685c590 - Area
Updating actor list [285/582] 4693480c - Area
Updating actor list [286/582] 46967338 - MusicianSpotBgmTag
Updating actor list [287/582] 46967bc4 - Npc_King_Vagrant001
Updating actor list [288/582] 46a4fb2c - Area
Updating actor list [289/582] 46a51474 - Weapon_Sword_500
Updating actor list [290/582] 469c1c5c - Area
Updating actor list [291/582] 46a73010 - FireWoodDie
Updating actor list [292/582] 469c2efc - FireWood
Updating actor list [293/582] 46a7c944 - Area
Updating actor list [294/582] 46936154 - Npc_Musician_AoC_BalladOfHeroes
Updating actor list [295/582] 46a9529c - Weapon_Lsword_032
Updating actor list [296/582] 46a4cb2c - BarrelBomb
Updating actor list [297/582] 46b0f8b8 - Weapon_Lsheath_032
Updating actor list [298/582] 46b00a40 - Weapon_Sword_006
Updating actor list [299/582] 46b1bebc - BoarMeat_Frame
Updating actor list [300/582] 46b2a08c - Enemy_Bokoblin_Junior
Updating actor list [301/582] 46b90798 - GrassCutTag
Updating actor list [302/582] 46bffb74 - EnemyFortressMgrTag
Updating actor list [303/582] 46b8ffbc - FireWood
Updating actor list [304/582] 46c3e798 - Area
Updating actor list [305/582] 46c0157c - BoarMeat
Updating actor list [306/582] 46cff4f8 - Area
Updating actor list [307/582] 46c57e90 - Weapon_Sword_021
Updating actor list [308/582] 46b0bb6c - Enemy_Bokoblin_Dark
Updating actor list [309/582] 46d30b04 - Item_Meat_01
Updating actor list [310/582] 46d0078c - Weapon_Sheath_021
Updating actor list [311/582] 46ce40a0 - MusicianSpotBgmTag
Updating actor list [312/582] 46d464bc - Area
Updating actor list [313/582] 46d398c4 - Npc_Musician_AoC_BalladOfHeroes
Updating actor list [314/582] 46dcac24 - Npc_Musician_AoC_BalladOfHeroes_Last
Updating actor list [315/582] 46d46a40 - Area
Updating actor list [316/582] 46e2063c - Area
Updating actor list [317/582] 46e20bc0 - Area
Updating actor list [318/582] 46e8813c - Area
Updating actor list [319/582] 46dc63e0 - TBox_Field_Stone
Updating actor list [320/582] 46e8947c - EventTag
Updating actor list [321/582] 46e78f98 - SwitchTimeLag
Updating actor list [322/582] 46e794e4 - SignalFlowchart
Updating actor list [323/582] 46dc5680 - Obj_RockBroken_A_02
Updating actor list [324/582] 46e8d9bc - DgnObj_WarpPoint_A_01
Updating actor list [325/582] 46e8c0cc - DgnObj_DungeonEntrance_A_01
Updating actor list [326/582] 46e86b08 - EventTag
Updating actor list [327/582] 46ec90d0 - Area
Updating actor list [328/582] 46e8632c - DgnObj_EntranceShutter_A_01
Updating actor list [329/582] 46eaeff0 - DgnObj_EntranceTerminal_A_01
Updating actor list [330/582] 46edbeb0 - SignalFlowchart
Updating actor list [331/582] 46eca36c - Area
Updating actor list [332/582] 46edc450 - EventTag
Updating actor list [333/582] 46ecb6ac - EventTag
Updating actor list [334/582] 46f7f8bc - DgnObj_EntranceTerminal_A_01
Updating actor list [335/582] 46f9d850 - EventTag
Updating actor list [336/582] 46fa905c - DgnObj_WarpPoint_A_01
Updating actor list [337/582] 46f9bb64 - Npc_King_Vagrant007
Updating actor list [338/582] 46fa9da4 - DgnObj_DungeonEntrance_A_01
Updating actor list [339/582] 46fa95c8 - DgnObj_EntranceShutter_A_01
Updating actor list [340/582] 47078d54 - Weapon_Sword_500
Updating actor list [341/582] 47015d00 - DgnObj_EntranceTerminal_A_01
Updating actor list [342/582] 4702b584 - BoxWater
Updating actor list [343/582] 46ffe85c - DgnObj_DungeonEntrance_A_01
Updating actor list [344/582] 46ffedc8 - DgnObj_WarpPoint_A_01
Updating actor list [345/582] 46fff334 - DgnObj_EntranceShutter_A_01
Updating actor list [346/582] 46fffb10 - EventTag
Updating actor list [347/582] 470831e0 - FireWoodDie
Updating actor list [348/582] 470839bc - SignalFireWood
Updating actor list [349/582] 4708f75c - Area
Updating actor list [350/582] 4708fce0 - Area
Updating actor list [351/582] 47090264 - Area
Updating actor list [352/582] 470a604c - SwitchTimeLag
Updating actor list [353/582] 470a740c - SignalFlowchart
Updating actor list [354/582] 470a79ac - EventTag
Updating actor list [355/582] 470a7f4c - BoxWater
Updating actor list [356/582] 470bbf6c - Area
Updating actor list [357/582] 470c579c - Area
Updating actor list [358/582] 470c5d20 - Area
Updating actor list [359/582] 470d06b0 - Area
Updating actor list [360/582] 470d1fa0 - LocationTag
Updating actor list [361/582] 470d3644 - SignalFlowchart
Updating actor list [362/582] 470d606c - Area
Updating actor list [363/582] 470d771c - Area
Updating actor list [364/582] 470d8adc - Area
Updating actor list [365/582] 470d9e9c - Area
Updating actor list [366/582] 470db26c - Area
Updating actor list [367/582] 470dbe58 - ChangeWeatherTag
Updating actor list [368/582] 470e132c - Area
Updating actor list [369/582] 470dd778 - LocationTag
Updating actor list [370/582] 470debac - MergedGrudge_D-6_0
Updating actor list [371/582] 470e26fc - SignalFlowchart
Updating actor list [372/582] 470e3abc - EventTag
Updating actor list [373/582] 470e4fa0 - SwitchTimeLag
Updating actor list [374/582] 470fb8f8 - EventTag
Updating actor list [375/582] 470fbe98 - SignalFlowchart
Updating actor list [376/582] 470fe650 - Area
Updating actor list [377/582] 471013a0 - Area
Updating actor list [378/582] 471029f0 - Area
Updating actor list [379/582] 47104334 - Area
Updating actor list [380/582] 471056ec - SwitchTimeLag
Updating actor list [381/582] 47106b5c - SignalFlowchart
Updating actor list [382/582] 47107f1c - EventTag
Updating actor list [383/582] 4710e1cc - Area
Updating actor list [384/582] 4710c1e0 - LookTagAoC
Updating actor list [385/582] 4710fb14 - DgnObj_EntranceElevatorSP
Updating actor list [386/582] 47121880 - DgnObj_WarpPointSP
Updating actor list [387/582] 4711f360 - DgnObj_EntranceShutterSP
Updating actor list [388/582] 4710d884 - EventTag
Updating actor list [389/582] 4710f46c - SwitchTimeLag
Updating actor list [390/582] 47110fec - EventTag
Updating actor list [391/582] 47120318 - LocationTag
Updating actor list [392/582] 47120864 - Area
Updating actor list [393/582] 4711fb3c - DgnObj_EntranceTerminalSP
Updating actor list [394/582] 471210a4 - DgnObj_DLC_ChampionsDungeonEntrance_A_01
Updating actor list [395/582] 471364ec - FldObj_MapTower_A_01
Updating actor list [396/582] 4712a810 - FldObj_DownloadTerminalBody_A_01
Updating actor list [397/582] 471531ec - EventTag
Updating actor list [398/582] 47154e30 - FldObj_DownloadTerminal_A_01
Updating actor list [399/582] 47154790 - DgnObj_WarpPoint_A_01
Updating actor list [400/582] 47152a10 - FldObj_MapTowerWingBefore_A_01
Updating actor list [401/582] 47155908 - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [402/582] 4715539c - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [403/582] 4718a1dc - Area
Updating actor list [404/582] 4718bb24 - LookTagAoC
Updating actor list [405/582] 4718d3b8 - DgnObj_DLC_ChampionsDungeonEntrance_A_01
Updating actor list [406/582] 4718b47c - EventTag
Updating actor list [407/582] 4718db94 - DgnObj_EntranceElevatorSP
Updating actor list [408/582] 4718cd6c - LocationTag
Updating actor list [409/582] 471a33ac - DgnObj_EntranceTerminalSP
Updating actor list [410/582] 4718f374 - Area
Updating actor list [411/582] 4718f8f8 - SwitchTimeLag
Updating actor list [412/582] 471a3b88 - EventTag
Updating actor list [413/582] 471d0780 - DgnObj_WarpPointSP
Updating actor list [414/582] 471b869c - DgnObj_EntranceShutterSP
Updating actor list [415/582] 471ac2e0 - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [416/582] 471b8e78 - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [417/582] 471cea90 - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [418/582] 471d00e0 - FldObj_FlagLarge_A_01
Updating actor list [419/582] 47201210 - FldObj_FlagLarge_A_01
Updating actor list [420/582] 4720177c - FldObj_FlagLarge_A_01
Updating actor list [421/582] 47202254 - Area
Updating actor list [422/582] 47201ce8 - FldObj_FlagLarge_A_01
Updating actor list [423/582] 47203b9c - SwitchTimeLag
Updating actor list [424/582] 472034ec - FldObj_FlagLarge_A_01
Updating actor list [425/582] 4722b920 - Area
Updating actor list [426/582] 472044e4 - FldObj_Mound_A_01
Updating actor list [427/582] 47213d6c - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [428/582] 4722bea4 - LookTagAoC
Updating actor list [429/582] 472345cc - DgnObj_DLC_ChampionsDungeonEntrance_A_01
Updating actor list [430/582] 4724bae4 - DgnObj_WarpPointSP
Updating actor list [431/582] 4724c2c0 - DgnObj_EntranceShutterSP
Updating actor list [432/582] 47234da8 - EventTag
Updating actor list [433/582] 4724b498 - LocationTag
Updating actor list [434/582] 47262b28 - DgnObj_EntranceTerminalSP
Updating actor list [435/582] 47262488 - Area
Updating actor list [436/582] 472638f4 - SwitchTimeLag
Updating actor list [437/582] 47263e40 - EventTag
Updating actor list [438/582] 472643e0 - FldObj_FlagLarge_A_01
Updating actor list [439/582] 4727b9a0 - DgnObj_EntranceElevatorSP
Updating actor list [440/582] 47279c9c - FldObj_DownloadTerminalBody_A_01
Updating actor list [441/582] 4727b354 - FldObj_DownloadTerminal_A_01
Updating actor list [442/582] 47285014 - DgnObj_WarpPoint_A_01
Updating actor list [443/582] 47285580 - FldObj_MapTowerWingBefore_A_01
Updating actor list [444/582] 47285d5c - FldObj_MapTower_A_01
Updating actor list [445/582] 4728fbb4 - Area
Updating actor list [446/582] 472a3214 - LookTagAoC
Updating actor list [447/582] 472a3760 - DgnObj_EntranceElevatorSP
Updating actor list [448/582] 472a3f3c - DgnObj_DLC_ChampionsDungeonEntrance_A_01
Updating actor list [449/582] 472d017c - DgnObj_EntranceShutterSP
Updating actor list [450/582] 472ae010 - DgnObj_WarpPointSP
Updating actor list [451/582] 47299f2c - EventTag
Updating actor list [452/582] 472d0958 - DgnObj_EntranceTerminalSP
Updating actor list [453/582] 472e7128 - LocationTag
Updating actor list [454/582] 472e6a88 - Area
Updating actor list [455/582] 472e82b0 - EventTag
Updating actor list [456/582] 472e7c64 - SwitchTimeLag
Updating actor list [457/582] 4730592c - FldObj_DownloadTerminalBody_A_01
Updating actor list [458/582] 472e8850 - FldObj_SoilRock_MapTower_A_01
Updating actor list [459/582] 47306fd8 - FldObj_MapTowerWingBefore_A_01_First
Updating actor list [460/582] 47307f90 - FldObj_DownloadTerminalBody_A_01
Updating actor list [461/582] 473077b4 - FldObj_MapTower_A_01_First
Updating actor list [462/582] 473084fc - EventTag
Updating actor list [463/582] 47311038 - DgnObj_WarpPoint_A_01
Updating actor list [464/582] 47310acc - FldObj_DownloadTerminal_A_01
Updating actor list [465/582] 47338364 - FldObj_MapTowerWingBefore_A_01
Updating actor list [466/582] 4731d1e0 - FldObj_MapTower_A_01
Updating actor list [467/582] 47341c14 - FldObj_FlagLarge_A_01
Updating actor list [468/582] 473426ec - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [469/582] 47342180 - DgnObj_DungeonEntrance_A_01_Far
Updating actor list [470/582] 4734af30 - AirWall
Updating actor list [471/582] 4734a9c4 - SandStorm_Distance_Battle
Updating actor list [472/582] 4735fcf0 - ChangeWeatherTag
Updating actor list [473/582] 4736023c - Area
Updating actor list [474/582] 47357e60 - Snow_Distance_Lanayru
Updating actor list [475/582] 4736c764 - FldObj_GanonBeast_BattleAreaLine_A_01
Updating actor list [476/582] 4736ccd0 - FirstHighlandFog
Updating actor list [477/582] 473a047c - Snow_Distance_Gerudo
Updating actor list [478/582] 473c0920 - ChangeWindTag
Updating actor list [479/582] 473c8ecc - Area
Updating actor list [480/582] 473a09e8 - Rain_Distance
Updating actor list [481/582] 473c9450 - Thundercloud_Distance
Updating actor list [482/582] 473c03b4 - TwnObj_Village_Korok_DekuTree_A_01_Far
Updating actor list [483/582] 473e7bc4 - Area
Updating actor list [484/582] 473e8148 - ChangeWeatherTag
Updating actor list [485/582] 473f3ac4 - ChangeWeatherTag
Updating actor list [486/582] 473f4010 - Area
Updating actor list [487/582] 4743a170 - FldObj_MapTower_A_01_Far
Updating actor list [488/582] 47400550 - SitRemainsWater_Far
Updating actor list [489/582] 4743c1fc - FldObj_MapTower_A_01_Far
Updating actor list [490/582] 47448aac - RemainsFire_Far
Updating actor list [491/582] 4743c768 - SitRemainsFire_Far
Updating actor list [492/582] 474513f0 - RemainsFire_Far
Updating actor list [493/582] 474492a0 - RemainsFire_Far
Updating actor list [494/582] 4745df64 - FldObj_MapTower_A_01_Far
Updating actor list [495/582] 4745e4d0 - FldObj_MapTower_A_01_Far
Updating actor list [496/582] 47483f6c - FldObj_MapTower_A_01_Far
Updating actor list [497/582] 474844d8 - FldObj_MapTower_A_01_Far
Updating actor list [498/582] 47484cec - FldObj_MapTower_A_01_Far
Updating actor list [499/582] 474853ac - FldObj_MapTower_A_01_Far
Updating actor list [500/582] 47485bc0 - FldObj_MapTower_A_01_Far
Updating actor list [501/582] 47486280 - SitRemainsElec_Far
Updating actor list [502/582] 47486a94 - FldObj_MapTower_A_01_Far
Updating actor list [503/582] 47487bd8 - RemainsWind_Far
Updating actor list [504/582] 47487154 - DungeonPit
Updating actor list [505/582] 47488520 - SitRemainsWind_Far
Updating actor list [506/582] 474922dc - Obj_TreeBroadleaf_A_LL
Updating actor list [507/582] 474a7484 - Item_Fruit_A
Updating actor list [508/582] 474a7c74 - Obj_TreeApple_A_M_01
Updating actor list [509/582] 474a8454 - Item_Fruit_A
Updating actor list [510/582] 474a8c44 - Item_Fruit_A
Updating actor list [511/582] 474a9434 - Obj_TreeBroadleaf_A_L
Updating actor list [512/582] 474a9c14 - Obj_TreeBroadleaf_A_L
Updating actor list [513/582] 474da6ac - Item_Plant_A
Updating actor list [514/582] 474d9ebc - Item_Plant_A
Updating actor list [515/582] 474dae9c - Item_Plant_A
Updating actor list [516/582] 3fa3f164 - DemoXLinkActor
Updating actor list [517/582] 3fa4261c - DemoXLinkActor
Updating actor list [518/582] 3fa44344 - DemoXLinkActor
Updating actor list [519/582] 474aa3f4 - EventControllerRumble
Updating actor list [520/582] 474db68c - EventMessageTransmitter1
Updating actor list [521/582] 474e20b4 - EventControllerRumble
Updating actor list [522/582] 474f0bd0 - Fader
Updating actor list [523/582] 474f111c - EventMessageTransmitter1
Updating actor list [524/582] 474f1668 - EventCameraRumble
Updating actor list [525/582] 474e2600 - WorldManagerControl
Updating actor list [526/582] 474f8690 - NPC_DRCVoice
Updating actor list [527/582] 4751dc24 - PlayerShockWave
Updating actor list [528/582] 47530824 - PlayerShockWave
Updating actor list [529/582] 4750b124 - FldObj_ShootingStar
Updating actor list [530/582] 3fa5d204 - DemoXLinkActor
Updating actor list [531/582] 3fa600dc - DemoXLinkActor
Updating actor list [532/582] 3fa62fe0 - DemoXLinkActor
Updating actor list [533/582] 45e5e904 - WorldManagerControl
Updating actor list [534/582] 45e607ac - Fader
Updating actor list [535/582] 45e60160 - EventControllerRumble
Updating actor list [536/582] 45e61c9c - EventMessageTransmitter1
Updating actor list [537/582] 45e621e8 - EventCameraRumble
Updating actor list [538/582] 45e96254 - NPC_DRCVoice
Updating actor list [539/582] 45e62734 - EventBgmCtrlTag
Updating actor list [540/582] 46088ecc - NPC_DRCVoice
Updating actor list [541/582] 45e932b0 - EventBgmCtrlTag
Updating actor list [542/582] 463a8d24 - NPC_DRCVoice
Updating actor list [543/582] 4637a91c - Animal_Insect_G
Updating actor list [544/582] 463a9a0c - Animal_Squirrel_A
Updating actor list [545/582] 463aa8a8 - Animal_Insect_G
Updating actor list [546/582] 463ab744 - Enemy_Keese
Updating actor list [547/582] 463ac5e0 - Animal_Insect_Q
Updating actor list [548/582] 46b8e738 - Enemy_Keese
Updating actor list [549/582] 4753f46c - Animal_LittleBird_A
Updating actor list [550/582] 4754e5d0 - Animal_Insect_Q
Updating actor list [551/582] 4755d760 - Enemy_Keese
Updating actor list [552/582] 4756c56c - Animal_LittleBird_A
Updating actor list [553/582] 4757b7cc - Animal_Hawk_A
Updating actor list [554/582] 475989bc - Animal_LittleBird_A
Updating actor list [555/582] 47599858 - Animal_Insect_Q
Updating actor list [556/582] 475ae2bc - Animal_Insect_Q
Updating actor list [557/582] 46089bb4 - Item_Fruit_L
Updating actor list [558/582] 475af158 - Enemy_Keese
Updating actor list [559/582] 475d3c38 - Enemy_Keese
Updating actor list [560/582] 475de070 - Animal_Insect_A
Updating actor list [561/582] 475f9c44 - Enemy_Keese
Updating actor list [562/582] 47603870 - Animal_Insect_G
Updating actor list [563/582] 4760d070 - Animal_Insect_A
Updating actor list [564/582] 47628544 - Animal_Insect_G
Updating actor list [565/582] 463ad47c - Item_Fruit_K
Updating actor list [566/582] 46b8f5d4 - Item_Fruit_K
Updating actor list [567/582] 47641d54 - Enemy_Keese
Updating actor list [568/582] 47642bf0 - Enemy_Keese
Updating actor list [569/582] 47643a8c - Animal_Insect_A
Updating actor list [570/582] 4764f03c - Enemy_Keese
Updating actor list [571/582] 4758accc - Item_Fruit_K
Updating actor list [572/582] 475a0634 - Item_Fruit_K
Updating actor list [573/582] 4765a93c - Animal_Insect_S
Updating actor list [574/582] 47945e98 - Animal_LittleBird_A
Updating actor list [575/582] 47961524 - Animal_Pigeon_A
Updating actor list [576/582] 475d4ad4 - Item_Fruit_L
Updating actor list [577/582] 47988ba0 - Animal_Insect_A
Updating actor list [578/582] 4799dc58 - Weapon_Sword_044
Updating actor list [579/582] 45ff9de4 - Weapon_Lsheath_056
Updating actor list [580/582] 479afebc - Animal_Insect_G
Updating actor list [581/582] 479bf858 - Animal_Boar_A
*/