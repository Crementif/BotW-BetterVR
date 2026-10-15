#include "pch.h"
#include "head_visibility.h"
#include "player.h"
#include "cemu/cemu_host.h"

bool HeadVisibility::IsFaceBone(std::string_view boneName) {
    if (boneName.starts_with("Eye" /*lid*/) || boneName.starts_with("Cheek") || boneName.starts_with("Lip") || boneName.starts_with("Hair") || boneName.starts_with("Ponytail") || boneName.starts_with("Momi")) {
        return true;
    }
    if (boneName == "Nose" || boneName == "Face_Root" || boneName == "Neck" || boneName == "Head" || boneName.starts_with("Teeth_") || boneName.starts_with("Chin")) {
        return true;
    }
    return false;
}

bool HeadVisibility::IsFaceMaterialTriggerBone(std::string_view boneName) {
    return boneName == "Face_Root" || boneName == "Head" || boneName == "Eyeball_L" || boneName == "Eyeball_R";
}

static constexpr uint32_t ActorGsysModelOffset = 0x330;
static constexpr uint32_t ModelUnitCountOffset = 0x1C;
static constexpr uint32_t ModelUnitArrayOffset = 0x24;
static constexpr uint32_t ModelUnitMaterialCountOffset = 0x9A;
static constexpr uint32_t ModelUnitMaterialArrayOffset = 0xA4;
static constexpr uint32_t ModelUnitMaterialStride = 0x40;
static constexpr uint32_t ModelUnitVtableOffset = 0x74;
static constexpr uint32_t ModelUnitVtableAddress = 0x1034AB98;
static constexpr uint32_t MaterialNameRelativeOffset = 0x04;
static constexpr uint32_t ModelUnitCountSanityMax = 256;
static constexpr int32_t MaterialNameMaxRelativeOffset = 0x04000000; // 64MB

static std::string_view GetModelUnitMaterialName(uint32_t materialArray, uint16_t materialIdx) {
    uint32_t materialPtr = 0;
    Guest::ReadBE(materialArray + materialIdx * ModelUnitMaterialStride, &materialPtr);
    if (!materialPtr) return {};
    uint32_t nameOffset = 0;
    Guest::ReadBE(materialPtr + MaterialNameRelativeOffset, &nameOffset);
    if (nameOffset == 0) return {};
    if ((int32_t)nameOffset > MaterialNameMaxRelativeOffset || (int32_t)nameOffset < -MaterialNameMaxRelativeOffset) return {};
    const uint32_t nameAddress = materialPtr + MaterialNameRelativeOffset + nameOffset;
    const char* materialName = Guest::CString(nameAddress);
    return std::string_view(materialName, strnlen(materialName, 63));
}

struct HeadMaterialRef {
    uint32_t unitPtr;
    uint16_t materialIdx;
};

static std::array<HeadMaterialRef, 32> s_playerHeadMaterials{};
static uint32_t s_playerHeadMaterialCount = 0;
static uint32_t s_playerHeadMaterialCursor = 0;
static bool s_playerHeadMaterialsRefreshedThisFrame = false;

void HeadVisibility::OnSkeletonRoot() {
    s_playerHeadMaterialsRefreshedThisFrame = false;
}

static void CollectHeadMaterialsFromModel(uint32_t gsysModelPtr) {
    uint32_t unitCount = 0;
    uint32_t unitArray = 0;
    Guest::ReadBE(gsysModelPtr + ModelUnitCountOffset, &unitCount);
    Guest::ReadBE(gsysModelPtr + ModelUnitArrayOffset, &unitArray);
    if (unitArray == 0) return;
    if (unitCount > ModelUnitCountSanityMax) return;

    for (uint32_t unitIdx = 0; unitIdx < unitCount; unitIdx++) {
        uint32_t unitHolder = 0;
        Guest::ReadBE(unitArray + unitIdx * 4, &unitHolder);
        if (unitHolder == 0) continue;
        uint32_t unitPtr = 0;
        Guest::ReadBE(unitHolder, &unitPtr);
        if (unitPtr == 0) continue;

        uint16_t materialCount = 0;
        uint32_t materialArray = 0;
        Guest::ReadBE(unitPtr + ModelUnitMaterialCountOffset, &materialCount);
        Guest::ReadBE(unitPtr + ModelUnitMaterialArrayOffset, &materialArray);
        if (materialArray == 0) continue;

        for (uint16_t materialIdx = 0; materialIdx < materialCount; materialIdx++) {
            if (GetModelUnitMaterialName(materialArray, materialIdx).empty()) continue;
            if (s_playerHeadMaterialCount >= s_playerHeadMaterials.size()) return;
            s_playerHeadMaterials[s_playerHeadMaterialCount++] = { unitPtr, materialIdx };
        }
    }
}

static void CollectHeadMaterialsFromActor(uint32_t actorPtr) {
    uint32_t gsysModelPtr = 0;
    Guest::ReadBE(actorPtr + ActorGsysModelOffset, &gsysModelPtr);
    if (gsysModelPtr == 0) return;
    CollectHeadMaterialsFromModel(gsysModelPtr);
}

static uint32_t ResolveBaseProcLinkActor(uint32_t baseProcLinkAddress) {
    BaseProcLink link = {};
    Guest::Read(baseProcLinkAddress, &link);
    const uint32_t linkDataPtr = link.baseProcLinkData.getLE();
    if (linkDataPtr == 0) return 0;

    BaseProcLinkData linkData = {};
    Guest::Read(linkDataPtr, &linkData);
    if (linkData.id.getLE() != link.id.getLE()) return 0;
    return linkData.actor.getLE();
}

static void RefreshPlayerHeadMaterials() {
    s_playerHeadMaterialCount = 0;

    const uint32_t playerActorPtr = Player::GetAddress();
    if (playerActorPtr == 0) return;

    const uint32_t headArmorLinkAddress = playerActorPtr + offsetof(PlayerActor, armors) + offsetof(PlayerArmors, armorHead);
    const uint32_t headArmorActorPtr = ResolveBaseProcLinkActor(headArmorLinkAddress);
    if (headArmorActorPtr == 0) return;
    CollectHeadMaterialsFromActor(headArmorActorPtr);
}

static bool IsLiveModelUnit(uint32_t unitPtr) {
    uint32_t vtablePtr = 0;
    Guest::ReadBE(unitPtr + ModelUnitVtableOffset, &vtablePtr);
    return vtablePtr == ModelUnitVtableAddress;
}

void HeadVisibility::SubmitNextMaterial(PPCInterpreter_t* hCPU, bool hideHead) {
    if (!s_playerHeadMaterialsRefreshedThisFrame) {
        RefreshPlayerHeadMaterials();
        s_playerHeadMaterialsRefreshedThisFrame = true;
    }
    if (s_playerHeadMaterialCount == 0) return;

    const HeadMaterialRef& headMaterial = s_playerHeadMaterials[s_playerHeadMaterialCursor++ % s_playerHeadMaterialCount];
    if (!IsLiveModelUnit(headMaterial.unitPtr)) {
        s_playerHeadMaterialCount = 0;
        return;
    }

    hCPU->gpr[9] = headMaterial.unitPtr;
    hCPU->gpr[10] = headMaterial.materialIdx;
    hCPU->gpr[11] = hideHead ? 0 : 1;
}
