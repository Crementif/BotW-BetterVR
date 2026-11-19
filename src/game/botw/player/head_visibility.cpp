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

static constexpr uint32_t ActorGsysModelOffset = 0x330;
static constexpr uint32_t ModelUnitCountOffset = 0x1C;
static constexpr uint32_t ModelUnitArrayOffset = 0x24;
static constexpr uint32_t ModelUnitMaterialCountOffset = 0x9A;
static constexpr uint32_t ModelUnitMaterialArrayOffset = 0xA4;
static constexpr uint32_t ModelUnitMaterialStride = 0x40;
static constexpr uint32_t ModelUnitVtableOffset = 0x74;
static constexpr uint32_t ModelUnitVtableAddress = 0x1034AB98;
static constexpr uint32_t ModelUnitDirtyFlagsOffset = 0x00;
static constexpr uint32_t ModelUnitViewOptionsOffset = 0x3C;
static constexpr uint32_t ModelUnitMaterialStatesOffset = 0x44;
static constexpr uint32_t ModelUnitViewNumOffset = 0x90;
static constexpr uint32_t MaterialResDataOffset = 0x48;
static constexpr uint32_t MaterialResRenderFlagsOffset = 0xC4;
static constexpr uint32_t MaterialNameRelativeOffset = 0x04;
static constexpr uint32_t ModelUnitCountSanityMax = 256;
static constexpr int32_t MaterialNameMaxRelativeOffset = 0x04000000; // 64MB
static constexpr uint16_t RenderViewOptionMainPass = 1 << 0;

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

    bool operator==(const HeadMaterialRef&) const = default;
};

static std::array<HeadMaterialRef, 32> s_playerHeadMaterials{};
static uint32_t s_playerHeadMaterialCount = 0;
static std::array<HeadMaterialRef, 32> s_hiddenMaterials{};
static uint32_t s_hiddenMaterialCount = 0;

static void CollectHeadMaterialsFromModel(uint32_t gsysModelPtr, bool headgear) {
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
            const std::string_view name = GetModelUnitMaterialName(materialArray, materialIdx);
            if (name.empty()) continue;
            if (!headgear && name != "Mt_Eyeball_L" && name != "Mt_Eyeball_R" && name != "Mt_Eyelashes" && name != "Mt_Face" && name != "Mt_Head") continue;
            if (s_playerHeadMaterialCount >= s_playerHeadMaterials.size()) return;
            s_playerHeadMaterials[s_playerHeadMaterialCount++] = { unitPtr, materialIdx };
        }
    }
}

static void CollectHeadMaterialsFromActor(uint32_t actorPtr) {
    uint32_t gsysModelPtr = 0;
    Guest::ReadBE(actorPtr + ActorGsysModelOffset, &gsysModelPtr);
    if (gsysModelPtr == 0) return;
    CollectHeadMaterialsFromModel(gsysModelPtr, true);
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

    const uint32_t playerModel = Guest::Get<uint32_t>(playerActorPtr + ActorGsysModelOffset).getLE();
    if (playerModel != 0)
        CollectHeadMaterialsFromModel(playerModel, false);

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

static bool IsMainPassEnabledByDefault(const HeadMaterialRef& material) {
    const uint32_t materialArray = Guest::Get<uint32_t>(material.unitPtr + ModelUnitMaterialArrayOffset).getLE();
    if (materialArray == 0) return true;
    const uint32_t materialPtr = Guest::Get<uint32_t>(materialArray + material.materialIdx * ModelUnitMaterialStride).getLE();
    if (materialPtr == 0) return true;
    const uint32_t materialResData = Guest::Get<uint32_t>(materialPtr + MaterialResDataOffset).getLE();
    if (materialResData == 0) return true;
    return (Guest::Get<uint32_t>(materialResData + MaterialResRenderFlagsOffset).getLE() & 1) != 0;
}

// same writes as gsys::ModelUnit's render view option setter at 0x039E4934, option 1 (shadow) is left alone
static void SetMainPassEnabled(const HeadMaterialRef& material, bool enabled) {
    const uint32_t viewOptions = Guest::Get<uint32_t>(material.unitPtr + ModelUnitViewOptionsOffset).getLE();
    if (viewOptions == 0) return;
    const uint8_t viewNum = Guest::Get<uint8_t>(material.unitPtr + ModelUnitViewNumOffset).getLE();

    bool changed = false;
    for (uint32_t view = 0; view < viewNum; view++) {
        const uint32_t optionsAddress = viewOptions + 12 * (material.materialIdx * viewNum + view) + 2;
        const uint16_t options = Guest::Get<uint16_t>(optionsAddress).getLE();
        const uint16_t newOptions = enabled ? (options | RenderViewOptionMainPass) : (options & ~RenderViewOptionMainPass);
        if (newOptions == options) continue;
        Guest::Set<uint16_t>(optionsAddress, newOptions);
        changed = true;
    }
    if (!changed) return;

    Guest::Set<uint8_t>(material.unitPtr + ModelUnitDirtyFlagsOffset, Guest::Get<uint8_t>(material.unitPtr + ModelUnitDirtyFlagsOffset).getLE() | 1);
    const uint32_t materialStates = Guest::Get<uint32_t>(material.unitPtr + ModelUnitMaterialStatesOffset).getLE();
    if (materialStates == 0) return;
    const uint32_t stateFlagsAddress = materialStates + 12 * material.materialIdx + 6;
    Guest::Set<uint16_t>(stateFlagsAddress, Guest::Get<uint16_t>(stateFlagsAddress).getLE() | 2);
}

static void RestoreMaterial(const HeadMaterialRef& material) {
    if (IsLiveModelUnit(material.unitPtr) && IsMainPassEnabledByDefault(material))
        SetMainPassEnabled(material, true);
}

void HeadVisibility::Apply(bool hideHead) {
    if (!hideHead) {
        for (uint32_t i = 0; i < s_hiddenMaterialCount; i++)
            RestoreMaterial(s_hiddenMaterials[i]);
        s_hiddenMaterialCount = 0;
        return;
    }

    RefreshPlayerHeadMaterials();
    const auto currentHead = std::span(s_playerHeadMaterials.data(), s_playerHeadMaterialCount);
    for (uint32_t i = 0; i < s_hiddenMaterialCount; i++) {
        if (std::ranges::find(currentHead, s_hiddenMaterials[i]) == currentHead.end())
            RestoreMaterial(s_hiddenMaterials[i]);
    }

    s_hiddenMaterialCount = 0;
    for (const HeadMaterialRef& material : currentHead) {
        if (!IsLiveModelUnit(material.unitPtr)) continue;
        SetMainPassEnabled(material, false);
        s_hiddenMaterials[s_hiddenMaterialCount++] = material;
    }
}
