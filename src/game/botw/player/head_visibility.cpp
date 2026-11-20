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
static constexpr uint32_t MaterialNameRelativeOffset = 0x04;
static constexpr uint32_t ModelUnitCountSanityMax = 256;
static constexpr int32_t MaterialNameMaxRelativeOffset = 0x04000000; // 64MB
// option 4 is the camera passes (the game clears it for uking_enable_hide_normal_pass), options 0/1 are the shadow
static constexpr uint16_t RenderViewOptionNormalPass = 1 << 4;
static constexpr uint32_t MaterialStateSelectableZPrePass = 0x8000;

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
struct HiddenMaterial {
    HeadMaterialRef ref;
    uint32_t hiddenViews;
    bool hiddenSelectableZPrePass;
};

static std::array<HiddenMaterial, 32> s_hiddenMaterials{};
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

static void MarkMaterialDirty(const HeadMaterialRef& material) {
    Guest::Set<uint8_t>(material.unitPtr + ModelUnitDirtyFlagsOffset, Guest::Get<uint8_t>(material.unitPtr + ModelUnitDirtyFlagsOffset).getLE() | 1);
    const uint32_t stateFlagsAddress = Guest::Get<uint32_t>(material.unitPtr + ModelUnitMaterialStatesOffset).getLE() + 12 * material.materialIdx + 6;
    Guest::Set<uint16_t>(stateFlagsAddress, Guest::Get<uint16_t>(stateFlagsAddress).getLE() | 2);
}

// same writes as the game's hide_normal_pass setup in sub_33EB704, the shadow options are left alone
static void HideFromNormalPass(HiddenMaterial& hidden) {
    const HeadMaterialRef& material = hidden.ref;
    const uint32_t viewOptions = Guest::Get<uint32_t>(material.unitPtr + ModelUnitViewOptionsOffset).getLE();
    const uint32_t materialStates = Guest::Get<uint32_t>(material.unitPtr + ModelUnitMaterialStatesOffset).getLE();
    const uint32_t viewNum = Guest::Get<uint8_t>(material.unitPtr + ModelUnitViewNumOffset).getLE();
    if (viewOptions == 0 || materialStates == 0 || viewNum > 32) return;

    bool changed = false;
    for (uint32_t view = 0; view < viewNum; view++) {
        const uint32_t optionsAddress = viewOptions + 12 * (material.materialIdx * viewNum + view) + 2;
        const uint16_t options = Guest::Get<uint16_t>(optionsAddress).getLE();
        if ((options & RenderViewOptionNormalPass) == 0) continue;
        Guest::Set<uint16_t>(optionsAddress, options & ~RenderViewOptionNormalPass);
        hidden.hiddenViews |= 1u << view;
        changed = true;
    }

    const uint32_t stateAddress = materialStates + 12 * material.materialIdx;
    const uint32_t state = Guest::Get<uint32_t>(stateAddress).getLE();
    if ((state & MaterialStateSelectableZPrePass) != 0) {
        Guest::Set<uint32_t>(stateAddress, state & ~MaterialStateSelectableZPrePass);
        hidden.hiddenSelectableZPrePass = true;
        changed = true;
    }

    if (changed)
        MarkMaterialDirty(material);
}

static void RestoreMaterial(const HiddenMaterial& hidden) {
    const HeadMaterialRef& material = hidden.ref;
    if (!IsLiveModelUnit(material.unitPtr)) return;
    const uint32_t viewOptions = Guest::Get<uint32_t>(material.unitPtr + ModelUnitViewOptionsOffset).getLE();
    const uint32_t materialStates = Guest::Get<uint32_t>(material.unitPtr + ModelUnitMaterialStatesOffset).getLE();
    const uint32_t viewNum = Guest::Get<uint8_t>(material.unitPtr + ModelUnitViewNumOffset).getLE();
    if (viewOptions == 0 || materialStates == 0 || viewNum > 32) return;

    for (uint32_t view = 0; view < viewNum; view++) {
        if ((hidden.hiddenViews & (1u << view)) == 0) continue;
        const uint32_t optionsAddress = viewOptions + 12 * (material.materialIdx * viewNum + view) + 2;
        Guest::Set<uint16_t>(optionsAddress, Guest::Get<uint16_t>(optionsAddress).getLE() | RenderViewOptionNormalPass);
    }

    if (hidden.hiddenSelectableZPrePass) {
        const uint32_t stateAddress = materialStates + 12 * material.materialIdx;
        Guest::Set<uint32_t>(stateAddress, Guest::Get<uint32_t>(stateAddress).getLE() | MaterialStateSelectableZPrePass);
    }

    MarkMaterialDirty(material);
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
    std::array<HiddenMaterial, 32> previouslyHidden = s_hiddenMaterials;
    const uint32_t previouslyHiddenCount = s_hiddenMaterialCount;
    for (uint32_t i = 0; i < previouslyHiddenCount; i++) {
        if (std::ranges::find(currentHead, previouslyHidden[i].ref) == currentHead.end())
            RestoreMaterial(previouslyHidden[i]);
    }

    s_hiddenMaterialCount = 0;
    for (const HeadMaterialRef& material : currentHead) {
        if (!IsLiveModelUnit(material.unitPtr)) continue;
        HiddenMaterial hidden = { material, 0, false };
        for (uint32_t i = 0; i < previouslyHiddenCount; i++) {
            if (previouslyHidden[i].ref == material)
                hidden = previouslyHidden[i];
        }
        HideFromNormalPass(hidden);
        s_hiddenMaterials[s_hiddenMaterialCount++] = hidden;
    }
}
