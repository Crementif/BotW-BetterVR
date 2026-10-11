#include "pch.h"

#include "instance.h"
#include "cemu_hooks.h"
#include "rendering/openxr.h"
#include "utils/debug_draw.h"

struct Bone {
    std::string name;
    glm::vec3 localPos;
    glm::vec3 localRotEuler; // in radians
    glm::mat4 localMatrix;
    glm::mat4 worldMatrix;
    int parentIndex = -1;
    std::vector<int> childrenIndices;
    int indentLevel = 0;
};

class Skeleton {
public:
    void Parse(const std::string& data) {
        m_bones.clear();
        m_boneNameMap.clear();

        std::stringstream ss(data);
        std::string line;

        // this parses the bone hierarchy using the indentation levels to calculate parent-child relationships and model space transforms
        std::vector<std::pair<int, int>> parentStack;
        parentStack.push_back({ -1, -1 }); // Root parent is -1

        while (std::getline(ss, line)) {
            if (line.empty()) continue;

            int indent = 0;
            while (indent < line.length() && line[indent] == ' ') indent++;

            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos) continue;

            std::string content = line.substr(start);
            size_t p1 = content.find('|');
            if (p1 == std::string::npos) continue;

            std::string currentName = content.substr(0, p1);
            size_t lastChar = currentName.find_last_not_of(' ');
            if (lastChar != std::string::npos) currentName = currentName.substr(0, lastChar + 1);

            size_t p2 = content.find('|', p1 + 1);
            if (p2 == std::string::npos) continue;

            glm::vec3 pos, rotEuler;
            std::stringstream ssPos(content.substr(p1 + 1, p2 - p1 - 1));
            ssPos >> pos.x >> pos.y >> pos.z;
            std::stringstream ssRot(content.substr(p2 + 1));
            ssRot >> rotEuler.x >> rotEuler.y >> rotEuler.z;

            Bone bone;
            bone.name = currentName;
            bone.localPos = pos;
            bone.localRotEuler = rotEuler;
            bone.indentLevel = indent;
            bone.localMatrix = glm::translate(glm::identity<glm::mat4>(), pos) * glm::eulerAngleZYX(rotEuler.x, rotEuler.y, rotEuler.z);

            while (parentStack.size() > 1 && parentStack.back().first >= indent) {
                parentStack.pop_back();
            }

            bone.parentIndex = parentStack.back().second;

            int newIndex = (int)m_bones.size();
            if (bone.parentIndex != -1) {
                m_bones[bone.parentIndex].childrenIndices.push_back(newIndex);
            }

            m_bones.push_back(bone);
            m_boneNameMap[bone.name] = newIndex;

            parentStack.push_back({ indent, newIndex });
        }

        UpdateWorldMatrices();
    }

    void UpdateWorldMatrices() {
        for (auto& bone : m_bones) {
            if (bone.parentIndex == -1) {
                bone.worldMatrix = bone.localMatrix;
            }
            else {
                bone.worldMatrix = m_bones[bone.parentIndex].worldMatrix * bone.localMatrix;
            }
        }
    }

    glm::mat4 CalculateLocalMatrixFromWorld(int boneIndex, const glm::mat4& targetWorldMatrix) {
        if (boneIndex < 0 || boneIndex >= m_bones.size()) return glm::identity<glm::mat4>();

        const Bone& bone = m_bones[boneIndex];
        if (bone.parentIndex == -1) {
            return targetWorldMatrix;
        }

        const glm::mat4& parentWorldMatrix = m_bones[bone.parentIndex].worldMatrix;
        return glm::inverse(parentWorldMatrix) * targetWorldMatrix;
    }

    void SolveTwoBoneIK(int rootIdx, int midIdx, int endIdx, const glm::vec3& targetPos, const glm::vec3& poleVector, float boneForwardSign, float upperArmStretchBias = 0.75f) {
        if (rootIdx < 0 || rootIdx >= m_bones.size() ||
            midIdx < 0 || midIdx >= m_bones.size() ||
            endIdx < 0 || endIdx >= m_bones.size()) {
            return;
        }

        Bone& rootBone = m_bones[rootIdx];
        Bone& midBone = m_bones[midIdx];
        Bone& endBone = m_bones[endIdx];

        // get parent world matrix (clavicle)
        glm::mat4 parentWorld = glm::identity<glm::mat4>();
        if (rootBone.parentIndex != -1) {
            parentWorld = m_bones[rootBone.parentIndex].worldMatrix;
        }

        glm::vec3 rootPos = glm::vec3(parentWorld * glm::vec4(rootBone.localPos, 1.0f));

        // get lengths
        float l1_orig = glm::length(midBone.localPos);
        float l2_orig = glm::length(endBone.localPos);
        float l1 = l1_orig;
        float l2 = l2_orig;
        float totalLength = l1 + l2;

        // solve IK
        glm::vec3 dir = targetPos - rootPos;
        float dist = glm::length(dir);

        // weighted stretch: upper arm absorbs more of the extra distance
        float epsilon = 0.001f;
        if (dist > totalLength - epsilon) {
            float extra = dist - totalLength + epsilon;
            l1 += extra * upperArmStretchBias;
            l2 += extra * (1.0f - upperArmStretchBias);
        }

        // clamp distance
        dist = glm::clamp(dist, epsilon, l1 + l2 - epsilon);

        // law of cosines for angle at shoulder (alpha)
        float cosAlpha = (l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist);
        float alpha = glm::acos(glm::clamp(cosAlpha, -1.0f, 1.0f));

        // plane construction
        glm::vec3 dirNorm = glm::normalize(dir);
        glm::vec3 planeNormal = glm::normalize(glm::cross(dirNorm, poleVector));
        glm::vec3 ortho = glm::normalize(glm::cross(planeNormal, dirNorm));

        // arm 1 direction (world)
        glm::vec3 arm1Dir = glm::normalize(dirNorm * cos(alpha) + ortho * sin(alpha));

        // arm 2 direction (world)
        glm::vec3 elbowPos = rootPos + arm1Dir * l1;
        glm::vec3 arm2Dir = glm::normalize(targetPos - elbowPos);

        // construct rotation matrices
        glm::vec3 x1 = arm1Dir * boneForwardSign;
        glm::vec3 z1 = planeNormal;
        glm::vec3 y1 = glm::cross(z1, x1);
        glm::mat3 rot1World = glm::mat3(x1, -y1, -z1);

        glm::vec3 x2 = arm2Dir * boneForwardSign;
        glm::vec3 z2 = planeNormal;
        glm::vec3 y2 = glm::cross(z2, x2);
        glm::mat3 rot2World = glm::mat3(x2, -y2, -z2);

        // convert to local space, stretching the upper arm position more
        glm::mat4 arm1Local = glm::inverse(parentWorld) * glm::mat4(rot1World);
        arm1Local[3] = glm::vec4(rootBone.localPos, 1.0f);

        glm::mat4 arm1World = parentWorld * arm1Local;
        glm::mat4 arm2Local = glm::inverse(arm1World) * glm::mat4(rot2World);
        arm2Local[3] = glm::vec4(midBone.localPos * (l1 / l1_orig), 1.0f);

        // update skeleton
        rootBone.localMatrix = arm1Local;
        midBone.localMatrix = arm2Local;
        UpdateWorldMatrices();
    }

    int GetBoneIndex(const std::string& name) const {
        auto it = m_boneNameMap.find(name);
        if (it != m_boneNameMap.end()) return it->second;
        return -1;
    }

    Bone* GetBone(int index) {
        if (index < 0 || index >= m_bones.size()) return nullptr;
        return &m_bones[index];
    }

    Bone* GetBone(const std::string& name) {
        int idx = GetBoneIndex(name);
        if (idx == -1) return nullptr;
        return &m_bones[idx];
    }

private:
    std::vector<Bone> m_bones;
    std::map<std::string, int> m_boneNameMap;
};

const std::string SKELETON_DATA = R"(
Root | 0 0 0 | 0 0 0
  Skl_Root | 0 0.99426 0 | 0 0 0
    Spine_1 | 0 0 0 | 1.5708 0 1.5708
      Spine_2 | 0.136 0 0 | 0 0 0
        Clavicle_L | 0.23961 -0.00002 0.03291 | 0 -1.5708 0
          Arm_1_L | 0.15 0 0.01074 | 0 0 0
            Arm_1_Assist_L | 0.06 0.00002 0 | 0 0 0
            Arm_2_L | 0.24 0 0 | 0 0 0
              Elbow_L | 0.04151 -0.02934 0.00021 | 0 0 0
              Wrist_Assist_L | 0.25809 0.00002 -0.00012 | 0 0 0
              Wrist_L | 0.27718 0 0 | 0 0 0
                Weapon_L | 0.1069 0.00002 0.02769 | 1.5708 0 3.14159
          Clavicle_Assist_L | 0.116 0 0.0107 | 0 0 0
        Clavicle_R | 0.2396 -0.00002 -0.03291 | 3.14159 -1.5708 0
          Arm_1_R | -0.15 0 -0.01074 | 0 0 0
            Arm_1_Assist_R | -0.06 -0.00002 0 | 0 0 0
            Arm_2_R | -0.24 0 0 | 0 0 0
              Elbow_R | -0.04151 0.02934 -0.0002 | 0 0 0
              Wrist_Assist_R | -0.25809 -0.00002 0.00012 | 0 0 0
              Wrist_R | -0.27718 0 0 | 0 0 0
                Weapon_R | -0.1069 -0.00002 -0.02769 | 1.5708 0 0
          Clavicle_Assist_R | -0.116 0 -0.0107 | 0 0 0
        Neck | 0.26326 0 0 | 0 0 0
          Head | 0.12447 0 0 | 0 0 0
            Face_Root | 0 0 0 | 0 0 0
              Chin | 0.04787 0.05757 0 | 0 0 2.53073
              Eyeball_L | 0.07017 0.12036 0.04815 | 0 0 0
              Eyeball_R | 0.07017 0.12036 -0.04815 | 0 0 0
)";

/*
    Waist | 0 0 0 | 1.5708 0 -1.5708
      Leg_1_L | 0.10854 0.0165 -0.11209 | 0 0 0
        Knee_L | 0.39619 0.0308 0 | 0 0 0
        Leg_2_L | 0.42 0 -0.08727 | 0 0 0
      Leg_1_R | 0.10854 0.0165 0.11209 | 0 0 3.14159
        Knee_R | -0.39619 -0.0308 0 | 0 0 0
        Leg_2_R | -0.42 0 -0.08727 | 0 0 0
 */

static bool isFaceBone(const std::string_view& boneName) {
    if (boneName.starts_with("Eye" /*lid*/) || boneName.starts_with("Cheek") || boneName.starts_with("Lip") || boneName.starts_with("Hair") || boneName.starts_with("Ponytail") || boneName.starts_with("Momi")) {
        return true;
    }
    if (boneName == "Nose" || boneName == "Face_Root" || boneName == "Neck" || boneName == "Head" || boneName.starts_with("Teeth_") || boneName.starts_with("Chin")) {
        return true;
    }
    return false;
}

static bool isFaceMaterialTriggerBone(const std::string_view& boneName) {
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
    CemuHooks::readMemoryBE(materialArray + materialIdx * ModelUnitMaterialStride, &materialPtr);
    if (!materialPtr) return {};
    uint32_t nameOffset = 0;
    CemuHooks::readMemoryBE(materialPtr + MaterialNameRelativeOffset, &nameOffset);
    if (nameOffset == 0) return {};
    if ((int32_t)nameOffset > MaterialNameMaxRelativeOffset || (int32_t)nameOffset < -MaterialNameMaxRelativeOffset) return {};
    const uint32_t nameAddress = materialPtr + MaterialNameRelativeOffset + nameOffset;
    const char* materialName = (const char*)(CemuHooks::s_memoryBaseAddress + nameAddress);
    return std::string_view(materialName, strnlen(materialName, 63));
}


struct HeadMaterialRef {
    uint32_t unitPtr;
    uint16_t materialIdx;
};

static std::array<HeadMaterialRef, 32> s_playerHeadMaterials{};
static uint32_t s_playerHeadMaterialCount = 0;
static uint32_t s_playerHeadMaterialCursor = 0;
// Reset once per frame in the "Skl_Root" branch of hook_ModifyBoneMatrix
static bool s_playerHeadMaterialsRefreshedThisFrame = false;

static void CollectHeadMaterialsFromModel(uint32_t gsysModelPtr) {
    uint32_t unitCount = 0;
    uint32_t unitArray = 0;
    CemuHooks::readMemoryBE(gsysModelPtr + ModelUnitCountOffset, &unitCount);
    CemuHooks::readMemoryBE(gsysModelPtr + ModelUnitArrayOffset, &unitArray);
    if (unitArray == 0) return;
    if (unitCount > ModelUnitCountSanityMax) return;

    for (uint32_t unitIdx = 0; unitIdx < unitCount; unitIdx++) {
        uint32_t unitHolder = 0;
        CemuHooks::readMemoryBE(unitArray + unitIdx * 4, &unitHolder);
        if (unitHolder == 0) continue;
        uint32_t unitPtr = 0;
        CemuHooks::readMemoryBE(unitHolder, &unitPtr);
        if (unitPtr == 0) continue;

        uint16_t materialCount = 0;
        uint32_t materialArray = 0;
        CemuHooks::readMemoryBE(unitPtr + ModelUnitMaterialCountOffset, &materialCount);
        CemuHooks::readMemoryBE(unitPtr + ModelUnitMaterialArrayOffset, &materialArray);
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
    CemuHooks::readMemoryBE(actorPtr + ActorGsysModelOffset, &gsysModelPtr);
    if (gsysModelPtr == 0) return;
    CollectHeadMaterialsFromModel(gsysModelPtr);
}

static uint32_t ResolveBaseProcLinkActor(uint32_t baseProcLinkAddress) {
    BaseProcLink link = {};
    CemuHooks::readMemory(baseProcLinkAddress, &link);
    const uint32_t linkDataPtr = link.baseProcLinkData.getLE();
    if (linkDataPtr == 0) return 0;

    BaseProcLinkData linkData = {};
    CemuHooks::readMemory(linkDataPtr, &linkData);
    if (linkData.id.getLE() != link.id.getLE()) return 0;
    return linkData.actor.getLE();
}

static void RefreshPlayerHeadMaterials() {
    s_playerHeadMaterialCount = 0;

    const uint32_t playerActorPtr = CemuHooks::s_playerAddress;
    if (playerActorPtr == 0) return;

    const uint32_t headArmorLinkAddress = playerActorPtr + offsetof(Player, armors) + offsetof(PlayerArmors, armorHead);
    const uint32_t headArmorActorPtr = ResolveBaseProcLinkActor(headArmorLinkAddress);
    if (headArmorActorPtr == 0) return;
    CollectHeadMaterialsFromActor(headArmorActorPtr);
}

static bool IsLiveModelUnit(uint32_t unitPtr) {
    uint32_t vtablePtr = 0;
    CemuHooks::readMemoryBE(unitPtr + ModelUnitVtableOffset, &vtablePtr);
    return vtablePtr == ModelUnitVtableAddress;
}

static void SubmitHeadMaterialVisibility(PPCInterpreter_t* hCPU, bool hideHead) {
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

static Skeleton s_skeleton;
static bool s_skeletonParsed = false;
static glm::vec3 s_manualBodyOffset = glm::vec3(0.0f, 0.0f, -0.075f);
static glm::mat4 s_handCorrectionRotationLeft = glm::mat4(1.0f);
static glm::mat4 s_handCorrectionRotationRight = glm::mat4(1.0f);
static glm::fquat s_rightToLeftCorrectedHandRot = glm::identity<glm::fquat>();
static constexpr float MaximumGameHandSeparationSq = 4.0f;
static constexpr std::array<std::array<const char*, 5>, 2> BowDrawArmChainBoneNames = { {
    { "Clavicle_L", "Arm_1_L", "Arm_2_L", "Wrist_L", "Weapon_L" },
    { "Clavicle_R", "Arm_1_R", "Arm_2_R", "Wrist_R", "Weapon_R" }
} };
static constexpr uint32_t BowDrawArmChainRequiredMask = 0b01110;
static std::array<std::array<glm::mat4, 5>, 2> s_gameAnimArmChainLocals = {};
static std::array<uint32_t, 2> s_gameAnimArmChainSeenMasks = { 0, 0 };
static glm::mat4 s_gameLeftToRightWeaponMtx = glm::identity<glm::mat4>();
static bool s_gameLeftToRightWeaponMtxValid = false;

static bool IsFiniteMatrix(const glm::mat4& matrix) {
    for (int column = 0; column < 4; column++) {
        for (int row = 0; row < 4; row++) {
            if (!std::isfinite(matrix[column][row]))
                return false;
        }
    }
    return true;
}

static void UpdateGameBowDrawRelativeTransform() {
    if (!s_skeletonParsed)
        return;

    std::array<glm::mat4, 2> gameWeaponMtxs = { glm::identity<glm::mat4>(), glm::identity<glm::mat4>() };
    for (uint32_t sideIdx = 0; sideIdx < 2; sideIdx++) {
        if ((s_gameAnimArmChainSeenMasks[sideIdx] & BowDrawArmChainRequiredMask) != BowDrawArmChainRequiredMask)
            return;

        for (uint32_t chainIdx = 0; chainIdx < BowDrawArmChainBoneNames[sideIdx].size(); chainIdx++) {
            if (s_gameAnimArmChainSeenMasks[sideIdx] & (1u << chainIdx)) {
                gameWeaponMtxs[sideIdx] = gameWeaponMtxs[sideIdx] * s_gameAnimArmChainLocals[sideIdx][chainIdx];
            }
            else {
                Bone* restBone = s_skeleton.GetBone(BowDrawArmChainBoneNames[sideIdx][chainIdx]);
                if (restBone == nullptr)
                    return;
                gameWeaponMtxs[sideIdx] = gameWeaponMtxs[sideIdx] * restBone->localMatrix;
            }
        }
    }

    // Preserve the game's right-hand placement in the left attachment bone's coordinate frame.
    const glm::mat4 relativeMtx = glm::inverse(gameWeaponMtxs[OpenXR::EyeSide::LEFT]) * gameWeaponMtxs[OpenXR::EyeSide::RIGHT];
    if (!IsFiniteMatrix(relativeMtx) || glm::length2(glm::vec3(relativeMtx[3])) > MaximumGameHandSeparationSq)
        return;

    s_gameLeftToRightWeaponMtx = relativeMtx;
    s_gameLeftToRightWeaponMtxValid = true;
}

static constexpr float BowFullDrawPullDistance = 0.30f;
static constexpr float BowSnapMaxEngageDistance = 0.35f;
static bool s_bowSnapTriggerHeld = false;
static bool s_bowSnapEngaged = false;
static bool s_arrowNearBowStart = false;

struct BowPullState {
    bool valid = false;
    float engageSeparation = 0.0f;
    float ratio = 0.0f;
};
static BowPullState s_bowPull;

bool CemuHooks::IsBowDrawEngaged() {
    return s_bowSnapEngaged;
}

bool CemuHooks::IsArrowNearBowStart() {
    return IsFirstPerson() && s_arrowNearBowStart;
}

static float GetBowDrawSnapWeight() {
    if (!s_bowSnapEngaged || !s_bowPull.valid)
        return 0.0f;

    const float weight = s_bowPull.ratio;
    return weight * weight * (3.0f - 2.0f * weight);
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

static glm::fvec3 RemoveHeadsetHorizontalOffset(glm::fvec3 trackedPos) {
    glm::fvec3 appliedHeadPos = CemuHooks::GetAppliedRoomscaleHeadPosition();
    trackedPos.x -= appliedHeadPos.x;
    trackedPos.z -= appliedHeadPos.z;
    return trackedPos;
}

// handle off-hand grab for two-handed weapons
static constexpr uint32_t TWO_HAND_WEAPON_ACTOR_MTX_OFFSET = 0x1F8;
static constexpr float TWO_HAND_MIN_HAND_SEPARATION = 0.05f; // numerical floor for the grip axis (rearToLeft/sep); off-hand reach is otherwise gated by the weapon AABB, not by distance from the primary hand
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

// blade-axis solver state
static glm::fquat s_lastAppliedRightWristRot = glm::identity<glm::fquat>();
static bool s_lastAppliedRightWristRotValid = false;
static uint32_t s_bladeAxisWeaponPtr = 0;
static glm::fvec3 s_bladeAxisInHand = glm::fvec3(0.0f);
static bool s_bladeAxisValid = false;
static glm::fvec3 s_lastGripAxisWorld = glm::fvec3(0.0f);

// grab latch state
static bool s_twoHandGripButtonHeld = false;
static bool s_twoHandGripLatched = false;
static glm::fvec3 s_latchedGripOffsetInRightHand = glm::fvec3(0.0f);
static float s_latchedBladeDirectionSign = 1.0f;

static glm::fquat MinimalArcQuat(const glm::fvec3& from, const glm::fvec3& to) {
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

static void DebugDrawWeaponAxes(uint32_t weaponPtr) {
    if (weaponPtr == 0)
        return;

    BEMatrix34 weaponMtxBE = {};
    CemuHooks::readMemory(weaponPtr + TWO_HAND_WEAPON_ACTOR_MTX_OFFSET, &weaponMtxBE);

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
    CemuHooks::readMemory(weaponPtr + offsetof(ActorWiiU, mtx), &weaponMtxBE);
    const glm::vec3 weaponPos = glm::vec3(weaponMtxBE.getLEMatrix()[3]);
    const glm::fquat weaponRot = weaponMtxBE.getRotLE();

    const glm::vec3 aabbMin = CemuHooks::getMemory<BEVec3>(weaponPtr + offsetof(ActorWiiU, aabb.min)).getLE();
    const glm::vec3 aabbMax = CemuHooks::getMemory<BEVec3>(weaponPtr + offsetof(ActorWiiU, aabb.max)).getLE();
    if (!IsAllFinite(aabbMin) || !IsAllFinite(aabbMax) || !IsAllFinite(weaponPos))
        return false;

    // ignore degenerate bounding boxes
    if (glm::all(glm::lessThanEqual(aabbMax - aabbMin, glm::vec3(0.0f))))
        return false;

    const glm::vec3 localPoint = glm::inverse(weaponRot) * (worldPoint - weaponPos);
    const glm::vec3 margin = glm::vec3(TWO_HAND_GRAB_AABB_MARGIN);
    return glm::all(glm::greaterThanEqual(localPoint, aabbMin - margin)) && glm::all(glm::lessThanEqual(localPoint, aabbMax + margin));
}

void CemuHooks::hook_ModifyBoneMatrix(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    hCPU->gpr[7] = 0;
    hCPU->gpr[8] = 0;
    hCPU->gpr[9] = 0;

    const uint32_t gsysModelPtr = hCPU->gpr[3];
    const uint32_t matrixPtr = hCPU->gpr[4];
    const uint32_t scalePtr = hCPU->gpr[5];
    const uint32_t boneNamePtr = hCPU->gpr[6];
    if (!gsysModelPtr || !matrixPtr || !scalePtr || !boneNamePtr) return;

    const auto modelName = getMemory<sead::FixedSafeString100>(gsysModelPtr + 0x128);
    if (modelName.getLE() != "GameROMPlayer") return;

    std::string boneName((char*)(s_memoryBaseAddress + boneNamePtr));
    const bool isLeft = boneName.ends_with("_L");
    const OpenXR::EyeSide side = isLeft ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;
    // capture the game's animated arm pose before it gets overwritten below
    for (uint32_t chainIdx = 0; chainIdx < BowDrawArmChainBoneNames[side].size(); chainIdx++) {
        if (boneName == BowDrawArmChainBoneNames[side][chainIdx]) {
            s_gameAnimArmChainLocals[side][chainIdx] = glm::mat4(getMemory<BEMatrix34>(matrixPtr).getLEMatrix());
            s_gameAnimArmChainSeenMasks[side] |= 1u << chainIdx;
            break;
        }
    }

    // reset grip state on root to prevent stranding on tracking loss
    if (boneName == "Skl_Root") {
        UpdateGameBowDrawRelativeTransform();
        s_twoHandGrip = {};
        CemuHooks::s_twoHandGripActive = false;
        s_playerHeadMaterialsRefreshedThisFrame = false;
    }

    // helpers to write back matrix and scale
    auto writeBoneMatrix = [&](const glm::mat4x3& mtx, const glm::fvec3& scale) {
        BEMatrix34 m{ mtx };
        setMemory(matrixPtr, m);
        setMemory(scalePtr, scale);
    };
    auto writeBoneQuat = [&](const glm::vec3& pos, const glm::quat& rot, const glm::fvec3& scale) {
        BEMatrix34 m{ pos, rot };
        setMemory(matrixPtr, m);
        setMemory(scalePtr, scale);
    };

    const bool isThirdPerson = IsThirdPerson();

    // one material per bone, so a whole head model converges well within a single frame
    SubmitHeadMaterialVisibility(hCPU, !isThirdPerson);

    if (isThirdPerson) {
        // wrist is game-animated in third person
        s_lastAppliedRightWristRotValid = false;
        if (isFaceMaterialTriggerBone(boneName)) {
            hCPU->gpr[7] = 1;
            hCPU->gpr[8] = 0;
        }
        if (isFaceBone(boneName)) {
            setMemory(scalePtr, glm::fvec3(1.0f));
        }
        return;
    }

    if (isFaceMaterialTriggerBone(boneName)) {
        hCPU->gpr[7] = 1;
        hCPU->gpr[8] = 1;
    }

    if (isFaceBone(boneName)) {
        return;
    }

    glm::fvec3 boneScale = getMemory<BEVec3>(scalePtr).getLE();
    const glm::fmat4 playerMtx4 = glm::fmat4(getMemory<BEMatrix34>(s_playerMtxAddress).getLEMatrix());
    const glm::mat4 cameraMtx = GetFreshCameraReferenceMtx();

    const OpenXR::InputState inputs = VRManager::instance().XR->m_input.load();
    if (!inputs.shared.pose[side].isActive) {
        if (boneName == "Skl_Root")
            s_lastAppliedRightWristRotValid = false;
        return;
    }

    // one-time skeleton and hand correction initialization
    if (!s_skeletonParsed) {
        s_skeleton.Parse(SKELETON_DATA);
        s_skeletonParsed = true;

        // left: 90 Y -> -90 Z -> 30 Z
        glm::fquat wristL = glm::identity<glm::fquat>();
        wristL *= glm::angleAxis(glm::radians(90.0f), glm::fvec3(0, 1, 0));
        wristL *= glm::angleAxis(glm::radians(-90.0f), glm::fvec3(0, 0, 1));
        wristL *= glm::angleAxis(glm::radians(30.0f), glm::fvec3(0, 0, 1));

        // right: -90 Z -> -180 Y -> 270 X -> 30 Z tweak
        glm::fquat wristR = glm::identity<glm::fquat>();
        wristR *= glm::angleAxis(glm::radians(-90.0f), glm::fvec3(0, 0, 1));
        wristR *= glm::angleAxis(glm::radians(-180.0f), glm::fvec3(0, 1, 0));
        wristR *= glm::angleAxis(glm::radians(270.0f), glm::fvec3(1, 0, 0));
        wristR *= glm::angleAxis(glm::radians(30.0f), glm::fvec3(0, 0, 1));

        s_handCorrectionRotationLeft = glm::mat4_cast(wristL);
        s_handCorrectionRotationRight = glm::mat4_cast(wristR);
        s_rightToLeftCorrectedHandRot = glm::inverse(wristL) * wristR;
    }

    int boneIndex = s_skeleton.GetBoneIndex(boneName);
    if (boneIndex == -1)
        return;

    // compute a controller target in model space
    auto calcControllerTargetModelForSide = [&](OpenXR::EyeSide handSide) -> glm::mat4 {
        const auto& handPose = inputs.shared.poseLocation[handSide];
        glm::fvec3 controllerPos = glm::fvec3();
        glm::fquat controllerRot = glm::identity<glm::fquat>();
        if (handPose.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
            controllerPos = RemoveHeadsetHorizontalOffset(ToGLM(handPose.pose.position));
        if (handPose.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
            controllerRot = ToGLM(handPose.pose.orientation);

        const bool targetIsLeft = handSide == OpenXR::EyeSide::LEFT;
        glm::mat4 handCorrectionMtx = targetIsLeft ? s_handCorrectionRotationLeft : s_handCorrectionRotationRight;
        glm::mat4 controllerMat = glm::translate(glm::identity<glm::mat4>(), controllerPos) * glm::mat4_cast(controllerRot) * handCorrectionMtx;
        glm::mat4 targetWorld = cameraMtx * controllerMat;

        const char* weaponName = targetIsLeft ? "Weapon_L" : "Weapon_R";
        if (Bone* weapon = s_skeleton.GetBone(weaponName)) {
            glm::vec3 weaponOffset = glm::vec3(weapon->localMatrix[3]);
            targetWorld = targetWorld * glm::translate(glm::identity<glm::mat4>(), -weaponOffset);
        }

        return glm::inverse(playerMtx4) * targetWorld;
    };

    auto calcControllerTargetModel = [&]() -> glm::mat4 {
        return calcControllerTargetModelForSide(side);
    };

    auto calcTwoHandGripTargetModel = [&]() -> glm::mat4 {
        const glm::vec3 gripOrigin = isLeft ? s_twoHandGrip.leftPinnedWorld : s_twoHandGrip.rearOriginWorld;
        const glm::fquat gripRot = isLeft ? s_twoHandGrip.leftGripRotWorld : s_twoHandGrip.rightGripRotWorld;
        glm::mat4 targetWorld = glm::translate(glm::identity<glm::mat4>(), gripOrigin) * glm::mat4_cast(gripRot);

        const char* weaponName = isLeft ? "Weapon_L" : "Weapon_R";
        if (Bone* weapon = s_skeleton.GetBone(weaponName)) {
            glm::vec3 weaponOffset = glm::vec3(weapon->localMatrix[3]);
            targetWorld = targetWorld * glm::translate(glm::identity<glm::mat4>(), -weaponOffset);
        }

        return glm::inverse(playerMtx4) * targetWorld;
    };

    auto calcBowDrawTargetModel = [&]() -> std::optional<glm::mat4> {
        if (!s_arrowNockedInRightHand || s_handWeaponTypes[OpenXR::EyeSide::LEFT] != WeaponType::Bow || !s_gameLeftToRightWeaponMtxValid || !inputs.shared.pose[OpenXR::EyeSide::LEFT].isActive)
            return std::nullopt;

        Bone* leftWeapon = s_skeleton.GetBone("Weapon_L");
        Bone* rightWeapon = s_skeleton.GetBone("Weapon_R");
        if (leftWeapon == nullptr || rightWeapon == nullptr)
            return std::nullopt;

        const glm::mat4 leftWristTargetMtx = calcControllerTargetModelForSide(OpenXR::EyeSide::LEFT);
        const glm::mat4 leftWeaponTargetMtx = leftWristTargetMtx * leftWeapon->localMatrix;
        // Re-anchor the original hand-to-hand transform at the VR-controlled left hand.
        const glm::mat4 rightWeaponTargetMtx = leftWeaponTargetMtx * s_gameLeftToRightWeaponMtx;
        return rightWeaponTargetMtx * glm::inverse(rightWeapon->localMatrix);
    };

    auto calcBowRightWristTargetModel = [&]() -> std::optional<glm::mat4> {
        if (isLeft)
            return std::nullopt;

        const float bowSnapWeight = GetBowDrawSnapWeight();
        if (bowSnapWeight <= 0.0f)
            return std::nullopt;

        const std::optional<glm::mat4> bowTargetMtx = calcBowDrawTargetModel();
        if (!bowTargetMtx.has_value())
            return std::nullopt;

        if (bowSnapWeight >= 1.0f)
            return bowTargetMtx;

        return BlendTransforms(calcControllerTargetModel(), bowTargetMtx.value(), bowSnapWeight);
    };

    auto calcHandTargetModel = [&]() -> glm::mat4 {
        if (const std::optional<glm::mat4> bowTargetMtx = calcBowRightWristTargetModel(); bowTargetMtx.has_value())
            return bowTargetMtx.value();
        return s_twoHandGrip.engaged ? calcTwoHandGripTargetModel() : calcControllerTargetModel();
    };

    glm::mat4 calculatedLocalMat = s_skeleton.GetBone(boneIndex)->localMatrix;

    // override the root transform so the body aligns with the headset yaw
    if (boneName == "Skl_Root") {
        auto headsetPose = VRManager::instance().XR->GetMiddlePose();
        glm::mat4 headsetMtx = headsetPose.value_or(ToMat4(glm::fvec3(0)));

        // calculate eye offset from eyeball bones (once)
        static glm::vec3 eyeOffset = glm::vec3(0.0f);
        static bool offsetCalculated = false;
        if (!offsetCalculated) {
            Bone* eyeL = s_skeleton.GetBone("Eyeball_L");
            Bone* eyeR = s_skeleton.GetBone("Eyeball_R");
            Bone* sklRoot = s_skeleton.GetBone("Skl_Root");
            if (eyeL && eyeR && sklRoot) {
                glm::vec3 eyePos = (glm::vec3(eyeL->worldMatrix[3]) + glm::vec3(eyeR->worldMatrix[3])) * 0.5f;
                eyeOffset = eyePos - glm::vec3(sklRoot->worldMatrix[3]);
                offsetCalculated = true;
            }
        }

        // headset in model space
        glm::mat4 headsetModel = glm::inverse(playerMtx4) * cameraMtx * headsetMtx;
        headsetModel[3].x = 0.0f;
        headsetModel[3].z = 0.0f;

        glm::fquat yawRot = ExtractPlayerBodyYaw(headsetModel);

        // position the root so that yawRot * eyeOffset lands at the headset position
        glm::vec3 targetPos = glm::vec3(headsetModel[3]) - (yawRot * eyeOffset) + (yawRot * s_manualBodyOffset);

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
                handPos = RemoveHeadsetHorizontalOffset(ToGLM(handPoseLocation.pose.position));
            if (handPoseLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
                handRot = ToGLM(handPoseLocation.pose.orientation);
            const glm::mat4 handCorrectionMtx = handSide == OpenXR::EyeSide::LEFT ? s_handCorrectionRotationLeft : s_handCorrectionRotationRight;
            return cameraMtx * glm::translate(glm::identity<glm::mat4>(), handPos) * glm::mat4_cast(handRot) * handCorrectionMtx;
        };

        const glm::mat4 rightHandWorld = calcHandWorldMat(OpenXR::EyeSide::RIGHT);
        const glm::fquat rightHandRotWorld = glm::quat_cast(glm::mat3(rightHandWorld));

        if (GetSettings().ShouldShowWeaponAxes()) {
            DebugDrawWeaponAxes(m_heldWeapons[OpenXR::EyeSide::LEFT]);
            DebugDrawWeaponAxes(m_heldWeapons[OpenXR::EyeSide::RIGHT]);

            const glm::vec3 rightGripPos = glm::vec3(rightHandWorld[3]);
            const glm::vec3 leftGripPos = glm::vec3(calcHandWorldMat(OpenXR::EyeSide::LEFT)[3]);
            DebugDraw::instance().Line(rightGripPos, leftGripPos, DebugDrawColor(0, 220, 255), 2.0f, true);
        }

        // measure blade direction in corrected hand frame
        const uint32_t rightWeaponPtr = m_heldWeapons[OpenXR::EyeSide::RIGHT];
        if (s_twoHandGripEnabled && rightWeaponPtr != 0 && s_lastAppliedRightWristRotValid) {
            BEMatrix34 weaponMtxBE = {};
            readMemory(rightWeaponPtr + TWO_HAND_WEAPON_ACTOR_MTX_OFFSET, &weaponMtxBE);

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

        // handle off-hand weapon grab latching
        const bool leftGripHeld = s_twoHandGripEnabled && IsTwoHandGripEngaged() && inputs.inGame.grab[EyeSide::LEFT].currentState > TwoHandGripButtonThreshold;
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

                // align wrists to grip axis and follow primary hand
                const glm::fvec3 leftBladeInHand = s_rightToLeftCorrectedHandRot * s_bladeAxisInHand;
                const glm::fquat solvedRightGripRotWorld = MinimalArcQuat(currentBladeWorld, targetBladeDir) * rightHandRotWorld;
                s_twoHandGrip.engaged = true;
                s_twoHandGrip.rearOriginWorld = rearPos;
                s_twoHandGrip.leftPinnedWorld = rearPos + solvedRightGripRotWorld * s_latchedGripOffsetInRightHand;
                s_twoHandGrip.rightGripRotWorld = solvedRightGripRotWorld;
                s_twoHandGrip.leftGripRotWorld = MinimalArcQuat(leftHandRotWorld * leftBladeInHand, targetBladeDir) * leftHandRotWorld;
            }
        }

        CemuHooks::s_twoHandGripActive = s_twoHandGrip.engaged;

        // the trigger starts the draw, pulling the arrow hand away from the bow hand charges it
        const bool bowTriggerHeld = inputs.inGame.useRightItem.currentState;
        const bool bowInLeftHand = s_handWeaponTypes[EyeSide::LEFT] == WeaponType::Bow;
        const float bowHandSeparation = glm::distance(rearPos, leftPos);

        const std::optional<glm::mat4> bowStartTargetMtx = calcBowDrawTargetModel();
        const glm::vec3 controllerHandPos = glm::vec3(calcControllerTargetModelForSide(EyeSide::RIGHT)[3]);
        s_arrowNearBowStart = bowStartTargetMtx.has_value() &&
            (inputs.shared.poseLocation[OpenXR::EyeSide::RIGHT].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
            glm::distance(glm::vec3(bowStartTargetMtx.value()[3]), controllerHandPos) <= BowSnapMaxEngageDistance;

        if (bowTriggerHeld && !s_bowSnapTriggerHeld) {
            s_bowSnapEngaged = s_arrowNearBowStart;
            s_bowPull = { bowInLeftHand, bowHandSeparation, 0.0f };
        }
        else if (!bowTriggerHeld) {
            s_bowSnapEngaged = false;
        }
        else if (s_bowPull.valid) {
            s_bowPull.ratio = glm::clamp((bowHandSeparation - s_bowPull.engageSeparation) / BowFullDrawPullDistance, 0.0f, 1.0f);
        }

        if (!bowInLeftHand) {
            s_bowPull.valid = false;
            s_arrowNearBowStart = false;
        }

        s_bowSnapTriggerHeld = bowTriggerHeld;

        s_lastAppliedRightWristRot = s_twoHandGrip.engaged ? s_twoHandGrip.rightGripRotWorld : rightHandRotWorld;
        s_lastAppliedRightWristRotValid = true;

        writeBoneQuat(targetPos, yawRot, boneScale);
        return;
    }

    // solve upper arm IK so the arms reach their active controller, grip, or bow target
    if (boneName == "Arm_1_L" || boneName == "Arm_1_R" || boneName == "Elbow_L" || boneName == "Elbow_R" || boneName == "Wrist_Assist_L" || boneName == "Wrist_Assist_R") {

        int arm1Index = s_skeleton.GetBoneIndex(isLeft ? "Arm_1_L" : "Arm_1_R");
        int arm2Index = s_skeleton.GetBoneIndex(isLeft ? "Arm_2_L" : "Arm_2_R");
        int wristIndex = s_skeleton.GetBoneIndex(isLeft ? "Wrist_L" : "Wrist_R");

        if (arm1Index != -1 && arm2Index != -1 && wristIndex != -1) {
            glm::vec3 targetPos = glm::vec3(calcHandTargetModel()[3]);

            // pole vector (elbow hint): left-down-back / right-down-back, rotated by body yaw
            glm::vec3 poleDir = isLeft ? glm::vec3(1.0f, -1.0f, -0.5f) : glm::vec3(-1.0f, -1.0f, -0.5f);
            if (Bone* rootBone = s_skeleton.GetBone("Skl_Root"))
                poleDir = glm::quat_cast(rootBone->localMatrix) * poleDir;

            s_skeleton.SolveTwoBoneIK(arm1Index, arm2Index, wristIndex, targetPos, poleDir, isLeft ? 1.0f : -1.0f);
            calculatedLocalMat = s_skeleton.GetBone(boneIndex)->localMatrix;
        }
    }

    // align wrists with the same target used by the arm solver
    if (boneName == "Wrist_L" || boneName == "Wrist_R" || boneName == "Wrist_Assist_L" || boneName == "Wrist_Assist_R") {
        calculatedLocalMat = s_skeleton.CalculateLocalMatrixFromWorld(boneIndex, calcHandTargetModel());
    }

    writeBoneMatrix(glm::mat4x3(calculatedLocalMat), boneScale);
}
