#pragma once

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

class BoneHierarchy {
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

        glm::mat4 parentWorld = glm::identity<glm::mat4>();
        if (rootBone.parentIndex != -1) {
            parentWorld = m_bones[rootBone.parentIndex].worldMatrix;
        }

        glm::vec3 rootPos = glm::vec3(parentWorld * glm::vec4(rootBone.localPos, 1.0f));

        float l1_orig = glm::length(midBone.localPos);
        float l2_orig = glm::length(endBone.localPos);
        float l1 = l1_orig;
        float l2 = l2_orig;
        float totalLength = l1 + l2;

        glm::vec3 dir = targetPos - rootPos;
        float dist = glm::length(dir);
        if (!std::isfinite(dist) || dist < 0.000001f || !std::isfinite(l1) || !std::isfinite(l2) || l1 < 0.000001f || l2 < 0.000001f)
            return;

        // weighted stretch: upper arm absorbs more of the extra distance
        float epsilon = 0.001f;
        if (dist > totalLength - epsilon) {
            float extra = dist - totalLength + epsilon;
            l1 += extra * upperArmStretchBias;
            l2 += extra * (1.0f - upperArmStretchBias);
        }

        dist = glm::clamp(dist, epsilon, l1 + l2 - epsilon);

        // law of cosines for angle at shoulder (alpha)
        float cosAlpha = (l1 * l1 + dist * dist - l2 * l2) / (2 * l1 * dist);
        float alpha = glm::acos(glm::clamp(cosAlpha, -1.0f, 1.0f));

        glm::vec3 dirNorm = glm::normalize(dir);
        glm::vec3 planeNormal = glm::cross(dirNorm, poleVector);
        if (!std::isfinite(glm::length2(planeNormal)))
            return;
        if (glm::length2(planeNormal) < 0.000001f) {
            // the target is in line with the elbow hint, so bend any way
            const glm::vec3 fallbackPole = std::abs(dirNorm.y) < 0.9f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
            planeNormal = glm::cross(dirNorm, fallbackPole);
        }
        planeNormal = glm::normalize(planeNormal);
        glm::vec3 ortho = glm::normalize(glm::cross(planeNormal, dirNorm));

        glm::vec3 arm1Dir = glm::normalize(dirNorm * std::cos(alpha) + ortho * std::sin(alpha));

        glm::vec3 elbowPos = rootPos + arm1Dir * l1;
        glm::vec3 arm2Dir = glm::normalize(targetPos - elbowPos);

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

    void SetLocalMatrix(int index, const glm::mat4& matrix) {
        Bone* bone = GetBone(index);
        if (bone == nullptr) return;
        bone->localMatrix = matrix;
        UpdateWorldMatrices();
    }

    void SetLocalMatrix(const std::string& name, const glm::mat4& matrix) {
        SetLocalMatrix(GetBoneIndex(name), matrix);
    }

private:
    std::vector<Bone> m_bones;
    std::map<std::string, int> m_boneNameMap;
};

namespace LinkSkeleton {
    extern const std::string RestPose;
    const glm::fquat& HandCorrection(uint32_t side);
}
