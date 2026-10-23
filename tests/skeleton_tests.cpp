#include "pch.h"
#include "../src/game/botw/player/link_skeleton.h"

static void Require(bool condition) {
    if (!condition) throw std::runtime_error("Skeleton regression check failed");
}

static void CheckFinite(BoneHierarchy& skeleton) {
    for (int i = 0; i < 4; i++)
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++)
                Require(std::isfinite(skeleton.GetBone(i)->worldMatrix[col][row]));
}

int main() {
    BoneHierarchy skeleton;
    skeleton.Parse("Root | 0 0 0 | 0 0 0\n  Shoulder | 0 0 0 | 0 0 0\n    Elbow | 1 0 0 | 0 0 0\n      Wrist | 1 0 0 | 0 0 0\n");
    skeleton.SolveTwoBoneIK(1, 2, 3, { 0, 0, 0 }, { 0, 1, 0 }, 1);
    CheckFinite(skeleton);
    skeleton.SolveTwoBoneIK(1, 2, 3, { 0, 1.5f, 0 }, { 0, 1, 0 }, 1);
    CheckFinite(skeleton);
    Require(glm::distance(glm::vec3(skeleton.GetBone(3)->worldMatrix[3]), glm::vec3(0, 1.5f, 0)) < 0.001f);
    skeleton.SolveTwoBoneIK(1, 2, 3, { std::numeric_limits<float>::quiet_NaN(), 0, 0 }, { 0, 1, 0 }, 1);
    CheckFinite(skeleton);
    skeleton.SolveTwoBoneIK(1, 2, 3, { 1, 1, 0 }, { 0, 0, 0 }, 1);
    CheckFinite(skeleton);
    std::cout << "PASS: zero target, parallel pole, NaN target, zero pole\n";
}
