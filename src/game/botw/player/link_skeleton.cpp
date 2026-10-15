#include "pch.h"
#include "link_skeleton.h"

const std::string LinkSkeleton::RestPose = R"(
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

static std::array<glm::fquat, 2> ComputeHandCorrections() {
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

    return { wristL, wristR };
}

const glm::fquat& LinkSkeleton::HandCorrection(uint32_t side) {
    static const std::array<glm::fquat, 2> s_corrections = ComputeHandCorrections();
    return s_corrections[side];
}
