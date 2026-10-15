#pragma once

namespace HeadVisibility {
    bool IsFaceBone(std::string_view boneName);
    bool IsFaceMaterialTriggerBone(std::string_view boneName);

    // Link's root bone is the first one of a frame, which is when the material list is rebuilt
    void OnSkeletonRoot();
    // returns one material per call through r9-r11 for the patch to show or hide
    void SubmitNextMaterial(PPCInterpreter_t* hCPU, bool hideHead);
}
