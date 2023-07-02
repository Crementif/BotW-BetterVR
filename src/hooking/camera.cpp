#include "cemu_hooks.h"
#include "instance.h"
#include "rendering/openxr.h"

#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/projection.hpp>
#include <glm/gtx/quaternion.hpp>


void CemuHooks::hook_UpdateProjectionMatrix(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    XrFovf viewFOV = VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentFOV();
    checkAssert(viewFOV.angleLeft <= viewFOV.angleRight, "OpenXR gave a left FOV that is larger than the right FOV! Behavior is unexpected!");
    checkAssert(viewFOV.angleDown <= viewFOV.angleUp, "OpenXR gave a top FOV that is larger than the bottom FOV! Behavior is unexpected!");

    // Convert the 4 OpenXR FOV values into a FoV, aspect ratio and projection center offset
    float totalHorizontalFov = viewFOV.angleRight - viewFOV.angleLeft;
    float totalVerticalFov = viewFOV.angleUp - viewFOV.angleDown;

    float aspectRatio = totalHorizontalFov / totalVerticalFov;
    float fovY = totalVerticalFov;
    float projectionCenter_offsetX = (viewFOV.angleRight + viewFOV.angleLeft) / 2.0f;
    float projectionCenter_offsetY = (viewFOV.angleUp + viewFOV.angleDown) / 2.0f;

    data_VRProjectionMatrixOut projectionMatrixOut = {
        .aspectRatio = aspectRatio,
        .fovY = fovY,
        .offsetX = projectionCenter_offsetX,
        .offsetY = projectionCenter_offsetY,
    };
    swapEndianness(projectionMatrixOut.aspectRatio);
    swapEndianness(projectionMatrixOut.fovY);
    swapEndianness(projectionMatrixOut.offsetX);
    swapEndianness(projectionMatrixOut.offsetY);
    uint32_t ppc_projectionMatrixOut = hCPU->gpr[28];
    writeMemory(ppc_projectionMatrixOut, &projectionMatrixOut);
}

// todo: for non-EAR versions it should use the same camera inputs for both eyes
void CemuHooks::hook_UpdateCamera(PPCInterpreter_t* hCPU) {
    //Log::print("Updated camera!");
    hCPU->instructionPointer = hCPU->gpr[7];

    // Read the camera matrix from the game's memory
    uint32_t ppc_cameraMatrixOffsetIn = hCPU->gpr[30];
    data_VRCameraIn origCameraMatrix = {};

    readMemory(ppc_cameraMatrixOffsetIn, &origCameraMatrix);
    swapEndianness(origCameraMatrix.posX);
    swapEndianness(origCameraMatrix.posY);
    swapEndianness(origCameraMatrix.posZ);
    swapEndianness(origCameraMatrix.targetX);
    swapEndianness(origCameraMatrix.targetY);
    swapEndianness(origCameraMatrix.targetZ);

    data_VRSettingsIn settings = VRManager::instance().Hooks->GetSettings();

    // Current VR headset camera matrix
    XrPosef currPose = VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentPose();

    glm::fvec3 currEyePos(currPose.position.x, currPose.position.y, currPose.position.z);
    glm::fquat currEyeQuat(currPose.orientation.w, currPose.orientation.x, currPose.orientation.y, currPose.orientation.z);
    //Log::print("Headset View: x={}, y={}, z={}, orientW={}, orientX={}, orientY={}, orientZ={}", currEyePos.x, currEyePos.y, currEyePos.z, currEyeQuat.w, currEyeQuat.x, currEyeQuat.y, currEyeQuat.z);

    // Current in-game camera matrix
    glm::fvec3 oldCameraPosition(origCameraMatrix.posX, origCameraMatrix.posY, origCameraMatrix.posZ);
    glm::fvec3 oldCameraTarget(origCameraMatrix.targetX, origCameraMatrix.targetY, origCameraMatrix.targetZ);
    float oldCameraDistance = glm::distance(oldCameraPosition, oldCameraTarget);
    //Log::print("Original Game Camera: x={}, y={}, z={}, targetX={}, targetY={}, targetZ={}", oldCameraPosition.x, oldCameraPosition.y, oldCameraPosition.z, oldCameraTarget.x, oldCameraTarget.y, oldCameraTarget.z);

    // Calculate game view directions
    glm::fvec3 forwardVector = glm::normalize(oldCameraTarget - oldCameraPosition);
    glm::fquat lookAtQuat = glm::quatLookAtRH(forwardVector, {0.0, 1.0, 0.0});

    // Calculate new view direction
    glm::fquat combinedQuat = glm::normalize(lookAtQuat * currEyeQuat);
    glm::fmat3 combinedMatrix = glm::toMat3(combinedQuat);

    // Rotate the headset position by the in-game rotation
    glm::fvec3 rotatedHmdPos = lookAtQuat * currEyePos;

    data_VRCameraOut updatedCameraMatrix = {
        .posX = oldCameraPosition.x + rotatedHmdPos.x,
        .posY = oldCameraPosition.y + rotatedHmdPos.y,
        .posZ = oldCameraPosition.z + rotatedHmdPos.z,
        // pos + rotated headset pos + inverted forward direction after combining both the in-game and HMD rotation
        .targetX = oldCameraPosition.x + rotatedHmdPos.x + ((combinedMatrix[2][0] * -1.0f) * oldCameraDistance),
        .targetY = oldCameraPosition.y + rotatedHmdPos.y + ((combinedMatrix[2][1] * -1.0f) * oldCameraDistance),
        .targetZ = oldCameraPosition.z + rotatedHmdPos.z + ((combinedMatrix[2][2] * -1.0f) * oldCameraDistance),
        .rotX = combinedMatrix[1][0],
        .rotY = combinedMatrix[1][1],
        .rotZ = combinedMatrix[1][2],
    };

    // Log::print("[{}] Rendering texture", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right");

    // Write the camera matrix to the game's memory
    // Log::print("[{}] New Game Camera: x={}, y={}, z={}, targetX={}, targetY={}, targetZ={}, rotX={}, rotY={}, rotZ={}", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", updatedCameraMatrix.posX, updatedCameraMatrix.posY, updatedCameraMatrix.posZ, updatedCameraMatrix.targetX, updatedCameraMatrix.targetY, updatedCameraMatrix.targetZ, updatedCameraMatrix.rotX, updatedCameraMatrix.rotY, updatedCameraMatrix.rotZ);
    swapEndianness(updatedCameraMatrix.posX);
    swapEndianness(updatedCameraMatrix.posY);
    swapEndianness(updatedCameraMatrix.posZ);
    swapEndianness(updatedCameraMatrix.targetX);
    swapEndianness(updatedCameraMatrix.targetY);
    swapEndianness(updatedCameraMatrix.targetZ);
    swapEndianness(updatedCameraMatrix.rotX);
    swapEndianness(updatedCameraMatrix.rotY);
    swapEndianness(updatedCameraMatrix.rotZ);
    uint32_t ppc_cameraMatrixOffsetOut = hCPU->gpr[31];
    writeMemory(ppc_cameraMatrixOffsetOut, &updatedCameraMatrix);
}