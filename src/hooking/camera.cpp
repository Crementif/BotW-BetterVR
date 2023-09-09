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


data_VRProjectionMatrixOut calculateProjectionMatrix(XrFovf viewFOV) {
    float totalHorizontalFov = viewFOV.angleRight - viewFOV.angleLeft;
    float totalVerticalFov = viewFOV.angleUp - viewFOV.angleDown;

    float aspectRatio = totalHorizontalFov / totalVerticalFov;
    float fovY = totalVerticalFov;
    float projectionCenter_offsetX = (viewFOV.angleRight + viewFOV.angleLeft) / 2.0f;
    float projectionCenter_offsetY = (viewFOV.angleUp + viewFOV.angleDown) / 2.0f;

    data_VRProjectionMatrixOut ret = {};
    ret.aspectRatio = aspectRatio;
    ret.fovY = fovY;
    ret.offsetX = projectionCenter_offsetX;
    ret.offsetY = projectionCenter_offsetY;
    return ret;
}

OpenXR::EyeSide s_currentEye = OpenXR::EyeSide::RIGHT;
std::pair<data_VRCameraRotationOut, OpenXR::EyeSide> s_currentCameraRotation = {};
data_VRProjectionMatrixOut s_currentProjectionMatrix = {};

// todo: for non-EAR versions it should use the same camera inputs for both eyes
void CemuHooks::hook_UpdateCameraPositionAndTarget(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    // Log::print("[{}] Updated camera position", s_currentEye == OpenXR::EyeSide::LEFT ? "left" : "right");

    // Read the camera matrix from the game's memory
    uint32_t ppc_cameraMatrixOffsetIn = hCPU->gpr[7];
    data_VRCameraIn origCameraMatrix = {};

    readMemory(ppc_cameraMatrixOffsetIn, &origCameraMatrix);

    // Current VR headset camera matrix
    XrPosef currPose = VRManager::instance().XR->GetRenderer()->m_layer3D.GetPose(s_currentEye);

    glm::fvec3 currEyePos(currPose.position.x, currPose.position.y, currPose.position.z);
    glm::fquat currEyeQuat(currPose.orientation.w, currPose.orientation.x, currPose.orientation.y, currPose.orientation.z);
    // Log::print("Headset View: x={}, y={}, z={}, orientW={}, orientX={}, orientY={}, orientZ={}", currEyePos.x, currEyePos.y, currEyePos.z, currEyeQuat.w, currEyeQuat.x, currEyeQuat.y, currEyeQuat.z);

    // Current in-game camera matrix
    glm::fvec3 oldCameraPosition(origCameraMatrix.posX, origCameraMatrix.posY, origCameraMatrix.posZ);
    glm::fvec3 oldCameraTarget(origCameraMatrix.targetX, origCameraMatrix.targetY, origCameraMatrix.targetZ);
    float oldCameraDistance = glm::distance(oldCameraPosition, oldCameraTarget);
    //Log::print("Original Game Camera: x={}, y={}, z={}, targetX={}, targetY={}, targetZ={}", oldCameraPosition.x, oldCameraPosition.y, oldCameraPosition.z, oldCameraTarget.x, oldCameraTarget.y, oldCameraTarget.z);

    // Calculate game view directions
    glm::fvec3 forwardVector = glm::normalize(oldCameraTarget - oldCameraPosition);
    glm::fquat lookAtQuat = glm::quatLookAtRH(forwardVector, { 0.0, 1.0, 0.0 });

    // Calculate new view direction
    glm::fquat combinedQuat = glm::normalize(lookAtQuat * currEyeQuat);
    glm::fmat3 combinedMatrix = glm::toMat3(combinedQuat);

    // Rotate the headset position by the in-game rotation
    glm::fvec3 rotatedHmdPos = lookAtQuat * currEyePos;

    data_VRCameraOut updatedCameraMatrix = {};
    updatedCameraMatrix.enabled = true;
    updatedCameraMatrix.posX = oldCameraPosition.x + rotatedHmdPos.x;
    updatedCameraMatrix.posY = oldCameraPosition.y + rotatedHmdPos.y;
    updatedCameraMatrix.posZ = oldCameraPosition.z + rotatedHmdPos.z;
    // pos + rotated headset pos + inverted forward direction after combining both the in-game and HMD rotation
    updatedCameraMatrix.targetX = oldCameraPosition.x + rotatedHmdPos.x + ((combinedMatrix[2][0] * -1.0f) * oldCameraDistance);
    updatedCameraMatrix.targetY = oldCameraPosition.y + rotatedHmdPos.y + ((combinedMatrix[2][1] * -1.0f) * oldCameraDistance);
    updatedCameraMatrix.targetZ = oldCameraPosition.z + rotatedHmdPos.z + ((combinedMatrix[2][2] * -1.0f) * oldCameraDistance);

    s_currentCameraRotation.first.rotY = combinedMatrix[1][1];
    s_currentCameraRotation.first.rotX = combinedMatrix[1][0];
    s_currentCameraRotation.first.rotZ = combinedMatrix[1][2];
    s_currentCameraRotation.second = s_currentEye;

    // Write the camera matrix to the game's memory
    // Log::print("[{}] New Game Camera: x={}, y={}, z={}, targetX={}, targetY={}, targetZ={}, rotX={}, rotY={}, rotZ={}", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", updatedCameraMatrix.posX, updatedCameraMatrix.posY, updatedCameraMatrix.posZ, updatedCameraMatrix.targetX, updatedCameraMatrix.targetY, updatedCameraMatrix.targetZ, updatedCameraMatrix.rotX, updatedCameraMatrix.rotY, updatedCameraMatrix.rotZ);
    uint32_t ppc_cameraMatrixOffsetOut = hCPU->gpr[3];
    writeMemory(ppc_cameraMatrixOffsetOut, &updatedCameraMatrix);

    s_framesSinceLastCameraUpdate = 0;
}

void CemuHooks::hook_UpdateCameraRotation(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    // Log::print("[{}] Updated camera rotation", s_currentCameraRotation.second == OpenXR::EyeSide::LEFT ? "left" : "right");

    data_VRCameraRotationOut updatedCameraMatrix = {};
    updatedCameraMatrix.enabled = true;
    updatedCameraMatrix.rotX = s_currentCameraRotation.first.rotX;
    updatedCameraMatrix.rotY = s_currentCameraRotation.first.rotY;
    updatedCameraMatrix.rotZ = s_currentCameraRotation.first.rotZ;

    uint32_t ppc_cameraMatrixOffsetOut = hCPU->gpr[3];
    writeMemory(ppc_cameraMatrixOffsetOut, &updatedCameraMatrix);
}


void CemuHooks::hook_UpdateCameraOffset(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    // Log::print("[{}] Updated camera FOV and projection offset", s_currentEye == OpenXR::EyeSide::LEFT ? "left" : "right");

    XrFovf viewFOV = VRManager::instance().XR->GetRenderer()->m_layer3D.GetFOV(s_currentEye);
    checkAssert(viewFOV.angleLeft <= viewFOV.angleRight, "OpenXR gave a left FOV that is larger than the right FOV! Behavior is unexpected!");
    checkAssert(viewFOV.angleDown <= viewFOV.angleUp, "OpenXR gave a top FOV that is larger than the bottom FOV! Behavior is unexpected!");

    data_VRProjectionMatrixOut projectionMatrix = calculateProjectionMatrix(viewFOV);

    data_VRCameraOffsetOut cameraOffsetOut = {
        .aspectRatio = projectionMatrix.aspectRatio,
        .fovY = projectionMatrix.fovY,
        .offsetX = projectionMatrix.offsetX,
        .offsetY = projectionMatrix.offsetY
    };
    uint32_t ppc_projectionMatrixOut = hCPU->gpr[11];
    writeMemory(ppc_projectionMatrixOut, &cameraOffsetOut);
}


void CemuHooks::hook_CalculateCameraAspectRatio(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    // Log::print("[{}] Updated camera aspect ratio", s_currentEye == OpenXR::EyeSide::LEFT ? "left" : "right");

    XrFovf viewFOV = VRManager::instance().XR->GetRenderer()->m_layer3D.GetFOV(s_currentEye);
    checkAssert(viewFOV.angleLeft <= viewFOV.angleRight, "OpenXR gave a left FOV that is larger than the right FOV! Behavior is unexpected!");
    checkAssert(viewFOV.angleDown <= viewFOV.angleUp, "OpenXR gave a top FOV that is larger than the bottom FOV! Behavior is unexpected!");

    data_VRProjectionMatrixOut projectionMatrix = calculateProjectionMatrix(viewFOV);

    data_VRCameraAspectRatioOut cameraOffsetOut = {
        .aspectRatio = projectionMatrix.aspectRatio,
        .fovY = projectionMatrix.fovY
    };
    uint32_t ppc_projectionMatrixOut = hCPU->gpr[28];
    writeMemory(ppc_projectionMatrixOut, &cameraOffsetOut);
}
