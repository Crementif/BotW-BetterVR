#include "cemu_hooks.h"
#include "instance.h"
#include "rendering/openxr.h"


void CemuHooks::hook_BeginCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[0] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    Log::print<RENDERING>("");
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);

    RND_Renderer* renderer = VRManager::instance().XR->GetRenderer();

    bool layersInitialized = renderer->m_layer3D && renderer->m_layer2D && renderer->m_imguiOverlay;
    if (layersInitialized && side == OpenXR::EyeSide::LEFT) {
        VRManager::instance().XR->GetRenderer()->StartFrame();
    }
}

static std::pair<glm::quat, glm::quat> swingTwistY(const glm::quat& q) {
    glm::vec3 yAxis(0, 1, 0);
    glm::vec3 r(q.x, q.y, q.z);
    float dot = glm::dot(r, yAxis);
    glm::vec3 proj = yAxis * dot;
    glm::quat twist = glm::normalize(glm::quat(q.w, proj.x, proj.y, proj.z));
    glm::quat swing = q * glm::conjugate(twist);
    return { swing, twist };
}

constexpr float hardcodedSwimOffset = 1.73f/* model height*/ - 0.3f /*head height*/;

glm::fvec3 s_wsCameraPosition = glm::fvec3();
glm::fquat s_wsCameraRotation = glm::identity<glm::fquat>();
bool s_isSwimming = false;
uint32_t s_isLadderClimbing = 0;

void CemuHooks::hook_UpdateCameraForGameplay(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return;
    }

    // read the camera matrix from the game's memory
    uint32_t ppc_cameraMatrixOffsetIn = hCPU->gpr[31];
    OpenXR::EyeSide side = hCPU->gpr[3] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;
    ActCamera actCam = {};
    readMemory(ppc_cameraMatrixOffsetIn, &actCam);

    // extract components from the existing camera matrix
    glm::fvec3 oldCameraPosition = actCam.finalCamMtx.pos.getLE();
    glm::fvec3 oldCameraTarget = actCam.finalCamMtx.target.getLE();
    glm::fvec3 oldCameraForward = glm::normalize(oldCameraTarget - oldCameraPosition);
    glm::fvec3 oldCameraUp = actCam.finalCamMtx.up.getLE();
    glm::fvec3 oldCameraUnknown = actCam.finalCamMtx.unknown.getLE();
    float extraValue0 = actCam.finalCamMtx.zNear.getLE();
    float extraValue1 = actCam.finalCamMtx.zFar.getLE();

    Log::print<RENDERING>("[{}] Getting gameplay camera (pos = {})", side, oldCameraPosition);

    // remove verticality from the camera position to avoid pitch changes that aren't from the VR headset
    oldCameraPosition.y = oldCameraTarget.y;

    // construct glm matrix from the existing camera parameters
    glm::mat4 existingGameMtx = glm::lookAtRH(oldCameraPosition, oldCameraTarget, oldCameraUp);

    // pass real camera position and rotation to be used elsewhere
    // GetRenderCamera recalculates the left and right eye positions from the base camera position
    s_wsCameraPosition = oldCameraPosition;
    s_wsCameraRotation = glm::quat_cast(glm::inverse(existingGameMtx));

    // rebase the rotation to the player position
    if (IsFirstPerson()) {
        // check if player is swimming
        Player actor;
        readMemory(s_playerAddress, &actor);

        PlayerMoveBitFlags moveBits = actor.moveBitFlags.getLE();
        s_isSwimming = (std::to_underlying(moveBits) & std::to_underlying(PlayerMoveBitFlags::SWIMMING_1024)) != 0;

        //Log::print<INFO>("{:08X}", std::to_underlying(moveBits));

        // read player MTX
        BEMatrix34& mtx = actor.mtx;
        glm::fvec3 playerPos = actor.mtx.getPos().getLE();

        playerPos.y += s_isSwimming ? hardcodedSwimOffset : 0.0f;

        if (auto settings = GetFirstPersonSettingsForActiveEvent()) {
            if (settings->ignoreCameraRotation) {
                glm::fquat playerRot = mtx.getRotLE();
                auto [swing, baseYaw] = swingTwistY(playerRot);
                s_wsCameraRotation = baseYaw * glm::angleAxis(glm::radians(180.0f), glm::fvec3(0.0f, 1.0f, 0.0f));
            }
        }

        if (s_isLadderClimbing > 0) {
            s_isLadderClimbing--;
        }

        //if (s_isLadderClimbing) {
        //    s_wsCameraRotation *= glm::angleAxis(glm::radians(180.0f), glm::fvec3(0, 1, 0));
        //}

        glm::mat4 playerMtx4 = glm::inverse(glm::translate(glm::identity<glm::mat4>(), playerPos) * glm::mat4(s_wsCameraRotation));
        existingGameMtx = playerMtx4;
    }

    // current VR headset camera matrix
    auto viewsOpt = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (!viewsOpt) {
        Log::print<ERROR>("hook_UpdateCameraForGameplay: No views available for the middle pose.");
        return;
    }
    auto& views = viewsOpt.value();

    // calculate final camera matrix
    glm::mat4 finalPose = glm::inverse(existingGameMtx) * views;

    // extract camera up, forward and position from the final matrix
    glm::fvec3 camPos = glm::fvec3(finalPose[3]);
    glm::fvec3 forward = -glm::normalize(glm::fvec3(finalPose[2]));
    glm::fvec3 up = glm::normalize(glm::fvec3(finalPose[1]));

    float oldCameraDistance = glm::distance(oldCameraPosition, oldCameraTarget);
    glm::fvec3 target = camPos + forward * oldCameraDistance;

    actCam.finalCamMtx.pos = camPos;
    actCam.finalCamMtx.target = target;
    actCam.finalCamMtx.up = up;
    //actCam.finalCamMtx.up = glm::fvec3(0.0f, 1.0f, 0.0f);

    // write back the modified camera matrix to the game's memory
    uint32_t ppc_cameraMatrixOffsetOut = hCPU->gpr[31];
    writeMemory(ppc_cameraMatrixOffsetOut, &actCam);
    s_framesSinceLastCameraUpdate = 0;
}

glm::mat4 CemuHooks::s_lastCameraMtx = glm::mat4(1.0f);

void CemuHooks::hook_GetRenderCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t cameraIn = hCPU->gpr[3];
    uint32_t cameraOut = hCPU->gpr[12];
    OpenXR::EyeSide side = hCPU->gpr[11] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return;
    }

    BESeadLookAtCamera camera = {};
    readMemory(cameraIn, &camera);

    Log::print<RENDERING>("[{}] Getting render camera", side);

    //s_lastCameraMtx = glm::fmat4x3(glm::inverse(glm::mat4(camera.mtx.getLEMatrix()))); // glm::inverse(glm::lookAtRH(camera.pos.getLE(), camera.at.getLE(), camera.up.getLE()));

    // in-game camera
    glm::mat4x3 viewMatrix = camera.mtx.getLEMatrix();
    glm::mat4 worldGame = glm::inverse(glm::mat4(viewMatrix));
    glm::vec3 basePos = glm::vec3(worldGame[3]);
    glm::quat baseRot = glm::quat_cast(worldGame);

    // overwrite with our stored camera pos/rot
    basePos = s_wsCameraPosition;
    baseRot = s_wsCameraRotation;
    auto [swing, baseYaw] = swingTwistY(baseRot);
    glm::fquat baseYawWithoutClimbingFix = baseYaw;

    if (IsFirstPerson()) {
        // take link's direction, then rotate the headset position
        BEMatrix34 mtx = {};
        readMemory(s_playerMtxAddress, &mtx);
        glm::fvec3 playerPos = mtx.getPos().getLE();

        playerPos.y += s_isSwimming ? hardcodedSwimOffset : 0.0f;

        basePos = playerPos;
        if (auto settings = GetFirstPersonSettingsForActiveEvent()) {
            if (settings->ignoreCameraRotation) {
                glm::fquat playerRot = mtx.getRotLE();
                auto [swing, yaw] = swingTwistY(playerRot);
                baseYaw = yaw * glm::angleAxis(glm::radians(180.0f), glm::fvec3(0.0f, 1.0f, 0.0f));
                baseYawWithoutClimbingFix = yaw * glm::angleAxis(glm::radians(180.0f), glm::fvec3(0.0f, 1.0f, 0.0f));
            }
        }
        
        //if (s_isLadderClimbing) {
        //    baseYaw *= glm::angleAxis(glm::radians(180.0f), glm::fvec3(0.0f, 1.0f, 0.0f));
        //}
    }

    s_lastCameraMtx = glm::fmat4x3(glm::translate(glm::identity<glm::fmat4>(), basePos) * glm::mat4(baseYawWithoutClimbingFix));

    // vr camera
    std::optional<XrPosef> currPoseOpt = VRManager::instance().XR->GetRenderer()->GetPose(side);
    if (!currPoseOpt.has_value())
        return;
    glm::fvec3 eyePos = ToGLM(currPoseOpt.value().position);
    glm::fquat eyeRot = ToGLM(currPoseOpt.value().orientation);

    glm::vec3 newPos = basePos + (baseYaw * eyePos);
    glm::fquat newRot = baseYaw * eyeRot;

    glm::mat4 newWorldVR = glm::translate(glm::mat4(1.0f), newPos) * glm::mat4_cast(newRot);
    glm::mat4 newViewVR = glm::inverse(newWorldVR);

    camera.mtx.setLEMatrix(newViewVR);

    camera.pos = newPos;

    // Set look-at point by offsetting position in view direction
    glm::vec3 viewDir = -glm::vec3(newViewVR[2]); // Forward direction is -Z in view space
    camera.at = newPos + viewDir;

    // Transform world up vector by new rotation
    glm::vec3 upDir = glm::vec3(newViewVR[1]); // Up direction is +Y in view space
    camera.up = upDir;


    //glm::mat4 workingMtx = glm::inverse(glm::lookAtRH(newPos, newPos + glm::vec3(newViewVR[2]), glm::fvec3(0, 1, 0)));
    //s_lastCameraMtx = workingMtx;

    writeMemory(cameraOut, &camera);
    hCPU->gpr[3] = cameraOut;
}

constexpr uint32_t seadOrthoProjection = 0x1027B5BC;
constexpr uint32_t seadPerspectiveProjection = 0x1027B54C;


// https://github.com/KhronosGroup/OpenXR-SDK/blob/858912260ca616f4c23f7fb61c89228c353eb124/src/common/xr_linear.h#L564C1-L632C2
// https://github.com/aboood40091/sead/blob/45b629fb032d88b828600a1b787729f2d398f19d/engine/library/modules/src/gfx/seadProjection.cpp#L166

static data_VRProjectionMatrixOut calculateFOVAndOffset(XrFovf viewFOV) {
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

static glm::mat4 calculateProjectionMatrix(float nearZ, float farZ, const XrFovf& fov) {
    float l = tanf(fov.angleLeft) * nearZ;
    float r = tanf(fov.angleRight) * nearZ;
    float b = tanf(fov.angleDown) * nearZ;
    float t = tanf(fov.angleUp) * nearZ;

    float invW = 1.0f / (r - l);
    float invH = 1.0f / (t - b);
    float invD = 1.0f / (farZ - nearZ);

    glm::mat4 dst = {};
    dst[0][0] = 2.0f * nearZ * invW;
    dst[1][1] = 2.0f * nearZ * invH;
    dst[0][2] = (r + l) * invW;
    dst[1][2] = (t + b) * invH;
    dst[2][2] = -(farZ + nearZ) * invD;
    dst[2][3] = -(2.0f * farZ * nearZ) * invD;
    dst[3][2] = -1.0f;
    dst[3][3] = 0.0f;

    return dst;
}

void CemuHooks::hook_GetRenderProjection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return;
    }

    uint32_t projectionIn = hCPU->gpr[3];
    uint32_t projectionOut = hCPU->gpr[12];
    OpenXR::EyeSide side = hCPU->gpr[0] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionIn, &perspectiveProjection);

    Log::print<RENDERING>("[{}] Render Proj. (LR: {:08X}): {}", side, hCPU->sprNew.LR, perspectiveProjection);

    if (perspectiveProjection.zFar == 10000.0f) {
        return;
    }

    perspectiveProjection.zFar = GetSettings().GetZFar();
    perspectiveProjection.zNear = GetSettings().GetZNear();

    if (!VRManager::instance().XR->GetRenderer()->GetFOV(side).has_value()) {
        return;
    }
    XrFovf currFOV = VRManager::instance().XR->GetRenderer()->GetFOV(side).value();
    auto newProjection = calculateFOVAndOffset(currFOV);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = calculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), currFOV);
    perspectiveProjection.matrix = newMatrix;

    // calculate device matrix
    glm::fmat4 newDeviceMatrix = newMatrix;

    float zScale = perspectiveProjection.deviceZScale.getLE();
    float zOffset = perspectiveProjection.deviceZOffset.getLE();

    newDeviceMatrix[2][0] *= zScale;
    newDeviceMatrix[2][1] *= zScale;
    newDeviceMatrix[2][2] = (newDeviceMatrix[2][2] + newDeviceMatrix[3][2] * zOffset) * zScale;
    newDeviceMatrix[2][3] = newDeviceMatrix[2][3] * zScale + newDeviceMatrix[3][3] * zOffset;

    perspectiveProjection.deviceMatrix = newDeviceMatrix;

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;

    writeMemory(projectionOut, &perspectiveProjection);
    hCPU->gpr[3] = projectionOut;
}


void CemuHooks::hook_ModifyLightPrePassProjectionMatrix(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        return;
    }

    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return;
    }

    uint32_t projectionIn = hCPU->gpr[3];
    OpenXR::EyeSide side = hCPU->gpr[11] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionIn, &perspectiveProjection);

    if (!VRManager::instance().XR->GetRenderer()->GetFOV(side).has_value()) {
        return;
    }

    Log::print<RENDERING>("[{}] Modify light prepass projection", side);


    XrFovf currFOV = VRManager::instance().XR->GetRenderer()->GetFOV(side).value();
    auto newProjection = calculateFOVAndOffset(currFOV);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = calculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), currFOV);
    perspectiveProjection.matrix = newMatrix;

    // calculate device matrix
    glm::fmat4 newDeviceMatrix = newMatrix;

    float zScale = perspectiveProjection.deviceZScale.getLE();
    float zOffset = perspectiveProjection.deviceZOffset.getLE();

    newDeviceMatrix[2][0] *= zScale;
    newDeviceMatrix[2][1] *= zScale;
    newDeviceMatrix[2][2] = (newDeviceMatrix[2][2] + newDeviceMatrix[3][2] * zOffset) * zScale;
    newDeviceMatrix[2][3] = newDeviceMatrix[2][3] * zScale + newDeviceMatrix[3][3] * zOffset;

    perspectiveProjection.deviceMatrix = newDeviceMatrix;

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;

    writeMemory(projectionIn, &perspectiveProjection);
}

void CemuHooks::hook_EndCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[3] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    // todo: sometimes this can deadlock apparently?
    if (VRManager::instance().XR->GetRenderer()->IsInitialized() && side == OpenXR::EyeSide::RIGHT) {
        VRManager::instance().XR->GetRenderer()->EndFrame();
        CemuHooks::m_heldWeaponsLastUpdate[0] = CemuHooks::m_heldWeaponsLastUpdate[0]++;
        CemuHooks::m_heldWeaponsLastUpdate[1] = CemuHooks::m_heldWeaponsLastUpdate[1]++;
        if (CemuHooks::m_heldWeaponsLastUpdate[0] >= 6) {
            CemuHooks::m_heldWeapons[0] = 0;
        }
        if (CemuHooks::m_heldWeaponsLastUpdate[1] >= 6) {
            CemuHooks::m_heldWeapons[1] = 0;
        }
    }

    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("");
}

void CemuHooks::hook_ReplaceCameraMode(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t currentCameraMode = hCPU->gpr[3];
    uint32_t cameraTailMode = hCPU->gpr[4]; // works best in VR since it ignores the pivot of the camera
    uint32_t currentCameraVtbl = hCPU->gpr[5];

    constexpr uint32_t kCameraChaseVtbl = 0x101B34F4;
    uint32_t kCameraTailVtbl = 0x101BC278;

    if (hCPU->gpr[5] == kCameraTailVtbl) {
        //Log::print<RENDERING>("Current camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currentCameraMode, cameraTailMode, currentCameraVtbl);
        if (IsFirstPerson()) {
            // overwrite to tail mode
            //hCPU->gpr[3] = cameraTailMode;

            //hCPU->gpr[3] = cameraTailMode;
        }
    }

    //Log::print<INFO>("Camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currentCameraMode, cameraTailMode, currentCameraVtbl);
}

void CemuHooks::hook_UseCameraDistance(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsFirstPerson()) {
        hCPU->fpr[13].fp0 = 0.0f;
    }
    else {
        hCPU->fpr[13].fp0 = hCPU->fpr[10].fp0;
    }
}

void CemuHooks::hook_SetActorOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    double toBeSetOpacity = hCPU->fpr[1].fp0;
    uint32_t actorPtr = hCPU->gpr[3];

    ActorWiiU actor;
    readMemory(actorPtr, &actor);

    // normal behavior if it wasn't the player or a held weapon
    if (actor.modelOpacity.getLE() != toBeSetOpacity) {
        uint8_t opacityOrDoFlushOpacityToGPU = 1;
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, modelOpacity), &toBeSetOpacity);
        writeMemoryBE(actorPtr + offsetof(ActorWiiU, opacityOrDoFlushOpacityToGPU), &opacityOrDoFlushOpacityToGPU);
    }
}

void CemuHooks::hook_CalculateModelOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // overwrites return value to 1 if in first person mode
    if (IsFirstPerson()) {
        hCPU->fpr[1].fp0 = 1.0f;
    }
}

std::string CemuHooks::s_currentEvent = {};
CemuHooks::HybridEventSettings CemuHooks::s_currentEventSettings = {};
std::unordered_map<std::string, CemuHooks::HybridEventSettings> CemuHooks::s_eventSettings = {};

constexpr CemuHooks::HybridEventSettings defaultFirstPersonSettings = {
    .firstPerson = true,
    .disablePlayerDrivenLinkHands = false,
    .ignoreCameraRotation = true
};

void CemuHooks::initCutsceneDefaultSettings(uint32_t ppc_TableOfCutsceneEventsSettingsOffset) {
    if (!s_eventSettings.empty()) {
        return;
    }

    char* currPtr = reinterpret_cast<char*>(s_memoryBaseAddress + ppc_TableOfCutsceneEventsSettingsOffset);
    while (true) {
        const std::string line(currPtr);
        if (line.empty()) {
            break;
        }
        size_t commaPos = line.find(',');
        if (commaPos != std::string::npos) {
            HybridEventSettings entry = {};
            std::string settingsStr = line.substr(commaPos + 1);
            std::string eventName = line.substr(0, commaPos);
            size_t pos = 0;
            while ((pos = settingsStr.find(',')) != std::string::npos) {
                std::string setting = settingsStr.substr(0, pos);
                if (setting == "FP_ON") entry.firstPerson = true;
                else if (setting == "FP_OFF")
                    entry.firstPerson = false;
                else if (setting == "HND_ON")
                    entry.disablePlayerDrivenLinkHands = false;
                else if (setting == "HND_OFF")
                    entry.disablePlayerDrivenLinkHands = true;
                else if (setting == "PAN_ON")
                    entry.ignoreCameraRotation = false;
                else if (setting == "PAN_OFF")
                    entry.ignoreCameraRotation = true;
                else if (setting == "CTRL_ON")
                    entry.demoEnableCameraInput = false;
                else if (setting == "CTRL_OFF")
                    entry.demoEnableCameraInput = true;
                else {
                    Log::print<WARNING>("Unknown cutscene default setting: {}", setting);
                }
                settingsStr.erase(0, pos + 1);
            }
            // last setting
            std::string setting = settingsStr;
            if (setting == "FP_ON") entry.firstPerson = true;
            else if (setting == "FP_OFF")
                entry.firstPerson = false;
            else if (setting == "HND_ON")
                entry.disablePlayerDrivenLinkHands = false;
            else if (setting == "HND_OFF")
                entry.disablePlayerDrivenLinkHands = true;
            else if (setting == "PAN_ON")
                entry.ignoreCameraRotation = false;
            else if (setting == "PAN_OFF")
                entry.ignoreCameraRotation = true;
            else if (setting == "CTRL_ON")
                entry.demoEnableCameraInput = false;
            else if (setting == "CTRL_OFF")
                entry.demoEnableCameraInput = true;
            else {
                Log::print<WARNING>("Unknown cutscene default setting: {}", setting);
            }
            s_eventSettings.insert_or_assign(eventName, entry);
        }
        currPtr += line.length() + 1;
    }

    Log::print<VERBOSE>("Initialized cutscene default settings for {} events.", s_eventSettings.size());
}


uint32_t CemuHooks::s_playerAddress = 0;

void CemuHooks::hook_GetEventName(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t isEventActive = hCPU->gpr[3];
    uint32_t ppcAddress = hCPU->gpr[4];

    if (isEventActive) {
        // Validate PPC address is in valid game memory range
        if (ppcAddress < 0x10000000 || ppcAddress > 0x50000000) {
            return;
        }

        const char* eventNamePtr = (const char*)(ppcAddress + s_memoryBaseAddress);

        // Validate the string starts with a printable ASCII character
        unsigned char firstChar = static_cast<unsigned char>(eventNamePtr[0]);
        if (firstChar < 0x20 || firstChar > 0x7E) {
            return;
        }

        std::string eventName = std::string(eventNamePtr);
        if (s_currentEvent == eventName) {
            return;
        }
        Log::print<INFO>("Event '{}' is now active.", eventName);
        s_currentEvent = eventName;

        auto it = s_eventSettings.find(eventName);
        if (it != s_eventSettings.end()) {
            HybridEventSettings settings = it->second;
            Log::print<INFO>(" - First Person: {}", settings.firstPerson ? "ON" : "OFF");
            Log::print<INFO>(" - Ignore Camera Rotation: {}", settings.ignoreCameraRotation ? "ON" : "OFF");
            Log::print<INFO>(" - Disable Player-Driven Link Hands: {}", settings.disablePlayerDrivenLinkHands ? "ON" : "OFF");
            s_currentEventSettings = settings;
        }
        else {
            Log::print<INFO>(" - No specific settings found for this event, using defaults.");
            s_currentEventSettings = defaultFirstPersonSettings;
        }

        // In cutscene's there's somethings a mention of Demo_EnableCameraInput/Demo_EnableCameraControlByUser/Demo_DisableCameraInput
        // These don't actually seem to be hooked up so won't do anything in real-time, but they do flag a cutscene as having camera control disabled for the player.
        // This can be read using the settings.demoEnableCameraInput in the HybridEventSettings struct.
    }
    else if (!s_currentEvent.empty()) {
        Log::print<INFO>("Event '{}' has now ended", s_currentEvent);
        s_currentEvent = "";
    }
}

constexpr uint32_t orig_GetStaticParam_float_funcAddr = 0x030E9BE0;

// hook for ksys::act::ai::ActionBase::getStaticParam<FLOAT> calls
void CemuHooks::hook_OverwriteCameraParam(PPCInterpreter_t* hCPU) {

    uint32_t actionPtr = hCPU->gpr[3];
    uint32_t destFloatPtr = hCPU->gpr[4];
    const char* paramName = (const char*)(s_memoryBaseAddress + getMemory<BEType<uint32_t>>(hCPU->gpr[5]).getLE());

    if (actionPtr == 0 || destFloatPtr == 0 || paramName == nullptr) {
        hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
        return;
    }
    
    std::string paramNameStr = paramName;

    hCPU->instructionPointer = hCPU->sprNew.LR;
    if (paramNameStr == "JumpHeight") {
        // override jump height to 1.2 in first person mode to temporarily workaround the increased gravity effect
        uint32_t superLowAddress = 0x100C50D0; // points to 1.2
        writeMemoryBE(hCPU->gpr[4], &superLowAddress);
        return;
    }

    if (GetSettings().IsFirstPersonMode()) {
        uint32_t superLowAddress = 0x102B3150; // points to 0.0000011920929
        writeMemoryBE(hCPU->gpr[4], &superLowAddress);
        return;
    }

    hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
}

void CemuHooks::hook_PlayerLadderFix(PPCInterpreter_t* hCPU) {
    hCPU->gpr[0] = hCPU->sprNew.LR;
    hCPU->instructionPointer = 0x02D07CEC;

    if (IsFirstPerson()) {
        s_isLadderClimbing = 2;
    }
}
