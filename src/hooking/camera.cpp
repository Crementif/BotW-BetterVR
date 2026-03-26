#include "cemu_hooks.h"
#include "instance.h"
#include "rendering/openxr.h"
#include "utils/debug_draw.h"

bool CemuHooks::UseMonoFrameBufferTemporarilyDuringMenusOrPictures() {
    return IsScreenOpen(ScreenId::PauseMenuInfo_00) || VRManager::instance().XR->GetRenderer()->IsGameCapturing3DFrameBuffer();
}

void CemuHooks::hook_BeginCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[0] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    Log::print<RENDERING>("");
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);
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

static glm::fvec3 ApplyCameraModeEyePosPolicy(glm::fvec3 eyePos, CameraMode cameraMode) {
    if (cameraMode == CameraMode::ORIGINAL) {
        eyePos.y = 0.0f;

        auto gameState = VRManager::instance().XR->m_gameState.load();
        if (gameState.is_riding_mount) {
            eyePos.y += GetSettings().GetOriginalRidingVerticalOffset();
        }
    }

    return eyePos;
}

static glm::fquat ApplyCameraModeEyeRotPolicy(const glm::fquat& baseYaw, const glm::fquat& baseRot, const glm::fquat& eyeRot, CameraMode cameraMode) {
    if (cameraMode == CameraMode::ORIGINAL) {
        return baseRot * eyeRot;
    }

    return baseYaw * eyeRot;
}

static glm::fvec3 ApplyCameraModeEyeOffsetPolicy(const glm::fquat& baseYaw, const glm::fquat& baseRot, const glm::fvec3& eyePos, CameraMode cameraMode) {
    if (cameraMode == CameraMode::ORIGINAL) {
        return baseRot * eyePos;
    }

    return baseYaw * eyePos;
}

static float GetStereoDepthScaleForCurrentContext() {
    if (CemuHooks::HasActiveCutscene()) {
        return GetSettings().GetCutsceneStereoDepthScale();
    }

    return GetSettings().GetGameplayStereoDepthScale();
}

static glm::fvec3 ApplyStereoDepthScalePolicy(const glm::fvec3& eyePos) {
    auto middlePoseOpt = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (!middlePoseOpt.has_value()) {
        return eyePos;
    }

    const glm::fvec3 middleEyePos = glm::fvec3(middlePoseOpt.value()[3]);
    const float depthScale = GetStereoDepthScaleForCurrentContext();
    return middleEyePos + (eyePos - middleEyePos) * depthScale;
}

static void SetLookAtCameraVectorsFromPose(BESeadLookAtCamera& camera, const glm::vec3& position, const glm::fquat& rotation) {
    const glm::vec3 forward = glm::normalize(rotation * glm::fvec3(0.0f, 0.0f, -1.0f));
    const glm::vec3 up = glm::normalize(rotation * glm::fvec3(0.0f, 1.0f, 0.0f));

    camera.pos = position;
    camera.at = position + forward;
    camera.up = up;
}


float hardcodedSwimOffset = 0.0f;
float hardcodedRidingOffset = 0.65f;
float hardcodedCrouchOffset = 0.3f;

glm::fvec3 s_wsCameraPosition = glm::fvec3();
glm::fquat s_wsCameraRotation = glm::identity<glm::fquat>();
bool s_isSwimming = false;
bool s_isCrouching = false;
bool s_wasCrouching = false;
float actualCrouchOffset = 0.0f;
std::chrono::steady_clock::time_point crouch_state_change_time;

uint32_t s_isLadderClimbing = 0;
uint32_t s_isRiding = 0;
uint32_t s_isRidingSandSeal = 0;

void CemuHooks::hook_UpdateCameraForGameplay(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (CemuHooks::UseBlackBarsDuringEvents()) {
        return;
    }

    // read the camera matrix from the game's memory
    uint32_t ppc_cameraMatrixOffsetIn = hCPU->gpr[31];
    OpenXR::EyeSide side = hCPU->gpr[3] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;
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

    if (GetSettings().GetCameraMode() == CameraMode::FIRST_PERSON) {
        // remove verticality from the camera position to avoid pitch changes that aren't from the VR headset
        oldCameraPosition.y = oldCameraTarget.y;
    }

    // construct glm matrix from the existing camera parameters
    glm::mat4 existingGameMtx = glm::lookAtRH(oldCameraPosition, oldCameraTarget, oldCameraUp);

    // pass real camera position and rotation to be used elsewhere
    // GetRenderCamera recalculates the left and right eye positions from the base camera position
    s_wsCameraPosition = oldCameraPosition;
    s_wsCameraRotation = glm::quat_cast(glm::inverse(existingGameMtx));

     // check if player is swimming
    Player actor;
    readMemory(s_playerAddress, &actor);

    PlayerMoveBitFlags moveBits = actor.moveBitFlags.getLE();
    s_isSwimming = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_SWIMMING);
    s_isCrouching = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_CROUCHING);

    // Todo: move those and their hooks in controls.cpp ?
    auto gameState = VRManager::instance().XR->m_gameState.load();
    // Unreliable flag, need to investigate
    gameState.is_climbing = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_SWIMMING_OR_CLIMBING | PlayerMoveBitFlags::IS_CLIMBING_WALL) || s_isLadderClimbing == 2;
    gameState.is_riding_mount = (s_isRiding == 2 || s_isRidingSandSeal == 2) ? true : false;
    gameState.is_paragliding = HAS_FLAG(moveBits, PlayerMoveBitFlags::IS_GLIDER_ACTIVE);
    VRManager::instance().XR->m_gameState.store(gameState);

    // rebase the rotation to the player position
    if (IsFirstPerson()) {
       

        auto now = std::chrono::steady_clock::now();
        std::chrono::milliseconds crouchLerpDuration{ 150 };
        if (s_isCrouching != s_wasCrouching) {
            crouch_state_change_time = now;
        }
        auto test = 0.8f;
        if (now <= crouch_state_change_time + crouchLerpDuration) {
            auto elapsed = std::chrono::duration<float>(now - crouch_state_change_time);
            auto duration = std::chrono::duration<float>(crouchLerpDuration);
            float t = elapsed.count() / duration.count();
            t = glm::clamp(t, 0.0f, 1.0f);
            if (s_isCrouching) {
                actualCrouchOffset = glm::mix(0.0f, test, t);
            }
            else {
                actualCrouchOffset = glm::mix(test, 0.0f, t);
            }
        }
        else {
            actualCrouchOffset = s_isCrouching ? test : 0.0f;
        }

        // read player MTX
        BEMatrix34& mtx = actor.mtx;
        glm::fvec3 playerPos = actor.mtx.getPos().getLE();

        if (s_isRiding) {
            playerPos.y -= hardcodedRidingOffset;
        }
        else if (s_isSwimming) {
            float playerHeight = VRManager::instance().XR->GetRenderer()->GetMiddlePose().value()[3].y;
            playerPos.y += 1.73f - playerHeight;
        }
        else {
            playerPos.y += GetSettings().GetPlayerHeightOffset() - actualCrouchOffset;
        }


        if (s_isLadderClimbing > 0) {
            s_isLadderClimbing--;
        }
        

        s_wasCrouching = s_isCrouching;

        glm::mat4 playerMtx4 = glm::inverse(glm::translate(glm::identity<glm::mat4>(), playerPos) * glm::mat4(s_wsCameraRotation));
        existingGameMtx = playerMtx4;
    }

    if (s_isRiding > 0) {
        s_isRiding--;
    }
    if (s_isRidingSandSeal > 0) {
        s_isRidingSandSeal--;
    }

    // current VR headset camera matrix
    auto viewsOpt = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (!viewsOpt) {
        Log::print<ERROR>("hook_UpdateCameraForGameplay: No views available for the middle pose.");
        return;
    }
    auto& views = viewsOpt.value();
    glm::mat4 viewsForGameplay = views;

    glm::fvec3 middleEyePos = glm::fvec3(viewsForGameplay[3]);
    middleEyePos = ApplyCameraModeEyePosPolicy(middleEyePos, GetSettings().GetCameraMode());
    viewsForGameplay[3] = glm::fvec4(middleEyePos, viewsForGameplay[3].w);

    // calculate final camera matrix
    glm::mat4 finalPose = glm::inverse(existingGameMtx) * viewsForGameplay;

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

void CemuHooks::hook_FixStaminaGaugeScreenPosition(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsThirdPerson() && GetSettings().GetCameraMode() != CameraMode::ORIGINAL) {
        return;
    }

    BEVec2 staminaGauge2DPos;
    readMemory(hCPU->gpr[4], &staminaGauge2DPos);
    BEVec3 playerPosToCameraPos;
    readMemory(hCPU->gpr[5], &playerPosToCameraPos);

    Log::print<PPC>("Fixing stamina gauge position (oldPos = {}, playerPosToCameraPos = {})", staminaGauge2DPos.getLE(), playerPosToCameraPos);

    staminaGauge2DPos = BEVec2{ -482.0f, +170.0f }; // nested under hearts
    //staminaGauge2DPos = BEVec2{ 380.0f, 300.0f }; // above the status bars but at the top of the screen
    //staminaGauge2DPos = BEVec2{ -220.0f, 300.0f }; // above the status bars but at the top of the screen
    playerPosToCameraPos = BEVec3{ 0.0f, 0.0, 0.0f };

    writeMemory(hCPU->gpr[4], &staminaGauge2DPos);
    writeMemory(hCPU->gpr[5], &playerPosToCameraPos);
}

void CemuHooks::hook_FixExtraStaminaGaugeIconPositions(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // original instruction that got replaced
    hCPU->fpr[29].fp0 = 1.0f;

    if (IsThirdPerson() && GetSettings().GetCameraMode() != CameraMode::ORIGINAL) {
        return;
    }

    // manually jump
    hCPU->instructionPointer = 0x02FB29C4;
}

glm::mat4 CemuHooks::s_lastCameraMtx = glm::mat4(1.0f);

void CemuHooks::hook_GetRenderCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t cameraIn = hCPU->gpr[3];
    uint32_t cameraOut = hCPU->gpr[12];
    EyeSide side = hCPU->gpr[11] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    if (UseBlackBarsDuringEvents()) {
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
    glm::fquat baseRot = glm::quat_cast(worldGame);

    // overwrite with our stored camera pos/rot
    basePos = s_wsCameraPosition;
    baseRot = s_wsCameraRotation;
    auto [swing, baseYaw] = swingTwistY(baseRot);
    glm::fquat baseYawWithoutClimbingFix = baseYaw;

    if (IsFirstPerson()) {
        // take link's direction, then rotate the headset position
        BEMatrix34 playerMtx = {};
        readMemory(s_playerMtxAddress, &playerMtx);
        glm::fvec3 playerPos = playerMtx.getPos().getLE();

        if (s_isRiding) {
            playerPos.y -= hardcodedRidingOffset;
        }
        else if (s_isSwimming) {
            playerPos.y += hardcodedSwimOffset + GetSettings().GetPlayerHeightOffset();
        }
        else {
            playerPos.y += GetSettings().GetPlayerHeightOffset() - actualCrouchOffset;
        }

        basePos = playerPos;
    }

    s_lastCameraMtx = glm::fmat4x3(glm::translate(glm::identity<glm::fmat4>(), basePos) * glm::mat4(baseYawWithoutClimbingFix));

    // vr camera
    std::optional<XrPosef> currPoseOpt = VRManager::instance().XR->GetRenderer()->GetPose(side);
    if (!currPoseOpt.has_value())
        return;
    glm::fvec3 eyePos = ToGLM(currPoseOpt.value().position);
    glm::fquat eyeRot = ToGLM(currPoseOpt.value().orientation);

    CameraMode cameraMode = GetSettings().GetCameraMode();
    eyePos = ApplyStereoDepthScalePolicy(eyePos);
    eyePos = ApplyCameraModeEyePosPolicy(eyePos, cameraMode);

    glm::vec3 newPos = basePos + ApplyCameraModeEyeOffsetPolicy(baseYaw, baseRot, eyePos, cameraMode);
    glm::fquat newRot = ApplyCameraModeEyeRotPolicy(baseYaw, baseRot, eyeRot, cameraMode);

    glm::mat4 newWorldVR = glm::translate(glm::mat4(1.0f), newPos) * glm::mat4_cast(newRot);
    glm::mat4 newViewVR = glm::inverse(newWorldVR);

    if (side == EyeSide::RIGHT && GetSettings().ShowDebugOverlay()) {
        glm::mat4 proj = calculateProjectionMatrix(GetSettings().GetZNear(), GetSettings().GetZFar(), VRManager::instance().XR->GetRenderer()->GetFOV(side).value());
            
        // transpose the sead-convention (row-major) projections to standard column-major
        glm::mat4 vrProj = glm::transpose(proj);

        glm::mat4 vrVP = vrProj * newViewVR;
        //DebugDraw::instance().Frustum(vrVP, IM_COL32(0, 255, 0, 255));

        // store the right-eye projection matrix to transform the debug lines with
        // use right-side due to that being the one shown in the 2D view
        DebugDraw::instance().SetViewProjection(vrVP);
    }

    camera.mtx.setLEMatrix(newViewVR);

    SetLookAtCameraVectorsFromPose(camera, newPos, newRot);


    //glm::mat4 workingMtx = glm::inverse(glm::lookAtRH(newPos, newPos + glm::vec3(newViewVR[2]), glm::fvec3(0, 1, 0)));
    //s_lastCameraMtx = workingMtx;

    writeMemory(cameraOut, &camera);
    hCPU->gpr[3] = cameraOut;
}

constexpr uint32_t seadOrthoProjection = 0x1027B5BC;
constexpr uint32_t seadPerspectiveProjection = 0x1027B54C;

void CemuHooks::hook_GetRenderProjection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (UseBlackBarsDuringEvents()) {
        return;
    }

    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
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

    if (UseBlackBarsDuringEvents()) {
        return;
    }
    
    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
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

void CemuHooks::hook_OverwriteSeadPerspectiveProjectionSet(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
}

void CemuHooks::hook_ModifyProjectionUsingCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        return;
    }

    if (UseBlackBarsDuringEvents()) {
        return;
    }

    if (UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }

    uint32_t projectionPtr = hCPU->gpr[4];
    uint32_t cameraPtr = hCPU->gpr[7];
    OpenXR::EyeSide side = hCPU->gpr[5] == 0 ? EyeSide::LEFT : EyeSide::RIGHT;

    // this is always true, since we currently only hook one caller
    if (hCPU->gpr[6] == 0x02C43454) {
        BESeadLookAtCamera camera = {};
        readMemory(cameraPtr, &camera);

        Log::print<RENDERING>("[{}] ModifyProjectionUsingCamera at {:08X}: {}", side, cameraPtr, camera);

        // in-game camera
        glm::mat4x3 viewMatrix = camera.mtx.getLEMatrix();
        glm::mat4 worldGame = glm::inverse(glm::mat4(viewMatrix));
        glm::vec3 basePos = glm::vec3(worldGame[3]);
        glm::fquat baseRot = glm::quat_cast(worldGame);

        // ignore the current rotation since it is already changed by the gameplay camera hooking
        baseRot = s_wsCameraRotation;
        auto [swing, baseYaw] = swingTwistY(baseRot);

        if (IsFirstPerson()) {
            // take link's direction, then rotate the headset position
            BEMatrix34 playerMtx = {};
            readMemory(s_playerMtxAddress, &playerMtx);
        }

        // vr camera
        std::optional<XrPosef> currPoseOpt = VRManager::instance().XR->GetRenderer()->GetPose(side);
        if (!currPoseOpt.has_value())
            return;
        glm::fvec3 eyePos = ToGLM(currPoseOpt.value().position);
        glm::fquat eyeRot = ToGLM(currPoseOpt.value().orientation);

        CameraMode cameraMode = GetSettings().GetCameraMode();
        eyePos = ApplyStereoDepthScalePolicy(eyePos);
        eyePos = ApplyCameraModeEyePosPolicy(eyePos, cameraMode);

        glm::vec3 newPos = basePos + ApplyCameraModeEyeOffsetPolicy(baseYaw, baseRot, eyePos, cameraMode);
        glm::fquat newRot = ApplyCameraModeEyeRotPolicy(baseYaw, baseRot, eyeRot, cameraMode);

        glm::mat4 newWorldVR = glm::translate(glm::mat4(1.0f), newPos) * glm::mat4_cast(newRot);
        glm::mat4 newViewVR = glm::inverse(newWorldVR);

        camera.mtx.setLEMatrix(newViewVR);

        SetLookAtCameraVectorsFromPose(camera, newPos, newRot);

        writeMemory(cameraPtr, &camera);
    }

    BESeadPerspectiveProjection perspectiveProjection = {};
    readMemory(projectionPtr, &perspectiveProjection);

    if (!VRManager::instance().XR->GetRenderer()->GetFOV(side).has_value()) {
        return;
    }

    Log::print<RENDERING>("[{}] ModifyProjectionUsingCamera: {}", side, perspectiveProjection);

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

    writeMemory(projectionPtr, &perspectiveProjection);
}

std::pair<glm::vec3, glm::fquat> CemuHooks::CalculateVRWorldPose(const BESeadLookAtCamera& camera, uint8_t side) {
    // in-game camera
    glm::mat4x3 viewMatrix = camera.mtx.getLEMatrix();
    glm::mat4 worldGame = glm::inverse(glm::mat4(viewMatrix));
    glm::vec3 basePos = glm::vec3(worldGame[3]);
    glm::fquat baseRot = glm::quat_cast(worldGame);

    // overwrite with our stored camera pos/rot
    basePos = s_wsCameraPosition;
    baseRot = s_wsCameraRotation;

    auto [swing, baseYaw] = swingTwistY(baseRot);
    if (IsFirstPerson()) {
        // take link's direction, then rotate the headset position
        BEMatrix34 playerMtx = {};
        readMemory(s_playerMtxAddress, &playerMtx);
        glm::fvec3 playerPos = playerMtx.getPos().getLE();

        if (s_isRiding) {
            playerPos.y -= hardcodedRidingOffset;
        }
        else if (s_isSwimming) {
            playerPos.y += hardcodedSwimOffset;
        }
        else {
            playerPos.y += GetSettings().GetPlayerHeightOffset() - actualCrouchOffset;
        }

        basePos = playerPos;
    }

    // vr camera
    std::optional<XrPosef> currPoseOpt = VRManager::instance().XR->GetRenderer()->GetPose((OpenXR::EyeSide)side);
    if (!currPoseOpt.has_value()) {
        return { basePos, baseRot };
    }

    glm::fvec3 eyePos = ToGLM(currPoseOpt.value().position);
    glm::fquat eyeRot = ToGLM(currPoseOpt.value().orientation);

    CameraMode cameraMode = GetSettings().GetCameraMode();
    eyePos = ApplyStereoDepthScalePolicy(eyePos);
    eyePos = ApplyCameraModeEyePosPolicy(eyePos, cameraMode);

    glm::vec3 newPos = basePos + ApplyCameraModeEyeOffsetPolicy(baseYaw, baseRot, eyePos, cameraMode);
    glm::fquat newRot = ApplyCameraModeEyeRotPolicy(baseYaw, baseRot, eyeRot, cameraMode);

    return { newPos, newRot };
}

void CemuHooks::hook_CheckIfCameraCanSeePos(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr) {
        hCPU->gpr[3] = 0;
        return;
    }

    uint32_t camPtr = hCPU->gpr[3];
    uint32_t posPtr = hCPU->gpr[4];
    float radius = hCPU->fpr[1].fp0;
    float nearClip = hCPU->fpr[2].fp0;
    float farClip = hCPU->fpr[3].fp0;

    BESeadLookAtCamera camera = {};
    readMemory(camPtr, &camera);

    BEVec3 center;
    readMemory(posPtr, &center);

    Frustum frustum;
    bool visible = false;

    for (int i = 0; i < 2; ++i) {
        OpenXR::EyeSide side = (i == 0) ? EyeSide::LEFT : EyeSide::RIGHT;
        if (auto fovOpt = VRManager::instance().XR->GetRenderer()->GetFOV(side)) {
            auto [pos, rot] = CalculateVRWorldPose(camera, side);

            // pull the camera backwards a bit to account for it being a third-person game that encompassed a bigger area
            pos += rot * glm::vec3(0.0f, 0.0f, 1.0f);

            glm::mat4 view = glm::inverse(glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(rot));
            glm::mat4 proj = glm::transpose(calculateProjectionMatrix(nearClip, farClip, fovOpt.value()));
            glm::mat4 vp = proj * view;

            frustum.update(vp);
            if (frustum.checkSphere(center.getLE(), radius)) {
                visible = true;
                break;
            }
        }
    }

    Log::print<PPC>("Checking visibility of {} (rad = {}, near = {}, far = {}): {}", center, radius, nearClip, farClip, visible ? "visible" : "invisible");

    hCPU->gpr[3] = visible ? 1 : 0;
}


void CemuHooks::hook_ModifyPixelUniformBlockData(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto currFovOpt = VRManager::instance().XR->GetRenderer()->GetFOV(EyeSide::RIGHT);
    if (!currFovOpt.has_value()) {
        return;
    }

    glm::fvec4 ubData = {};
    readMemory(hCPU->gpr[5], &ubData);

    XrFovf currFOV = currFovOpt.value();
    auto newProjection = calculateFOVAndOffset(currFOV);

    ubData.x = 0.5f + newProjection.offsetX.getLE();
    ubData.y = 0.5f + newProjection.offsetY.getLE();

    writeMemory(hCPU->gpr[5], &ubData);
}

void CemuHooks::hook_EndCameraSide(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    OpenXR::EyeSide side = hCPU->gpr[3] == 0 ? OpenXR::EyeSide::LEFT : OpenXR::EyeSide::RIGHT;

    // todo: sometimes this can deadlock apparently?
    if (VRManager::instance().XR->GetRenderer()->IsInitialized() && side == OpenXR::EyeSide::RIGHT) {
        m_heldWeaponsLastUpdate[0]++;
        m_heldWeaponsLastUpdate[1]++;
        if (m_heldWeaponsLastUpdate[0] >= 6) {
            m_heldWeapons[0] = 0;
        }
        if (m_heldWeaponsLastUpdate[1] >= 6) {
            m_heldWeapons[1] = 0;
        }
    }

    Log::print<RENDERING>("{0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0} {0}", side);
    Log::print<RENDERING>("===============================================================================");
    Log::print<RENDERING>("");
}

void CemuHooks::hook_UseCameraDistance(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsFirstPerson()) {
        hCPU->fpr[13].fp0 = 0.0f;
    }
    else if (GetSettings().GetCameraMode() == CameraMode::THIRD_PERSON || GetSettings().GetCameraMode() == CameraMode::ORIGINAL) {
        hCPU->fpr[13].fp0 = GetSettings().thirdPlayerDistance;
    }
    else {
        hCPU->fpr[13].fp0 = 0.5f; // use default distance when using the first-person camera
    }
}

void CemuHooks::hook_SetActorOpacity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    float toBeSetOpacity = hCPU->fpr[1].fp0;
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

// todo: Fix sped-up cutscenes. You can go in-game into a save file that didn't have DLC yet, and it'll give you a sped up Demo200_0.
void CemuHooks::hook_GetEventName(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t isEventActive = hCPU->gpr[3];
    uint32_t eventNamePtr = hCPU->gpr[4];
    uint32_t entryPointNamePtr = hCPU->gpr[5];

    if (isEventActive) {
        std::string eventName = std::string((char*)s_memoryBaseAddress + eventNamePtr);
        std::string entryPointName = std::string((char*)s_memoryBaseAddress + entryPointNamePtr);

        if (s_currentEvent == eventName) {
            return;
        }
        Log::print<INFO>("Event '{}' is now active (using entry point '{}').", eventName, entryPointName);
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

void CemuHooks::hook_ShouldSkipEventCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (IsFirstPerson()) {
        // disable camera rotation for first-person events, according to the event settings
        if (auto eventSettings = GetFirstPersonSettingsForActiveEvent()) {
            if (eventSettings->ignoreCameraRotation) {
                hCPU->gpr[3] = 1;
                return;
            }
        }
    }

    // return 0 to just follow regular camera rotation
    hCPU->gpr[3] = 0;
}

struct CameraParamValueOffset {
    std::string name;
    uint32_t offsetInsideCamera;
    bool storedOriginalValue = false;
    float originalValue;
};

// key = vtable address, value = list of parameter names and their offsets inside the camera object
std::mutex storedCameraParametersLock;
std::unordered_map<uint32_t, std::vector<CameraParamValueOffset>> storedCameraParameters;


void CemuHooks::hook_ReplaceCameraMode(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t currCameraInstance = hCPU->gpr[3];
    uint32_t cameraChaseInstance = hCPU->gpr[4]; // this is currently a pointer to the regular camera mode (CameraChase)
    uint32_t currentCameraVtbl = hCPU->gpr[5];


    // check if any patched parameters exist for this camera vtbl
    {
        std::scoped_lock(storedCameraParametersLock);

        auto it = storedCameraParameters.find(currCameraInstance);
        if (it != storedCameraParameters.end()) {
            for (auto& paramEntry : it->second) {
                uint32_t originalValuePtr = getMemory<BEType<uint32_t>>(paramEntry.offsetInsideCamera).getLE();
                BEType<float>* paramValueBE = (BEType<float>*)(s_memoryBaseAddress + originalValuePtr);

                // on first patch, store original value
                if (!paramEntry.storedOriginalValue) {
                    paramEntry.originalValue = paramValueBE->getLE();
                    paramEntry.storedOriginalValue = true;
                }

                if (IsFirstPerson()) {
                    // set to zero in first person
                    *paramValueBE = 0.0f;
                }
                else {
                    // restore original value in third person
                    *paramValueBE = paramEntry.originalValue;
                }
            }
        }
    }

    constexpr uint32_t kCameraChaseVtbl = 0x101B34F4;
    constexpr uint32_t kCameraTailVtbl = 0x101BC278;
    constexpr uint32_t kCameraAiming2Vtbl = 0X101B2EB4;
    constexpr uint32_t kCameraMagneCatchVtbl = 0x101BAB4C;

    //hCPU->gpr[3] = cameraChaseInstance;

    if (hCPU->gpr[5] == kCameraMagneCatchVtbl) {
        if (IsFirstPerson()) {
            //hCPU->gpr[3] = cameraChaseMode;
        }
    }

    if (hCPU->gpr[5] == kCameraTailVtbl) {
        //Log::print<RENDERING>("Current camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currentCameraMode, cameraTailMode, currentCameraVtbl);
        if (IsFirstPerson()) {
            // overwrite to tail mode
            //hCPU->gpr[3] = cameraTailMode;
        }
    }

    //Log::print<INFO>("Camera mode: {:#X}, tail mode: {:#X}, vtbl: {:#X}", currentCameraMode, cameraTailMode, currentCameraVtbl);
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
    
    {
        std::scoped_lock(storedCameraParametersLock);

        auto& paramList = storedCameraParameters[actionPtr];
        // store parameter offset if not already stored
        auto it = std::find_if(paramList.begin(), paramList.end(), [&paramNameStr](const CameraParamValueOffset& entry) {
            return entry.name == paramNameStr;
        });
        if (it == paramList.end()) {
            // get offset by calling original function first
            hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;

            Log::print<PPC>("Storing camera param '{}' offset {:08X} for camera action at {:08X}", paramNameStr, destFloatPtr, actionPtr);
            paramList.push_back({ paramNameStr, destFloatPtr });
        }
    }

    hCPU->instructionPointer = orig_GetStaticParam_float_funcAddr;
}

void CemuHooks::hook_FixLadder(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto input = VRManager::instance().XR->m_input.load();

    if (input.shared.in_game && s_isLadderClimbing == 0) {
        return;
    }

    if (input.inGame.move.currentState.y >= -0.05) {
        //Log::print<INFO>("PLAYER LADDER MODE UP {}", input.inGame.move.currentState.y);
        hCPU->gpr[3] = 4; // allows pressing A to jump upwards, regardless of camera orientation
    }
    else {
        //Log::print<INFO>("PLAYER LADDER MODE DOWN {}", input.inGame.move.currentState.y);
        hCPU->gpr[3] = 1; // allows sliding down when holding move stick downwards
    }
}

void CemuHooks::hook_PlayerIsRiding(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    bool isRiding = hCPU->gpr[3] == 1;
    if (isRiding && (IsFirstPerson() || GetSettings().GetCameraMode() == CameraMode::ORIGINAL)) {
        s_isRiding = 2;
    }
}

void CemuHooks::hook_PlayerIsRidingSandSeal(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    bool isRiding = hCPU->gpr[3] == 1;
    if (isRiding && (IsFirstPerson() || GetSettings().GetCameraMode() == CameraMode::ORIGINAL)) {
        s_isRidingSandSeal = 2;
    }
}

void CemuHooks::hook_PlayerLadderFix(PPCInterpreter_t* hCPU) {
    hCPU->gpr[0] = hCPU->sprNew.LR;
    hCPU->instructionPointer = 0x02D07CEC;

    if (IsFirstPerson()) {
        s_isLadderClimbing = 2;
    }
}

void CemuHooks::hook_VisualizeRayCastHits(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (VRManager::instance().XR->GetRenderer() == nullptr || !GetSettings().ShowDebugOverlay()) {
        return;
    }

    if (!VRManager::instance().Hooks->m_entityDebugger || !VRManager::instance().Hooks->m_entityDebugger->ShouldDrawRaycastLines()) {
        return;
    }

    uint32_t rayCastResultPtr = hCPU->gpr[3];
    glm::fvec3 raycastHitPos = getMemory<BEVec3>(hCPU->gpr[4]).getLE();

    ksys::phys::RayCast rayCast = {};
    readMemory(rayCastResultPtr, &rayCast);

    glm::fvec3 rayStart = rayCast.from.getLE();
    glm::fvec3 rayEnd = rayCast.to.getLE();
    
    rayStart.y += 0.5f;
    rayEnd.y += 0.5f;
    raycastHitPos.y += 0.5f;

    DebugDraw::instance().Line(rayStart, raycastHitPos, IM_COL32(255, 0, 255, 255));
    DebugDraw::instance().Line(raycastHitPos, rayEnd, IM_COL32(128, 0, 128, 128));
}
