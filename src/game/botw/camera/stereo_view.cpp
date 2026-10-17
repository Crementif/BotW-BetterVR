#include "pch.h"
#include "stereo_view.h"
#include "camera.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/pack_shared_memory.h"
#include "instance.h"
#include "utils/debug_draw.h"
#include "utils/render_utils.h"

using EyeSide = OpenXR::EyeSide;

static std::atomic_uint32_t s_sharedMemoryAddress = 0;
static std::atomic<float> s_flatCameraFovYRadians = glm::radians(90.0f);

uint32_t StereoView::GetSharedMemoryAddress() {
    return s_sharedMemoryAddress.load(std::memory_order_relaxed);
}

void StereoView::ResetFlatCameraFov() {
    s_flatCameraFovYRadians.store(glm::radians(90.0f), std::memory_order_relaxed);
}

static std::optional<XrFovf> TryGetRenderFOV(const std::optional<Tracking::HeadViews>& views, EyeSide side) {
    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer == nullptr || !views.has_value()) {
        return std::nullopt;
    }

    return RenderUtils::GetRenderFov(views->Fov(side), renderer->m_widenedRenderAspectRatio);
}

static std::optional<XrFovf> TryGetCombinedFOV(const std::optional<Tracking::HeadViews>& views) {
    auto leftFovOpt = TryGetRenderFOV(views, EyeSide::LEFT);
    auto rightFovOpt = TryGetRenderFOV(views, EyeSide::RIGHT);
    if (!leftFovOpt.has_value() || !rightFovOpt.has_value()) {
        return std::nullopt;
    }

    return XrFovf{
        .angleLeft = std::min(leftFovOpt->angleLeft, rightFovOpt->angleLeft),
        .angleRight = std::max(leftFovOpt->angleRight, rightFovOpt->angleRight),
        .angleUp = std::max(leftFovOpt->angleUp, rightFovOpt->angleUp),
        .angleDown = std::min(leftFovOpt->angleDown, rightFovOpt->angleDown),
    };
}

struct SharedEyeView {
    BEMatrix34& view;
    BEMatrix44& proj;
    BEType<float>& zNear;
    BEType<float>& zFar;
    BEType<float>& fovY;
    BEType<float>& aspect;
    BEType<float>& tanHalfFovY;
};

static SharedEyeView GetSharedEyeView(VRSharedMemory& sharedMem, EyeSide side) {
    if (side == EyeSide::LEFT) {
        return { sharedMem.leftView, sharedMem.leftProj, sharedMem.leftNear, sharedMem.leftFar, sharedMem.leftFovy, sharedMem.leftAspect, sharedMem.leftStuff5 };
    }
    return { sharedMem.rightView, sharedMem.rightProj, sharedMem.rightNear, sharedMem.rightFar, sharedMem.rightFovy, sharedMem.rightAspect, sharedMem.rightStuff5 };
}

template <typename Fn>
static void UpdateSharedMemory(Fn&& update) {
    const uint32_t sharedAddress = StereoView::GetSharedMemoryAddress();
    if (sharedAddress == 0) {
        return;
    }

    const uint32_t sharedMemAddress = sharedAddress + offsetof(VRSharedMemoryRoot, shared);
    VRSharedMemory sharedMem = {};
    Guest::Read(sharedMemAddress, &sharedMem);
    sharedMem.dllActive = 1;
    update(sharedMem);
    Guest::Write(sharedMemAddress, &sharedMem);
}

static void PublishFlatCameraProjectionToSharedMemory(const BESeadPerspectiveProjection& projection) {
    s_flatCameraFovYRadians.store(projection.fovYRadiansOrAngle.getLE(), std::memory_order_relaxed);
    UpdateSharedMemory([&](VRSharedMemory& sharedMem) {
        for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
            SharedEyeView eye = GetSharedEyeView(sharedMem, side);
            eye.proj = projection.matrix;
            eye.zNear = projection.zNear;
            eye.zFar = projection.zFar;
            eye.fovY = projection.fovYRadiansOrAngle;
            eye.aspect = projection.aspect;
            eye.tanHalfFovY = projection.fovyTan;
        }
    });
}

// set zNear/zFar before calling this
static void ApplyProjectionFov(BESeadPerspectiveProjection& perspectiveProjection, const XrFovf& fov) {
    data_VRProjectionMatrixOut newProjection = RenderUtils::CalculateFOVAndOffset(fov);

    perspectiveProjection.aspect = newProjection.aspectRatio;
    perspectiveProjection.fovYRadiansOrAngle = newProjection.fovY;
    float halfAngle = newProjection.fovY.getLE() * 0.5f;
    perspectiveProjection.fovySin = sinf(halfAngle);
    perspectiveProjection.fovyCos = cosf(halfAngle);
    perspectiveProjection.fovyTan = tanf(halfAngle);
    perspectiveProjection.offset.x = newProjection.offsetX;
    perspectiveProjection.offset.y = newProjection.offsetY;

    glm::fmat4 newMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), fov);
    perspectiveProjection.matrix = newMatrix;
    perspectiveProjection.deviceMatrix = RenderUtils::ApplyDeviceZTransform(newMatrix, perspectiveProjection.deviceZScale.getLE(), perspectiveProjection.deviceZOffset.getLE());

    perspectiveProjection.dirty = false;
    perspectiveProjection.deviceDirty = false;
}

static void SetCameraWorld(BESeadLookAtCamera& camera, const glm::mat4& cameraWorld) {
    const glm::mat4 view = glm::inverse(cameraWorld);
    camera.mtx.setLEMatrix(view);
    camera.pos = glm::vec3(cameraWorld[3]);
    camera.at = glm::vec3(cameraWorld[3]) - glm::vec3(view[2]);
    camera.up = glm::vec3(view[1]);
}

static void hook_GetRenderCamera(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    uint32_t cameraIn = hCPU->gpr[3];
    uint32_t cameraOut = hCPU->gpr[12];

    if (GameState::UseBlackBarsDuringEvents()) {
        return;
    }

    BESeadLookAtCamera camera = {};
    Guest::Read(cameraIn, &camera);

    if (GameState::UseFlatCameraPresentation()) {
        Camera::SetReferenceMtx(glm::inverse(glm::mat4(camera.mtx.getLEMatrix())));
        UpdateSharedMemory([&](VRSharedMemory& sharedMem) {
            sharedMem.leftView = camera.mtx;
            sharedMem.rightView = camera.mtx;
        });
        return;
    }

    auto [gameplayPos, gameplayRot] = Camera::ResolveGameplayBasePose(camera);
    Camera::UpdateReferenceMtx(gameplayPos, gameplayRot);

    // culling and both eyes have to use the same head pose
    const std::optional<Tracking::HeadViews> views = Tracking::Views();

    // Gameplay camera is set to the middle pose between the eyes
    auto [newPos, newRot] = Camera::BuildMiddlePose(views, gameplayPos, gameplayRot);

    SetCameraWorld(camera, glm::translate(glm::identity<glm::fmat4>(), newPos) * glm::mat4_cast(newRot));

    Guest::Write(cameraOut, &camera);
    hCPU->gpr[3] = cameraOut;

    // where each eye looks, or straight ahead without eye tracking
    const std::optional<glm::fvec3> gaze = Tracking::EyeGaze();
    std::optional<std::array<std::array<float, 2>, 2>> focus;
    UpdateSharedMemory([&](VRSharedMemory& sharedMem) {
        if (!views.has_value()) {
            return;
        }

        focus.emplace();
        for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
            auto [eyePos, eyeRot] = Camera::BuildEyePose(views, gameplayPos, gameplayRot, side);
            SharedEyeView eye = GetSharedEyeView(sharedMem, side);
            eye.view.setLEMatrix(glm::mat4x3(glm::inverse(glm::translate(glm::identity<glm::fmat4>(), eyePos) * glm::mat4_cast(eyeRot))));

            auto eyeFovOpt = TryGetRenderFOV(views, side);
            if (!eyeFovOpt.has_value()) {
                eye.proj = BEMatrix44();
                continue;
            }

            const float nearZ = GetSettings().GetZNear();
            const float farZ = GetSettings().GetZFar();
            const auto newProjection = RenderUtils::CalculateFOVAndOffset(eyeFovOpt.value());
            eye.proj = RenderUtils::CalculateProjectionMatrix(nearZ, farZ, eyeFovOpt.value());
            const glm::fvec3 gazeInEye = gaze.has_value() ? glm::inverse(ToGLM(views->EyePose(side).orientation)) * gaze.value() : glm::fvec3(0.0f, 0.0f, -1.0f);
            (*focus)[(size_t)side] = RenderUtils::ProjectDirectionToPicture(eyeFovOpt.value(), gazeInEye);
            eye.zNear = nearZ;
            eye.zFar = farZ;
            eye.fovY = newProjection.fovY;
            eye.aspect = newProjection.aspectRatio;
            // ViewFrustumCulling::update_2 (0x3B262B4) stores tan(fovy/2) at VFC+0x154, which the deferred passes and shadows use
            eye.tanHalfFovY = 0.5f * (tanf(eyeFovOpt->angleUp) - tanf(eyeFovOpt->angleDown));
        }
    });
    if (focus.has_value()) {
        CemuHost::SetFoveationCentres((*focus)[(size_t)EyeSide::LEFT].data(), (*focus)[(size_t)EyeSide::RIGHT].data());
    }

    Camera::UpdateDebugEyeViews(views, gameplayPos, gameplayRot);
}

static void hook_GetRenderProjection(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t projectionIn = hCPU->gpr[3];
    uint32_t projectionOut = hCPU->gpr[12];

    BESeadPerspectiveProjection perspectiveProjection = {};
    Guest::Read(projectionIn, &perspectiveProjection);

    if (GameState::UseBlackBarsDuringEvents() || perspectiveProjection.zFar == 10000.0f) {
        return;
    }

    if (GameState::UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        std::optional<float> gameFovY = RenderUtils::SanitizeGameFovY(perspectiveProjection.fovYRadiansOrAngle.getLE());
        if (!gameFovY.has_value()) {
            return;
        }

        ApplyProjectionFov(perspectiveProjection, RenderUtils::CreateSymmetricFov(gameFovY.value(), RenderUtils::CapturedImageAspectRatio));

        if (GameState::UseFlatCameraPresentation()) {
            PublishFlatCameraProjectionToSharedMemory(perspectiveProjection);
        }

        Guest::Write(projectionOut, &perspectiveProjection);
        hCPU->gpr[3] = projectionOut;
        return;
    }

    perspectiveProjection.zFar = GetSettings().GetZFar();
    perspectiveProjection.zNear = GetSettings().GetZNear();

    const std::optional<Tracking::HeadViews> views = Tracking::Views();
    auto combinedFovOpt = TryGetCombinedFOV(views);
    if (!combinedFovOpt.has_value()) {
        return;
    }
    ApplyProjectionFov(perspectiveProjection, combinedFovOpt.value());

    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        glm::fmat4 debugProjMatrix = RenderUtils::CalculateProjectionMatrix(perspectiveProjection.zNear.getLE(), perspectiveProjection.zFar.getLE(), views->Fov(side));
        glm::fmat4 debugDeviceMatrix = RenderUtils::ApplyDeviceZTransform(debugProjMatrix, perspectiveProjection.deviceZScale.getLE(), perspectiveProjection.deviceZOffset.getLE());
        DebugDraw::instance().UpdateEyeProjection(side, glm::transpose(debugDeviceMatrix));
    }

    Guest::Write(projectionOut, &perspectiveProjection);
    hCPU->gpr[3] = projectionOut;
}

static void hook_CheckIfCameraCanSeePos(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!Tracking::IsSessionRunning()) {
        hCPU->gpr[3] = 0;
        return;
    }

    uint32_t camPtr = hCPU->gpr[3];
    uint32_t posPtr = hCPU->gpr[4];
    float radius = hCPU->fpr[1].fp0;
    float nearClip = hCPU->fpr[2].fp0;
    float farClip = hCPU->fpr[3].fp0;

    BESeadLookAtCamera camera = {};
    Guest::Read(camPtr, &camera);
    const glm::fvec3 center = Guest::Get<BEVec3>(posPtr).getLE();

    Frustum frustum;

    if (GameState::UseFlatCameraPresentation()) {
        XrFovf flatCameraFov = RenderUtils::CreateSymmetricFov(s_flatCameraFovYRadians.load(std::memory_order_relaxed), RenderUtils::CapturedImageAspectRatio);
        glm::mat4 view = glm::mat4(camera.mtx.getLEMatrix());
        glm::mat4 proj = glm::transpose(RenderUtils::CalculateProjectionMatrix(nearClip, farClip, flatCameraFov));
        frustum.update(proj * view);
        hCPU->gpr[3] = frustum.checkSphere(center, radius) ? 1 : 0;
        return;
    }

    auto [gameplayPos, gameplayRot] = Camera::ResolveGameplayBasePose(camera);
    const std::optional<Tracking::HeadViews> views = Tracking::Views();
    for (EyeSide side : { EyeSide::LEFT, EyeSide::RIGHT }) {
        auto fovOpt = TryGetRenderFOV(views, side);
        if (!fovOpt.has_value()) {
            continue;
        }

        auto [pos, rot] = Camera::BuildEyePose(views, gameplayPos, gameplayRot, side);

        // pull the camera backwards a bit to account for it being a third-person game that encompassed a bigger area
        pos += rot * glm::vec3(0.0f, 0.0f, 1.0f);

        glm::mat4 view = glm::inverse(glm::translate(glm::mat4(1.0f), pos) * glm::mat4_cast(rot));
        glm::mat4 proj = glm::transpose(RenderUtils::CalculateProjectionMatrix(nearClip, farClip, fovOpt.value()));
        frustum.update(proj * view);
        if (frustum.checkSphere(center, radius)) {
            hCPU->gpr[3] = 1;
            return;
        }
    }

    hCPU->gpr[3] = 0;
}

static void hook_ModifyPixelUniformBlockData(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::UseMonoFrameBufferTemporarilyDuringMenusOrPictures()) {
        return;
    }

    auto currFovOpt = TryGetRenderFOV(Tracking::Views(), EyeSide::LEFT);
    if (!currFovOpt.has_value()) {
        return;
    }

    glm::fvec4 ubData = {};
    Guest::Read(hCPU->gpr[5], &ubData);

    auto newProjection = RenderUtils::CalculateFOVAndOffset(currFovOpt.value());
    // the center is on the opposite side of the projection offset
    ubData.x = 0.5f - newProjection.offsetX.getLE();
    ubData.y = 0.5f + newProjection.offsetY.getLE();

    Guest::Write(hCPU->gpr[5], &ubData);
}

static void hook_RegisterVRSharedMemory(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t sharedAddress = hCPU->gpr[3];
    if (s_sharedMemoryAddress.exchange(sharedAddress) != sharedAddress) {
        Log::print<RENDERING>("Registered VRSharedMemoryRoot address: 0x{:08X}", sharedAddress);
    }

    if (sharedAddress != 0) {
        uint32_t active = Tracking::IsSessionRunning() ? 1u : 0u;
        uint32_t sharedMemAddress = sharedAddress + offsetof(VRSharedMemoryRoot, shared);
        Guest::Set(sharedMemAddress + offsetof(VRSharedMemory, vrActive), active);
        Guest::Set(sharedMemAddress + offsetof(VRSharedMemory, dllActive), active);
        const FoveatedRenderingMode foveation = GetSettings().foveatedRendering;
        CemuHost::SetFoveatedShadingActive(foveation != FoveatedRenderingMode::OFF);
        CemuHost::SetFoveationExaggerated(foveation == FoveatedRenderingMode::EXAGGERATED);
    }
}

// the game copies the 3D image for camera rune photos, save files and the inventory background
static void hook_FixCameraSaveFilesAndInventory(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t originCaller = hCPU->gpr[0];
    uint32_t tempLayerCopyObject = hCPU->gpr[3];
    uint32_t frameIdx = hCPU->gpr[4];
    EyeSide side = (EyeSide)hCPU->gpr[5];

    Log::print<PPC>("[{:08X}] hook_FixCameraSaveFilesAndInventory: tempLayerCopyObject={:08X}, side={}, frameIdx={}", originCaller, tempLayerCopyObject, side, frameIdx);
    if (auto* renderer = VRManager::instance().XR->GetRenderer(); renderer != nullptr) {
        renderer->SignalGameCapturing3DFrameBuffer((long)frameIdx);
    }
}

void StereoView::RegisterHooks() {
    CemuHost::RegisterHook("hook_RegisterVRSharedMemory", &hook_RegisterVRSharedMemory);
    CemuHost::RegisterHook("hook_GetRenderCamera", &hook_GetRenderCamera);
    CemuHost::RegisterHook("hook_GetRenderProjection", &hook_GetRenderProjection);
    CemuHost::RegisterHook("hook_CheckIfCameraCanSeePos", &hook_CheckIfCameraCanSeePos);
    CemuHost::RegisterHook("hook_ModifyPixelUniformBlockData", &hook_ModifyPixelUniformBlockData);
    CemuHost::RegisterHook("hook_FixCameraSaveFilesAndInventory", &hook_FixCameraSaveFilesAndInventory);
}
