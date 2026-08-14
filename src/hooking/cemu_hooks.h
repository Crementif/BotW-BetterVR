#pragma once
#include "entity_debugger.h"
#include "utils/mod_settings.h"

class CemuHooks {
public:
    CemuHooks() {
        m_cemuHandle = GetModuleHandleA(NULL);
        checkAssert(m_cemuHandle != NULL, "Failed to get handle of Cemu process which is required for interfacing with Cemu!");

        gameMeta_getTitleId = (gameMeta_getTitleIdPtr_t)GetProcAddress(m_cemuHandle, "gameMeta_getTitleId");
        memory_getBase = (memory_getBasePtr_t)GetProcAddress(m_cemuHandle, "memory_getBase");
        osLib_registerHLEFunction = (osLib_registerHLEFunctionPtr_t)GetProcAddress(m_cemuHandle, "osLib_registerHLEFunction");
        checkAssert(gameMeta_getTitleId != nullptr && memory_getBase != nullptr && osLib_registerHLEFunction != nullptr, "Failed to get function pointers of Cemu functions! Is this hook being used on Cemu?");

        bool isSupportedTitleId = gameMeta_getTitleId() == 0x00050000101C9300 || gameMeta_getTitleId() == 0x00050000101C9400 || gameMeta_getTitleId() == 0x00050000101C9500;
        checkAssert(isSupportedTitleId, std::format("Expected title IDs for Breath of the Wild (00050000-101C9300, 00050000-101C9400 or 00050000-101C9500) but received {:16x}!", gameMeta_getTitleId()).c_str());

        s_memoryBaseAddress = (uint64_t)memory_getBase();
        checkAssert(s_memoryBaseAddress != 0, "Failed to get memory base address of Cemu process!");

        InitWindowHandles();

        osLib_registerHLEFunction("coreinit", "hook_UpdateSettings", &hook_UpdateSettings);

        // Actor Hooks
        osLib_registerHLEFunction("coreinit", "hook_UpdateActorList", &hook_UpdateActorList);
        osLib_registerHLEFunction("coreinit", "hook_CreateNewActor", &hook_CreateNewActor);

        osLib_registerHLEFunction("coreinit", "hook_SetRigidBodyVelocity", &hook_SetRigidBodyVelocity);
        osLib_registerHLEFunction("coreinit", "hook_BeginRoomscaleMovement", &hook_BeginRoomscaleMovement);
        osLib_registerHLEFunction("coreinit", "hook_PrepareRoomscaleRaycast", &hook_PrepareRoomscaleRaycast);
        osLib_registerHLEFunction("coreinit", "hook_ConsumeRoomscaleRaycast", &hook_ConsumeRoomscaleRaycast);
        osLib_registerHLEFunction("coreinit", "hook_BuildRoomscaleWarpTransform", &hook_BuildRoomscaleWarpTransform);
        osLib_registerHLEFunction("coreinit", "hook_SetRigidBodyTransform", &hook_SetRigidBodyTransform);
        osLib_registerHLEFunction("coreinit", "hook_SetRigidBodyScale", &hook_SetRigidBodyScale);
        osLib_registerHLEFunction("coreinit", "hook_SetRigidBodyPosition", &hook_SetRigidBodyPosition);
        osLib_registerHLEFunction("coreinit", "hook_SetRigidBodyPositionAndRotation", &hook_SetRigidBodyPositionAndRotation);

        // Stereo Rendering/Camera Hooks
        osLib_registerHLEFunction("coreinit", "hook_BeginCameraSide", &hook_BeginCameraSide);
        osLib_registerHLEFunction("coreinit", "hook_ModifyLightPrePassProjectionMatrix", &hook_ModifyLightPrePassProjectionMatrix);
        osLib_registerHLEFunction("coreinit", "hook_OverwriteSeadPerspectiveProjectionSet", &hook_OverwriteSeadPerspectiveProjectionSet);
        osLib_registerHLEFunction("coreinit", "hook_ModifyProjectionUsingCamera", &hook_ModifyProjectionUsingCamera);
        osLib_registerHLEFunction("coreinit", "hook_CheckIfCameraCanSeePos", &hook_CheckIfCameraCanSeePos);
        osLib_registerHLEFunction("coreinit", "hook_UpdateCameraForGameplay", &hook_UpdateCameraForGameplay);
        osLib_registerHLEFunction("coreinit", "hook_AdjustGameplayCameraPivot", &hook_AdjustGameplayCameraPivot);
        osLib_registerHLEFunction("coreinit", "hook_SnapTurnCameraPivot", &hook_SnapTurnCameraPivot);
        osLib_registerHLEFunction("coreinit", "hook_SnapTurnCameraTailPivot", &hook_SnapTurnCameraTailPivot);
        osLib_registerHLEFunction("coreinit", "hook_AddGameCameraStickYaw", &hook_AddGameCameraStickYaw);
        osLib_registerHLEFunction("coreinit", "hook_CaptureClimbWallSurface", &hook_CaptureClimbWallSurface);
        osLib_registerHLEFunction("coreinit", "hook_GetRenderCamera", &hook_GetRenderCamera);
        osLib_registerHLEFunction("coreinit", "hook_GetRenderProjection", &hook_GetRenderProjection);
        osLib_registerHLEFunction("coreinit", "hook_EndCameraSide", &hook_EndCameraSide);
        osLib_registerHLEFunction("coreinit", "hook_RouteActorJob", &hook_RouteActorJob);

        osLib_registerHLEFunction("coreinit", "hook_UseCameraDistance", &hook_UseCameraDistance);
        osLib_registerHLEFunction("coreinit", "hook_ReplaceCameraMode", &hook_ReplaceCameraMode);
        osLib_registerHLEFunction("coreinit", "hook_GetEventName", &hook_GetEventName);
        osLib_registerHLEFunction("coreinit", "hook_PlayerNormalChangeState", &hook_PlayerNormalChangeState);
        osLib_registerHLEFunction("coreinit", "hook_ShouldSkipEventCamera", &hook_ShouldSkipEventCamera);
        osLib_registerHLEFunction("coreinit", "hook_OverwriteFloatParam", &hook_OverwriteFloatParam);
        osLib_registerHLEFunction("coreinit", "hook_StoreLadderState", &hook_StoreLadderState);
        osLib_registerHLEFunction("coreinit", "hook_PlayerIsRiding", &hook_PlayerIsRiding);
        osLib_registerHLEFunction("coreinit", "hook_PlayerIsRidingSandSeal", &hook_PlayerIsRidingSandSeal);
        osLib_registerHLEFunction("coreinit", "hook_FixStaminaGaugeScreenPosition", &hook_FixStaminaGaugeScreenPosition);
        osLib_registerHLEFunction("coreinit", "hook_FixExtraStaminaGaugeIconPositions", &hook_FixExtraStaminaGaugeIconPositions);
        osLib_registerHLEFunction("coreinit", "hook_ModifyPixelUniformBlockData", &hook_ModifyPixelUniformBlockData);

        // First-Person Model Hooks
        osLib_registerHLEFunction("coreinit", "hook_SetActorOpacity", &hook_SetActorOpacity);
        osLib_registerHLEFunction("coreinit", "hook_CalculateModelOpacity", &hook_CalculateModelOpacity);
        osLib_registerHLEFunction("coreinit", "hook_ModifyBoneMatrix", &hook_ModifyBoneMatrix);
        osLib_registerHLEFunction("coreinit", "hook_ChangeWeaponMtx", &hook_ChangeWeaponMtx);
        osLib_registerHLEFunction("coreinit", "hook_OverrideThrowDirection", &hook_OverrideThrowDirection);
        osLib_registerHLEFunction("coreinit", "hook_OverrideGuardDirection", &hook_OverrideGuardDirection);

        // First-Person Weapon Hooks
        osLib_registerHLEFunction("coreinit", "hook_EquipWeapon", &hook_EquipWeapon);
        osLib_registerHLEFunction("coreinit", "hook_DropEquipment", &hook_DropEquipment);
        osLib_registerHLEFunction("coreinit", "hook_EnableWeaponAttackSensor", &hook_EnableWeaponAttackSensor);
        osLib_registerHLEFunction("coreinit", "hook_SetPlayerWeaponScale", &hook_SetPlayerWeaponScale);
        osLib_registerHLEFunction("coreinit", "hook_GetContactLayerOfAttack", &hook_GetContactLayerOfAttack);

        // Input Hooks
        osLib_registerHLEFunction("coreinit", "hook_InjectXRInput", &hook_InjectXRInput);
        osLib_registerHLEFunction("coreinit", "hook_XRRumble_VPADControlMotor", &hook_XRRumble_VPADControlMotor);
        osLib_registerHLEFunction("coreinit", "hook_XRRumble_VPADStopMotor", &hook_XRRumble_VPADStopMotor);
        osLib_registerHLEFunction("coreinit", "hook_FixLadder", &hook_FixLadder);

        // Misc. Hooks
        osLib_registerHLEFunction("coreinit", "hook_OSReportToConsole", &hook_OSReportToConsole);
        osLib_registerHLEFunction("coreinit", "hook_DropWeaponLogging", &hook_DropWeaponLogging);
        osLib_registerHLEFunction("coreinit", "hook_FindWeaponEquipSlot", &hook_FindWeaponEquipSlot);
        osLib_registerHLEFunction("coreinit", "hook_ModifyHandModelAccessSearch", &hook_ModifyHandModelAccessSearch);
        osLib_registerHLEFunction("coreinit", "hook_CreateNewScreen", &hook_CreateNewScreen);
        osLib_registerHLEFunction("coreinit", "hook_FixUIBlending", &hook_FixUIBlending);
        osLib_registerHLEFunction("coreinit", "hook_FixCameraSaveFilesAndInventory", &hook_FixCameraSaveFilesAndInventory);
        osLib_registerHLEFunction("coreinit", "hook_ProfileSectionBegin", &hook_ProfileSectionBegin);
        osLib_registerHLEFunction("coreinit", "hook_ProfileSectionEnd", &hook_ProfileSectionEnd);
        osLib_registerHLEFunction("coreinit", "hook_LoadDynamicVec3", &hook_LoadDynamicVec3);
        osLib_registerHLEFunction("coreinit", "hook_LoadDynamicBool", &hook_LoadDynamicBool);
        osLib_registerHLEFunction("coreinit", "hook_OverrideArrowShotTransform", &hook_OverrideArrowShotTransform);
        osLib_registerHLEFunction("coreinit", "hook_CaptureArrowShootDecision", &hook_CaptureArrowShootDecision);
        osLib_registerHLEFunction("coreinit", "hook_ArrowFpsScale", &hook_ArrowFpsScale);
        osLib_registerHLEFunction("coreinit", "hook_VisualizeRayCastHits", &hook_VisualizeRayCastHits);
    };
    ~CemuHooks() {
        FreeLibrary(m_cemuHandle);
    };

    static HWND m_cemuTopWindow;
    static HWND m_cemuRenderWindow;
    static uint64_t s_memoryBaseAddress;

    std::unique_ptr<class EntityDebugger> m_entityDebugger;
    static std::array<class WeaponMotionAnalyser, 2> m_motionAnalyzers;
    static std::array<uint32_t, 2> m_heldWeapons;
    static std::array<uint32_t, 2> m_heldWeaponsLastUpdate;
    static std::array<WeaponType, 2> s_handWeaponTypes;
    static bool s_arrowNockedInRightHand;

    static constexpr bool s_twoHandGripEnabled = true;
    static constexpr float TwoHandGripButtonThreshold = 0.5f;
    static bool s_twoHandGripActive;
    static bool s_twoHandGripConsumesGrabInput;
    static std::atomic_uint32_t s_recordingOutputMode;
    static uint32_t s_playerAddress;
    static uint32_t s_damageStateNameAddress;
    static uint32_t s_playerMtxAddress;
    static uint32_t s_cameraMtxAddress;
    static glm::fvec3 s_playerPos;
    static glm::mat4 s_lastCameraMtx;
    static glm::mat4 GetFreshCameraReferenceMtx();
    // If the user is unable to control the camera, we can guess that they're in a cutscene
    struct HybridEventSettings {
        bool firstPerson;                  // use Link's perspective, ignore the animated event camera
        bool disablePlayerDrivenLinkHands; // let event control the hands instead of the VR controllers
        bool ignoreCameraRotation;         // some events will pan the camera, but in first-person it should usually be ignored to avoid nausea. Doors opening is okay, but panning down to a chest is not.
        bool demoEnableCameraInput;        // there's already events that allow user camera control. This isn't used or overwritten atm.
    };

    static uint32_t GetFramesSinceLastCameraUpdate() { return s_framesSinceLastCameraUpdate.load(); }
    static bool IsInGame() {
        // todo: check if 3 frames is the right threshold
        return GetFramesSinceLastCameraUpdate() <= 4 && !IsScreenOpen(ScreenId::PauseMenuInfo_00);
    }
    static bool IsShowingMenu() {
        return !IsInGame() || IsScreenOpen(ScreenId::ShopBG_00) || IsScreenOpen(ScreenId::MessageDialog);
    }
    static bool IsRiding(bool ignoreSandSeal = false) {
        if (ignoreSandSeal) {
            return s_isRiding > 0;
        }
        return s_isRiding > 0 || s_isRidingSandSeal > 0;
    }
    static bool UseMonoFrameBufferTemporarilyDuringMenusOrPictures();
    // experimental mono-throughput mode: the game renders one (left) eye and the
    // compositor currently mirrors it; depth reprojection is not yet validated
    static bool ShouldUseSynthesizedRightEye();
    // Requests the forked Cemu record-once/issue-twice path. This is the safety
    // predicate only; entering the mono guest loop additionally requires a fresh
    // ABI acknowledgement checked by IpcControl.
    static bool ShouldRequestStereoInstancing();
    // Right-eye queue reuse keeps the game's two-pass housekeeping cadence, but the
    // right pass intentionally does not rebuild model draw lists. The compositor must
    // therefore submit the already-copied left eye for both OpenXR views.
    static bool ShouldSynthesizeRightEyeFromQueueReuse();
    static bool ShouldCaptureSynthStereoMatrices();
    static std::optional<glm::fmat4> GetSynthReprojectionMatrix(bool sourceIsRight = false);
    static bool GetStereoInstancingPayload(std::array<glm::fmat4, 5>& matrices,
        BESeadLookAtCamera& rightCamera, BESeadPerspectiveProjection& rightProjection);
    // the render-skip bitmask with load/cutscene safety gating applied
    static uint32_t GetEffectiveRenderSkipMask();

    static std::string s_currentEvent;
    static std::string s_currentPlayerNormalState;
    static std::string s_lastRequestedPlayerNormalState;
    static HybridEventSettings s_currentEventSettings;
    static std::unordered_map<std::string, HybridEventSettings> s_eventSettings;
    static void initCutsceneDefaultSettings(uint32_t ppc_TableOfCutsceneEventsSettingsOffset);

    static bool HasActiveCutscene();
    static EventMode GetEventModeWithOverride();
    static std::optional<HybridEventSettings> GetFirstPersonSettingsForActiveEvent();
    static bool IsFirstPerson();
    static bool IsThirdPerson();
    static bool IsTwoHandGripEngaged();
    static bool IsBowDrawEngaged();
    static bool IsArrowNearBowStart();
    static bool UseBlackBarsDuringEvents();
    static bool IsScreenOpen(ScreenId screen);
    static bool IsScreenVisible(ScreenId screen);
    static bool IsAnyFadeScreenVisible();
    static glm::fvec3 GetAppliedRoomscaleHeadPosition();
    static float GetRoomscaleFadeAmount();
    static void UpdateFloatParamOverrides();

    uint64_t GetCurrentTitleId() const {
        if (gameMeta_getTitleId == nullptr) {
            return 0;
        }

        return gameMeta_getTitleId();
    }

private:
    HMODULE m_cemuHandle;

    osLib_registerHLEFunctionPtr_t osLib_registerHLEFunction;
    memory_getBasePtr_t memory_getBase;
    gameMeta_getTitleIdPtr_t gameMeta_getTitleId;

    static std::atomic_uint32_t s_framesSinceLastCameraUpdate;
    static uint32_t s_isLadderClimbing;
    static uint32_t s_isRiding;
    static uint32_t s_isRidingSandSeal;

    static void InitWindowHandles();

    static std::pair<glm::vec3, glm::fquat> CalculateVRWorldPose(const BESeadLookAtCamera& camera, uint8_t side);

    static void hook_UpdateSettings(PPCInterpreter_t* hCPU);

    // Actor Hooks
    static void hook_UpdateActorList(PPCInterpreter_t* hCPU);
    static void hook_CreateNewActor(PPCInterpreter_t* hCPU);

    static void hook_SetRigidBodyVelocity(PPCInterpreter_t* hCPU);
    static void hook_BeginRoomscaleMovement(PPCInterpreter_t* hCPU);
    static void hook_PrepareRoomscaleRaycast(PPCInterpreter_t* hCPU);
    static void hook_ConsumeRoomscaleRaycast(PPCInterpreter_t* hCPU);
    static void hook_BuildRoomscaleWarpTransform(PPCInterpreter_t* hCPU);
    static void hook_SetRigidBodyTransform(PPCInterpreter_t* hCPU);
    static void hook_SetRigidBodyScale(PPCInterpreter_t* hCPU);
    static void hook_SetRigidBodyPosition(PPCInterpreter_t* hCPU);
    static void hook_SetRigidBodyPositionAndRotation(PPCInterpreter_t* hCPU);

    // Camera Hooks
    static void hook_BeginCameraSide(PPCInterpreter_t* hCPU);
    static void hook_ModifyLightPrePassProjectionMatrix(PPCInterpreter_t* hCPU);
    static void hook_ModifyProjectionUsingCamera(PPCInterpreter_t* hCPU);
    static void hook_CheckIfCameraCanSeePos(PPCInterpreter_t* hCPU);
    static void hook_OverwriteSeadPerspectiveProjectionSet(PPCInterpreter_t* hCPU);
    static void hook_UpdateCameraForGameplay(PPCInterpreter_t* hCPU);
    static void hook_AdjustGameplayCameraPivot(PPCInterpreter_t* hCPU);
    static void hook_SnapTurnCameraPivot(PPCInterpreter_t* hCPU);
    static void hook_SnapTurnCameraTailPivot(PPCInterpreter_t* hCPU);
    static void hook_AddGameCameraStickYaw(PPCInterpreter_t* hCPU);
    static void hook_CaptureClimbWallSurface(PPCInterpreter_t* hCPU);
    static void hook_GetRenderCamera(PPCInterpreter_t* hCPU);
    static void hook_GetRenderProjection(PPCInterpreter_t* hCPU);
    static void hook_EndCameraSide(PPCInterpreter_t* hCPU);
    static void hook_RouteActorJob(PPCInterpreter_t* hCPU);

    static void hook_UseCameraDistance(PPCInterpreter_t* hCPU);
    static void hook_ReplaceCameraMode(PPCInterpreter_t* hCPU);
    static void hook_GetEventName(PPCInterpreter_t* hCPU);
    static void hook_PlayerNormalChangeState(PPCInterpreter_t* hCPU);
    static void hook_ShouldSkipEventCamera(PPCInterpreter_t* hCPU);
    static void hook_OverwriteFloatParam(PPCInterpreter_t* hCPU);
    static void hook_StoreLadderState(PPCInterpreter_t* hCPU);
    static void hook_VisualizeRayCastHits(PPCInterpreter_t* hCPU);
    static void hook_FixLadder(PPCInterpreter_t* hCPU);
    static void hook_PlayerIsRiding(PPCInterpreter_t* hCPU);
    static void hook_PlayerIsRidingSandSeal(PPCInterpreter_t* hCPU);
    static void hook_ModifyPixelUniformBlockData(PPCInterpreter_t* hCPU);

    // First-Person Model Hooks
    static void hook_SetActorOpacity(PPCInterpreter_t* hCPU);
    static void hook_CalculateModelOpacity(PPCInterpreter_t* hCPU);
    static void hook_ModifyBoneMatrix(PPCInterpreter_t* hCPU);
    static void hook_ChangeWeaponMtx(PPCInterpreter_t* hCPU);
    static void hook_OverrideThrowDirection(PPCInterpreter_t* hCPU);
    static void hook_OverrideGuardDirection(PPCInterpreter_t* hCPU);
    static void hook_FixStaminaGaugeScreenPosition(PPCInterpreter_t* hCPU);
    static void hook_FixExtraStaminaGaugeIconPositions(PPCInterpreter_t* hCPU);

    // First-Person Weapon Hooks
    static void hook_EquipWeapon(PPCInterpreter_t* hCPU);
    static void hook_DropEquipment(PPCInterpreter_t* hCPU);
    static void hook_EnableWeaponAttackSensor(PPCInterpreter_t* hCPU);
    static void hook_SetPlayerWeaponScale(PPCInterpreter_t* hCPU);
    static void hook_GetContactLayerOfAttack(PPCInterpreter_t* hCPU);

    // Input Hooks
    static void hook_InjectXRInput(PPCInterpreter_t* hCPU);
    static void hook_XRRumble_VPADControlMotor(PPCInterpreter_t* hCPU);
    static void hook_XRRumble_VPADStopMotor(PPCInterpreter_t* hCPU);

    // Misc Hooks
    static void hook_OSReportToConsole(PPCInterpreter_t* hCPU);
    static void hook_DropWeaponLogging(PPCInterpreter_t* hCPU);
    static void hook_FindWeaponEquipSlot(PPCInterpreter_t* hCPU);
    static void hook_ModifyHandModelAccessSearch(PPCInterpreter_t* hCPU);
    static void hook_CreateNewScreen(PPCInterpreter_t* hCPU);
    static void hook_FixUIBlending(PPCInterpreter_t* hCPU);
    static void hook_FixCameraSaveFilesAndInventory(PPCInterpreter_t* hCPU);
    static void hook_ProfileSectionBegin(PPCInterpreter_t* hCPU);
    static void hook_ProfileSectionEnd(PPCInterpreter_t* hCPU);
    static void hook_LoadDynamicVec3(PPCInterpreter_t* hCPU);
    static void hook_LoadDynamicBool(PPCInterpreter_t* hCPU);
    static void hook_OverrideArrowShotTransform(PPCInterpreter_t* hCPU);
    static void hook_CaptureArrowShootDecision(PPCInterpreter_t* hCPU);
    static void hook_ArrowFpsScale(PPCInterpreter_t* hCPU);

public:
    template <typename T>
    static void writeMemoryBE(uint64_t offset, T* valuePtr) {
        *valuePtr = swapEndianness(*valuePtr);
        memcpy((void*)(s_memoryBaseAddress + offset), (void*)valuePtr, sizeof(T));
    }

    template <typename T>
    static void writeMemory(uint64_t offset, T* valuePtr) {
        memcpy((void*)(s_memoryBaseAddress + offset), (void*)valuePtr, sizeof(T));
    }

    template <typename T>
    static void readMemoryBE(uint64_t offset, T* resultPtr) {
        uint64_t memoryAddress = s_memoryBaseAddress + offset;
        memcpy(resultPtr, (void*)memoryAddress, sizeof(T));
        *resultPtr = swapEndianness(*resultPtr);
    }

    template <typename T>
    static void readMemory(uint64_t offset, T* resultPtr) {
        uint64_t memoryAddress = s_memoryBaseAddress + offset;
        memcpy(resultPtr, (void*)memoryAddress, sizeof(T));
    }

    template <typename T>
    static auto getMemory(uint64_t offset) {
        if constexpr (is_BEType_v<T>) {
            T result;
            readMemory(offset, &result);
            return result;
        }
        else {
            BEType<T> result;
            readMemory(offset, &result);
            return result;
        }
    }

    template <typename T>
    static void setMemory(uint64_t offset, T value) {
        if constexpr (is_BEType_v<T>) {
            writeMemory(offset, &value);
        }
        else {
            BEType<T> beValue = value;
            writeMemory(offset, &beValue);
        }
    }
};

