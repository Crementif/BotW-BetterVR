[BetterVR_CameraControls_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; disable camera recentering using the shield button
0x02B96E10 = li r3, 0

; ==================================================================================
; For gameplay and culling reasons, we update the camera position earlier then just before rendering (see GetRenderCamera hook)
; GetRenderCamera() is still required since this function is only ran once per each pair of eyes being rendered.
; GetRenderCamera() discards the position and eyes that this function sets, since it can't easily apply only the differential.
; This function also lacks the ability to modify the up vector, so roll is not possible here, but its sufficient for gameplay.
updateCameraPositionAndTarget:
; repeat instructions from either branches
lfs f0, 0xEC0(r31)
stfs f0, 0x5CC(r31)
lfs f13, 0xEB8(r31)
stfs f13, 0x5C4(r31)

mflr r0
stwu r1, -0x58(r1)
stw r0, 0x5C(r1)
stw r3, 0x54(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
bl import.coreinit.hook_UpdateCameraForGameplay

exit_updateCameraPositionAndTarget:
lwz r3, 0x54(r1)
lwz r0, 0x5C(r1)
mtlr r0
addi r1, r1, 0x58

addi r3, r31, 0xE78
blr


0x02C054FC = bla updateCameraPositionAndTarget
0x02C05590 = bla updateCameraPositionAndTarget


; ==================================================================================

; CameraChase::buildAnchorWorldPos serves all six anchor modes, and 0x02B97C78 is its last write to
; out_world_pos, inside the guard that means an anchor was built. r30 is out_world_pos, r31 is
; current_spherical, and LR is reloaded from the frame afterwards, so the bla may clobber it.
adjustGameplayCameraPivot_AnchorOutput:
stfs f12, 4(r30)

stwu r1, -0x20(r1)
mflr r0
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

mr r3, r29
mr r4, r30
mr r5, r31
bl import.coreinit.hook_AdjustGameplayCameraPivot

mr r4, r30
mr r5, r31
bl import.coreinit.hook_SnapTurnCameraPivot

lwz r0, 0x24(r1)
mtlr r0
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
addi r1, r1, 0x20
blr

0x02B97C78 = bla adjustGameplayCameraPivot_AnchorOutput


; ==================================================================================

cameraModePtr:
.int 0

storeCameraModePtr:
mr r3, r31

mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

lis r4, cameraModePtr@ha
stw r3, cameraModePtr@l(r4)

lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x02B966A4 = bla storeCameraModePtr

useCameraFinder:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

; r3 is modified by the hook, if needed
lis r4, cameraModePtr@ha
lwz r4, cameraModePtr@l(r4)
lwz r5, 0x0C(r3) ; load vtable pointer from current camera mode

bl import.coreinit.hook_ReplaceCameraMode

lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0

mr r31, r3
blr

0x02B8FCA4 = bla useCameraFinder


; ==================================================================================
; disables CameraChase's atMoveOffset
; also prevents camera from slowly drifting panning towards where Link is walking
0x02B9D164 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D184 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D1A4 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D1C4 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D1E4 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D204 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D224 = bla import.coreinit.hook_OverwriteFloatParam

0x02B9D244 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D264 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D284 = bla import.coreinit.hook_OverwriteFloatParam

; prevent camera from connecting to things
0x02B9D2E4 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D304 = bla import.coreinit.hook_OverwriteFloatParam
0x02B9D324 = bla import.coreinit.hook_OverwriteFloatParam


; ==================================================================================

snapTurnCameraTailAndConvert:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)

mr r3, r31
bl import.coreinit.hook_SnapTurnCameraTailPivot

lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)

lis r12, 0x02E5
ori r12, r12, 0x5864
mtctr r12
bctrl

lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x02BEEE48 = bla snapTurnCameraTailAndConvert
0x02BEF014 = bla snapTurnCameraTailAndConvert


; ==================================================================================
; Report the yaw delta both camera modes derive from the camera stick, so first-person's held yaw
; can follow it instead of correcting it away every frame.

; CameraChase adds the frame-scaled delta at 0x194(r1) to its yaw while in anchor mode 3
captureCameraChaseStickYaw:
stwu r1, -0x20(r1)
mflr r0
stw r0, 0x24(r1)

lfs f1, 0x1B4(r1) ; the delta, 0x20 higher up than in the game's own frame
bl import.coreinit.hook_AddGameCameraStickYaw

lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20

lfs f12, 0x194(r1) ; replaced instruction
blr

0x02B9C96C = bla captureCameraChaseStickYaw

; CameraTail adds f28 * f24 to its yaw right after this load. Both are non-volatile and final here.
captureCameraTailStickYaw:
stwu r1, -0x20(r1)
mflr r0
stw r0, 0x24(r1)

fmuls f1, f28, f24 ; the same delta the following fmadds applies
bl import.coreinit.hook_AddGameCameraStickYaw

lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20

lfs f1, 0x98(r31) ; replaced instruction
blr

0x02BEEBCC = bla captureCameraTailStickYaw


; ==================================================================================
; The climb action's wall state holds the contact point at 0x60 and the wall normal at 0x54. This is
; the last place in the sweep where both are final, with the state still in r31.
captureClimbWallSurface:
stwu r1, -0x20(r1)
mflr r0
stw r0, 0x24(r1)
stw r4, 0x08(r1)
stw r5, 0x0C(r1)
stw r6, 0x10(r1)
stw r7, 0x14(r1)
stw r8, 0x18(r1)

mr r3, r31
bl import.coreinit.hook_CaptureClimbWallSurface

lwz r4, 0x08(r1)
lwz r5, 0x0C(r1)
lwz r6, 0x10(r1)
lwz r7, 0x14(r1)
lwz r8, 0x18(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20

mr r3, r31 ; replaced instruction
blr

0x03354C08 = bla captureClimbWallSurface


; workaround for ladder climbing issue
; Always sets the ladder mode to 4 which allows pressing A to jump up ladders
; Sets the ladder mode to 1 when player is moving the stick downwards to allow sliding down ladders
0x02D69E04 = ba import.coreinit.hook_FixLadder

0x02D07CE8 = ba import.coreinit.hook_StoreLadderState
