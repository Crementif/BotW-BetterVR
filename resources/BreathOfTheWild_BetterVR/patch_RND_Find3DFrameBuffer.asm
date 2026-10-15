[BetterVR_Find3DFrameBuffer_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; r26 holds the agl::RenderBuffer object
hookPostHDRComposedImage:
mflr r0
stwu r1, -0x40(r1)
stw r0, 0x44(r1)
stw r3, 0x3C(r1)
stw r4, 0x38(r1)
stw r5, 0x34(r1)
stw r6, 0x30(r1)
stw r7, 0x2C(r1)
stw r8, 0x28(r1)
stfs f1, 0x24(r1)
stfs f2, 0x20(r1)
stfs f3, 0x1C(r1)
stfs f4, 0x18(r1)
stfs f5, 0x14(r1)
stfs f6, 0x10(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

; Cemu hands the scene's color and depth images to the module through host events
mr r3, r26
bl hook_emitCaptureEvents3D

exit_hookPostHDRComposedImage:
lwz r0, 0x44(r1)
mtlr r0
lwz r0, 0x0C(r1)
lwz r0, 0x08(r1)
lwz r3, 0x3C(r1)
lwz r4, 0x38(r1)
lwz r5, 0x34(r1)
lwz r6, 0x30(r1)
lwz r7, 0x2C(r1)
lwz r8, 0x28(r1)
lfs f1, 0x24(r1)
lfs f2, 0x20(r1)
lfs f3, 0x1C(r1)
lfs f4, 0x18(r1)
lfs f5, 0x14(r1)
lfs f6, 0x10(r1)
addi r1, r1, 0x40

lmw r18, 0xB0(r1) ; original instruction
blr

0x039B3044 = bla hookPostHDRComposedImage


; for one frame, disable clearing the framebuffer (used for separating 2D hud from 3D scene)
custom_turnOnTempLayerCopy:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)
stw r5, 0x04(r1)

; publish the capture onto the GPU timeline before anything else, so the Vulkan layer
; learns about it in command order rather than from a CPU-side flag it races with
; (see sMonoCaptureFrames in src/updateSharedMemory.cpp). Clobbers r3, so reload it.
bl hook_signalMonoCapture
lwz r3, 0x0C(r1)

lwz r3, 0x86C(r3)
li r4, 1
stb r4, 0x318(r3)

; also uses r0 which contains the LR register to decide if this is the camera or a save game snapshot
lis r4, currentFrameCounter@ha
lwz r4, currentFrameCounter@l(r4)
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
bla import.coreinit.hook_FixCameraSaveFilesAndInventory

lwz r5, 0x04(r1)
lwz r4, 0x08(r1)
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x0340A8A8 = ba custom_turnOnTempLayerCopy