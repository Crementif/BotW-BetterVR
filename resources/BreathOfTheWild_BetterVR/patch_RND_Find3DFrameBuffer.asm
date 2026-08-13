[BetterVR_Find3DFrameBuffer_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

str_printClear3DBuffer_left:
.string "Clearing both 3D buffers with left eye"
str_printClear3DBuffer_right:
.string "Clearing both 3D buffers with right eye"

magic3DClearingValues:
.float (0.0 / 32.0)
magic3DColorValue_leftSide:
.float 0.123456789
magic3DColorValue_rightSide:
.float 0.987654321

; capture index 1 marks a mono capture whose right eye the host synthesizes
magic3DClearingValueMono:
.float (1.0 / 32.0)

magic3DDepthValue_leftSide:
.float 0.0123456789
magic3DDepthValue_rightSide:
.float 0.163987654

magic3DColorValue_cntr0:
.float 0.0
magic3DColorValue_cntr1:
.float 1.0

; r10 holds the agl::RenderBuffer object
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

; get pointers to the agl::RenderBuffer::mColorBuffer and agl::RenderBuffer::mDepthTarget arrays

mr r3, r26 ; r26 holds the agl::RenderBuffer object
addi r3, r3, 0x1C ; r3 is now the agl::RenderBuffer::mColorBuffer array
lwz r3, 0(r3) ; r3 is now the agl::RenderBuffer::mColorBuffer[0] object
cmpwi r3, 0
beq exit_hookPostHDRComposedImage
addi r3, r3, 0xBC ; r3 is now the agl::RenderBuffer::mColorBuffer[0]::mGX2FrameBuffer object

mr r4, r26 ; r26 holds the agl::RenderBuffer object
addi r4, r4, 0x3C ; r4 is now the agl::RenderBuffer::mDepthTarget array
lwz r4, 0(r4) ; r4 is now the agl::RenderBuffer::mDepthTarget object
cmpwi r4, 0
beq exit_hookPostHDRComposedImage
addi r4, r4, 0xBC ; r4 is now the agl::RenderBuffer::mDepthTarget::mGX2FrameBuffer object

; use 0 and 1 as alpha clear values for the left/right eye, and use 1 and 2 as color clear values
lis r7, currentEyeSide@ha
lwz r7, currentEyeSide@l(r7)
cmpwi r7, 0
beq leftEye3DValues
b rightEye3DValues

leftEye3DValues:
; store the left and right values in a certain order to indicate which eye is being cleared
lis r7, magic3DClearingValues@ha
lfs f2, magic3DClearingValues@l+0x4(r7)
lfs f3, magic3DClearingValues@l+0x8(r7)

; depth clear value
lis r7, magic3DDepthValue_leftSide@ha
lfs f5, magic3DDepthValue_leftSide@l(r7)

; log clearing action
li r5, 0
mr r8, r3
lis r3, str_printClear3DBuffer_left@ha
addi r3, r3, str_printClear3DBuffer_left@l
bla import.coreinit.hook_OSReportToConsole
mr r3, r8
b continueTo3DClear

rightEye3DValues:
; store the left and right values in a certain order to indicate which eye is being cleared
lis r7, magic3DClearingValues@ha
lfs f3, magic3DClearingValues@l+0x4(r7)
lfs f2, magic3DClearingValues@l+0x8(r7)
; depth clear value
lis r7, magic3DDepthValue_rightSide@ha
lfs f5, magic3DDepthValue_rightSide@l(r7)

; log clearing action
li r5, 1
mr r8, r3
lis r3, str_printClear3DBuffer_right@ha
addi r3, r3, str_printClear3DBuffer_right@l
bla import.coreinit.hook_OSReportToConsole
mr r3, r8
b continueTo3DClear

; void GX2ClearBuffersEx(GX2ColorBuffer* colorBuffer, GX2DepthBuffer* depthBuffer, float r, float g, float b, float a, float depthClearValue, uint8 stencilClearValue, GX2ClearFlags clearFlags)
continueTo3DClear:
; check if clear calls should be skipped
lis r12, useStubHooks_preventClearCalls@ha
lwz r12, useStubHooks_preventClearCalls@l(r12)
cmpwi r12, 1
beq exit_hookPostHDRComposedImage

; identifier type of clear (0 = stereo 3D capture, 1 = mono 3D capture)
lis r7, magic3DClearingValues@ha
lfs f1, magic3DClearingValues@l+0x0(r7)
lis r7, currentFrameIsMono@ha
lwz r7, currentFrameIsMono@l(r7)
cmpwi r7, 0
beq monoCaptureIdxDone
lis r7, magic3DClearingValueMono@ha
lfs f1, magic3DClearingValueMono@l(r7)
monoCaptureIdxDone:

; store frame counter in alpha channel
lis r12, currentFrameCounter@ha
lwz r0, currentFrameCounter@l(r12)
cmpwi r0, 0
bne loadOneInAlphaChannel
loadZeroInAlphaChannel:
lis r12, magic3DColorValue_cntr0@ha
lfs f4, magic3DColorValue_cntr0@l(r12)
li r5, 0 ; stencil clear value for the depth buffer clearing to know its frame 0
b continueAfterAlphaChannel
loadOneInAlphaChannel:
lis r12, magic3DColorValue_cntr1@ha
lfs f4, magic3DColorValue_cntr1@l(r12)
li r5, 1 ; stencil clear value for the depth buffer clearing to know its frame 1
continueAfterAlphaChannel:
; GX2ClearFlags: clear depth and stencil using the provided values
li r6, 3
bla import.gx2.GX2ClearBuffersEx

exit_hookPostHDRComposedImage:
lwz r0, 0x44(r1)
mtlr r0
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


;stwu r1, -0x20(r1)
;stw r3, 0x1C(r1)
;stw r4, 0x18(r1)
;stw r5, 0x14(r1)
;stw r6, 0x10(r1)
;
;lis r3, currentEyeSide@ha
;lwz r3, currentEyeSide@l(r3)
;cmpwi r3, 0
;lis r3, str_printClear3DDepthBuffer_left@ha
;addi r3, r3, str_printClear3DDepthBuffer_left@l
;beq actualPrint_2
;
;lis r3, str_printClear3DDepthBuffer_right@ha
;addi r3, r3, str_printClear3DDepthBuffer_right@l
;actualPrint_2:
;li r4, 10
;crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
;bl import.coreinit.hook_OSReportToConsole
;
;lwz r3, 0x1C(r1)
;lwz r4, 0x18(r1)
;lwz r5, 0x14(r1)
;lwz r6, 0x10(r1)
;addi r1, r1, 0x20


0x039B3044 = bla hookPostHDRComposedImage
;0x0397AB30 = cmpw r3, r3


; for one frame, disable clearing the framebuffer (used for separating 2D hud from 3D scene)
custom_turnOnTempLayerCopy:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)
stw r5, 0x04(r1)

; original code
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