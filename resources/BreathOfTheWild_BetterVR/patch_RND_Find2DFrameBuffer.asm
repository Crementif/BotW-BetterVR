[BetterVR_Find2DFrameBuffer_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


magic2DColorValue:
.float (2.0 / 32.0)
.float 0.123456789
.float 0.987654321

magic2DColorValue_cntr0:
.float 0.0
magic2DColorValue_cntr1:
.float 1.0

str_printClear2DColorBuffer_left:
.string "Clearing 2D color buffer with left eye"
str_printClear2DColorBuffer_right:
.string "Clearing 2D color buffer with right eye"

clear2DColorBuffer:
mflr r5
stwu r1, -0x20(r1)
stw r5, 0x24(r1)

stfs f1, 0x20(r1)
stfs f2, 0x1C(r1)
stfs f3, 0x18(r1)
stfs f4, 0x14(r1)
stw r3, 0x10(r1)
stw r4, 0x0C(r1)

; ignore the right eye for now
; todo: have the GUI update for both eyes, and then capture the right eye since it'd be more responsive
; lis r3, currentEyeSide@ha
; lwz r3, currentEyeSide@l(r3)
; cmpwi r3, 1
; beq skipClearing2DColorBuffer

; in synthesized-right-eye mode there is only one pass per frame, and it has to carry
; the HUD capture (which the host consumes from the left-eye-ordered magic values)
lis r3, currentFrameIsMono@ha
lwz r3, currentFrameIsMono@l(r3)
cmpwi r3, 0
bne leftEye2DValues

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq leftEye2DValues
b rightEye2DValues

leftEye2DValues:
; store the left and right values in a certain order to indicate left eye is being cleared
lis r3, magic2DColorValue@ha
lfs f2, magic2DColorValue@l+0x4(r3)
lfs f3, magic2DColorValue@l+0x8(r3)

; log clearing action
lis r3, str_printClear2DColorBuffer_right@ha
addi r3, r3, str_printClear2DColorBuffer_right@l
bl import.coreinit.hook_OSReportToConsole
b continueTo2DClear

rightEye2DValues:
; store the left and right values in a certain order to indicate right eye is being cleared
lis r3, magic2DColorValue@ha
lfs f3, magic2DColorValue@l+0x4(r3)
lfs f2, magic2DColorValue@l+0x8(r3)

; log clearing action
lis r3, str_printClear2DColorBuffer_right@ha
addi r3, r3, str_printClear2DColorBuffer_right@l
bl import.coreinit.hook_OSReportToConsole
b continueTo2DClear

continueTo2DClear:
; check if clear calls should be skipped
lis r12, useStubHooks_preventClearCalls@ha
lwz r12, useStubHooks_preventClearCalls@l(r12)
cmpwi r12, 1
beq skipClearing2DColorBuffer

lis r3, magic2DColorValue@ha
lfs f1, magic2DColorValue@l+0x0(r3)

; load current frame counter into alpha
lis r3, currentFrameCounter@ha
lwz r3, currentFrameCounter@l(r3)
cmpwi r3, 0
beq useClearValue_cntr0
b useClearValue_cntr1
useClearValue_cntr0:
lis r3, magic2DColorValue_cntr0@ha
lfs f4, magic2DColorValue_cntr0@l(r3)
b continueAfterLoadingAlpha
useClearValue_cntr1:
lis r3, magic2DColorValue_cntr1@ha
lfs f4, magic2DColorValue_cntr1@l(r3)
continueAfterLoadingAlpha:

; clear the GX2 texture for the right eye, which translates to a vkCmdClearColorImage call that identifies the Vulkan image
mr r3, r30 ; r3 is now the agl::RenderBuffer object
cmpwi r3, 0
beq skipClearing2DColorBuffer
addi r3, r3, 0x1C ; r3 is now the agl::RenderBuffer::mColorBuffer array
lwz r3, 0(r3) ; r3 is now the agl::RenderBuffer::mColorBuffer[0] object
cmpwi r3, 0
beq skipClearing2DColorBuffer
addi r3, r3, 0xBC ; r3 is now the agl::RenderBuffer::mColorBuffer[0]::mGX2FrameBuffer object

bl import.gx2.GX2ClearColor

; GX2DrawDone shouldn't be used since it's slow...
; but it makes sure that Cemu translates the clear command directly instead of (sometimes) next frame
;bl import.gx2.GX2DrawDone

skipClearing2DColorBuffer:
lfs f1, 0x20(r1)
lfs f2, 0x1C(r1)
lfs f3, 0x18(r1)
lfs f4, 0x14(r1)
lwz r3, 0x10(r1)

lwz r5, 0x24(r1)
mtlr r5
addi r1, r1, 0x20

mr r5, r30 ; original instruction
blr

0x03A147B4 = bla clear2DColorBuffer
