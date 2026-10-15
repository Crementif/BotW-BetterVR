[BetterVR_StereoRendering_CameraAndProjection_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x1046CF88 = Camera__sInstance:

const_0:
.float 0.0
0x10323118 = const_epsilon:

0x105978C4 = sDefaultCameraMatrix:

modifiedCopy_seadLookAtCamera:
.float 0,0,0,0  ; mtx
.float 0,0,0,0
.float 0,0,0,0
.int 0x1027B294 ; vtable
.float 10,2,-20 ; pos
.float 10,4,-19 ; target
.float 0,1,0    ; up

agl__lyr__Layer__getRenderCamera:
lhz r0, 0x52(r3)
clrlwi. r12, r0, 31
bne testCameraFlags
extrwi. r11, r0, 1,26
beq returnCameraFrom1stParam

testCameraFlags:
extrwi. r11, r0, 1,30
beq returnCameraFrom1stParam
lwz r12, 0x19C(r3)
cmpwi r12, 0
bne returnDefaultCamera

returnCameraFrom1stParam:
lwz r0, 0x48(r3)
lis r3, sDefaultCameraMatrix@ha
addi r3, r3, sDefaultCameraMatrix@l
cmpwi r0, 0
beqlr ; if (layer->renderCamera == nullptr) return sDefaultCameraMatrix;
mr r3, r0 ; r3 = layer->renderCamera

; camera.pos.x.getLE() != 0.0f
lis r12, const_0@ha
lfs f12, const_0@l(r12)
lfs f13, 0x34(r3)
fcmpu cr0, f12, f13
beqlr ; return layer->renderCamera

; std::fabs(camera.at.z.getLE()) < std::numeric_limits<float>::epsilon()
fmr f13, f1
lfs f1, 0x48(r3)
.int 0xFC200A10 ; fabs f1, f1
fmr f12, f1
fmr f1, f13
lis r12, const_epsilon@ha
lfs f13, const_epsilon@l(r12)
fcmpu cr0, f12, f13
bltlr ; return layer->renderCamera

lis r12, sMonoWithoutSession@ha
lwz r12, sMonoWithoutSession@l(r12)
cmpwi r12, 1
beqlr ; no VR session: return the plain render camera

lis r12, import.coreinit.hook_GetRenderCamera@ha
addi r12, r12, import.coreinit.hook_GetRenderCamera@l
mtctr r12
lis r11, currentEyeSide@ha
lwz r11, currentEyeSide@l(r11)
lis r12, modifiedCopy_seadLookAtCamera@ha
addi r12, r12, modifiedCopy_seadLookAtCamera@l
bctr ; jump to DLL hook_GetRenderCamera

returnDefaultCamera:
addi r3, r12, 4
blr

0x03AE4AA0 = ba agl__lyr__Layer__getRenderCamera


modifiedCopy_seadPerspectiveProjection:
.byte 0,0,0,0 ; dirty, deviceDirty, pad, pad
.float 0,0,0,0 ; mtx
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0 ; deviceMtx
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0
.int 0 ; devicePosture
.float 0 ; deviceZScale
.float 0 ; deviceZOffset
.int 0x1027B54C ; vtable
.float 0.1 ; near
.float 25000 ; far
.float 0.8726647 ; angle
.float 0.4226183 ; fovySin
.float 0.9063078 ; fovyCos
.float 0.4663077 ; fovyTan
.float 1.7777778 ; aspect
.float 0 ; offsetX
.float 0 ; offsetY


0x1027B54C = seadPerspectiveProjection_vtbl:
0x10597928 = sDefaultSeadProjection:

agl__lyr__Layer__getRenderProjection:
lhz r0, 0x54(r3)
extrwi. r0, r0, 1,30
beq returnProjectionFrom1stParam
lwz r12, 0x19C(r3)
cmpwi r12, 0
bne useSpecialProjection

returnProjectionFrom1stParam:
lwz r12, 0x4C(r3)
lis r3, sDefaultSeadProjection@ha
addi r3, r3, sDefaultSeadProjection@l
cmpwi r12, 0
beqlr
mr r3, r12

; prevent modifying anything but sead::PerspectiveProjection
lis r11, seadPerspectiveProjection_vtbl@ha
addi r11, r11, seadPerspectiveProjection_vtbl@l
lwz r12, 0x90(r3)
cmpw r12, r11
bnelr

lis r12, sMonoWithoutSession@ha
lwz r12, sMonoWithoutSession@l(r12)
cmpwi r12, 1
beqlr ; no VR session: return the plain projection

lis r12, import.coreinit.hook_GetRenderProjection@ha
addi r12, r12, import.coreinit.hook_GetRenderProjection@l
mtctr r12
lis r12, currentEyeSide@ha
lwz r0, currentEyeSide@l(r12)
lis r12, modifiedCopy_seadPerspectiveProjection@ha
addi r12, r12, modifiedCopy_seadPerspectiveProjection@l
bctr ; jump to DLL hook_GetRenderProjection

useSpecialProjection:
lwz r3, 0xEC(r12)
blr

0x03AE4AEC = ba agl__lyr__Layer__getRenderProjection

; =================================================================================
; captures the light UniformBlock at the dcbz call in updateLightUniformBuffers (0x33DC1D8), r3 = the block
; =================================================================================

0x03A83CBC = agl__UniformBlock__dcbz:

capturedXLow:
.int 0

capture_dcbz_wrapper:
stwu r1, -0x20(r1)
mflr r0
stw r0, 0x24(r1)

lis r12, capturedXLow@ha
stw r3, capturedXLow@l(r12)

lis r12, agl__UniformBlock__dcbz@ha
addi r12, r12, agl__UniformBlock__dcbz@l
mtctr r12
bctrl

lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

0x33DC1D8 = bla capture_dcbz_wrapper