[BetterVR_UpdateCamera_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


data_cameraMatrixIn:
oldPosX: ; Input for the calculations done in the Vulkan layer
.float 0.0
oldPosY:
.float 0.0
oldPosZ:
.float 0.0
oldTargetX:
.float 0.0
oldTargetY:
.float 0.0
oldTargetZ:
.float 0.0

data_cameraMatrixOut:
newPosX: ; New post-calculated values from the Vulkan layer
.float 0.0
newPosY:
.float 0.0
newPosZ:
.float 0.0
newTargetX:
.float 0.0
newTargetY:
.float 0.0
newTargetZ:
.float 0.0
newRotX:
.float 0.0
newRotY:
.float 0.0
newRotZ:
.float 0.0

CAM_OFFSET_POS = 0x5C0
CAM_OFFSET_TARGET = 0x5CC
CAM_OFFSET_FOV = 0x5E4


tempStuff:
.int 0

calcCameraMatrix:
lwz r0, 0x1c(r1) ; original instruction

lfs f0, CAM_OFFSET_POS+0x0(r31)
lis r7, oldPosX@ha
stfs f0, oldPosX@l(r7)
lfs f0, CAM_OFFSET_POS+0x4(r31)
lis r7, oldPosY@ha
stfs f0, oldPosY@l(r7)
lfs f0, CAM_OFFSET_POS+0x8(r31)
lis r7, oldPosZ@ha
stfs f0, oldPosZ@l(r7)

lfs f0, CAM_OFFSET_TARGET+0x0(r31)
lis r7, oldTargetX@ha
stfs f0, oldTargetX@l(r7)
lfs f0, CAM_OFFSET_TARGET+0x4(r31)
lis r7, oldTargetY@ha
stfs f0, oldTargetY@l(r7)
lfs f0, CAM_OFFSET_TARGET+0x8(r31)
lis r7, oldTargetZ@ha
stfs f0, oldTargetZ@l(r7)

lis r7, tempStuff@ha
stw r31, tempStuff@l(r7)


lis r7, continueCodeAddr@ha
addi r7, r7, continueCodeAddr@l
lis r30, data_cameraMatrixIn@ha
addi r30, r30, data_cameraMatrixIn@l
lis r31, data_cameraMatrixOut@ha
addi r31, r31, data_cameraMatrixOut@l
b import.coreinit.hook_UpdateCamera

blr ; will return if the hook isn't connected since otherwise it'll jump to continueCodeAddr

continueCodeAddr:
lis r7, tempStuff@ha
lwz r31, tempStuff@l(r7)


lis r7, newPosX@ha
lfs f0, newPosX@l(r7)
stfs f0, CAM_OFFSET_POS(r31)
lis r7, newPosY@ha
lfs f0, newPosY@l(r7)
stfs f0, CAM_OFFSET_POS+0x4(r31)
lis r7, newPosZ@ha
lfs f0, newPosZ@l(r7)
stfs f0, CAM_OFFSET_POS+0x8(r31)

lis r7, newTargetX@ha
lfs f0, newTargetX@l(r7)
stfs f0, CAM_OFFSET_TARGET+0x0(r31)
lis r7, newTargetY@ha
lfs f0, newTargetY@l(r7)
stfs f0, CAM_OFFSET_TARGET+0x4(r31)
lis r7, newTargetZ@ha
lfs f0, newTargetZ@l(r7)
stfs f0, CAM_OFFSET_TARGET+0x8(r31)

blr

0x02C05500 = bla calcCameraMatrix
0x02C05598 = bla calcCameraMatrix


changeCameraRotation:
stfs f10, 0x18(r31)

lis r8, newRotX@ha
lfs f10, newRotX@l(r8)
stfs f10, 0x18(r31)

lis r8, newRotY@ha
lfs f10, newRotY@l(r8)
stfs f10, 0x1C(r31)

lis r8, newRotZ@ha
lfs f10, newRotZ@l(r8)
stfs f10, 0x20(r31)

blr

0x02E57FF0 = bla changeCameraRotation