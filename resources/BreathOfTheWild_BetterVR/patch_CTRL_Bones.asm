[BetterVR_HandBones_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

ModelUnitVtableOffset = 0x74
ModelUnitGetMaterialNumVtableOffset = 0xBC
ModelUnitSetMaterialVisibleVtableOffset = 0x1B4
gsys_ModelUnit_setXluAlpha = 0x039E4E08

0x03C6FE5C = copyMatrix34:

0x03821B64 = ksys_phys_ModelBoneAccessor_getBoneName:

custom_gsys_ModelUnit_getBoneLocalMatrix:
stwu r1, -0x30(r1)
mflr r0
stw r0, 0x34(r1)
stw r31, 0x0C(r1)
stw r30, 0x08(r1)
mr r30, r5
stw r29, 0x14(r1)
stw r28, 0x18(r1)
stw r27, 0x1C(r1)
mr r29, r3
lwz r9, 0x04(r12) ; store key->modelAccessHandle->name
stw r9, 0x20(r1)

; r3 = gsys::ModelUnit*
; r4 = sead::Matrix34*
; r5 = Vec3*
; r6 = int boneIdx
lwz r9, 0x9C(r3)
lwz r12, 0xC(r9)
slwi r0, r6, 6
lhz r10, 4(r9)
add r31, r12, r0
ori r10, r10, 4
lis r3, copyMatrix34@ha
addi r3, r3, copyMatrix34@l
mtctr r3
addi r3, r31, 0x10
sth r10, 4(r9) ; src transform from model unit
; r4 is already the destination matrix
bctrl ; bl copyMatrix34
lfs f0, 4(r31)
stfs f0, 0(r30)
lfs f0, 8(r31)
stfs f0, 4(r30)
lfs f0, 0xC(r31)
stfs f0, 8(r30)

; call custom bone matrix function
; r6 = char* boneName
; r5 = Vec3* scale
; r4 = sead::Matrix34*
; r3 = gsys::ModelUnit*

mr r27, r6

lwz r6, 0x20(r1) ; load name

lwz r3, 0x0C(r1) ; r31 = ksys::phys::ModelBoneAccessor*
lwz r3, 0x10(r3) ; ModelBoneAccessor::gsysModel
mr r7, r29 ; pass the source gsys::ModelUnit* to the HLE hook

bla import.coreinit.hook_ModifyBoneMatrix

mr r6, r27

mr r27, r7 ; r7 is volatile, keep the face-material flag across the vtable call below

; r9 = gsys::ModelUnit*, r10 = material index, r11 = visible
cmpwi r9, 0
beq custom_gsys_ModelUnit_getBoneLocalMatrix_HeadMaterialDone
stw r8, 0x24(r1)
mr r3, r9
mr r4, r10
mr r5, r11
lwz r12, ModelUnitVtableOffset(r3)
lwz r12, ModelUnitSetMaterialVisibleVtableOffset(r12)
mtctr r12
bctrl
lwz r8, 0x24(r1)

custom_gsys_ModelUnit_getBoneLocalMatrix_HeadMaterialDone:
cmpwi r27, 0
beq custom_gsys_ModelUnit_getBoneLocalMatrix_Restore

lis r12, player_face_model_unit@ha
addi r12, r12, player_face_model_unit@l
stw r29, 0(r12)
lis r12, hide_face_materials@ha
addi r12, r12, hide_face_materials@l
stw r8, 0(r12)

cmpwi r8, 0
beq custom_gsys_ModelUnit_getBoneLocalMatrix_Restore

mr r3, r29
lwz r12, ModelUnitVtableOffset(r3)
lwz r12, ModelUnitGetMaterialNumVtableOffset(r12)
mtctr r12
bctrl
mr r28, r3

lis r30, material_alpha_zero@ha
addi r30, r30, material_alpha_zero@l
b custom_gsys_ModelUnit_getBoneLocalMatrix_Apply

custom_gsys_ModelUnit_getBoneLocalMatrix_Apply:
lis r31, gsys_ModelUnit_setXluAlpha@ha
addi r31, r31, gsys_ModelUnit_setXluAlpha@l

; loop material indices 3-7: Mt_Eyeball_L, Mt_Eyeball_R, Mt_Eyelashes, Mt_Face, Mt_Head
li r27, 3
b custom_gsys_ModelUnit_getBoneLocalMatrix_CheckLoop

custom_gsys_ModelUnit_getBoneLocalMatrix_LoopBody:
mr r3, r29
mr r4, r27
stw r27, 0x28(r1)
li r5, -1
lfs f1, 0(r30)
mtctr r31
bctrl
lwz r27, 0x28(r1)
addi r27, r27, 1

custom_gsys_ModelUnit_getBoneLocalMatrix_CheckLoop:
cmpw r27, r28
bge custom_gsys_ModelUnit_getBoneLocalMatrix_Restore
cmpwi r27, 7
bgt custom_gsys_ModelUnit_getBoneLocalMatrix_Restore
b custom_gsys_ModelUnit_getBoneLocalMatrix_LoopBody

custom_gsys_ModelUnit_getBoneLocalMatrix_Restore:

lwz r0, 0x34(r1)
mtlr r0

lwz r27, 0x1C(r1)
lwz r28, 0x18(r1)
lwz r29, 0x14(r1)
lwz r31, 0x0C(r1)
lwz r30, 0x08(r1)
addi r1, r1, 0x30
blr

player_face_model_unit:
.int 0
hide_face_materials:
.int 0

material_alpha_zero:
.float 0.0

setXluAlpha_return_addr = 0x039E4E18

custom_setXluAlpha_interceptor:
lis r12, player_face_model_unit@ha
addi r12, r12, player_face_model_unit@l
lwz r0, 0(r12)
cmpw r3, r0
bne run_original_setXluAlpha

lis r12, hide_face_materials@ha
addi r12, r12, hide_face_materials@l
lwz r0, 0(r12)
cmpwi r0, 0
beq run_original_setXluAlpha

cmpwi r4, 3
blt run_original_setXluAlpha
cmpwi r4, 7
bgt run_original_setXluAlpha

force_alpha_zero:
lis r12, material_alpha_zero@ha
addi r12, r12, material_alpha_zero@l
lfs f1, 0(r12)

run_original_setXluAlpha:
stwu r1, -0x30(r1)
stmw r26, 0x08(r1)
mflr r0
stfd f31, 0x20(r1)
lis r12, setXluAlpha_return_addr@ha
addi r12, r12, setXluAlpha_return_addr@l
mtctr r12
bctr

; hooks ksys::phys::ModelBoneAccessor::copyModelPoseToHavok at 0382094C to get the custom bone matrix instead of the original one
0x03820C38 = lis r8, custom_gsys_ModelUnit_getBoneLocalMatrix@ha
0x03820C3C = addi r9, r8, custom_gsys_ModelUnit_getBoneLocalMatrix@l

; hooks gsys::ModelUnit::setXluAlpha to intercept opacity writes for the player's face model unit
0x039E4E08 = lis r12, custom_setXluAlpha_interceptor@ha
0x039E4E0C = addi r12, r12, custom_setXluAlpha_interceptor@l
0x039E4E10 = mtctr r12
0x039E4E14 = bctr
