[BetterVR_BowString_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; the held bow's ModelNW, published by the DLL once per game frame; 1 until the DLL has the slot
bowStringModelUnit:
.int 1

0x0399728C = game_ModelNW_postCalcWorldBody:

; gsys::ModelNW::m_78_postCalcWorld entry: every model unit passes here after its world matrices are final
; and before CalcMtxBlock turns them into skinning matrices
bowStringPostCalcWorld:
lis r12, bowStringModelUnit@ha
lwz r11, bowStringModelUnit@l(r12)
cmpw r3, r11
beq bowStringPose
cmpwi r11, 1
beq bowStringRegister
b bowStringResume

bowStringPose:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)
stw r5, 0x10(r1)
stw r6, 0x14(r1)
stw r7, 0x18(r1)
stw r8, 0x1C(r1)
bl import.coreinit.hook_PoseBowString
b bowStringRestore

bowStringRegister:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)
stw r5, 0x10(r1)
stw r6, 0x14(r1)
stw r7, 0x18(r1)
stw r8, 0x1C(r1)
lis r3, bowStringModelUnit@ha
addi r3, r3, bowStringModelUnit@l
bl import.coreinit.hook_RegisterBowStringSlot

bowStringRestore:
lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lwz r5, 0x10(r1)
lwz r6, 0x14(r1)
lwz r7, 0x18(r1)
lwz r8, 0x1C(r1)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30

; replays the displaced mflr r0 and resumes through LR, see notify_uniformBlockDestroy for why not bctr
bowStringResume:
mflr r0
lis r12, game_ModelNW_postCalcWorldBody@ha
addi r12, r12, game_ModelNW_postCalcWorldBody@l
mtlr r12
blr
.int 0x00000000

0x03997288 = ba bowStringPostCalcWorld
