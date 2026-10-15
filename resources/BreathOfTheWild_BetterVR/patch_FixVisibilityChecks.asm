[BetterVR_FixVisibilityChecks_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x03191BA8 = Camera__getAltPerspectiveProjection:
0x03191BA0 = Camera__getPerspectiveProjection:

custom_checkIfCameraCanSeePos:
mflr r11
stwu r1, -0x20(r1)
stw r11, 0x24(r1)
stw r3, 0x1C(r1)
stw r5, 0x18(r1)
stw r6, 0x14(r1)
stw r7, 0x10(r1)
stw r8, 0x0C(r1)

mr r8, r0

lis r3, Camera__getPerspectiveProjection@ha
addi r3, r3, Camera__getPerspectiveProjection@l
mtctr r3
lis r3, Camera__sInstance@ha
lwz r3, Camera__sInstance@l(r3)
bctrl
mr r5, r3

lis r3, Camera__getAltPerspectiveProjection@ha
addi r3, r3, Camera__getAltPerspectiveProjection@l
mtctr r3
lis r3, Camera__sInstance@ha
lwz r3, Camera__sInstance@l(r3)
bctrl
mr r6, r3

lwz r3, 0x1C(r1)
; r5 = perspective projection
; r6 = alt perspective projection
lwz r7, 0x24(r1) ; load LR
mr r0, r8 ; restore original r0
bla import.coreinit.hook_CheckIfCameraCanSeePos

lwz r8, 0x0C(r1)
lwz r7, 0x10(r1)
lwz r6, 0x14(r1)
lwz r5, 0x18(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

; fix all visibility checks to use our custom function that will do a query twice for each eye
0x0318FFA8 = li r0, 0
0x0318FFAC = ba custom_checkIfCameraCanSeePos

0x03190118 = li r0, 1
0x0319011C = ba custom_checkIfCameraCanSeePos
