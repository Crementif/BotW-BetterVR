[BetterVR_StereoRendering_ScreenChecks_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; --------------------------------------------------------------------------------
; Hook the uniform buffer that is used to test the screen position (normally centered) for the magnesis target

hook_GX2SetPixelUniformBlock:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)
stw r7, 0x0C(r1)

; check size of uniform block being set
; r3 is uniform block handle
; r4 is the size of the uniform block data in bytes
; r5 is the pointer to the uniform block data to be set
cmpwi r4, 0x20
bne skipHook_GX2SetPixelUniformBlock

; modify values in the uniform block data to be set
bla import.coreinit.hook_ModifyPixelUniformBlockData

skipHook_GX2SetPixelUniformBlock:
bla import.gx2.GX2SetPixelUniformBlock

lwz r7, 0x0C(r1)
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x038B2B2C = bla hook_GX2SetPixelUniformBlock

; --------------------------------------------------------------------------------
; Fix the stamina gauge position that's normally attached to the player position

0x02FB2468 = orig_StaminaGaugeScreenPositionFn:

hook_fixStaminaGaugeToScreenPosition:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

lis r3, orig_StaminaGaugeScreenPositionFn@ha
addi r3, r3, orig_StaminaGaugeScreenPositionFn@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; call original function to get the original screen position

lwz r4, 0x18(r1) ; r4 is a pointer to the 2D screen position that we need to overwrite
lwz r5, 0x14(r1) ; r5 is a pointer to the world position
bla import.coreinit.hook_FixStaminaGaugeScreenPosition

lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

; replace the player pos to screen position with static screen coords
0x02FB2608 = bla hook_fixStaminaGaugeToScreenPosition

; force extra stamina gauge icons to just stay in place instead of dynamically hanging onto the regular stamina wheel once it appears
0x02FB2760 = bla import.coreinit.hook_FixExtraStaminaGaugeIconPositions