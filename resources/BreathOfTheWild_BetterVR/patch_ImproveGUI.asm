[BetterVR_ImproveGUI_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


; the game normally draws most of the UI on top of the 3D color buffer
; the VR mod uses the alpha channel of the 2D UI as a mask to present a transparent version of the HUD on top of the 3D framebuffer
; this patch basically fixes a few elements not using the right blending options
; for example, the minimap, dialogue boxes and various other things to overwrite the alpha of previously rendered 2D elements
; which cause gaps inside the image that's shown in VR
hook_FixUIBlending:
stwu r1, -0x10(r1)
mflr r0
stw r0, 0x14(r1)
stw r3, 0xC(r1)
stw r4, 0x8(r1)

bl import.coreinit.hook_FixUIBlending

bl import.gx2.GX2SetBlendControl

lwz r4, 0x8(r1)
lwz r3, 0xC(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x03C58BF8 = bla hook_FixUIBlending
0x03C58CC0 = bla hook_FixUIBlending
0x03C58D68 = bla hook_FixUIBlending

; hook for Screen::createNewScreen to track when certain screens are created

ENABLED_SCREEN_STACK_OFFSET = 0x04

createNewScreenHook:
mflr r0
stwu r1, -0x0C(r1)
stw r0, 0x10(r1)

cmpwi r5, 0x63

bl import.coreinit.hook_CreateNewScreen

exit_createNewScreenHook:
lwz r0, 0x10(r1)
mtlr r0
addi r1, r1, 0x0C
blr

0x0305EAF8 = bla createNewScreenHook

; sead::Projection::cameraPosToScreenPos, so name labels and markers line up with the floating HUD panel
0x030C1134 = ba import.coreinit.hook_PlaceScreenMarker
