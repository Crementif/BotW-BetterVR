[BetterVR_ImproveGUI_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

ENABLED_SCREEN_STACK_OFFSET = 0x04

createNewScreenHook:
mflr r0
stwu r1, -0x0C(r1)
stw r0, 0x10(r1)

cmpwi r5, 0x63

; li r3, 0
; stw r3, ENABLED_SCREEN_STACK_OFFSET(r1)
; addi r3, r1, 0x04
bl import.coreinit.hook_CreateNewScreen
; lwz r3, ENABLED_SCREEN_STACK_OFFSET(r1)
; cmpwi r3, 0
; beq exit_createNewScreenHook

exit_createNewScreenHook:
;lwz r3, 0x08(r1)
lwz r0, 0x10(r1)
mtlr r0
addi r1, r1, 0x0C
blr

0x0305EAF8 = bla createNewScreenHook