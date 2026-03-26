[BetterVR_HookInput_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; r3 = channel
; r4 = status
; r5 = count
; r6 = error
; ret r3 = 0 for success, 1 for failure
hooked_VPADRead:
; prologue
stwu r1, -0x14(r1)
stw r7, 0x14(r1) ; save r7 to use as a temporary register
mflr r7
stw r7, 0x18(r1)

; save registers and run the original function to allow for combined inputs
stw r6, 0x10(r1)
stw r5, 0x0C(r1)
stw r4, 0x08(r1)
stw r3, 0x04(r1)
bl import.vpad.VPADRead

; check if VPAD_CHANNEL parameter is 0 (in r3 register)
lwz r7, 0x04(r1)
cmpwi r7, 0
bne exitHookInput

; check error pointer for OK status (in r6 register)
lwz r7, 0x10(r1)
lwz r7, 0x00(r7)
cmpwi r7, 0
;beq exitHookInput ; uncomment to only enable XR input when also using gamepad input

; override input with XR input
lwz r7, 0x04(r1)
bl import.coreinit.hook_InjectXRInput
; override status with OK if injection was successful
cmpwi r3, 1
bne exitHookInput
lwz r7, 0x10(r1)
li r3, 0
stw r3, 0x00(r7)
li r3, 1

exitHookInput:
; epilogue
lwz r7, 0x18(r1)
mtlr r7
; preserve original function's return values
; lwz r3, 0x04(r1)
; lwz r4, 0x08(r1)
; lwz r5, 0x0C(r1)
; lwz r6, 0x10(r1)
lwz r7, 0x14(r1)
addi r1, r1, 0x14
blr

0x030D9F48 = bla hooked_VPADRead
