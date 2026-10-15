[BetterVR_HookInput_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; r3 = channel, r4 = status, r5 = capacity, r6 = optional error pointer.
; VPADRead returns the number of samples, not a success flag.
hooked_VPADRead:
stwu r1, -0x30(r1)
mflr r0
stw r0, 0x34(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)
stw r5, 0x10(r1)
stw r6, 0x14(r1)
stw r7, 0x18(r1)
bl import.vpad.VPADRead
stw r3, 0x1C(r1)

lwz r7, 0x08(r1)
cmpwi r7, 0
bne exitHookInput
; Cemu currently returns at most one sample. Leave unsupported batches untouched.
cmpwi r3, 1
bgt exitHookInput
lwz r4, 0x0C(r1)
lwz r5, 0x10(r1)
lwz r6, 0x14(r1)
bl import.coreinit.hook_InjectXRInput
cmpwi r3, 1
bne exitHookInput
stw r3, 0x1C(r1)
lwz r7, 0x14(r1)
cmpwi r7, 0
beq exitHookInput
li r0, 0
stw r0, 0(r7)

exitHookInput:
lwz r3, 0x1C(r1)
lwz r0, 0x34(r1)
mtlr r0
lwz r7, 0x18(r1)
addi r1, r1, 0x30
blr

0x030D9F48 = bla hooked_VPADRead
