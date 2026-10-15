[BetterVR_PlayerDirectionFixes_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x02C16EA4 = ba import.coreinit.hook_OverrideGuardDirection
0x02C91728 = ba import.coreinit.hook_OverrideThrowDirection

0x02D5A5D8 = Player__getYawIdx:
0x02D66930 = Player__setYawIdx:

; replaces the current-yaw read that the Wait/Move stick turn (0x02DA0E5C) compares against the stick's target yaw at player+0x16A0
snapToStickDirection:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)

bl import.coreinit.hook_ShouldSnapToStickDirection
cmpwi r3, 0
beq skipSnapToStickDirection

lwz r3, 0x08(r1)
addi r4, r3, 0x16A0
bl Player__setYawIdx
lwz r3, 0x08(r1)
lwz r0, 0x16A0(r3)
stw r0, 0x1694(r3)

skipSnapToStickDirection:
lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
bl Player__getYawIdx

lwz r0, 0x14(r1)
mtlr r0
addi r1, r1, 0x10
blr

0x02DA0FE4 = bla snapToStickDirection
