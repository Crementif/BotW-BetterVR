[BetterVR_PerfCameraCollision_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; ==================================================================================
; Third-person camera collision. sub_2E61FC0 only produces a camera matrix the headset pose overwrites, so first person skips it.
; 0x02E61FF4 lbz r0, 0xC(r1) is the flag byte, 0x02E61FF8 cmpwi r0, 0 is taken over, 0x02E61FFC bne loc_2E622C0 is the early return.
; LR is free here (the function stashed it at 0x02E61FD8), so only r0 and r3 need preserving.

skipCameraCollisionWhenOverridden:
stwu r1, -0x20(r1)
stw r0, 0x18(r1)
mflr r0
stw r0, 0x1C(r1)
stw r3, 0x14(r1)
stw r4, 0x10(r1)

bl import.coreinit.hook_SkipCameraCollision

cmpwi r3, 0
beq keepGameCameraCollisionFlag
; any non-zero value makes the game's bne take the early return
li r0, 1
b restoreCameraCollisionFlag

keepGameCameraCollisionFlag:
lwz r0, 0x18(r1)

restoreCameraCollisionFlag:
lwz r4, 0x10(r1)
lwz r3, 0x14(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
; leave CR0 describing r0, which is what the instruction this replaced did
cmpwi r0, 0
blr

0x02E61FF8 = bla skipCameraCollisionWhenOverridden


; ==================================================================================
; The second collision pass: sub_2E65180 runs it behind the sub_2E622FC predicate, whose only two call sites are taken over here.

cameraCollisionPredicateOverride:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x18(r1)

bl import.coreinit.hook_SkipCameraCollision
cmpwi r3, 0
bne cameraCollisionPredicateSkip

lwz r3, 0x18(r1)
lis r12, 0x02E6
ori r12, r12, 0x22FC
mtctr r12
bctrl
b cameraCollisionPredicateDone

cameraCollisionPredicateSkip:
li r3, 1

cameraCollisionPredicateDone:
lwz r0, 0x1C(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x02E65238 = bla cameraCollisionPredicateOverride
0x02E6539C = bla cameraCollisionPredicateOverride
