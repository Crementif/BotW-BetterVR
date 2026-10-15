[BetterVR_EntityController_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

RoomscaleScratch = 0x20
RoomscaleCurrent = RoomscaleScratch + 0x00
RoomscaleCastFrom = RoomscaleScratch + 0x0C
RoomscaleCastDelta = RoomscaleScratch + 0x18
RoomscaleHit = RoomscaleScratch + 0x24
RoomscaleGroundHit = RoomscaleScratch + 0x30
RoomscaleQueryType = RoomscaleScratch + 0x34
RoomscaleSweepRadius = RoomscaleScratch + 0x38
RoomscaleHitNormal = RoomscaleScratch + 0x3C
RoomscaleSavedCurrent = 0x68
RoomscaleOwnLayer = 0x74
RoomscaleSavedLR = 0x78
RoomscaleQuery = 0x88
RoomscaleShapeVtable = RoomscaleQuery + 0x3C
RoomscaleIterator = 0xD0
RoomscaleCapsuleQuery = 0x118
RoomscaleCapsuleShapeVtable = RoomscaleCapsuleQuery + 0x3C

ApplyRoomscaleAfterPlayerVelocity:
stwu r1, -0x180(r1)
mflr r0
stw r0, 0x184(r1)
stw r30, 0x14(r1)
stw r31, 0x18(r1)

mr r3, r31
addi r4, r1, RoomscaleScratch
bla import.coreinit.hook_BeginRoomscaleMovement
lwz r31, 0x18(r1)

cmpwi r3, 0
beq ApplyRoomscaleAfterPlayerVelocity_Continue

mr r3, r31
addi r4, r1, RoomscaleCurrent
bl RigidBody_getPosition

ApplyRoomscaleAfterPlayerVelocity_Loop:
addi r3, r1, RoomscaleScratch
bla import.coreinit.hook_PrepareRoomscaleRaycast
lwz r31, 0x18(r1)
cmpwi r3, 2
beq ApplyRoomscaleAfterPlayerVelocity_Warp
cmpwi r3, 1
bne ApplyRoomscaleAfterPlayerVelocity_Continue

lwz r0, RoomscaleQueryType(r1)
cmpwi r0, 1
beq ApplyRoomscaleAfterPlayerVelocity_GroundProbe
cmpwi r0, 2
beq ApplyRoomscaleAfterPlayerVelocity_GroundProbe

li r0, 3
stw r0, RoomscaleOwnLayer(r1)
li r6, 0
li r7, 0x80
li r8, 0
stw r8, 0x08(r1)
lis r9, nullPosition@ha
addi r9, r9, nullPosition@l
lis r10, emptyString@ha
addi r10, r10, emptyString@l
addi r3, r1, RoomscaleQuery
addi r4, r1, RoomscaleOwnLayer
addi r5, r1, RoomscaleGroundHit
lfs f1, RoomscaleSweepRadius(r1)
bl RoomscaleSphereCast_ctor

lwz r12, RoomscaleQuery + 0x04(r1)
addi r3, r1, RoomscaleQuery
addi r4, r31, 0x208
bl ShapeCast_copyLayers

addi r3, r1, RoomscaleQuery
addi r4, r1, RoomscaleCastFrom
addi r5, r1, RoomscaleHit
bl ShapeCast_setStartAndEnd

lwz r5, RoomscaleQuery + 0x04(r1)
cmpwi r5, 0
beq ApplyRoomscaleAfterPlayerVelocity_SweepCleanup

lwz r11, 0xE8(r31)
extrwi. r0, r11, 1, 15
beq ApplyRoomscaleAfterPlayerVelocity_SweepUseMainBody
lwz r4, 0x230(r31)
b ApplyRoomscaleAfterPlayerVelocity_SweepBuildCast

ApplyRoomscaleAfterPlayerVelocity_SweepUseMainBody:
lwz r4, 0x4(r31)

ApplyRoomscaleAfterPlayerVelocity_SweepBuildCast:
addi r3, r1, RoomscaleCapsuleQuery
li r6, 0
bl ShapeCastWithInfo_ctor

addi r3, r1, RoomscaleCapsuleQuery
addi r4, r31, 0x208
bl ShapeCast_copyLayers

addi r3, r1, RoomscaleCapsuleQuery
addi r4, r1, RoomscaleCastFrom
addi r5, r1, RoomscaleHit
bl ShapeCast_setStartAndEnd

lwz r12, RoomscaleCapsuleShapeVtable(r1)
lwz r0, 0x1C(r12)
mtctr r0
li r4, 1
addi r3, r1, RoomscaleCapsuleQuery
bctrl
mr r30, r3

cmpwi r30, 0
beq ApplyRoomscaleAfterPlayerVelocity_SweepCleanupCapsule

lwz r6, RoomscaleCapsuleQuery + 0x04(r1)
cmpwi r6, 0
beq ApplyRoomscaleAfterPlayerVelocity_SweepCleanupCapsule

lwz r5, 0x08(r6)
cmpwi r5, 0
beq ApplyRoomscaleAfterPlayerVelocity_SweepCleanupCapsule

addi r3, r1, RoomscaleIterator
addi r4, r6, 0x34
bl ContactPointInfoIterator_ctor
lis r0, ContactPointInfoIterator_HitPosVtable@h
ori r0, r0, ContactPointInfoIterator_HitPosVtable@l
stw r0, RoomscaleIterator + 0x10(r1)

lwz r0, RoomscaleIterator + 0x00(r1)
lwz r5, RoomscaleIterator + 0x08(r1)
cmpw r0, r5
beq ApplyRoomscaleAfterPlayerVelocity_SweepCleanupCapsule

addi r3, r1, RoomscaleIterator
addi r4, r1, RoomscaleHit
addi r5, r1, RoomscaleCapsuleQuery
bl ShapeCastContactPointToHitPos
lwz r6, RoomscaleIterator + 0x00(r1)
lwz r6, 0(r6)
lfs f0, 0x14(r6)
stfs f0, RoomscaleHitNormal + 0x00(r1)
lfs f0, 0x18(r6)
stfs f0, RoomscaleHitNormal + 0x04(r1)
lfs f0, 0x1C(r6)
stfs f0, RoomscaleHitNormal + 0x08(r1)

ApplyRoomscaleAfterPlayerVelocity_SweepCleanupCapsule:
addi r3, r1, RoomscaleCapsuleQuery
li r4, 2
bl ShapeCastWithInfo_dtor

ApplyRoomscaleAfterPlayerVelocity_SweepCleanup:

addi r3, r1, RoomscaleQuery
li r4, 2
bl RoomscaleSphereCast_dtor
b ApplyRoomscaleAfterPlayerVelocity_Notify

ApplyRoomscaleAfterPlayerVelocity_GroundProbe:

addi r3, r1, RoomscaleQuery
li r4, 0
addi r5, r1, RoomscaleGroundHit
bl RayCastBodyQuery_ctor

addi r3, r1, RoomscaleQuery
addi r4, r31, 0x208
bl RayCast_setLayers

addi r3, r1, RoomscaleQuery
addi r4, r1, RoomscaleCastFrom
addi r5, r1, RoomscaleCastDelta
bl RayCast_setFromAndDelta

addi r3, r1, RoomscaleQuery
li r4, 0
bl RayCastBodyQuery_worldRayCast
mr r30, r3
cmpwi r30, 0
beq ApplyRoomscaleAfterPlayerVelocity_Consume

addi r3, r1, RoomscaleQuery
addi r4, r1, RoomscaleHit
bl RayCast_getHitPosition

ApplyRoomscaleAfterPlayerVelocity_Consume:
addi r3, r1, RoomscaleQuery
li r4, 2
bl RayCastBodyQuery_dtor

ApplyRoomscaleAfterPlayerVelocity_Notify:
addi r3, r1, RoomscaleScratch
mr r4, r30
bla import.coreinit.hook_ConsumeRoomscaleRaycast
lwz r31, 0x18(r1)
cmpwi r3, 2
beq ApplyRoomscaleAfterPlayerVelocity_Warp
cmpwi r3, 1
beq ApplyRoomscaleAfterPlayerVelocity_Loop
b ApplyRoomscaleAfterPlayerVelocity_Continue

ApplyRoomscaleAfterPlayerVelocity_Warp:
lfs f0, RoomscaleCurrent + 0x00(r1)
stfs f0, RoomscaleSavedCurrent + 0x00(r1)
lfs f0, RoomscaleCurrent + 0x04(r1)
stfs f0, RoomscaleSavedCurrent + 0x04(r1)
lfs f0, RoomscaleCurrent + 0x08(r1)
stfs f0, RoomscaleSavedCurrent + 0x08(r1)

lwz r11, 0xE8(r31)
extrwi. r0, r11, 1, 15
beq ApplyRoomscaleAfterPlayerVelocity_WarpMainBody

lwz r3, 0x230(r31)
addi r4, r1, RoomscaleScratch
bl RigidBody_getTransform
addi r3, r1, RoomscaleSavedCurrent
addi r4, r1, RoomscaleScratch
bla import.coreinit.hook_BuildRoomscaleWarpTransform
lwz r31, 0x18(r1)

lwz r3, 0x230(r31)
addi r4, r1, RoomscaleScratch
li r5, 1
mflr r12
stw r12, RoomscaleSavedLR(r1)
lis r0, ReturnAfterOriginalSetRigidBodyTransformSecondary@h
ori r0, r0, ReturnAfterOriginalSetRigidBodyTransformSecondary@l
lis r12, OriginalSetRigidBodyTransform@ha
addi r12, r12, OriginalSetRigidBodyTransform@l
mtctr r12
bctr

ReturnAfterOriginalSetRigidBodyTransformSecondary:
lwz r12, RoomscaleSavedLR(r1)
mtlr r12
b ApplyRoomscaleAfterPlayerVelocity_Continue

ApplyRoomscaleAfterPlayerVelocity_WarpMainBody:
lwz r3, 0x4(r31)
addi r4, r1, RoomscaleScratch
bl RigidBody_getTransform
addi r3, r1, RoomscaleSavedCurrent
addi r4, r1, RoomscaleScratch
bla import.coreinit.hook_BuildRoomscaleWarpTransform
lwz r31, 0x18(r1)

lwz r3, 0x4(r31)
addi r4, r1, RoomscaleScratch
li r5, 1
mflr r12
stw r12, RoomscaleSavedLR(r1)
lis r0, ReturnAfterOriginalSetRigidBodyTransformMain@h
ori r0, r0, ReturnAfterOriginalSetRigidBodyTransformMain@l
lis r12, OriginalSetRigidBodyTransform@ha
addi r12, r12, OriginalSetRigidBodyTransform@l
mtctr r12
bctr

ReturnAfterOriginalSetRigidBodyTransformMain:
lwz r12, RoomscaleSavedLR(r1)
mtlr r12
mr r3, r31
bl SyncPhysicsFieldAfterSetPosition

ApplyRoomscaleAfterPlayerVelocity_Continue:
lwz r31, 0x18(r1)
lwz r30, 0x14(r1)
lwz r0, 0x184(r1)
addi r1, r1, 0x180
mtlr r0
lwz r3, 0x4(r31)
lis r12, ContinueAfterApplyRoomscaleAfterPlayerVelocity@ha
addi r12, r12, ContinueAfterApplyRoomscaleAfterPlayerVelocity@l
mtctr r12
bctr

0x0344DCF8 = bla ApplyRoomscaleAfterPlayerVelocity

; aim the magnesis hold point with the controller before AppearMagneForce derives it from the sticks
0x020D858C = bla import.coreinit.hook_DriveMagnesisAim

; turn the magnesis-held object with the glove hand, right after the game has pushed it toward the hold point
0x03488AEC = ksys__phys__RigidBody__getLinearVelocity:
0x03488A30 = ksys__phys__RigidBody__getTransform:
0x03489868 = ksys__phys__RigidBody__setAngularVelocity:

MagnesisSpinEpsilon:
.float 0.00000011920929

SpinMagnesisHeldBody:
mflr r0
stwu r1, -0x60(r1)
stw r0, 0x64(r1)
stw r31, 0x5C(r1)
mr r31, r3

lis r12, ksys__phys__RigidBody__getLinearVelocity@ha
addi r12, r12, ksys__phys__RigidBody__getLinearVelocity@l
mtctr r12
bctrl

mr r3, r31
addi r4, r1, 0x10 ; sead::Matrix34 of the body
lis r12, ksys__phys__RigidBody__getTransform@ha
addi r12, r12, ksys__phys__RigidBody__getTransform@l
mtctr r12
bctrl

addi r3, r1, 0x10
addi r4, r1, 0x40 ; angular velocity to apply
bl import.coreinit.hook_GetMagnesisHeldBodySpin
cmpwi r3, 0
beq skipSpinMagnesisHeldBody

mr r3, r31
addi r4, r1, 0x40
lis r12, MagnesisSpinEpsilon@ha
lfs f1, MagnesisSpinEpsilon@l(r12)
lis r12, ksys__phys__RigidBody__setAngularVelocity@ha
addi r12, r12, ksys__phys__RigidBody__setAngularVelocity@l
mtctr r12
bctrl

skipSpinMagnesisHeldBody:
lwz r31, 0x5C(r1)
lwz r0, 0x64(r1)
addi r1, r1, 0x60
mtlr r0
blr

0x02B0117C = bla SpinMagnesisHeldBody
0x02B011C8 = bla SpinMagnesisHeldBody

; align the real player-fired arrow launch transform to the headset pose
0x02706270 = bla import.coreinit.hook_OverrideArrowShotTransform

; capture whether AI__Arrow::handleStateShoot selected normal, homing, or sky shooting
0x0270640C = bla import.coreinit.hook_CaptureArrowShootDecision
0x027064D8 = bla import.coreinit.hook_CaptureArrowShootDecision
0x02706518 = bla import.coreinit.hook_CaptureArrowShootDecision

; keep the bow arc / player-shot override synced with live Arrow param loads
0x030EC840 = ba import.coreinit.hook_LoadDynamicVec3
0x030EC8C0 = bla import.coreinit.hook_LoadDynamicBool

; capture FPS-scaled arrow speed from f0 for bow arc vfrScale
; hooks the FPS++ nop at 0x03793384 (right after lfs f0, averageFPS1Inv@l)
0x03793384 = bla import.coreinit.hook_ArrowFpsScale

0x03489710 = ContinueAfterLogSetRigidBodyVelocity:
0x0344DCFC = ContinueAfterApplyRoomscaleAfterPlayerVelocity:
0x0344B864 = SyncPhysicsFieldAfterSetPosition:
0x0344C0B8 = RigidBody_getPosition:
0x03488A30 = RigidBody_getTransform:
0x03486D84 = OriginalSetRigidBodyPosition:
0x03486CD8 = OriginalSetRigidBodyTransform:
0x034D0170 = ShapeCast_setStartAndEnd:
0x034C9C70 = RayCast_setLayers:
0x034C9E1C = RayCast_setFromAndDelta:
0x034C9F68 = RayCast_getHitPosition:
0x034CC1F8 = RayCastBodyQuery_ctor:
0x034CC2CC = RayCastBodyQuery_worldRayCast:
0x034CC278 = RayCastBodyQuery_dtor:
0x034CEFAC = ContactPointInfoIterator_ctor:
0x034C991C = ShapeCastContactPointToHitPos:
0x034D09D4 = ShapeCast_copyLayers:
0x034D1614 = ShapeCastWithInfo_ctor:
0x034D16A0 = ShapeCastWithInfo_dtor:
0x034D1B60 = RoomscaleSphereCast_ctor:
0x034D1DD0 = RoomscaleSphereCast_dtor:
0x102CCD20 = ContactPointInfoIterator_HitPosVtable:
0x10549DD0 = nullPosition:
0x10549FEC = emptyString:
