[BetterVR_StereoRendering_ActorJobs_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

0x037A6EC4 = real_actor_job1_1:
0x101E5FFC = Player_vtable:

;0x0335AD78 = nop
;0x0335AD70 = nop
;0x0335AD38 = nop
;0x037A71B4 = nop

custom_left_version_player:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x08(r1)
stw r10, 0x0C(r1)
stw r11, 0x10(r1)

; check if actor has LODState
lwz r3, 0x08(r1) ; reload actor pointer
lwz r10, 0x3B0(r3)
cmpwi r10, 0
bne skipLODStateCheck ; if it has an LOD state, skip

; check if player vtable is present
lwz r3, 0x08(r1) ; reload actor pointer
lwz r11, 0xE8(r3) ; load vtable of Actor
lis r10, Player_vtable@ha
addi r10, r10, Player_vtable@l
cmpw r10, r11
bne skipLODStateCheck

; call Player::m_70_updatePosition_maybe_fallHandleMaybe
lwz r3, 0x08(r1) ; reload actor pointer
lwz r10, 0xE8(r3)
lwz r11, 0x23C(r10)
mtctr r11
bctrl ; Player::m_70_updatePosition_maybe_fallHandleMaybe

skipLODStateCheck:
lwz r11, 0x10(r1)
lwz r10, 0x0C(r1)
lwz r3, 0x08(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr


custom_right_version_other:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x08(r1)
stw r10, 0x0C(r1)
stw r11, 0x10(r1)

; check if actor has LODState
lwz r3, 0x08(r1) ; reload actor pointer
lwz r10, 0x3B0(r3)
cmpwi r10, 0
bne exit_custom_right_version_other ; if it has an LOD state, skip

; check if player vtable is present
lwz r3, 0x08(r1) ; reload actor pointer
lwz r11, 0xE8(r3) ; load vtable of Actor
lis r10, Player_vtable@ha
addi r10, r10, Player_vtable@l
cmpw r10, r11
beq exit_custom_right_version_other

lwz r3, 0x08(r1) ; reload actor pointer
lwz r10, 0xE8(r3)
lwz r11, 0x23C(r10)
mtctr r11
bctrl ; Actor::m_70_updatePositionSomething

exit_custom_right_version_other:
lwz r11, 0x10(r1)
lwz r10, 0x0C(r1)
lwz r3, 0x08(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

;0x037A71B4 = mr r11, r30
;0x037A71A8 = mr r3, r31
;0x037A71AC = mflr r10
;0x037A71B0 = bla custom_right_version_other
;0x037A71B4 = mtlr r10







; r3 = format string
; r4 = actor pointer
logActorJob:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)
stw r7, 0x0C(r1)

lis r3, printToCemuConsoleWithFormat@ha
addi r3, r3, printToCemuConsoleWithFormat@l
mtctr r3


lwz r4, 0x08(r3)

mr r5, r3
mr r6, r4

bctrl ; bl printToCemuConsoleWithFormat

exit_logActorJob:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr



; timeout for skipping job pushes
; normally the game skips job pushes by setting this counter to 2
; however, in stereo rendering this code is executed twice per frame (once per eye)
; so this timeout is now doubled to 4 to account for both eyes
; this fixes LinkTag and EventTag actors not updating properly in stereo rendering
; which breaks dungeon doors and many other things
0x037A1B3C = li r12, 4

; EventTag::shouldSkipJobPush
;0x0310C670 = lis r3, currentEyeSide@ha
;0x0310C674 = li r3, 1 ; lwz r3, currentEyeSide@l(r3)
;0x0310C678 = blr

; LinkTag::shouldSkipJobPush
;0x031216BC = lis r3, currentEyeSide@ha
;0x031216C0 = li r3, 1 ; lwz r3, currentEyeSide@l(r3)
;0x031216C4 = blr

; should skip job push for baseproc
;0x0378BD2C = li r3, 0


custom_Actor_decrementSkipJobPushTimer:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
;beqlr

lbz r12, 0x42C(r3)
cmpwi r12, 0
beqlr
addi r0, r12, -1
stb r0, 0x42C(r3)
blr

;0x0379E270 = ba custom_Actor_decrementSkipJobPushTimer

; =====================================================

hook_sub_37B524C:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
beqlr

mr r12, r3
lwz r11, 0x1C(r12)
cmpwi r11, 0
beqlr
lha r0, 0x22(r12)
cmpwi r0, 0
beqlr
lha r10, 0x20(r12)
cmpwi r0, 0
add r3, r11, r10
bge loc_37B5284
lwz r11, 0x24(r12)
mtctr r11
bctr

loc_37B5284:
lha r12, 0x26(r12)
lwzx r10, r3, r12
slwi r0, r0, 3
add r9, r10, r0
lwz r11, 4(r9)
mtctr r11
bctr
blr

;0x037B524C = ba hook_sub_37B524C

; ======================================================

0x0310C82C = Actor__deleteIfPlacementStuff:

custom_EventTag__calcAi:
mflr r0
stwu r1, -0x10(r1)
stw r31, 0x0C(r1)
stw r0, 0x14(r1)

lis r31, currentEyeSide@ha
lwz r31, currentEyeSide@l(r31)
cmpwi r31, 1
beq loc_310C85C ; skip if right eye

lis r31, Actor__deleteIfPlacementStuff@ha
addi r31, r31, Actor__deleteIfPlacementStuff@l
mtctr r31
mr r31, r3
bctrl ; bl Actor__deleteIfPlacementStuff
cmpwi r3, 0
bne loc_310C85C
lwz r3, 0x390(r31)
cmpwi r3, 0
beq loc_310C854
lwz r12, 0xC(r3)
lwz r0, 0xA4(r12)
mtctr r0
bctrl

loc_310C854:
li r0, 0
stb r0, 0x58C(r31)

loc_310C85C:
lwz r0, 0x14(r1)
lwz r31, 0x0C(r1)
mtlr r0
addi r1, r1, 0x10
blr

;0x0310C818 = ba custom_EventTag__calcAi

; ======================================================


0x031222F8 = LinkTag_job_run:

custom_LinkTag_job_run:
mflr r0
stwu r1, -0x0C(r1)
stw r0, 0x10(r1)
stw r31, 0x08(r1)
stw r30, 0x04(r1)

; skip if right eye
lis r31, currentEyeSide@ha
lwz r31, currentEyeSide@l(r31)
cmpwi r31, 1
beq exit_LinkTag_job_run

; call original function
lis r30, LinkTag_job_run@ha
addi r30, r30, LinkTag_job_run@l
mtctr r30
bctrl ; bl LinkTag_job_run

exit_LinkTag_job_run:
lwz r0, 0x10(r1)
mtlr r0
lwz r31, 0x08(r1)
lwz r30, 0x04(r1)
addi r1, r1, 0x0C
blr

;0x0311F404 = lis r26, custom_LinkTag_job_run@ha
;0x0311F420 = addi r26, r26, custom_LinkTag_job_run@l


; ======================================================
; ======================================================

0x03415C6C = ksys__CalcAttentionAndVibration:
0x03414FE8 = ksys__CalcPlayReportAndStatsMgr:
0x1046CA64 = ActorDebug__sInstance:
0x03414BF8 = ksys__CalcQuestAndEventAndActorDeleteOrPreload:

ksys__CalcFrameJob1__run:
mflr r0
stw r0, 0x04(r1)
stwu r1, -0x08(r1)

lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
beq exit_ksys__CalcFrameJob1__run ; skip if right eye

lis r12, ksys__CalcAttentionAndVibration@ha
addi r12, r12, ksys__CalcAttentionAndVibration@l
mtctr r12
bctrl ; bl ksys__CalcAttentionAndVibration
lis r12, ksys__CalcPlayReportAndStatsMgr@ha
addi r12, r12, ksys__CalcPlayReportAndStatsMgr@l
mtctr r12
bctrl ; bl ksys__CalcPlayReportAndStatsMgr
lis r12, ActorDebug__sInstance@ha
lwz r12, ActorDebug__sInstance@l(r12)
cmpwi r12, 0
beq loc_31FDC38
lwz r0, 0x18(r12)

;rotlwi r0, r0, 2
;li r12, 1
;and r0, r0, r12
;cmpwi cr0, r0, 0

.long 0x540017FF
beq exit_ksys__CalcFrameJob1__run

loc_31FDC38:
lis r12, ksys__CalcQuestAndEventAndActorDeleteOrPreload@ha
addi r12, r12, ksys__CalcQuestAndEventAndActorDeleteOrPreload@l
mtctr r12
bctrl ; bl ksys__CalcQuestAndEventAndActorDeleteOrPreload

exit_ksys__CalcFrameJob1__run:
lwz r0, 0x0C(r1)
mtlr r0
addi r1, r1, 8
blr


;0x031FDC08 = ba ksys__CalcFrameJob1__run


; ======================================================

;;;; 0x0313EC98 = ksys__map__ObjectLinkArray__checkLink:
;;;; 
;;;; custom_actor_checkSignal:
;;;; ;bla import.coreinit.log_actorCheckSignal
;;;; lwz r12, 0x4FC(r3)
;;;; cmpwi r12, 0
;;;; beq loc_379F1F4
;;;; lwz r0, 0x28(r12)
;;;; cmpwi r0, 0
;;;; bne loc_379F1FC
;;;; 
;;;; loc_379F1F4:
;;;; li r3, 0
;;;; blr
;;;; 
;;;; loc_379F1FC:
;;;; lwz r12, 0x28(r12)
;;;; li r5, 0
;;;; lis r3, ksys__map__ObjectLinkArray__checkLink@ha
;;;; addi r3, r3, ksys__map__ObjectLinkArray__checkLink@l
;;;; mtctr r3
;;;; addi r3, r12, 0x20
;;;; bctr ; b ksys__map__ObjectLinkArray__checkLink
;;;; blr


custom_player_m_81_getCalcTiming:
lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
li r3, 999
beqlr ; if right eye, return 999
li r3, 0
blr

;0x02D794C4 = ba custom_player_m_81_getCalcTiming


playerNormal:
.int 0

storePlayerNormal:
lis r3, playerNormal@ha
stw r31, playerNormal@l(r3)
mr r3, r31
blr

0x02D0A168 = bla storePlayerNormal

0x037966A8 = getPhysicsField60:

onlyRunPlayerNormalStateForLeftEye:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

lis r4, currentEyeSide@ha
lwz r4, currentEyeSide@l(r4)
cmpwi r4, 1
li r3, 0
beq skip_onlyRunPlayerNormalStateForLeftEye ; skip if right eye

; get actor physics field
lis r4, getPhysicsField60@ha
addi r4, r4, getPhysicsField60@l
mtctr r4
lwz r3, 0x0C(r1)
bctrl ; bl getPhysicsField60

skip_onlyRunPlayerNormalStateForLeftEye:
lwz r4, 0x08(r1)
;lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x02D149DC = bla onlyRunPlayerNormalStateForLeftEye


onlyRunPlayerNormalForLeftEye:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

lis r4, currentEyeSide@ha
lwz r4, currentEyeSide@l(r4)
cmpwi r4, 1
li r3, 0
beq skip_onlyRunPlayerNormalForLeftEye ; skip if right eye

; get actor physics field
lis r4, getPhysicsField60@ha
addi r4, r4, getPhysicsField60@l
mtctr r4
lwz r3, 0x0C(r1)
bctrl ; bl getPhysicsField60

skip_onlyRunPlayerNormalForLeftEye:
lwz r4, 0x08(r1)
;lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

;0x02D16554 = bla onlyRunPlayerNormalForLeftEye


hook_updatePlayerNormalStateForClimbing:
lis r25, currentEyeSide@ha
lwz r25, currentEyeSide@l(r25)
cmpwi r25, 1
beq .+0x08 ; skip if right eye
li r3, 0
cmpwi r3, 0
blr

;0x02D149E0 = bla hook_updatePlayerNormalStateForClimbing

; ======================================================
; ======================================================
; ======================================================

setClimbingState:
li r25, 0
;lbz r25, 0x16E8(r30)
stb r25, 0x16E8(r30)
blr
0x02D5C67C = bla setClimbingState

scopedDeltaSetter_float2:
.float 1.0
.float 0.0

0x037A5DB0 = real_actor_job0_1:
0x037A5080 = actor_job_common_calcAI_andMore:

0x02D149A0 = updatePlayerNormalStateForClimbing:
0x02D164E8 = playerNormal_calc:

strActor_job0_1:
.string "job0_1"

stub_routeActorJob_job0_1:
li r6, 0
cmpwi r5, 0
bne exit_stub_routeActorJob_job0_1
lwz r11, 0xE8(r3)
lis r10, Player_vtable@ha
addi r10, r10, Player_vtable@l
cmpw r11, r10
li r6, 1
bne exit_stub_routeActorJob_job0_1
li r6, 2

exit_stub_routeActorJob_job0_1:
mr r3, r6
blr

stub_routeActorJob_skipRight:
li r3, 0
cmpwi r5, 1
bnelr
li r3, 1
blr

hook_actor_job0_1:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job0_1@ha
addi r4, r4, strActor_job0_1@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_job0_1@ha
addi r12, r12, stub_routeActorJob_job0_1@l
beq routeActorJob_job0_1
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job0_1:
mtctr r12
bctrl

cmpwi r3, 0
beq job0_1_normal
cmpwi r3, 1
beq finish_hook_actor_job0_1
cmpwi r3, 2
beq job0_1_altered

job0_1_normal:
lis r4, real_actor_job0_1@ha
addi r4, r4, real_actor_job0_1@l
mtctr r4
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job0_1
b finish_hook_actor_job0_1

job0_1_altered:
; set climbing state
lwz r3, 0x1C(r1)
li r4, 0
stb r4, 0x16E8(r3)

; ; FOR PLAYER ALONE, run actor update HP
; lwz r3, 0x1C(r1)
; lwz r3, 0xE8(r3) ; load vtable of Actor
; lwz r3, 0x264(r3) ; load updateHP function pointer
; mtctr r3
; lis r4, scopedDeltaSetter_float2@ha
; addi r4, r4, scopedDeltaSetter_float2@l
; lwz r3, 0x1C(r1)
; bctrl ; ba updateHP

;lis r4, playerNormal_calc@ha
;addi r4, r4, playerNormal_calc@l
;mtctr r4
;lis r3, playerNormal@ha
;lwz r3, playerNormal@l(r3)
;bctrl ; ba playerNormal_calc

lis r4, updatePlayerNormalStateForClimbing@ha
addi r4, r4, updatePlayerNormalStateForClimbing@l
mtctr r4
lis r3, playerNormal@ha
lwz r3, playerNormal@l(r3)
bctrl ; ba updatePlayerNormalStateForClimbing

;   ; FOR PLAYER ALONE, run on the common calcAI_andMore on the LEFT EYE too.
;   lis r4, actor_job_common_calcAI_andMore@ha
;   addi r4, r4, actor_job_common_calcAI_andMore@l
;   mtctr r4
;   lwz r3, 0x1C(r1)
;   bctrl ; ba actor_job_common_calcAI_andMore

;lis r4, real_actor_job0_1@ha
;addi r4, r4, real_actor_job0_1@l
;mtctr r4
;lwz r3, 0x1C(r1)
;bctrl ; ba real_actor_job0_1
b finish_hook_actor_job0_1

finish_hook_actor_job0_1:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x03795750 = lis r27, hook_actor_job0_1@ha
0x03795760 = addi r27, r27, hook_actor_job0_1@l

; ======================================================

0x037A7D3C = real_actor_job0_2:

strActor_job0_2:
.string "job0_2"

hook_actor_job0_2:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job0_2@ha
addi r4, r4, strActor_job0_2@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job0_2
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job0_2:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job0_2

lis r3, real_actor_job0_2@ha
addi r3, r3, real_actor_job0_2@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job0_2

finish_hook_actor_job0_2:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x0379575C = lis r25, hook_actor_job0_2@ha
0x03795768 = addi r25, r25, hook_actor_job0_2@l

; ======================================================

strActor_job1_1:
.string "job1_1"

hook_actor_job1_1:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job1_1@ha
addi r4, r4, strActor_job1_1@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job1_1
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job1_1:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job1_1

lis r3, real_actor_job1_1@ha
addi r3, r3, real_actor_job1_1@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job1_1

finish_hook_actor_job1_1:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x03795834 = lis r25, hook_actor_job1_1@ha
0x03795840 = addi r25, r25, hook_actor_job1_1@l


; ======================================================

0x037A7CB8 = real_actor_job1_2:

strActor_job1_2:
.string "job1_2"

hook_actor_job1_2:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job1_2@ha
addi r4, r4, strActor_job1_2@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job1_2
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job1_2:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job1_2

lis r3, real_actor_job1_2@ha
addi r3, r3, real_actor_job1_2@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job1_2

finish_hook_actor_job1_2:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x03795844 = lis r20, hook_actor_job1_2@ha
0x03795850 = addi r20, r20, hook_actor_job1_2@l

; ======================================================

0x037A7438 = real_actor_job2_1_ragdoll_related:

strActor_job2_1:
.string "job2_1_ragdoll_related"

hook_actor_job2_1_ragdoll_related:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job2_1@ha
addi r4, r4, strActor_job2_1@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job2_1
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job2_1:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job2_1

lis r3, real_actor_job2_1_ragdoll_related@ha
addi r3, r3, real_actor_job2_1_ragdoll_related@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job2_1_ragdoll_related

finish_hook_actor_job2_1:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x037958F8 = lis r20, hook_actor_job2_1_ragdoll_related@ha
0x03795904 = addi r20, r20, hook_actor_job2_1_ragdoll_related@l

; ======================================================

0x037A7E30 = real_actor_job2_2:

strActor_job2_2:
.string "job2_2"

hook_actor_job2_2:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job2_2@ha
addi r4, r4, strActor_job2_2@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job2_2
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job2_2:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job2_2

lis r3, real_actor_job2_2@ha
addi r3, r3, real_actor_job2_2@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job2_2

finish_hook_actor_job2_2:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x03795908 = lis r19, hook_actor_job2_2@ha
0x03795914 = addi r19, r19, hook_actor_job2_2@l

; ======================================================

0x037A7C00 = real_actor_job4:

strActor_job4:
.string "job4"

hook_actor_job4:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lwz r3, 0x1C(r1)
lis r4, strActor_job4@ha
addi r4, r4, strActor_job4@l
lis r5, currentEyeSide@ha
lwz r5, currentEyeSide@l(r5)
lis r6, currentFrameIsMono@ha
lwz r6, currentFrameIsMono@l(r6)
lis r12, useStubHooks@ha
lwz r12, useStubHooks@l(r12)
cmpwi r12, 1
lis r12, stub_routeActorJob_skipRight@ha
addi r12, r12, stub_routeActorJob_skipRight@l
beq routeActorJob_job4
lis r12, import.coreinit.hook_RouteActorJob@ha
addi r12, r12, import.coreinit.hook_RouteActorJob@l
routeActorJob_job4:
mtctr r12
bctrl

cmpwi r3, 1
beq finish_hook_actor_job4

lis r3, real_actor_job4@ha
addi r3, r3, real_actor_job4@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl ; ba real_actor_job4

finish_hook_actor_job4:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x037959C0 = lis r21, hook_actor_job4@ha
0x037959D0 = addi r21, r21, hook_actor_job4@l


; ======================================================

; this fixes the doubled player gravity and various animations of the player

0x0344CECC = originalPlayerVelocityUpdater:

OnlyRunPlayerUpdateJobOnce:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

; in single-pass mode this job normally runs on the right eye's pass, which no
; longer exists, so run it on the only pass instead
lis r3, currentFrameIsMono@ha
lwz r3, currentFrameIsMono@l(r3)
cmpwi r3, 0
bne runPlayerUpdateJob_mono

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 0
beq exit_OnlyRunPlayerUpdateJobOnce

runPlayerUpdateJob_mono:
lis r3, originalPlayerVelocityUpdater@ha
addi r3, r3, originalPlayerVelocityUpdater@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl

exit_OnlyRunPlayerUpdateJobOnce:
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x037FFE34 = bla OnlyRunPlayerUpdateJobOnce


; ======================================================

; fix events running twice as fast

OnlyRunEventUpdateJobOnce:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

; in single-pass mode this job normally runs on the right eye's pass, which no
; longer exists, so run it on the only pass instead
lis r3, currentFrameIsMono@ha
lwz r3, currentFrameIsMono@l(r3)
cmpwi r3, 0
bne runEventUpdateJob_mono

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 0
beq exit_OnlyRunEventUpdateJobOnce

runEventUpdateJob_mono:
lis r3, FixedSizeJQ_enque_job@ha
addi r3, r3, FixedSizeJQ_enque_job@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl

exit_OnlyRunEventUpdateJobOnce:
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr


0x031FBC78 = bla OnlyRunEventUpdateJobOnce

; ======================================================

; fix weather and WorldMgr::calc being called twice per frame

0x030DF7E8 = FixedSizeJQ_enque_job:

OnlyRunWorldUpdateJobOnce:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_OnlyRunWorldUpdateJobOnce

lis r3, FixedSizeJQ_enque_job@ha
addi r3, r3, FixedSizeJQ_enque_job@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl

exit_OnlyRunWorldUpdateJobOnce:
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x031FBC84 = bla OnlyRunWorldUpdateJobOnce

; ===================================================

0x0317A128 = ksys_act_Awareness_calc:

; run awareness job once

OnlyRunAwarenessJobOnce:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_OnlyRunAwarenessJobOnce

lis r3, ksys_act_Awareness_calc@ha
addi r3, r3, ksys_act_Awareness_calc@l
mtctr r3
lwz r3, 0x1C(r1)
bctrl

exit_OnlyRunAwarenessJobOnce:
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x031FDE3C = ba OnlyRunAwarenessJobOnce