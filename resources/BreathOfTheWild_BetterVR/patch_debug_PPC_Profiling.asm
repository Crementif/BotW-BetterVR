[BetterVR_PPC_Profiling_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; BetterVRProfiler::Section values. Keep in sync with src/utils/profiler.h.
PPC_SystemPreCalc                    = 12
PPC_SystemStateMachine               = 13
PPC_SystemPostCalc                   = 14
PPC_CalcPlacementMgr                 = 15
PPC_PhysicsPostBgBaseProcMgr         = 16
PPC_ActorUpdateJobs                  = 17
PPC_GraphicsCalc                     = 18
PPC_SystemTaskPreCalc               = 19
PPC_SystemTaskPostCalc              = 20
PPC_SystemTaskDrawTV                = 21
PPC_SystemTaskDrawDRC               = 22
PPC_SystemTaskPostDrawTV            = 23
PPC_SystemTaskPostDrawDRC           = 24
PPC_Layer3DDraw                     = 25
PPC_Layer3DCalcView                 = 26
PPC_Layer3DCalcViewGPU              = 27
PPC_Layer3DDrawBG                   = 28
PPC_Layer3DDrawOpaque               = 29
PPC_Layer3DDrawXlu                  = 30
PPC_Layer3DDrawPostEffects          = 31
PPC_Layer3DDrawFinalImage           = 32
PPC_ActorJob0_1                     = 33
PPC_ActorJob0_2                     = 34
PPC_ActorJob1_1                     = 35
PPC_ActorJob1_2                     = 36
PPC_ActorJob2_1Ragdoll              = 37
PPC_ActorJob2_2                     = 38
PPC_ActorJob4                       = 39

0x02C57E54 = uking__frm__System__preCalc_continue:
0x02C57E84 = uking__frm__System__calcAndRunStateMachine_continue:
0x02C57EE0 = uking__frm__System__postCalc_continue:
0x0340EEE4 = ksys__CalcPlacementMgr_continue:
0x031FCE74 = MCMgr__calcPostBgBaseProcMgr_continue:
0x03414D7C = runActorUpdateStuff_continue:
0x03416598 = gameScene__CalcGraphicsStuff_continue:

0x03A128DC = gsys__SystemTask__preCalc_continue:
0x03A135A8 = gsys__SystemTask__postCalc_continue:
0x03A146E8 = gsys__SystemTask__drawTV_continue:
0x03A1480C = gsys__SystemTask__drawDRC_continue:
0x03A148DC = gsys__SystemTask__postDrawTV_continue:
0x03A14978 = gsys__SystemTask__postDrawDRC_continue:

0x037A5DB8 = real_actor_job0_1_continue:
0x037A7D44 = real_actor_job0_2_continue:
0x037A6ECC = real_actor_job1_1_continue:
0x037A7CC0 = real_actor_job1_2_continue:
0x037A7448 = real_actor_job2_1_ragdoll_related_continue:
0x037A7E34 = real_actor_job2_2_continue:
0x037A7C08 = real_actor_job4_continue:

Profile_uking__frm__System__preCalc:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)

li r3, PPC_SystemPreCalc
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lis r0, ReturnAfter_uking__frm__System__preCalc@h
ori r0, r0, ReturnAfter_uking__frm__System__preCalc@l
stw r0, 0x04(r1)
lis r12, uking__frm__System__preCalc_continue@ha
addi r12, r12, uking__frm__System__preCalc_continue@l
mtctr r12
bctr

ReturnAfter_uking__frm__System__preCalc:
li r3, PPC_SystemPreCalc
bla import.coreinit.hook_ProfileSectionEnd
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_uking__frm__System__calcAndRunStateMachine:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)

li r3, PPC_SystemStateMachine
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lis r0, ReturnAfter_uking__frm__System__calcAndRunStateMachine@h
ori r0, r0, ReturnAfter_uking__frm__System__calcAndRunStateMachine@l
stwu r1, -0x18(r1)
lis r12, uking__frm__System__calcAndRunStateMachine_continue@ha
addi r12, r12, uking__frm__System__calcAndRunStateMachine_continue@l
mtctr r12
bctr

ReturnAfter_uking__frm__System__calcAndRunStateMachine:
stw r3, 0x08(r1)
li r3, PPC_SystemStateMachine
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_uking__frm__System__postCalc:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)
stw r4, 0x0C(r1)

li r3, PPC_SystemPostCalc
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lwz r4, 0x0C(r1)
lis r0, ReturnAfter_uking__frm__System__postCalc@h
ori r0, r0, ReturnAfter_uking__frm__System__postCalc@l
stw r0, 0x04(r1)
lis r12, uking__frm__System__postCalc_continue@ha
addi r12, r12, uking__frm__System__postCalc_continue@l
mtctr r12
bctr

ReturnAfter_uking__frm__System__postCalc:
li r3, PPC_SystemPostCalc
bla import.coreinit.hook_ProfileSectionEnd
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_ksys__CalcPlacementMgr:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)

li r3, PPC_CalcPlacementMgr
bla import.coreinit.hook_ProfileSectionBegin

lis r0, ReturnAfter_ksys__CalcPlacementMgr@h
ori r0, r0, ReturnAfter_ksys__CalcPlacementMgr@l
stwu r1, -0x40(r1)
lis r12, ksys__CalcPlacementMgr_continue@ha
addi r12, r12, ksys__CalcPlacementMgr_continue@l
mtctr r12
bctr

ReturnAfter_ksys__CalcPlacementMgr:
li r3, PPC_CalcPlacementMgr
bla import.coreinit.hook_ProfileSectionEnd
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_MCMgr__calcPostBgBaseProcMgr:
stwu r1, -0x20(r1)
mflr r12
stw r12, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_PhysicsPostBgBaseProcMgr
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r12, ReturnAfter_MCMgr__calcPostBgBaseProcMgr@ha
addi r12, r12, ReturnAfter_MCMgr__calcPostBgBaseProcMgr@l
mtlr r12
stwu r1, -0xB0(r1)
stmw r23, 0x8C(r1)
lis r12, MCMgr__calcPostBgBaseProcMgr_continue@ha
addi r12, r12, MCMgr__calcPostBgBaseProcMgr_continue@l
mtctr r12
bctr

ReturnAfter_MCMgr__calcPostBgBaseProcMgr:
stw r3, 0x08(r1)
li r3, PPC_PhysicsPostBgBaseProcMgr
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_runActorUpdateStuff:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)

li r3, PPC_ActorUpdateJobs
bla import.coreinit.hook_ProfileSectionBegin

lis r0, ReturnAfter_runActorUpdateStuff@h
ori r0, r0, ReturnAfter_runActorUpdateStuff@l
stwu r1, -0x10(r1)
lis r12, runActorUpdateStuff_continue@ha
addi r12, r12, runActorUpdateStuff_continue@l
mtctr r12
bctr

ReturnAfter_runActorUpdateStuff:
stw r3, 0x08(r1)
li r3, PPC_ActorUpdateJobs
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gameScene__CalcGraphicsStuff:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_GraphicsCalc
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_gameScene__CalcGraphicsStuff@h
ori r0, r0, ReturnAfter_gameScene__CalcGraphicsStuff@l
stwu r1, -0x18(r1)
lis r12, gameScene__CalcGraphicsStuff_continue@ha
addi r12, r12, gameScene__CalcGraphicsStuff_continue@l
mtctr r12
bctr

ReturnAfter_gameScene__CalcGraphicsStuff:
stw r3, 0x08(r1)
li r3, PPC_GraphicsCalc
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__preCalc:
stwu r1, -0x20(r1)
mflr r12
stw r12, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskPreCalc
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r12, ReturnAfter_gsys__SystemTask__preCalc@ha
addi r12, r12, ReturnAfter_gsys__SystemTask__preCalc@l
mtlr r12
stwu r1, -0x48(r1)
stmw r20, 0x18(r1)
lis r12, gsys__SystemTask__preCalc_continue@ha
addi r12, r12, gsys__SystemTask__preCalc_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__preCalc:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskPreCalc
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__postCalc:
stwu r1, -0x20(r1)
mflr r12
stw r12, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskPostCalc
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r12, ReturnAfter_gsys__SystemTask__postCalc@ha
addi r12, r12, ReturnAfter_gsys__SystemTask__postCalc@l
mtlr r12
stwu r1, -0xA8(r1)
stmw r22, 0x80(r1)
lis r12, gsys__SystemTask__postCalc_continue@ha
addi r12, r12, gsys__SystemTask__postCalc_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__postCalc:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskPostCalc
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__drawTV:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskDrawTV
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_gsys__SystemTask__drawTV@h
ori r0, r0, ReturnAfter_gsys__SystemTask__drawTV@l
stwu r1, -0x48(r1)
lis r12, gsys__SystemTask__drawTV_continue@ha
addi r12, r12, gsys__SystemTask__drawTV_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__drawTV:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskDrawTV
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__drawDRC:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskDrawDRC
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_gsys__SystemTask__drawDRC@h
ori r0, r0, ReturnAfter_gsys__SystemTask__drawDRC@l
stwu r1, -0x30(r1)
lis r12, gsys__SystemTask__drawDRC_continue@ha
addi r12, r12, gsys__SystemTask__drawDRC_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__drawDRC:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskDrawDRC
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__postDrawTV:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskPostDrawTV
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_gsys__SystemTask__postDrawTV@h
ori r0, r0, ReturnAfter_gsys__SystemTask__postDrawTV@l
stwu r1, -0x28(r1)
lis r12, gsys__SystemTask__postDrawTV_continue@ha
addi r12, r12, gsys__SystemTask__postDrawTV_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__postDrawTV:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskPostDrawTV
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_gsys__SystemTask__postDrawDRC:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_SystemTaskPostDrawDRC
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_gsys__SystemTask__postDrawDRC@h
ori r0, r0, ReturnAfter_gsys__SystemTask__postDrawDRC@l
stwu r1, -0x28(r1)
lis r12, gsys__SystemTask__postDrawDRC_continue@ha
addi r12, r12, gsys__SystemTask__postDrawDRC_continue@l
mtctr r12
bctr

ReturnAfter_gsys__SystemTask__postDrawDRC:
stw r3, 0x08(r1)
li r3, PPC_SystemTaskPostDrawDRC
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job0_1:
stwu r1, -0x20(r1)
mflr r12
stw r12, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob0_1
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r12, ReturnAfter_Actor__job0_1@ha
addi r12, r12, ReturnAfter_Actor__job0_1@l
mtlr r12
stwu r1, -0x98(r1)
stmw r17, 0x5C(r1)
lis r12, real_actor_job0_1_continue@ha
addi r12, r12, real_actor_job0_1_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job0_1:
stw r3, 0x08(r1)
li r3, PPC_ActorJob0_1
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job0_2:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob0_2
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_Actor__job0_2@h
ori r0, r0, ReturnAfter_Actor__job0_2@l
stwu r1, -0x10(r1)
lis r12, real_actor_job0_2_continue@ha
addi r12, r12, real_actor_job0_2_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job0_2:
stw r3, 0x08(r1)
li r3, PPC_ActorJob0_2
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job1_1:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob1_1
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_Actor__job1_1@h
ori r0, r0, ReturnAfter_Actor__job1_1@l
stwu r1, -0x20(r1)
lis r12, real_actor_job1_1_continue@ha
addi r12, r12, real_actor_job1_1_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job1_1:
stw r3, 0x08(r1)
li r3, PPC_ActorJob1_1
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job1_2:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob1_2
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_Actor__job1_2@h
ori r0, r0, ReturnAfter_Actor__job1_2@l
stwu r1, -0x10(r1)
lis r12, real_actor_job1_2_continue@ha
addi r12, r12, real_actor_job1_2_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job1_2:
stw r3, 0x08(r1)
li r3, PPC_ActorJob1_2
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job2_1_ragdoll_related:
stwu r1, -0x20(r1)
mflr r12
stw r12, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob2_1Ragdoll
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r12, ReturnAfter_Actor__job2_1_ragdoll_related@ha
addi r12, r12, ReturnAfter_Actor__job2_1_ragdoll_related@l
mtlr r12
stwu r1, -0x310(r1)
stmw r23, 0x2DC(r1)
stfd f31, 0x300(r1)
ps_merge10 f31, f31, f31
lis r12, real_actor_job2_1_ragdoll_related_continue@ha
addi r12, r12, real_actor_job2_1_ragdoll_related_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job2_1_ragdoll_related:
stw r3, 0x08(r1)
li r3, PPC_ActorJob2_1Ragdoll
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

Profile_Actor__job2_2:
stwu r1, -0x20(r1)
mflr r11
stw r11, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob2_2
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r11, ReturnAfter_Actor__job2_2@ha
addi r11, r11, ReturnAfter_Actor__job2_2@l
mtlr r11
lwz r12, 0x364(r3)
lis r11, real_actor_job2_2_continue@ha
addi r11, r11, real_actor_job2_2_continue@l
mtctr r11
bctr

ReturnAfter_Actor__job2_2:
stw r3, 0x08(r1)
li r3, PPC_ActorJob2_2
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r11, 0x1C(r1)
mtlr r11
addi r1, r1, 0x20
blr

Profile_Actor__job4:
stwu r1, -0x20(r1)
stw r0, 0x1C(r1)
stw r3, 0x08(r1)

li r3, PPC_ActorJob4
bla import.coreinit.hook_ProfileSectionBegin

lwz r3, 0x08(r1)
lis r0, ReturnAfter_Actor__job4@h
ori r0, r0, ReturnAfter_Actor__job4@l
stwu r1, -0x10(r1)
lis r12, real_actor_job4_continue@ha
addi r12, r12, real_actor_job4_continue@l
mtctr r12
bctr

ReturnAfter_Actor__job4:
stw r3, 0x08(r1)
li r3, PPC_ActorJob4
bla import.coreinit.hook_ProfileSectionEnd
lwz r3, 0x08(r1)
lwz r12, 0x1C(r1)
mtlr r12
addi r1, r1, 0x20
blr

0x02C57E50 = ba Profile_uking__frm__System__preCalc
0x02C57E80 = ba Profile_uking__frm__System__calcAndRunStateMachine
0x02C57EDC = ba Profile_uking__frm__System__postCalc
0x0340EEE0 = ba Profile_ksys__CalcPlacementMgr
0x031FCE6C = ba Profile_MCMgr__calcPostBgBaseProcMgr
0x03414D78 = ba Profile_runActorUpdateStuff
0x03416594 = ba Profile_gameScene__CalcGraphicsStuff

0x03A128D4 = ba Profile_gsys__SystemTask__preCalc
0x03A135A0 = ba Profile_gsys__SystemTask__postCalc
0x03A146E4 = ba Profile_gsys__SystemTask__drawTV
0x03A148D8 = ba Profile_gsys__SystemTask__postDrawTV
; drawDRC_/postDrawDRC_ entry hooks are owned by hook_maybeSkipDrawDRC/hook_maybeSkipPostDrawDRC
; in patch_RND_StereoRendering_Optimizations.asm, so they can't also be profiled here

0x037A5DB0 = ba Profile_Actor__job0_1
0x037A7D40 = ba Profile_Actor__job0_2
0x037A6EC8 = ba Profile_Actor__job1_1
0x037A7CBC = ba Profile_Actor__job1_2
0x037A7438 = ba Profile_Actor__job2_1_ragdoll_related
0x037A7E30 = ba Profile_Actor__job2_2
0x037A7C04 = ba Profile_Actor__job4
