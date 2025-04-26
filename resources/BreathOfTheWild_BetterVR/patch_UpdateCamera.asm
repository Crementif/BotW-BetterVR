[BetterVR_UpdateCamera_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


currentEyeSide:
.int 0

0x031FB1B4 = sub_31FB1B4_getTimeForGameUpdateMaybe:
0x0309F72C = sead_GameFramework_lockFrameDrawContext:
0x0309F744 = sead_GameFramework_unlockFrameDrawContext:
0x030A27CC = sead__TaskMgr__calcDestruction:
0x031FAF00 = calculateUIMaybe:

custom_sead__GameFramework__procFrame:
mflr r0
stwu r1, -0x10(r1)
stw r30, 8(r1)
mr r30, r3
stw r31, 0xC(r1)
stw r0, 0x14(r1)
lis r3, sub_31FB1B4_getTimeForGameUpdateMaybe@ha
addi r3, r3, sub_31FB1B4_getTimeForGameUpdateMaybe@l
mtctr r3
addi r3, r30, 0x388
bctrl ; bl sub_31FB1B4
lis r3, sead_GameFramework_lockFrameDrawContext@ha
addi r3, r3, sead_GameFramework_lockFrameDrawContext@l
mtctr r3
mr r3, r30
bctrl ; bl sead_GameFramework_lockFrameDrawContext
lis r3, sead__TaskMgr__calcDestruction@ha
addi r3, r3, sead__TaskMgr__calcDestruction@l
mtctr r3
lwz r3, 0x1C(r30)
bctrl ; bl sead__TaskMgr__calcDestruction
lis r3, calculateUIMaybe@ha
addi r3, r3, calculateUIMaybe@l
mtctr r3
bctrl
;bl calculateUIMaybe

; ========================================================================
; FIRST EYE SIDE
; ========================================================================

li r0, 0
lis r12, currentEyeSide@ha
stw r0, currentEyeSide@l(r12)
li r3, 0
bl import.coreinit.hook_BeginCameraSide

lwz r12, 0(r30)
lwz r0, 0xF4(r12)
mtctr r0
mr r3, r30
bctrl ; sead__GameFrameworkCafe__procDraw

lwz r12, 0(r30)
lwz r11, 0xFC(r12)
mtctr r11
mr r3, r30
bctrl ; sead__GameFrameworkCafe__calcDraw

lwz r12, 0(r30)
lwz r0, 0x7C(r12)
mtctr r0
mr r3, r30
bctrl ; sead__Framework__procReset

lwz r12, 0(r30)
lwz r12, 0x10C(r12)
mtctr r12
mr r3, r30
bctrl ; sead__GameFrameworkCafe__presentAndDrawDone

li r3, 0
bl import.coreinit.hook_EndCameraSide

; ========================================================================
; SECOND EYE SIDE
; ========================================================================

li r0, 1
lis r12, currentEyeSide@ha
stw r0, currentEyeSide@l(r12)
li r3, 1
bl import.coreinit.hook_BeginCameraSide

lwz r12, 0(r30)
lwz r0, 0xF4(r12)
mtctr r0
mr r3, r30
bctrl ; sead__GameFrameworkCafe__procDraw

lwz r12, 0(r30)
lwz r11, 0xFC(r12)
mtctr r11
mr r3, r30
bctrl ; sead__GameFrameworkCafe__calcDraw

lwz r12, 0(r30)
lwz r0, 0x7C(r12)
mtctr r0
mr r3, r30
bctrl ; sead__Framework__procReset

lwz r12, 0x74(r30)
clrlwi. r11, r12, 31
li r31, 1
beq loc_31FA90C
extrwi. r0, r12, 1,30
beq loc_31FA90C
li r31, 0

loc_31FA90C:
lwz r0, 0x4C(r30)
cmpwi r0, 0
clrlwi r31, r31, 24
beq loc_31FA928
mtctr r0
li r3, 1
bctrl ; some lockAndUnlockFrameFunc call

loc_31FA928:
cmpwi r31, 0
beq loc_31FA944
lwz r12, 0(r30)
lwz r12, 0x10C(r12)
mtctr r12
mr r3, r30
bctrl ; sead__GameFrameworkCafe__presentAndDrawDone

loc_31FA944:
lwz r0, 0x4C(r30)
cmpwi r0, 0
beq loc_31FA95C
mtctr r0
li r3, 0
bctrl ; lockOrUnlockDrawContextMgr

loc_31FA95C:
li r0, 0
lis r12, currentEyeSide@ha
stw r0, currentEyeSide@l(r12)
li r3, 1
bl import.coreinit.hook_EndCameraSide

; ========================================================================
; ========================================================================

continueWithRendering:
lis r3, sead_GameFramework_unlockFrameDrawContext@ha
addi r3, r3, sead_GameFramework_unlockFrameDrawContext@l
mtctr r3
mr r3, r30
bctrl ; bl sead::GameFramework::unlockFrameDrawContext((void))
bl import.coreinit.OSGetSystemTime
lwz r11, 0x84(r30)
lwz r0, 0x80(r30)
subfc r12, r11, r4
subfe r0, r0, r3
stw r12, 0x7C(r30)
stw r0, 0x78(r30)
bl import.coreinit.OSGetSystemTime
lwz r11, 0x28(r30)
stw r4, 0x84(r30)
cmpwi r11, 1
stw r3, 0x80(r30)
lwz r12, 0x74(r30)
bne loc_31FA9A4
li r0, 2
stw r0, 0x28(r30)

loc_31FA9A4:
clrlwi. r0, r12, 31
beq loc_31FA9B4
xori r0, r12, 2
stw r0, 0x74(r30)

loc_31FA9B4:
lwz r12, 0(r30)
lwz r0, 0x104(r12)
mtctr r0
mr r3, r30
bctrl ; waitForVsync
lwz r0, 0x14(r1)
lwz r30, 0x08(r1)
mtlr r0
lwz r31, 0x0C(r1)
addi r1, r1, 0x10
blr

0x031FA880 = ba custom_sead__GameFramework__procFrame



; ==================================================================================
; ==================================================================================


0x02C57E4C = uking__frm__System__preCalc:
0x02C57E7C = calcAndRunStateMachine:
0x02C57ED8 = uking__frm__System__postCalc:

0x10463E7C = byte_10463E7C:
0x03414828 = gameScene__preCalc_:


0x0340EEDC = ksys__CalcPlacementMgr:
0x1047C370 = HavokWorkerMgr__sInstance:
0x1046D594 = MCMgr__sInstance:
0x031FCE6C = MCMgr__calcPostBgBaseProcMgr:
0x034155E4 = pushJobQueue2:
0x03415600 = ksys__PreCalcWorldPre:
0x03415584 = requestLODMgrModelsAndUpdateDebugInput:
0x03414D74 = runActorUpdateStuff:


custom_calcAndRunStateMachine:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)
stw r5, 0x04(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq dontRunStateMachine

lis r3, uking__frm__System__preCalc@ha
addi r3, r3, uking__frm__System__preCalc@l
mtctr r3
lwz r3, 0x0C(r1)
lwz r4, 0x08(r1)
lwz r5, 0x04(r1)
bctrl

lis r3, calcAndRunStateMachine@ha
addi r3, r3, calcAndRunStateMachine@l
mtctr r3
lwz r3, 0x0C(r1)
lwz r4, 0x08(r1)
lwz r5, 0x04(r1)
bctrl

lis r3, uking__frm__System__postCalc@ha
addi r3, r3, uking__frm__System__postCalc@l
mtctr r3
lwz r3, 0x0C(r1)
lwz r4, 0x08(r1)
lwz r5, 0x04(r1)
bctrl

b continuecalcAndRunStateMachine__run

dontRunStateMachine:
; HERE WE RUN OUR CUSTOM CODE

;lis r3, uking__frm__System__preCalc@ha
;addi r3, r3, uking__frm__System__preCalc@l
;mtctr r3
;lwz r3, 0x0C(r1)
;lwz r4, 0x08(r1)
;lwz r5, 0x04(r1)
;bctrl

; ===== PRE CALC STATE MACHINE =====
lis r3, byte_10463E7C@ha
lbz r3, byte_10463E7C@l(r3)
bl gameScene__preCalc_


; lis r3, requestLODMgrModelsAndUpdateDebugInput@ha
; addi r3, r3, requestLODMgrModelsAndUpdateDebugInput@l
; mtctr r3
; bctrl ; bl requestLODMgrModelsAndUpdateDebugInput

; 0x031FBA24 = MCMgr__preCalc:
; lis r3, MCMgr__preCalc@ha
; addi r3, r3, MCMgr__preCalc@l
; mtctr r3
; lis r3, MCMgr__sInstance@ha
; lwz r3, MCMgr__sInstance@l(r3)
; bctrl ; bl MCMgr__preCalc



; ===== RUN STATE MACHINE =====

; lis r3, ksys__PreCalcWorldPre@ha
; addi r3, r3, ksys__PreCalcWorldPre@l
; mtctr r3
; bctrl ; bl ksys__PreCalcWorldPre

;0x03417E5C = teraStuff:
;lis r3, teraStuff@ha
;addi r3, r3, teraStuff@l
;mtctr r3
;bctrl ; bl teraStuff

; run ksys::PreCalcWorldPre things
; lis r3, ksys__CalcPlacementMgr@ha
; addi r3, r3, ksys__CalcPlacementMgr@l
; mtctr r3
; bctrl ; bl ksys__CalcPlacementMgr

; this runs all actor related update code + uses gsys::Model::requestDraw() which is needed to get the models to render
; ; run MCMgr__calcPostBgBaseProcMgr(MCMgr::sInstance)
; lis r3, MCMgr__calcPostBgBaseProcMgr@ha
; addi r3, r3, MCMgr__calcPostBgBaseProcMgr@l
; mtctr r3
; lis r3, MCMgr__sInstance@ha
; lwz r3, MCMgr__sInstance@l(r3)
; bctrl ; bl MCMgr__calcPostBgBaseProcMgr

; lis r3, runActorUpdateStuff@ha
; addi r3, r3, runActorUpdateStuff@l
; mtctr r3
; bctrl ; bl runActorUpdateStuff

; lis r3, gsys__SystemTask__invokeCalcFrame_@ha
; addi r3, r3, gsys__SystemTask__invokeCalcFrame_@l
; mtctr r3
; lis r3, gsys__SystemTask__sInstance_0@ha
; lwz r3, gsys__SystemTask__sInstance_0@l(r3)
; bctrl ; bl gsys__SystemTask__invokeCalcFrame_
; ;bla gsys__SystemTask__invokeCalcFrame_

; lis r3, pushJobQueue2@ha
; addi r3, r3, pushJobQueue2@l
; mtctr r3
; li r3, 2
; bctrl ; bl pushJobQueue2

; 0x031FD360 = MCMgr__calc:
; lis r3, MCMgr__calc@ha
; addi r3, r3, MCMgr__calc@l
; mtctr r3
; lis r3, MCMgr__sInstance@ha
; lwz r3, MCMgr__sInstance@l(r3)
; bctrl ; bl MCMgr__calc

; 0x03415CD0 = ksys__calcEntryJob:
;lis r3, ksys__calcEntryJob@ha
;addi r3, r3, ksys__calcEntryJob@l
;mtctr r3
;bctrl ; bl ksys__calcEntryJob

; run gameScene::calcGraphicsStuff(*a2)
0x03416590 = gameScene__calcGraphicsStuff:

lis r3, gameScene__calcGraphicsStuff@ha
addi r3, r3, gameScene__calcGraphicsStuff@l
mtctr r3
lwz r3, 0x0C(r1)
lwz r4, 0x08(r1)
lwz r5, 0x04(r1)
lwz r3, 0x0(r4)
bctrl

;lis r3, uking__frm__System__postCalc@ha
;addi r3, r3, uking__frm__System__postCalc@l
;mtctr r3
;lwz r3, 0x0C(r1)
;lwz r4, 0x08(r1)
;lwz r5, 0x04(r1)
;bctrl

continuecalcAndRunStateMachine__run:
lwz r0, 0x14(r1)
mtlr r0
lwz r3, 0x0C(r1)
lwz r4, 0x08(r1)
lwz r5, 0x04(r1)
addi r1, r1, 0x10
blr


0x02C4BFE0 = bl custom_calcAndRunStateMachine
0x02C4BFEC = nop
0x02C4BFF8 = nop

; ==================================================================================
; ==================================================================================
; ==================================================================================
; ==================================================================================

0x03A14D74 = sead__Delegate_gsys__SystemTask___invoke:
0x030A0594 = sead__Delegate__RootTaskAndControllerMgr__invoke:
0x0309D890 = sead__Delegate__Root38AndControllerMgr__invoke:

; 0x02DAFABC = NOT USED
; 0x037D4B40 = NOT USED
; 0x0320549C = NOT USED
; 0x030A2C00 = NOT USED

str_callRec_didCall:
.string "sead::MethodTreeNode::callRec( %08X ) did get called %c"

str_callRec_didNotCall:
.string "sead::MethodTreeNode::callRec( %08X ) did not get called %c"

hook_skipCallRecFunctions:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 0
beq doCallRec

lis r3, sead__Delegate_gsys__SystemTask___invoke@ha
addi r3, r3, sead__Delegate_gsys__SystemTask___invoke@l
cmpw r10, r3
beq doCallRec

lis r3, sead__Delegate__RootTaskAndControllerMgr__invoke@ha
addi r3, r3, sead__Delegate__RootTaskAndControllerMgr__invoke@l
cmpw r10, r3
beq doCallRec


;b doCallRec
b dontCallRec

doCallRec:
; --- LOGGING ---
stwu r1, -0x20(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)
li r5, 10
mr r4, r10
lis r3, str_callRec_didCall@ha
addi r3, r3, str_callRec_didCall@l
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
bl import.coreinit.OSReport
lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
lwz r5, 0x14(r1)
lwz r6, 0x10(r1)
addi r1, r1, 0x20
; --- LOGGING ---

lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
bctrl

b continue_callRec

dontCallRec:
; --- LOGGING ---
stwu r1, -0x20(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)
li r5, 10
mr r4, r10
lis r3, str_callRec_didNotCall@ha
addi r3, r3, str_callRec_didNotCall@l
crxor 4*cr1+eq, 4*cr1+eq, 4*cr1+eq
bl import.coreinit.OSReport
lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
lwz r5, 0x14(r1)
lwz r6, 0x10(r1)
addi r1, r1, 0x20
; --- LOGGING ---

b continue_callRec

continue_callRec:
lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

0x0309FA60 = bl hook_skipCallRecFunctions

; ==================================================================================
; ==================================================================================
; ==================================================================================
; ==================================================================================


preventUnrequestingDraw:
lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq keepRequestDraw

unsetRequestDraw:
ori r11, r11, 2

keepRequestDraw:
stb r11, 0x7D(r31)
li r3, 1
blr

0x03987CA4 = bla preventUnrequestingDraw


0x03A2406C = gsys__ModelJobQueue__clear:

preventModelQueueClear:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_preventModelQueueClear

lis r3, gsys__ModelJobQueue__clear@ha
addi r3, r3, gsys__ModelJobQueue__clear@l
mtctr r3

lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
lwz r5, 0x14(r1)
bctrl


exit_preventModelQueueClear:
lwz r3, 0x1C(r1)
lwz r4, 0x18(r1)
lwz r5, 0x14(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

0x039A8D80 = bla preventModelQueueClear

; ==================================================================================
; ==================================================================================
; ==================================================================================
; ==================================================================================

; ---------------- RENDER CAMERA ----------------
;0x03AEA94C = agl__lyr__RenderInfo__RenderInfo_returnAddr:

0x105978C4 = sDefaultCameraMatrix:

modifiedCopy_seadLookAtCamera:
.float 0,0,0,0  ; mtx
.float 0,0,0,0
.float 0,0,0,0
.int 0x123 ; vtable
.float 10,2,-20 ; pos
.float 10,4,-19 ; target
.float 0,1,0    ; up

testForOverflow:
.int 0x60000000


agl__lyr__Layer__getRenderCamera:
lhz r0, 0x52(r3)
clrlwi. r12, r0, 31
bne testCameraFlags
extrwi. r11, r0, 1,26
beq returnCameraFrom1stParam

testCameraFlags:
extrwi. r11, r0, 1,30
beq returnCameraFrom1stParam
lwz r12, 0x19C(r3)
cmpwi r12, 0
bne returnDefaultCamera

returnCameraFrom1stParam:
lwz r0, 0x48(r3)
lis r3, sDefaultCameraMatrix@ha
addi r3, r3, sDefaultCameraMatrix@l
cmpwi r0, 0
beqlr
mr r3, r0
lis r12, modifiedCopy_seadLookAtCamera@ha
addi r12, r12, modifiedCopy_seadLookAtCamera@l
ba import.coreinit.hook_GetRenderCamera


returnDefaultCamera:
addi r3, r12, 4
blr

0x03AE4AA0 = ba agl__lyr__Layer__getRenderCamera

modifiedCopy_seadPerspectiveProjection:
.byte 0,0,0,0 ; dirty, deviceDirty, pad, pad
.int 0 ; devicePosture
.float 0,0,0,0 ; mtx
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0 ; deviceMtx
.float 0,0,0,0
.float 0,0,0,0
.float 0,0,0,0
.float 0 ; deviceZScale
.float 0 ; deviceZOffset
.int 0x1337 ; vtable
.float 0 ; near
.float 25000 ; far
.float 0.8726647 ; angle
.float 0.4226183 ; fovySin
.float 0.9063078 ; fovyCos
.float 0.4663077 ; fovyTan
.float 1.7777778 ; aspect
.float 0 ; offsetX
.float 0 ; offsetY



0x10597928 = sDefaultSeadProjection:

agl__lyr__Layer__getRenderProjection:
lhz r0, 0x54(r3)
extrwi. r0, r0, 1,30
beq returnProjectionFrom1stParam
lwz r12, 0x19C(r3)
cmpwi r12, 0
bne useSpecialProjection

returnProjectionFrom1stParam:
lwz r12, 0x4C(r3)
lis r3, sDefaultSeadProjection@ha
addi r3, r3, sDefaultSeadProjection@l
cmpwi r12, 0
beqlr
mr r3, r12
lis r12, modifiedCopy_seadPerspectiveProjection@ha
addi r12, r12, modifiedCopy_seadPerspectiveProjection@l
ba import.coreinit.hook_GetRenderProjection

useSpecialProjection:
lwz r3, 0xEC(r12)
blr

0x03AE4AEC = ba agl__lyr__Layer__getRenderProjection