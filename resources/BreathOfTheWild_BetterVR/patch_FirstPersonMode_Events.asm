[BetterVR_FirstPersonMode_Events_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


0x02C036E4 = act__getCamera:
0x030EA2CC = ksys__act__ai__ActionBase__setFinished:

hook_calcCameraDuringEvent_trampoline:
; at entry r3 = this+0x10 (arg for getCamera), LR = return addr in caller
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)

; c++ hook decides whether to skip the event camera updates based on the current event's settings
bla import.coreinit.hook_ShouldSkipEventCamera

cmpwi r3, 0
bne skipEventCamera

lwz r3, 0x0C(r1)
lis r4, act__getCamera@ha
addi r4, r4, act__getCamera@l
mtctr r4
bctrl
b done_calcCameraDuringEvent_trampoline

skipEventCamera:
; mark the action as finished so the game doesn't get stuck waiting for it, and return 0 to skip the camera update
lwz r3, 0x0C(r1)
addi r3, r3, -0x10 ; r3 = ActionBase this (getCamera arg was this+0x10)
lis r4, ksys__act__ai__ActionBase__setFinished@ha
addi r4, r4, ksys__act__ai__ActionBase__setFinished@l
mtctr r4
bctrl
li r3, 0

done_calcCameraDuringEvent_trampoline:
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

; patch CameraEventAnim (going down the elevator after loading into the shrine, shrine of resurrection wake up, etc.)
0x02BA3214 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventMovePos (teleport for example)
0x02BB9D30 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventMove (enter shrine elevator from overworld)
0x02BB1F68 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventGameOver
0x02BAA378 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventLook and CameraEventLookDirect
0x02BACE10 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventMultiTalk
0x02BBE844 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventTalk
0x02BC7014 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventTurn
0x02BCD3F4 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventPolarCoordPlayerRel (shrine with test of strength whenever the boss comes up from the ground)
0x02BC1A28 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventTalkManualCtrl/CameraEventTalkManualCtrlRet (repro by talking to koroks)
0x02BCB414 = bla hook_calcCameraDuringEvent_trampoline
; patch CameraEventIdling (seems to cause jumps, seen after cliff scene @ shrine of resurrection)
0x02BAABC0 = bla hook_calcCameraDuringEvent_trampoline


; --------------------------------------------------------------------------------------
; run a function every frame to check the current event and update settings accordingly

0x1046D3AC = EventMgr__sInstance:
0x031CA1C0 = EventMgr__getActiveEventName:

vr_updateSettings:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x1C(r1)
stw r4, 0x18(r1)
stw r5, 0x14(r1)
stw r6, 0x10(r1)

lis r6, data_TableOfCutsceneEventsSettings@ha
addi r6, r6, data_TableOfCutsceneEventsSettings@l
bl import.coreinit.hook_UpdateSettings
lwz r5, 0x14(r1)
lwz r6, 0x10(r1)

; get event and entrypoint strings
lis r3, EventMgr__sInstance@ha
lwz r3, EventMgr__sInstance@l(r3)
cmpwi r3, 0
beq skipGetEventName

; get active event name
lis r3, EventMgr__getActiveEventName@ha
addi r3, r3, EventMgr__getActiveEventName@l
mtctr r3
li r3, 0
addi r5, r1, 0x0C ; ptr to store entrypoint name
stw r3, 0x0C(r1)
addi r4, r1, 0x08 ; ptr to store event name
stw r3, 0x08(r1)
lis r3, EventMgr__sInstance@ha
lwz r3, EventMgr__sInstance@l(r3)
bctrl ; bl EventMgr::getActiveEventName

; call C++ hook to handle the results
lwz r4, 0x08(r1) ; event name
lwz r5, 0x0C(r1) ; entrypoint name
bla import.coreinit.hook_GetEventName

skipGetEventName:
lwz r6, 0x10(r1)
lwz r5, 0x14(r1)
lwz r4, 0x18(r1)
lwz r3, 0x1C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0

li r4, -1 ; Execute the instruction that got replaced
blr

0x031FAAF0 = bla vr_updateSettings