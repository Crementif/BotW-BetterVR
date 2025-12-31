[BetterVR_UpdateSettings_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

data_settingsOffset:
CameraModeSetting:
.int $cameraMode

LeftHandModeSetting:
.int $leftHanded

GUIFollowModeSetting:
.int $guiFollowMode

PlayerHeightSetting:
.float $cameraHeight

Enable2DViewSetting:
.int $enable2DView

CropFlatTo16_9Setting:
.int $cropFlatTo16_9

EnableDebugOverlaySetting:
.int $enableDebugOverlay

BuggyAngularVelocitySetting:
.int $buggyAngularVelocity

CutsceneCameraMode:
.int $cutsceneCameraMode

CutsceneBlackBars:
.int $cutsceneBlackBars



eventName:
.int 0
.int 0
.int 0
.int 0

entryPointName:
.int 0
.int 0
.int 0
.int 0

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
stw r7, 0x0C(r1)
stw r8, 0x08(r1)

lis r5, data_settingsOffset@ha
addi r5, r5, data_settingsOffset@l
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
lis r3, EventMgr__sInstance@ha
lwz r3, EventMgr__sInstance@l(r3)
addis r3, r3, 1
lwz r3, 0x75B0(r3)
; EventMgr::sInstance->activeEventContext
cmpwi r3, 0
beq noActiveEvent

; r3 = EventMgr::sInstance->activeEventContext

; load active event name
lwz r4, 0x18C(r3) ; load index to event indices
addi r8, r3, 0x184 ; load array start
cmplwi r4, 8 ;.int 0x28040008 ;cmplwi r4, 8 ; .long 0x28040008
lwz r6, 4(r3)
bge loc_31D8C8C
add r8, r8, r4

loc_31D8C8C:
; r4 is unused from here
li r4, 0
li r7, 0
; -------------------------

lbz r5, 0(r8)
extsb r5, r5 ; .int 0x7CA50774 ; extsb r5, r5
cmplw r5, r6
li r4, 0
bge loc_31D8CAC
lwz r7, 0x0C(r3)
slwi r8, r5, 2
lwzx r4, r8, r7

loc_31D8CAC:
cmpwi r4, 0
beq noActiveEvent ; skip if no valid event pointer
lwz r4, 8(r4)
li r3, 1
bla import.coreinit.hook_GetEventName
b skipGetEventName
;lis r3, EventMgr__sInstance@ha
;lwz r3, EventMgr__sInstance@l(r3)
;bctrl

noActiveEvent:
li r3, 0
bla import.coreinit.hook_GetEventName

skipGetEventName:

; spawn check
li r3, 0
bl import.coreinit.hook_CreateNewActor
cmpwi r3, 1
bne notSpawnActor
;bl vr_spawnEquipment
notSpawnActor:

;bl checkIfDropWeapon

lwz r8, 0x08(r1)
lwz r7, 0x0C(r1)
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