[BotW_Cutscene_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

cutsceneMode:
.int $cutsceneFPSLimitMode

lastCutsceneId:
.int 0

cutsceneFPSLimit:
.float 0


_setCutsceneFPSLimit:
stw r0, 4(r31) ; Original instruction

; Register r9 still holds the ID of this cutscene
; Register r29 is the parameter structure that this object is created with which also holds the event name 
lwz r12, 8(r29)

; Check if the event name is not 0
cmpwi r12, 0
beqlr

; Check if the to-be-created event is a cutscene by checking if the event name starts with *emo
lbz r3, 1(r12)
cmpwi r3, 0x65 ; e
bnelr

lbz r3, 2(r12)
cmpwi r3, 0x6D ; m
bnelr

lbz r3, 3(r12)
cmpwi r3, 0x6F ; o
bnelr

; Enforce cutscene mode settings
lis r3, cutsceneMode@ha
lwz r3, cutsceneMode@l(r3)
cmpwi r3, 0
beqlr
cmpwi r3, 2
beq _setCutscene60FPSLimit
cmpwi r3, 3
beq _setCutscene30FPSLimit

; Load each digit of the "Demo000_0" string pointer in the params, then add the numbers together
li r22, 0

lbz r3, 4(r12)
addi r3, r3, -0x30
mulli r3, r3, 1000
add r22, r22, r3

lbz r3, 5(r12)
addi r3, r3, -0x30
mulli r3, r3, 100
add r22, r22, r3

lbz r3, 6(r12)
addi r3, r3, -0x30
mulli r3, r3, 10
add r22, r22, r3

lbz r3, 8(r12)
addi r3, r3, -0x30
mulli r3, r3, 1
add r22, r22, r3

cmpwi r22, 3480 ; demo348_0 ; Could be part of the crash at the Rito Village bridge
beq _setCutscene30FPSLimit

cmpwi r22, 3481 ; demo348_1 ; Crash at the Rito Village bridge
beq _setCutscene30FPSLimit

cmpwi r22, 1620 ; demo162_0 ; Perhaps the crash at the end of vah ruta
beq _setCutscene30FPSLimit
cmpwi r22, 1630 ; demo163_0 ; Perhaps the crash at the end of vah ruta
beq _setCutscene30FPSLimit
cmpwi r22, 3340 ; demo334_0 ; Perhaps the crash at the end of vah ruta
beq _setCutscene30FPSLimit

# cmpwi r22, 1020 ; demo102_0 ; Start Game - Bright light waking up link on blackscreen
# beq _setCutscene30FPSLimit
cmpwi r22, 1690 ; demo169_0 ; Start Game - Zelda waking up link dialogue on blackscreen
beq _setCutscene30FPSLimit

cmpwi r22, 1030 ; demo103_0 ; Start Game - Walking out of shrine of resurrection
beq _setCutscene60FPSLimit

; A bunch of divine beast enter and exit events. Could definitely be narrowed down in the future.
cmpwi r22, 1100 ; demo110_0
beq _setCutscene30FPSLimit
cmpwi r22, 1110 ; demo111_0
beq _setCutscene30FPSLimit
cmpwi r22, 1120 ; demo112_0
beq _setCutscene30FPSLimit
cmpwi r22, 1130 ; demo113_0
beq _setCutscene30FPSLimit
cmpwi r22, 1140 ; demo114_0
beq _setCutscene30FPSLimit
cmpwi r22, 1150 ; demo115_0
beq _setCutscene30FPSLimit
cmpwi r22, 1160 ; demo116_0
beq _setCutscene30FPSLimit
cmpwi r22, 1170 ; demo117_0
beq _setCutscene30FPSLimit
cmpwi r22, 1180 ; demo118_0
beq _setCutscene30FPSLimit
cmpwi r22, 1190 ; demo119_0
beq _setCutscene30FPSLimit
cmpwi r22, 1200 ; demo120_0
beq _setCutscene30FPSLimit
cmpwi r22, 1210 ; demo121_0
beq _setCutscene30FPSLimit
cmpwi r22, 1220 ; demo122_0
beq _setCutscene30FPSLimit
cmpwi r22, 1230 ; demo123_0
beq _setCutscene30FPSLimit
cmpwi r22, 1240 ; demo124_0
beq _setCutscene30FPSLimit
cmpwi r22, 1250 ; demo125_0
beq _setCutscene30FPSLimit
blr

; 30FPS limit case
_setCutscene30FPSLimit:
lis r3, const_30@ha
lfs f10, const_30@l(r3)
b _returnWithCutsceneFPSLimit

; 60FPS limit case
_setCutscene60FPSLimit:
lis r3, const_60@ha
lfs f10, const_60@l(r3)
b _returnWithCutsceneFPSLimit

_returnWithCutsceneFPSLimit:
lis r3, lastCutsceneId@ha
stw r9, lastCutsceneId@l(r3)
lis r3, cutsceneFPSLimit@ha
stfs f10, cutsceneFPSLimit@l(r3)
blr


_removeCutsceneFPSLimit:
lwz r11, 0x27C(r30) ; Original instruction

lwz r9, 0x274(r30)
lis r8, lastCutsceneId@ha
lwz r8, lastCutsceneId@l(r8)
cmpw r9, r8
bnelr

lis r8, const_0.0@ha
lfs f10, const_0.0@l(r8)
lis r8, cutsceneFPSLimit@ha
stfs f10, cutsceneFPSLimit@l(r8)
blr


; Check if a cutscene is ongoing with a set FPS limit
_checkCutsceneFPSLimit:
lis r11, const_0.0@ha
lfs f7, const_0.0@l(r11)
lis r11, cutsceneFPSLimit@ha
lfs f12, cutsceneFPSLimit@l(r11)
fcmpu cr0, f12, f7
beq _calculateDynamicFPS

; If a cutscene FPS limit is set, lower FPS
_lowerCutsceneFPS:
; Calculate how many ticks a frame has to take to render at the given FPS limit
lis r11, const_1@ha
lfs f7, const_1@l(r11)
fdivs f12, f7, f12
lis r11, timerTickSpeed@ha
lfs f7, timerTickSpeed@l(r11)
fmuls f12, f7, f12

; Subtract the time that it took for the actual frame to render and make sure that it's not negative (which means a frame already took longer to render then the FPS limit)
fsubs f12, f12, f10
lis r11, const_0.0@ha
lfs f7, const_0.0@l(r11)
fcmpu cr0, f7, f12
ble .+0x8
fmr f12, f7

; Convert the remaining ticks that should be spend idling to an integer (but it requires storing it in memory)
fctiwz f12, f12
lis r11, floatConvHa@ha
stfd f12, floatConvHa@l(r11)

; Load the converted integer ticks and sleep for the given amount of time
mr r12, r4
li r3, 0
lis r11, floatConvL@ha
lwz r4, floatConvL@l(r11)
mflr r11
bl import.coreinit.OSSleepTicks
mtlr r11
mr r4, r12

lis r11, cutsceneFPSLimit@ha
lfs f10, cutsceneFPSLimit@l(r11)

; Skip dynamic FPS code when cutscene FPS limit is in place since the game should always run at 30FPS
b _setGamespeed


; Hook evt::EventFlowBase's ctor (event object creation)
0x031D2D2C = bla _setCutsceneFPSLimit

; Hook evt::EventFlowBase's dtor (event object destruction)
0x031D3798 = bla _removeCutsceneFPSLimit