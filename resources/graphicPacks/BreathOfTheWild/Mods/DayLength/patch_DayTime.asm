[BotW_DayTime_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


const_timeMultiplier:
.float $timeMultiplier

const_cloudMultiplier:
.float $cloudMultiplier + (($cloudMultiplier == 0) * $timeMultiplier)


; Normal Time Mode - Time
multiplyTimeStep:
lfs f7, 0xA4(r30) ; original instruction to load timestep
lis r4, const_timeMultiplier@ha
lfs f8, const_timeMultiplier@l(r4)
fmuls f7, f7, f8
blr

0x0365FF78 = bla multiplyTimeStep

; Normal Time Mode - Clouds
multiplyCloudStep:
lis r4, const_cloudMultiplier@ha
lfs f11, const_cloudMultiplier@l(r4)
fmuls f6, f6, f11
stfs f6, 0xB0(r30)
blr

0x03660018 = bla multiplyCloudStep

; OnlyUpdateTimeOfDay Mode - Time & Clouds

multiplyOnlyTimeStep:
lfs f9, 0xA4(r30)
lis r4, const_timeMultiplier@ha
lfs f6, const_timeMultiplier@l(r4)
fmuls f9, f9, f6
blr

0x03660154 = bla multiplyOnlyTimeStep


; Change mode to one of the forced ones when forced time is enabled
calcForceTime:
li r12, $timeCycleMode
cmpwi r12, 1
beq noForcedTime

li r12, $dayTimeEnum
cmpwi r0, 0
blr
noForcedTime:
lbz r12, 0x129(r30)
cmpwi r0, 0
blr

0x0365FB18 = bla calcForceTime

calcForceTime2:
li r12, $timeCycleMode
cmpwi r12, 1
beq noForcedTime1

li r12, $dayTimeEnum ; Load forced time
cmpwi r0, 0
blr
noForcedTime1:
lbz r12, 0x129(r30) ; Normal load instruction
cmpwi r0, 0
blr

0x0365FE0C = bla calcForceTime2


clockUnits:
li r0, $clockAdjust
cmpwi r0, 0
beq clockSkip
subf r0, r12, r3 ; original instruction
clockSkip:
blr

0x307C130 = bla clockUnits


0x10301850 = .float $bloodMoonTime
