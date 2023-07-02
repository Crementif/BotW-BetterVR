[BetterVR_UpdateSettings_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


data_settingsOffset:
ModeSetting:
.int $mode
AlternatingEyeRenderingSetting:
.int $AER
EyeSeparationSetting:
.float $eyeSeparation
HeadPositionSensitivitySetting:
.float $headPositionSensitivity
HeightPositionOffsetSetting:
.float $heightPositionOffset
HUDScaleSetting:
.float $hudScale
MenuScaleSetting:
.float $menuScale


vr_updateSettings:
addi r1, r1, -12
stw r3, 0x0(r1)
stw r5, 0x4(r1)
mflr r5
stw r5, 0x8(r1)

li r4, -1 ; Execute the instruction that got replaced

lis r5, data_settingsOffset@ha
addi r5, r5, data_settingsOffset@l
bl import.coreinit.hook_UpdateSettings

lwz r5, 0x8(r1)
mtlr r5
lwz r5, 0x4(r1)
lwz r3, 0x0(r1)
addi r1, r1, 12
b 0x031faaf4

0x031FAAF0 = ba vr_updateSettings