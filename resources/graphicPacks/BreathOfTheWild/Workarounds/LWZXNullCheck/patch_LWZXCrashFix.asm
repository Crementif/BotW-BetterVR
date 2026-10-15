[BotW_LWZXCrashFix_Shared]
moduleMatches = 0xD91A406D,0x0F748D9C,0x9A61FF4C,0x8E3324A9,0xD71D859D,0x6FD41A61,0x9A2CA0C7,0x29DBB52A,0xFD091F9F,0xD472D8A5,0x6267BFD0

.origin = codecave

; null checked version of 'lwzx r31, r9, r0'
_lwzxCheck:
add r30, r9, r0
xor r31, r31, r31
cmpwi r30, 0
beq .+0x8
lwzx r31, r9, r0
blr

; null checked version of 'lwz r8, 0(r31)'
_lwzCheck:
xor  r8, r8, r8
oris r8, r8, 0xD15A
ori  r8, r8, 0xB1ED
cmpwi r31, 0
beq .+0x8
lwz r8, 0(r31)
blr


[BotW_LWZXCrashFix_V33]
moduleMatches = 0xD91A406D

; restore the 2 instructions possibly overwritten by cemu itself
0x03721B00 = stwu r1, -0x28(r1)
0x03721B04 = mflr r0

; replace simple loads with calls to the patched functions
0x03721CA8 = bla _lwzxCheck
0x03721CB4 = bla _lwzCheck

0x03721D20 = bla _lwzxCheck
0x03721D2C = bla _lwzCheck

[BotW_LWZXCrashFix_V48]
moduleMatches = 0x0F748D9C

; restore the 2 instructions possibly overwritten by cemu itself
0x03721F0C = stwu r1, -0x28(r1)
0x03721F10 = mflr r0

; replace simple loads with calls to these funcs
0x037220B4 = bla _lwzxCheck
0x037220C0 = bla _lwzCheck

0x0372212C = bla _lwzxCheck
0x03722138 = bla _lwzCheck

[BotW_LWZXCrashFix_V64]
moduleMatches = 0x9A61FF4C

; restore the 2 instructions possibly overwritten by cemu itself
0x037235F4 = stwu r1, -0x28(r1)
0x037235F8 = mflr r0

; replace simple loads with calls to these funcs
0x0372379C = bla _lwzxCheck
0x037237A8 = bla _lwzCheck

0x03723814 = bla _lwzxCheck
0x03723820 = bla _lwzCheck


[BotW_LWZXCrashFix_V80]
moduleMatches = 0x8E3324A9

; restore the 2 instructions possibly overwritten by cemu itself
0x03744368 = stwu r1, -0x28(r1)
0x0374436C = mflr r0

; replace simple loads with calls to these funcs
0x03744510 = bla _lwzxCheck
0x0374451C = bla _lwzCheck

0x03744588 = bla _lwzxCheck
0x03744594 = bla _lwzCheck

[BotW_LWZXCrashFix_V97]
moduleMatches = 0xD71D859D

; restore the 2 instructions possibly overwritten by cemu itself
0x03799E3C = stwu r1, -0x28(r1)
0x03799E40 = mflr r0

; replace simple loads with calls to these funcs
0x03799FE4 = bla _lwzxCheck
0x03799FF0 = bla _lwzCheck

0x0379A05C = bla _lwzxCheck
0x0379A068 = bla _lwzCheck

[BotW_LWZXCrashFix_V112]
moduleMatches = 0x6FD41A61

; restore the 2 instructions possibly overwritten by cemu itself
0x0379AF6C = stwu r1, -0x28(r1)
0x0379AF70 = mflr r0

; replace simple loads with calls to these funcs
0x0379B114 = bla _lwzxCheck
0x0379B120 = bla _lwzCheck

0x0379B18C = bla _lwzxCheck
0x0379B198 = bla _lwzCheck

[BotW_LWZXCrashFix_V144]
moduleMatches = 0x9A2CA0C7

; restore the 2 instructions possibly overwritten by cemu itself
0x037CF648 = stwu r1, -0x28(r1)
0x037CF64C = mflr r0

; replace simple loads with calls to these funcs
0x037CF7F0 = bla _lwzxCheck
0x037CF7FC = bla _lwzCheck

0x037CF868 = bla _lwzxCheck
0x037CF874 = bla _lwzCheck

[BotW_LWZXCrashFix_V160]
moduleMatches = 0x29DBB52A

; restore the 2 instructions possibly overwritten by cemu itself
0x037CF514 = stwu r1, -0x28(r1)
0x037CF518 = mflr r0

; replace simple loads with calls to these funcs
0x037CF6BC = bla _lwzxCheck
0x037CF6C8 = bla _lwzCheck

0x037CF734 = bla _lwzxCheck
0x037CF740 = bla _lwzCheck

[BotW_LWZXCrashFix_V176V192]
moduleMatches = 0xFD091F9F,0xD472D8A5

; restore the 2 instructions possibly overwritten by cemu itself
0x038E3A30 = stwu r1, -0x28(r1)
0x038E3A34 = mflr r0

; replace simple loads with calls to these funcs
0x038E3BD8 = bla _lwzxCheck
0x038E3BE4 = bla _lwzCheck

0x038E3C50 = bla _lwzxCheck
0x038E3C5C = bla _lwzCheck

[BotW_LWZXCrashFix_V208]
moduleMatches = 0x6267BFD0

; restore the 2 instructions possibly overwritten by cemu itself
0x038E46CC = stwu r1, -0x28(r1)
0x038E46D0 = mflr r0

; replace simple loads with calls to these funcs
0x038E4874 = bla _lwzxCheck
0x038E4880 = bla _lwzCheck

0x038E48EC = bla _lwzxCheck
0x038E48F8 = bla _lwzCheck