[BotW_VSync_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

setSwapInterval:
li r11, $keepVsync
cmpwi r11, 1
bne setSwapIntervalTo0

lis r11, fpsLimit@ha
lfs f12, fpsLimit@l(r11)

lis r11, const_30@ha
lfs f10, const_30@l(r11)
fcmpu cr0, f10, f12
beq setSwapIntervalTo2
# lis r11, const_60@ha
# lfs f10, const_60@l(r11)
# fcmpu cr0, f10, f12
# beq setSwapIntervalTo1
; Disable vsync for everything else
b setSwapIntervalTo0


setSwapIntervalTo0: ; Disable vsync
mflr r11
li r3, 0
bl import.gx2.GX2SetSwapInterval
mtlr r11
b _convertTicksToFrametime

setSwapIntervalTo1: ; Double Buffered 30FPS
mflr r11
li r3, 1
bl import.gx2.GX2SetSwapInterval
mtlr r11
b _convertTicksToFrametime

setSwapIntervalTo2: ; Double Buffered 30FPS
mflr r11
li r3, 2
bl import.gx2.GX2SetSwapInterval
mtlr r11
b _convertTicksToFrametime


waitForVsyncAddr:
.ptr 0x031FACD4

conditionalVsyncWait:
stwu r1, -0x20(r1)

li r12, $keepVsync
cmpwi r12, 1
beqlr

lis r12, waitForVsyncAddr@ha
lwz r12, waitForVsyncAddr@l(r12)
mtlr r12
blr

0x031FACCC = bla conditionalVsyncWait


conditionalSwapStatus:
li r6, $keepVsync
cmpwi r6, 1
beq continueSwapStatus
cmpw r12, r12
blr
continueSwapStatus:
cmplw r12, r0
blr

0x031FACF0 = bla conditionalSwapStatus


# Disable vsync entirely if not running at 30FPS
# 0x031FACD0 = .uint (($keepVsync == 1) * 0x4914DAB9) + (($keepVsync == 0) * 0x60000000)
# 0x031FACF4 = .uint (($keepVsync == 1) * 0x4180FFDC) + (($keepVsync == 0) * 0x60000000)