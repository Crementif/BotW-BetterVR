[BotW_DeferredReadbacks_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; GX2DrawDone makes Cemu wait until the GPU has handed back every texture readback and occlusion query.
; The game only uses those results a frame later, so this only waits for Cemu to process the submitted commands.
; It's the same as turning off "Full sync at GX2DrawDone()" in OpenGL.

; the terrain bakes water colours into a 64x1 target that the CPU reads a frame later, so those frames keep the full sync
fullSyncDrawDones:
.int 0

drawDoneWithoutReadbacks:
li r0, $deferGpuReadbacks
cmpwi r0, 0
beq drawDoneWithoutReadbacks_original
lis r11, fullSyncDrawDones@ha
lwz r12, fullSyncDrawDones@l(r11)
cmpwi r12, 0
beq drawDoneWithoutReadbacks_deferred
addi r12, r12, -1
stw r12, fullSyncDrawDones@l(r11)
b drawDoneWithoutReadbacks_original
drawDoneWithoutReadbacks_deferred:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
bl import.gx2.GX2Flush
bl import.gx2.GX2GetLastSubmittedTimeStamp
bl import.gx2.GX2WaitTimeStamp
lwz r0, 0x14(r1)
mtlr r0
addi r1, r1, 0x10
blr
drawDoneWithoutReadbacks_original:
ba import.gx2.GX2DrawDone

0x031FAA10 = bla drawDoneWithoutReadbacks ; sead::GameFrameworkCafe::presentAndDrawDone
0x031FAAB8 = bla drawDoneWithoutReadbacks
0x031FAB20 = bla drawDoneWithoutReadbacks
0x0309D0A8 = bla drawDoneWithoutReadbacks ; sead::GameFrameworkCafe::swapBuffer_
0x0309D140 = bla drawDoneWithoutReadbacks

requestFullSync:
lis r11, fullSyncDrawDones@ha
li r12, 4
stw r12, fullSyncDrawDones@l(r11)
blr

bakeTerrainColorUnorm:
stwu r1, -0x10(r1)
mflr r0
stw r0, 0x14(r1)
bl requestFullSync
lis r12, 0x036E
ori r12, r12, 0x9290
mtctr r12
bctrl
lwz r0, 0x14(r1)
mtlr r0
addi r1, r1, 0x10
blr

bakeTerrainColorFloat:
stwu r1, -0x10(r1)
mflr r0
stw r0, 0x14(r1)
bl requestFullSync
lis r12, 0x036E
ori r12, r12, 0x9954
mtctr r12
bctrl
lwz r0, 0x14(r1)
mtlr r0
addi r1, r1, 0x10
blr

0x036E9EF0 = bla bakeTerrainColorUnorm ; Terrain colour bake, RGBA8
0x036E9F0C = bla bakeTerrainColorFloat ; Terrain colour bake, RGBA32F

; rune targeting reads its ID readback and coverage query a frame later, deferring them made the target flicker
idAnalyzerQueryBegin:
lis r11, fullSyncDrawDones@ha
li r12, 4
stw r12, fullSyncDrawDones@l(r11)
ba import.gx2.GX2QueryBegin

0x038B29BC = bla idAnalyzerQueryBegin ; gfx::IDAnalyzer

; Cemu reports a GPU occlusion query that hasn't finished as having 0 samples, which would hide objects.
; A CPU query reports that it isn't ready yet instead, which the game handles. Also fixes distant NPCs stuttering
; in OpenGL with "Full sync at GX2DrawDone()" turned off.
0x03B25BB0 = li r3, 0 ; agl::cull::OcclusionQueryDrawer::draw_, distance culling of actors
0x038B29B4 = li r3, 0 ; gfx::IDAnalyzer, used to target objects with runes
