[BotW_DeferredReadbacks_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; GX2DrawDone makes Cemu wait until the GPU has handed back every texture readback and occlusion query.
; The game only uses those results a frame later, so this only waits for Cemu to process the submitted commands.
; It's the same as turning off "Full sync at GX2DrawDone()" in OpenGL.

drawDoneWithoutReadbacks:
li r0, $deferGpuReadbacks
cmpwi r0, 0
beq drawDoneWithoutReadbacks_original
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

; Cemu reports a GPU occlusion query that hasn't finished as having 0 samples, which would hide objects.
; A CPU query reports that it isn't ready yet instead, which the game handles. Also fixes distant NPCs stuttering
; in OpenGL with "Full sync at GX2DrawDone()" turned off.
0x03B25BB0 = li r3, 0 ; agl::cull::OcclusionQueryDrawer::draw_, distance culling of actors
0x038B29B4 = li r3, 0 ; gfx::IDAnalyzer, used to target objects with runes
