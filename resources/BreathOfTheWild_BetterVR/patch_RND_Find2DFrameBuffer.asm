[BetterVR_Find2DFrameBuffer_V208]
moduleMatches = 0x6267BFD0

.origin = codecave


; r3 = agl::RenderBuffer*
capture2DFromRenderBuffer:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)

stfs f1, 0x1C(r1)
stfs f2, 0x18(r1)
stfs f3, 0x14(r1)
stfs f4, 0x10(r1)
stw r3, 0x2C(r1)
stw r4, 0x28(r1)
stw r5, 0x24(r1)
stw r12, 0x20(r1)

; Cemu hands the HUD's color image to the module through a host event
bl hook_emitCaptureEvent2D

lwz r12, 0x20(r1)
lwz r5, 0x24(r1)
lwz r4, 0x28(r1)
lwz r3, 0x2C(r1)
lfs f4, 0x10(r1)
lfs f3, 0x14(r1)
lfs f2, 0x18(r1)
lfs f1, 0x1C(r1)

lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30

blr
