[BetterVR_StereoRendering_Optimizations_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; ==================================================================================
; Swap GX2SwapScanBuffers and GX2DrawDone calls around to allow VR headset to wait for vsync, and then actually sync the GPU for the next frame using GX2DrawDone, which should reduce stuttering and input lag
0x031FAB24 = returnAddr_doGx2DrawDoneAfterSwapBuffers:

doGx2DrawDoneAfterSwapBuffers:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

; telemetry: PPC frame counter tick (once per presented frame, from boot)
lis r11, _cFrame@ha
lwz r12, _cFrame@l(r11)
addi r12, r12, 1
stw r12, _cFrame@l(r11)

lwz r3, 0x0C(r1)
bl import.gx2.GX2SwapScanBuffers

lwz r3, 0x0C(r1)
bl import.gx2.GX2DrawDone

lis r3, returnAddr_doGx2DrawDoneAfterSwapBuffers@ha
addi r3, r3, returnAddr_doGx2DrawDoneAfterSwapBuffers@l
mtlr r3

lwz r4, 0x08(r1)
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
;mtlr r0
blr

0x031FAB1C = bla doGx2DrawDoneAfterSwapBuffers
0x031FAA10 = nop

; --------------------------------------------------------------------------------
; Patches below disable the shadow map projection matrices updates to prevent a mismatch when applying the old one

hook_skipShadowUpdateShadowMatrix:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_SkipShadowCalcView

0x03B0ACD0 = depthShadow_updateShadowMatrix:
lis r3, depthShadow_updateShadowMatrix@ha
addi r3, r3, depthShadow_updateShadowMatrix@l
mtctr r3
lwz r3, 0x0C(r1)
bctrl ; bl depthShadow_updateShadowMatrix

exit_SkipShadowCalcView:
lwz r4, 0x08(r1)
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x039DE028 = bla hook_skipShadowUpdateShadowMatrix


hook_skipShadowCalcViewMasked:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_SkipShadowCalcViewMasked

0x039DEA48 = modelSceneShadow_calcViewMaskedLightShadow:
lis r3, modelSceneShadow_calcViewMaskedLightShadow@ha
addi r3, r3, modelSceneShadow_calcViewMaskedLightShadow@l
mtctr r3
lwz r3, 0x0C(r1)
bctrl ; bl modelSceneShadow_calcViewMaskedLightShadow

exit_SkipShadowCalcViewMasked:
lwz r4, 0x08(r1)
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x039AC9B0 = bla hook_skipShadowCalcViewMasked

; --------------------------------------------------------------------------------
; Patches below disable cascaded shadow map updating draw calls for the right eye

hook_shadowSetSizeLeftOnly:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_ShadowSetSizeLeftOnly

0x03B159F0 = shadowMap_setSize:
lis r3, shadowMap_setSize@ha
addi r3, r3, shadowMap_setSize@l
mtctr r3
lwz r3, 0x0C(r1)
bctrl ; bl shadowMap_setSize

exit_ShadowSetSizeLeftOnly:
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x039E02F8 = bla hook_shadowSetSizeLeftOnly

hook_shadowAllocBufferLeftOnly:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_ShadowAllocBufferLeftOnly

0x03B0ADC4 = shadowMap_allocDepthBuffer:
lis r3, shadowMap_allocDepthBuffer@ha
addi r3, r3, shadowMap_allocDepthBuffer@l
mtctr r3
lwz r3, 0x0C(r1)
bctrl ; bl shadowMap_allocDepthBuffer

exit_ShadowAllocBufferLeftOnly:
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x039E0304 = bla hook_shadowAllocBufferLeftOnly

; --------------------------------------------------------------------------------
; Host-writable render-skip bitmask. Lives next to the other host flags (see
; DISABLE_PPC_LOGGING_GET at 0x10416BF0, recording mode at 0x10416BF4) so the C++
; side can toggle each skip at runtime without a rebuild.
;   bit 0  (0x0001): skip DRC/GamePad draw passes (both eyes - nobody sees them in VR)
; The remaining bits skip per-frame render *preparation* on the RIGHT eye pass only,
; reusing the left eye's results (queues survive thanks to preventModelQueueClear and
; the union-frustum visibility patch already makes cull sets cover both eyes):
;   bit 1  (0x0002): type-2 "model calc" worker-job dispatch (via the ModelMgr::invoke
;                    gate; the old calcModel whole-function skip is disabled)
;   bit 2  (0x0004): type-3 world/effects worker-job dispatch (via the invoke gate;
;                    the old calcWorld whole-function skip is disabled)
;   bit 3  (0x0008): gsys::ModelMgr::calcPreSortQueue
;   bit 4  (0x0010): gsys::ModelScene::calcView (main culling/queue build)
;   bit 5  (0x0020): gsys::ModelScene::calcViewGPU
;   bit 6  (0x0040): gsys::ModelScene::invokeCalcWorld
;   bit 7  (0x0080): gsys::ModelRenderQueue::sort
;   bit 8  (0x0100): gsys::ModelSceneFx::calcView (particles)
;   bit 9  (0x0200): gsys::ModelSceneLight::calcViewLightPrePass
;   bit 10 (0x0400): gsys::ModelSceneGI::calcViewLightProbe
;   bit 11 (0x0800): gsys::ModelSceneGI::calcViewLocalReflection
;   bit 12 (0x1000): UpdateModelJobQueue
;   bit 13 (0x2000): queue-preserve - on right-eye pass, gsys::ModelScene::clearQueue
;                    runs ONLY the draw-cursor reset (swapDrawList) and skips the
;                    clear / changeRequestFlag / updateGPU mutations, so the draw list,
;                    per-model request flags, and the clear/flag alternation state all
;                    survive for redraw. REQUIRED companion for bits 1-12: without it
;                    the skipped calc leaves the double-buffered draw list empty on
;                    alternating frames (world flickers in and out).

0x10416BF8 = VR_RENDER_SKIP_MASK:
0x10416BF8 = .int 0x00000001 ; default: only the DRC skip

0x03A1480C = drawDRC_body:
0x03A14978 = postDrawDRC_body:

hook_maybeSkipDrawDRC:
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0001
beq run_drawDRC_body
blr
run_drawDRC_body:
stwu r1, -0x30(r1) ; original first instruction of drawDRC_
lis r12, drawDRC_body@ha
addi r12, r12, drawDRC_body@l
mtctr r12
bctr

0x03A14808 = ba hook_maybeSkipDrawDRC

hook_maybeSkipPostDrawDRC:
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0001
beq run_postDrawDRC_body
blr
run_postDrawDRC_body:
stwu r1, -0x28(r1) ; original first instruction of postDrawDRC_
lis r12, postDrawDRC_body@ha
addi r12, r12, postDrawDRC_body@l
mtctr r12
bctr

0x03A14974 = ba hook_maybeSkipPostDrawDRC

; --------------------------------------------------------------------------------
; Right-eye render-preparation skips (bits 1-12 above). Each hook: on the right eye
; pass with its bit set, return immediately, reusing the left eye's results. All the
; hooked functions are void calc methods hooked at their entry instruction; skipping
; happens before any stack setup so a plain blr is safe. cr0/r12/ctr are volatile at
; function entry per the PPC EABI.

; continuation labels = entry + 4
0x03994268 = rskip_calcModel_cont:
0x03994428 = rskip_calcWorld_cont:
0x03994394 = rskip_preSortQueue_cont:
0x039A9A94 = rskip_sceneCalcView_cont:
0x039AAFC8 = rskip_calcViewGPU_cont:
0x039ABC04 = rskip_invokeCalcWorld_cont:
0x0399D548 = rskip_queueSort_cont:
0x039D131C = rskip_fxCalcView_cont:
0x039D5D00 = rskip_lightPrePass_cont:
0x039D6C5C = rskip_giLightProbe_cont:
0x039D6D2C = rskip_giLocalReflection_cont:
0x03A24878 = rskip_updateModelJobQueue_cont:

hook_rskip_calcModel:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_calcModel_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0002
beq rskip_calcModel_run
; telemetry: count skips + append event 7
lis r11, _cCalcModelSkip@ha
lwz r12, _cCalcModelSkip@l(r11)
addi r12, r12, 1
stw r12, _cCalcModelSkip@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x1C00
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
blr
rskip_calcModel_run:
; telemetry: count runs + append event 6
lis r11, _cCalcModelRun@ha
lwz r12, _cCalcModelRun@l(r11)
addi r12, r12, 1
stw r12, _cCalcModelRun@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x1800
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
mflr r0 ; original first instruction
lis r12, rskip_calcModel_cont@ha
addi r12, r12, rskip_calcModel_cont@l
mtctr r12
bctr
; DISABLED: whole-function skip starves the draw list (the staging-list consumer
; sub_3A24A38/UpdateModelJobQueue lives inside calcModel). Replaced by the
; type-filtered ModelMgr::invoke gate below - bit 1 now gates only the type-2
; "ModelScene Calc" job dispatch.
;0x03994264 = ba hook_rskip_calcModel

hook_rskip_calcWorld:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_calcWorld_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0004
beq rskip_calcWorld_run
blr
rskip_calcWorld_run:
mflr r0 ; original first instruction
lis r12, rskip_calcWorld_cont@ha
addi r12, r12, rskip_calcWorld_cont@l
mtctr r12
bctr
; DISABLED: replaced by the type-filtered ModelMgr::invoke gate (bit 2 now gates
; only the type-3 world/effects job dispatch; ModelScene::calcWorld_ is nothing
; but invoke(scene, wm, 3, 7, 0)).
;0x03994424 = ba hook_rskip_calcWorld

hook_rskip_preSortQueue:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_preSortQueue_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0008
beq rskip_preSortQueue_run
blr
rskip_preSortQueue_run:
mflr r0 ; original first instruction
lis r12, rskip_preSortQueue_cont@ha
addi r12, r12, rskip_preSortQueue_cont@l
mtctr r12
bctr
; DISABLED: bit 3 repurposed for the calcFrame_ contextClear gate (freeze-set bisection)
;0x03994390 = ba hook_rskip_preSortQueue

hook_rskip_sceneCalcView:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_sceneCalcView_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0010
beq rskip_sceneCalcView_run
blr
rskip_sceneCalcView_run:
stwu r1, -0x28(r1) ; original first instruction
lis r12, rskip_sceneCalcView_cont@ha
addi r12, r12, rskip_sceneCalcView_cont@l
mtctr r12
bctr
; DISABLED: bit 4 repurposed for the calcFrame_ requestDraw-pass gate (freeze-set bisection)
;0x039A9A90 = ba hook_rskip_sceneCalcView

hook_rskip_calcViewGPU:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_calcViewGPU_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0020
beq rskip_calcViewGPU_run
blr
rskip_calcViewGPU_run:
mflr r0 ; original first instruction
lis r12, rskip_calcViewGPU_cont@ha
addi r12, r12, rskip_calcViewGPU_cont@l
mtctr r12
bctr
0x039AAFC4 = ba hook_rskip_calcViewGPU

hook_rskip_invokeCalcWorld:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_invokeCalcWorld_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0040
beq rskip_invokeCalcWorld_run
blr
rskip_invokeCalcWorld_run:
mflr r0 ; original first instruction
lis r12, rskip_invokeCalcWorld_cont@ha
addi r12, r12, rskip_invokeCalcWorld_cont@l
mtctr r12
bctr
0x039ABC00 = ba hook_rskip_invokeCalcWorld

hook_rskip_queueSort:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_queueSort_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0080
beq rskip_queueSort_run
blr
rskip_queueSort_run:
mflr r0 ; original first instruction
lis r12, rskip_queueSort_cont@ha
addi r12, r12, rskip_queueSort_cont@l
mtctr r12
bctr
0x0399D544 = ba hook_rskip_queueSort

hook_rskip_fxCalcView:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_fxCalcView_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0100
beq rskip_fxCalcView_run
blr
rskip_fxCalcView_run:
mflr r0 ; original first instruction
lis r12, rskip_fxCalcView_cont@ha
addi r12, r12, rskip_fxCalcView_cont@l
mtctr r12
bctr
0x039D1318 = ba hook_rskip_fxCalcView

hook_rskip_lightPrePass:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_lightPrePass_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0200
beq rskip_lightPrePass_run
blr
rskip_lightPrePass_run:
mflr r0 ; original first instruction
lis r12, rskip_lightPrePass_cont@ha
addi r12, r12, rskip_lightPrePass_cont@l
mtctr r12
bctr
0x039D5CFC = ba hook_rskip_lightPrePass

hook_rskip_giLightProbe:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_giLightProbe_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0400
beq rskip_giLightProbe_run
blr
rskip_giLightProbe_run:
mflr r0 ; original first instruction
lis r12, rskip_giLightProbe_cont@ha
addi r12, r12, rskip_giLightProbe_cont@l
mtctr r12
bctr
0x039D6C58 = ba hook_rskip_giLightProbe

hook_rskip_giLocalReflection:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_giLocalReflection_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0800
beq rskip_giLocalReflection_run
blr
rskip_giLocalReflection_run:
mflr r0 ; original first instruction
lis r12, rskip_giLocalReflection_cont@ha
addi r12, r12, rskip_giLocalReflection_cont@l
mtctr r12
bctr
0x039D6D28 = ba hook_rskip_giLocalReflection

hook_rskip_updateModelJobQueue:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_updateModelJobQueue_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x1000
beq rskip_updateModelJobQueue_run
blr
rskip_updateModelJobQueue_run:
stwu r1, -0x50(r1) ; original first instruction
lis r12, rskip_updateModelJobQueue_cont@ha
addi r12, r12, rskip_updateModelJobQueue_cont@l
mtctr r12
bctr
0x03A24874 = ba hook_rskip_updateModelJobQueue

; --------------------------------------------------------------------------------
; SUB-FUNCTION GATING - the working form of the right-eye calc reuse (bits 1 and 2).
; The whole-function skips proved unfixable: the heavy math and the draw-list build
; are interleaved in the same functions, and skipping the build starves/corrupts the
; double-buffered draw list (see the queue-preserve notes below). But ALL the heavy
; parallel calc funnels through one choke point: gsys::ModelMgr::invoke (0x039A9638),
; which enqueues typed jobs into the scene's FixedSizeJQ and pushes them to
; sead::WorkerMgr ("ModelScene Calc"). Job types (from every call site):
;   type 0 x12: ModelScene::calcRenderView_  - per-context render/cull  (MUST RUN)
;   type 1 x4 : calcDrawBackGround_ phase 2                             (MUST RUN)
;   type 2 x1 : sortModelObj/calcModel       - per-model matrices etc.  (bit 1 gates)
;   type 3 x7 : calcWorld_ -> invokeCalcWorld switch: env/fx/decal/light/pfx/occl
;                                                                       (bit 2 gates)
;   type 4 x3 : calcDrawBackGround_ phase 1                             (MUST RUN)
;   type 5 x7 : calcGPU_                                                (MUST RUN)
; Gating here skips ONLY the worker-job execution; the queue build around it
; (sub_3A24A38: staging-list heapSort + UpdateModelJobQueue) runs on BOTH eye
; passes, so requestDraw/DO_DRAW/clearQueue alternation all stay in their normal
; per-pass rhythm and the right pass simply reuses the left pass's job OUTPUT
; (world matrices, effect state) - which is per-model persistent data, not
; per-pass data. No queue-preserve (bit 13) needed with this scheme.

0x039A963C = invokeGate_cont:

hook_invokeGate:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne invokeGate_run
cmpwi r5, 2
beq invokeGate_type2
cmpwi r5, 3
beq invokeGate_type3
b invokeGate_run
invokeGate_type2:
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0002
beq invokeGate_run
; telemetry: count type-2 skips + append event 7
lis r11, _cCalcModelSkip@ha
lwz r12, _cCalcModelSkip@l(r11)
addi r12, r12, 1
stw r12, _cCalcModelSkip@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x1C00
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
blr
invokeGate_type3:
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0004
beq invokeGate_run
; telemetry: append event 9 (type-3 dispatch skipped)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x2400
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
blr
invokeGate_run:
mflr r0 ; original first instruction
lis r12, invokeGate_cont@ha
addi r12, r12, invokeGate_cont@l
mtctr r12
bctr
0x039A9638 = ba hook_invokeGate

; --------------------------------------------------------------------------------
; bit 13: queue-preserve. gsys::ModelScene::clearQueue normally alternates between
; a clear+mark branch and an erase+request-flag-flip branch every frame (the model
; job queue is a double-buffered draw list; the calc pass refills what these consume).
; When the right-eye calc work is skipped, that alternation empties the draw list on
; every other frame. On skip frames, run ONLY the atomic draw-cursor reset
; (gsys::ModelJobQueue::swapDrawList on scene+0x40C0) so the previous frame's draw
; list, per-model request flags (model+125), and branch-alternation state survive.

; --------------------------------------------------------------------------------
; Telemetry counter block (host-readable, see src/utils/ipc_control.cpp). Lives in
; THE CODECAVE - the only memory guaranteed unused by the game. (A first attempt at
; 0x10416C00 clobbered live .data that PhysicsMemSys reads at boot -> NULL-pointer
; crash in initHavokSystems; only the four words 0x10416BF0-BFC inside the unused
; "Unab..." error string are safe, and they're all taken.) The host finds this block
; by scanning the cave region 0x01800000..0x01880000 for the BE magic 'BVRC'.
; PPC hooks increment the counters; the host drains them into BetterVR_state.json.
; The event ring stores one word per event: (eventId << 26) | (frame & 0x03FFFFFF).
; Event IDs: 1 clearQueue-full, 2 clearQueue-reduced, 3 mgrClearQueue-run,
;            4 mgrClearQueue-skip, 5 swapDrawList, 6 calcModel-run, 7 calcModel-skip.
; Counter races between calc job threads lose the odd increment - fine for telemetry.
; HOST OFFSET MAP (from magic): +0x00 magic, +0x04 frame, +0x08 requestDraw,
; +0x0C lastSwapCursor, +0x10 swapCount, +0x14 clearQFull, +0x18 clearQReduced,
; +0x1C mgrRun, +0x20 mgrSkip, +0x24 calcModelRun, +0x28 calcModelSkip,
; +0x2C evtWriteIdx, +0x30 event ring (64 words), +0x130 ABI extension.

_bvrCounters:
.int 0x42565243 ; 'BVRC'
_cFrame:
.int 0
_cRequestDraw:
.int 0
_cLastSwapCursor:
.int 0
_cSwapCount:
.int 0
_cClearQFull:
.int 0
_cClearQReduced:
.int 0
_cMgrRun:
.int 0
_cMgrSkip:
.int 0
_cCalcModelRun:
.int 0
_cCalcModelSkip:
.int 0
_cEvtWriteIdx:
.int 0
_bvrEvtRing:
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0
.int 0

; Versioned host/guest control extension. Keep the legacy counters above at their
; existing offsets so older diagnostic builds fail soft. The host validates this
; second magic, version, and size before trusting any control-state fields.
_bvrAbiMagic:
.int 0x42565232 ; 'BVR2'
_bvrAbiVersion:
.int 2
_bvrAbiSize:
.int 0x00000160
_bvrSnapshotSeq:
.int 0
_bvrActiveMask:
.int 0
_bvrDesiredMask:
.int 0
_bvrMaskEpoch:
.int 0
_bvrActivationFrame:
.int 0
_bvrEyePhase:
.int 2 ; 0=left calc phase, 1=right calc phase, 2=between stereo generations
_bvrFaultFlags:
.int 0
_bvrTelemetryLevel:
.int 1 ; 0=minimal, 1=counters, 2=counters+event ring
_bvrLastControlEvent:
.int 0

; counter hook: gsys::Model::requestDraw call count (hot path - counter only, no event)
0x039859E8 = cnt_requestDraw_cont:

hook_cnt_requestDraw:
lis r11, _cRequestDraw@ha
lwz r12, _cRequestDraw@l(r11)
addi r12, r12, 1
stw r12, _cRequestDraw@l(r11)
lbz r10, 0x7D(r3) ; original first instruction
lis r12, cnt_requestDraw_cont@ha
addi r12, r12, cnt_requestDraw_cont@l
mtctr r12
bctr
0x039859E4 = ba hook_cnt_requestDraw

; counter hook: gsys::ModelJobQueue::swapDrawList - latches the draw-list cursor value
; (queue+196) BEFORE the reset zeroes it, so the host can see whether the right-eye
; frame actually had entries to draw
0x03A2408C = cnt_swapDrawList_cont:

hook_cnt_swapDrawList:
lwz r11, 196(r3)
lis r12, _cLastSwapCursor@ha
stw r11, _cLastSwapCursor@l(r12)
lis r11, _cSwapCount@ha
lwz r12, _cSwapCount@l(r11)
addi r12, r12, 1
stw r12, _cSwapCount@l(r11)
; append event 5
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x1400
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
addi r11, r3, 0xC4 ; original first instruction
lis r12, cnt_swapDrawList_cont@ha
addi r12, r12, cnt_swapDrawList_cont@l
mtctr r12
bctr
0x03A24088 = ba hook_cnt_swapDrawList

0x039A8D58 = rskip_clearQueue_cont:
0x03A24088 = gsys__ModelJobQueue__swapDrawList:

hook_rskip_clearQueue:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_clearQueue_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x2000
beq rskip_clearQueue_run

; reduced body: do NOTHING. Telemetry proved swapDrawList must not run here: its
; cursor is the FILL cursor of a double-buffered list, so swapping on right frames
; exposes an empty half (right calc never refills it) and an odd number of extra
; swaps leaves the halves permanently out of phase - the screen stays white even
; after the mask is restored. Preserving the queue means leaving it COMPLETELY
; untouched; the next left frame's full body runs the normal alternation.

; telemetry: count reduced runs + append event 2
lis r11, _cClearQReduced@ha
lwz r12, _cClearQReduced@l(r11)
addi r12, r12, 1
stw r12, _cClearQReduced@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x0800
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)

li r3, 0
blr

rskip_clearQueue_run:
; telemetry: count full runs + append event 1
lis r11, _cClearQFull@ha
lwz r12, _cClearQFull@l(r11)
addi r12, r12, 1
stw r12, _cClearQFull@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x0400
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
mflr r0 ; original first instruction
lis r12, rskip_clearQueue_cont@ha
addi r12, r12, rskip_clearQueue_cont@l
mtctr r12
bctr
0x039A8D54 = ba hook_rskip_clearQueue

; --------------------------------------------------------------------------------
; bit 13 continued: gsys::ModelScene::calcFrame_ (0x039A8E70) is the per-pass tick
; that resets every ModelRenderContext, re-requests draws, and re-submits the model
; job queue - and ALSO bakes the per-eye camera via getRenderCamera/getRenderProjection.
; On right-eye reuse frames the camera/matrix path must still run (that IS the right
; eye), but the state mutations below must not, or the contexts sit in their
; "cleared" state with nothing rebuilt (world flickers white):
;   0x039A918C bl gsys::Model::requestDraw        - draw request stream (left owns it)
;   0x039A9194 bl sub_3A01B30                     - occlusion query drawer tick
;   0x039A91C4 bl sub_3A241E8                     - model job queue frame prep/reset
;   0x039A9228 bl gsys::ModelRenderContext::clear - per-context reset
;   0x039A958C bl ModelJobQueueSceneUpdater::updateGPU - GPU upload (left already did)
; Each stub either tail-calls the real function (LR still points into calcFrame_) or
; returns immediately on skip frames. r12/ctr/cr0 are volatile at call sites.

0x039859E4 = gsys__Model__requestDraw:
0x03A01B30 = occlusionDrawerTick_3A01B30:
0x03A241E8 = modelJobQueue_framePrep_3A241E8:
0x039BCCA8 = gsys__ModelRenderContext__clear:
0x039A5F78 = gsys__ModelJobQueueSceneUpdater__updateGPU:

; bit 13 continued: gsys::ModelMgr::clearQueue (0x03993634) ALSO flushes the pending
; model-destroy list every frame (under a critical section). On right-eye reuse frames
; the preserved draw list still references those models, so destroying them here causes
; use-after-free crashes (0xC0000005 at load transitions and during actor streaming).
; A FULL skip is wrong though (proven live via the counter telemetry): it also skips
; the per-scene ModelScene::clearQueue calls, so the bit-13 reduced body (the
; swapDrawList draw-cursor reset) never runs, the right pass draws a consumed list,
; and worse - once ModelJobQueue::clear runs with models still DO_DRAW-flagged, the
; flags never get cleared by changeRequestFlag (it only walks queued entries), every
; later requestDraw early-outs, and the screen stays white PERMANENTLY.
; So the reduced body here replicates ONLY the per-scene loop (scene array at
; Mgr+0x20, count at Mgr+0x18, per the 0x03993634 disassembly) and skips the
; destroy flush + free-list bookkeeping, deferring them to the next left frame.

0x03993638 = rskip_mgrClearQueue_cont:

hook_rskip_mgrClearQueue:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne rskip_mgrClearQueue_run
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x2000
beq rskip_mgrClearQueue_run
; telemetry: count reduced runs + append event 4
lis r11, _cMgrSkip@ha
lwz r12, _cMgrSkip@l(r11)
addi r12, r12, 1
stw r12, _cMgrSkip@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x1000
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)

; reduced body: run the per-scene clearQueue loop (each call self-reduces to the
; draw-cursor reset via hook_rskip_clearQueue), skip the destroy flush
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r29, 0x10(r1)
stw r30, 0x08(r1)
stw r31, 0x0C(r1)
mr r29, r3
mr r31, r4
lwz r12, 0x18(r29) ; modelScenesWhenClearing.size
cmpwi r12, 0
beq mgrReduced_done
lwz r30, 0x20(r29) ; modelScenesWhenClearing.ptrs
mgrReduced_loop:
lwz r3, 0(r30)
mr r4, r31
li r5, 1
lis r12, mgr_perScene_clearQueue@ha
addi r12, r12, mgr_perScene_clearQueue@l
mtctr r12
bctrl
lwz r12, 0x18(r29)
rlwinm r12, r12, 2, 0, 29
lwz r11, 0x20(r29)
add r12, r11, r12
addi r30, r30, 4
cmpw r30, r12
blt mgrReduced_loop
mgrReduced_done:
lwz r29, 0x10(r1)
lwz r30, 0x08(r1)
lwz r31, 0x0C(r1)
lwz r0, 0x24(r1)
addi r1, r1, 0x20
mtlr r0
blr

0x039A8D54 = mgr_perScene_clearQueue:

rskip_mgrClearQueue_run:
; telemetry: count runs + append event 3
lis r11, _cMgrRun@ha
lwz r12, _cMgrRun@l(r11)
addi r12, r12, 1
stw r12, _cMgrRun@l(r11)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
rlwinm r12, r12, 2, 24, 29
lis r11, _bvrEvtRing@ha
addi r11, r11, _bvrEvtRing@l
add r12, r12, r11
lis r11, _cFrame@ha
lwz r11, _cFrame@l(r11)
rlwinm r11, r11, 0, 6, 31
oris r11, r11, 0x0C00
stw r11, 0(r12)
lis r11, _cEvtWriteIdx@ha
lwz r12, _cEvtWriteIdx@l(r11)
addi r12, r12, 1
stw r12, _cEvtWriteIdx@l(r11)
stwu r1, -0x30(r1) ; original first instruction
lis r12, rskip_mgrClearQueue_cont@ha
addi r12, r12, rskip_mgrClearQueue_cont@l
mtctr r12
bctr
0x03993634 = ba hook_rskip_mgrClearQueue

hook_cf_skip_requestDraw:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne cf_requestDraw_call
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0010
beq cf_requestDraw_call
blr
cf_requestDraw_call:
lis r12, gsys__Model__requestDraw@ha
addi r12, r12, gsys__Model__requestDraw@l
mtctr r12
bctr
; ENABLED as part of the right-pass freeze (with bit 13): the draw-request walk only
; restages models the left pass already staged; skipping it keeps the staging/DO_DRAW
; protocol a pure left-pass stream.
0x039A918C = bla hook_cf_skip_requestDraw

hook_cf_skip_occlusionTick:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne cf_occlusionTick_call
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x2000
beq cf_occlusionTick_call
blr
cf_occlusionTick_call:
lis r12, occlusionDrawerTick_3A01B30@ha
addi r12, r12, occlusionDrawerTick_3A01B30@l
mtctr r12
bctr
; DISABLED: skipping the occlusion-query drawer tick on the right pass HANGS the
; game frame loop hard (ppc frame counter freezes; no crash). Pending GPU occlusion
; queries are never retired and a later stage waits on them forever. The tick is
; cheap - let it run.
;0x039A9194 = bla hook_cf_skip_occlusionTick

hook_cf_skip_jobQueuePrep:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne cf_jobQueuePrep_call
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x2000
beq cf_jobQueuePrep_call
blr
cf_jobQueuePrep_call:
lis r12, modelJobQueue_framePrep_3A241E8@ha
addi r12, r12, modelJobQueue_framePrep_3A241E8@l
mtctr r12
bctr
; MUST STAY DISABLED: sub_3A241E8 just resets the submission cursor (queue+216) that
; sub_3A24214 post-increments per context. Gating prep while submit runs is the exact
; iteration-2 crash: the cursor runs away past the 548-byte record array capacity
; (0xC0000005 in JIT code). Prep is 11 instructions - there is nothing to save.
;0x039A91C4 = bla hook_cf_skip_jobQueuePrep

hook_cf_skip_contextClear:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne cf_contextClear_call
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x0008
beq cf_contextClear_call
blr
cf_contextClear_call:
lis r12, gsys__ModelRenderContext__clear@ha
addi r12, r12, gsys__ModelRenderContext__clear@l
mtctr r12
bctr
; ENABLED: preserving the per-context draw lists is the whole point of the freeze -
; UpdateModelJobQueue (bit 12) is skipped on the right pass, so nothing would refill
; cleared contexts.
0x039A9228 = bla hook_cf_skip_contextClear

hook_cf_skip_updateGPU:
lis r12, currentEyeSide@ha
lwz r12, currentEyeSide@l(r12)
cmpwi r12, 1
bne cf_updateGPU_call
lis r12, _bvrActiveMask@ha
lwz r12, _bvrActiveMask@l(r12)
andi. r12, r12, 0x2000
beq cf_updateGPU_call
li r3, 0 ; report "no GPU update" so the follow-up vtable upload is skipped too
blr
cf_updateGPU_call:
lis r12, gsys__ModelJobQueueSceneUpdater__updateGPU@ha
addi r12, r12, gsys__ModelJobQueueSceneUpdater__updateGPU@l
mtctr r12
bctr
; MUST STAY DISABLED: updateGPU uploads the per-pass camera/uniforms - the right pass
; needs it to render the preserved lists with the RIGHT eye's matrices (calcFrame_'s
; getRenderCamera/getRenderProjection baking also stays live).
;0x039A958C = bla hook_cf_skip_updateGPU

; --------------------------------------------------------------------------------

hook_shadowDrawMapLeftOnly:
mflr r0
stwu r1, -0x10(r1)
stw r0, 0x14(r1)
stw r3, 0x0C(r1)
stw r4, 0x08(r1)

lis r3, currentEyeSide@ha
lwz r3, currentEyeSide@l(r3)
cmpwi r3, 1
beq exit_ShadowDrawMapLeftOnly

0x03B0ADF0 = depthShadow_drawShadowMap:
lis r3, depthShadow_drawShadowMap@ha
addi r3, r3, depthShadow_drawShadowMap@l
mtctr r3
lwz r3, 0x0C(r1)
bctrl ; bl depthShadow_drawShadowMap

exit_ShadowDrawMapLeftOnly:
lwz r4, 0x08(r1)
lwz r3, 0x0C(r1)
lwz r0, 0x14(r1)
addi r1, r1, 0x10
mtlr r0
blr

0x039E0310 = bla hook_shadowDrawMapLeftOnly
