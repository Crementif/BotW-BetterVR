[BetterVR_StereoReplay_V208]
moduleMatches = 0x6267BFD0

.origin = codecave

; disable gamepad rendering
0x03A1464C = nop

; external function references
0x03B4F0AC = agl__lyr__RenderDisplay__callDisplayList:

0x399C16C = gsys__ModelRenderContext__calcGPU:
0x399BCF0 = gsys__ModelRenderContext__calcView:
0x03A0B78C = sub_3A0B78C:
0x033F6A08 = aglPfxSky_drawSky_regUniforms:
0x03A59CF4 = sub_3A59CF4_cloudUniforms:

; globals
currentEyeSide:
.int 0
currentFrameCounter:
.int 0
sModelRenderContextPtr:
.int 0
sCaptureDone:
.int 0
halfEyeOffset:
.float 0.9
sFrameCounter:
.int 0

; the visible sky's projection, wraps bl aglPfxSky_drawSky_regUniforms at 0x03403308
; r6 = VFC[view]+0xC0 is the only argument replaced
skyProjForEye:
mflr r0
stwu r1, -0x40(r1)
stw r0, 0x44(r1)
stw r3, 0x10(r1)
stw r4, 0x14(r1)
stw r5, 0x18(r1)
stw r6, 0x2C(r1)
stw r7, 0x1C(r1)
stfs f1, 0x20(r1)
stfs f2, 0x24(r1)
stfs f3, 0x28(r1)
lwz r3, 0x2C(r1)
bl hook_getVisibleSkyProjection
mr r6, r3
lwz r3, 0x10(r1)
lwz r4, 0x14(r1)
lwz r5, 0x18(r1)
lwz r7, 0x1C(r1)
lfs f1, 0x20(r1)
lfs f2, 0x24(r1)
lfs f3, 0x28(r1)
lis r12, aglPfxSky_drawSky_regUniforms@ha
addi r12, r12, aglPfxSky_drawSky_regUniforms@l
mtctr r12
bctrl
stw r3, 0x38(r1)
stw r4, 0x3C(r1)
bl hook_endVisibleSkyDraw
lwz r3, 0x38(r1)
lwz r4, 0x3C(r1)
lwz r0, 0x44(r1)
addi r1, r1, 0x40
mtlr r0
blr

; the same for the cloud layers, wraps bl sub_3A59CF4 at 0x039D2034
; r3,r4 = cloudMgr/ctx, r5 = view34 (VFC+0x00), r6 = VFC[view]+0xC0
cloudProjForEye:
mflr r0
stwu r1, -0x40(r1)
stw r0, 0x44(r1)
stw r3, 0x10(r1)
stw r4, 0x14(r1)
stw r5, 0x18(r1)
stw r6, 0x30(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
stfs f1, 0x24(r1)
stfs f2, 0x28(r1)
lwz r3, 0x30(r1)
bl hook_getStereoProjection
mr r6, r3
lwz r3, 0x10(r1)
lwz r4, 0x14(r1)
lwz r5, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lfs f1, 0x24(r1)
lfs f2, 0x28(r1)
lis r12, sub_3A59CF4_cloudUniforms@ha
addi r12, r12, sub_3A59CF4_cloudUniforms@l
mtctr r12
bctrl
; the cloud uniform blocks now hold the mono transforms, hook_recalcCloudUniforms rebuilds them per eye
stw r3, 0x2C(r1)                         ; preserve sub_3A59CF4's return value
lwz r3, 0x10(r1)                         ; cloud manager
lwz r4, 0x14(r1)                         ; view index
lwz r5, 0x20(r1)                         ; layer base (a6)
lwz r6, 0x18(r1)                         ; VFC[view]+0x00, the mono view34
bl hook_captureCloudUniforms
lwz r3, 0x2C(r1)
lwz r0, 0x44(r1)
addi r1, r1, 0x40
mtlr r0
blr

; capture the first ModelRenderContext that shows up after a replay finishes
; hooked at both 0x039A94BC and 0x039A94EC
; r27 = ModelRenderContext*, r28 = ModelScene*
captureViewFrustumCulling:
lis r12, sCaptureDone@ha
lwz r12, sCaptureDone@l(r12)
cmpwi r12, 0
bne captureSkip

lis r12, sFrameCounter@ha
lwz r11, sFrameCounter@l(r12)
addi r11, r11, 1
stw r11, sFrameCounter@l(r12)

; Update currentFrameCounter = sFrameCounter % 2
clrlwi r11, r11, 31
lis r12, currentFrameCounter@ha
stw r11, currentFrameCounter@l(r12)

lis r12, sModelRenderContextPtr@ha
stw r27, sModelRenderContextPtr@l(r12)
lis r12, sCaptureDone@ha
li r0, 1
stw r0, sCaptureDone@l(r12)

captureSkip:
mr r3, r28
blr




; ================================================================================
; the frame's display list, replayed once for both eyes
; ================================================================================

; called from 0x03AE9D34, returns to 0x03AE9D38
stereoCallDisplayList:
mflr r0
stwu r1, -0x50(r1)
stw r0, 0x54(r1)
stmw r22, 0x28(r1)
stw r3, 0x10(r1)
stw r4, 0x14(r1)
stw r5, 0x18(r1)

; register the shared memory with the module
lis r3, gVRSharedMemory@ha
addi r3, r3, gVRSharedMemory@l
bl import.coreinit.hook_RegisterVRSharedMemory
lis r12, sModelRenderContextPtr@ha
lwz r31, sModelRenderContextPtr@l(r12)    ; r31 = ModelRenderContext*

lis r12, currentEyeSide@ha
li r0, 0
stw r0, currentEyeSide@l(r12)

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_updateVRSharedMemory

; without a VR session the recorded mono frame is what the desktop shows, left untouched
lis r12, sMonoWithoutSession@ha
lwz r0, sMonoWithoutSession@l(r12)
cmpwi r0, 0
bne skipLeftEyeRecalc

cmpwi r31, 0
beq skipRecalcGpuView1
mr r3, r31                               ; r3 = ModelRenderContext*
lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcGpuView
skipRecalcGpuView1:

lwz r3, 0x10(r1)                         ; r3 = RenderDisplay*
lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcBakedRegUniforms

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcReflectionUniforms

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcLightUniforms

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcCloudUniforms

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcParticleUniforms

lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset
fneg f1, f1                              ; f1 = -halfEyeOffset
bl hook_recalcPointLightUniforms

skipLeftEyeRecalc:
mr r3, r31                               ; r3 = ModelRenderContext*
lis r12, halfEyeOffset@ha
lfs f1, halfEyeOffset@l(r12)             ; f1 = halfEyeOffset, the right eye's values for the stereo scope
bl hook_beginStereoIssue
bl gpuWriteFlush                         ; the left eye's writes, ahead of its replay

lwz r3, 0x10(r1)
lwz r4, 0x14(r1)
lwz r5, 0x18(r1)
lis r12, agl__lyr__RenderDisplay__callDisplayList@ha
addi r12, r12, agl__lyr__RenderDisplay__callDisplayList@l
mtctr r12
bctrl
bl hook_endStereoIssue

lwz r3, 0x10(r1)                         ; r3 = RenderDisplay*
cmpwi r3, 0
beq skipClear2DAfterLeftDisplayList
lwz r3, 0x8(r3)                           ; r3 = RenderDisplay framebuffer/render buffer
bl capture2DFromRenderBuffer
skipClear2DAfterLeftDisplayList:

; loading screens drain the replay before the next frame records
bl import.coreinit.hook_IsLoadingScreenVisible
cmpwi r3, 0
beq skipLoadingDrain
bl import.gx2.GX2DrawDone
skipLoadingDrain:

; the record phase reads the right side, as it always has
lis r12, currentEyeSide@ha
li r0, 1
stw r0, currentEyeSide@l(r12)

bl hook_resetParticleBufs                ; clear captured particle list before next frame's record
bl hook_resetShadowEntries               ; clear captured shadow-matrix list before next frame's record
bl hook_resetBakedRegEntries             ; clear captured baked-register list before next frame's record
bl hook_resetReflectionEntries           ; clear captured reflection-plane list before next frame's record
bl hook_resetEnvViewEntries              ; clear captured env view->world list before next frame's record
bl hook_resetPointLightUBOs              ; clear captured per-light UBO list before next frame's record
bl gpuWriteFlush                         ; the mono restores hook_resetPointLightUBOs queued

lis r12, sCaptureDone@ha
li r0, 0
stw r0, sCaptureDone@l(r12)

lmw r22, 0x28(r1)
lwz r0, 0x54(r1)
mtlr r0
addi r1, r1, 0x50
blr

; ================================================================================
; particles: wraps bl sub_3B684D8 at 0x3B66F38 (r3 = handle {VA,size}, r4 = 1), before it swaps the block to little-endian
; ================================================================================
0x3B684D8 = game_sub_3B684D8:

capture_particleFinalize:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
bl hook_captureParticleBuf               ; hook_captureParticleBuf(r3 = handle)
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lis r12, game_sub_3B684D8@ha
addi r12, r12, game_sub_3B684D8@l
mtctr r12
bctrl                                    ; sub_3B684D8(handle, 1): byte-swap + flush
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

; Star camera: "bl agl::UniformBlock::setData_" at 0x38C4E70, r4=buf, r6=&src (mat44 BE), r8=4.
capture_starCamera:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r3, r4
mr r4, r6
bl hook_captureStarCamera
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; shadows: the two writers of the view-space->shadowUV cascade matrices, see updateShadows.cpp
; ================================================================================

; A) slot 17 "DepthShadowProj[cascade]", the view-space branch's bl 0x3A0AA90 at 0x39DF234
; r3=ShaderContext, r4=cascade (0..3), r5=&source mat4
0x3A0AA90 = game_setDepthShadowProj17:

capture_depthShadowProj:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
bl hook_captureDepthShadowProj           ; (r3=shaderCtx, r4=cascade, r5=src)
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lis r12, game_setDepthShadowProj17@ha
addi r12, r12, game_setDepthShadowProj17@l
mtctr r12
bctrl                                    ; original slot-17 setter
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

; B) ShadowPrePass slot 0 (3 mat4s), bl setData_ at 0x3B1C1CC
; r3=UB, r4=buf, r5=0, r6=&src, r7=0, r8=12
0x3A83F6C = game_aglUBSetData:

capture_prePassShadow:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r4, r6                                ; hook args: r3=UB, r4=src
bl hook_capturePrePassShadow
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl                                    ; original setData_(UB, buf, 0, src, 0, 12)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; env block view->world rows: the blob upload bl sub_3A84008 at 0x3A0C8A0, see updateEnvUniforms.cpp
; r3=env UniformBlock, r4=buffer, r5=34 (the blob member), r6=blob, r9=blob size
; ================================================================================
0x3A84008 = game_aglUBSetDataBulk:

capture_envViewRows:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
stw r9, 0x24(r1)
bl hook_captureEnvViewRows               ; (r3=UB, r4=buf, r5=slot, r6=blob)
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lwz r9, 0x24(r1)
lis r12, game_aglUBSetDataBulk@ha
addi r12, r12, game_aglUBSetDataBulk@l
mtctr r12
bctrl                                    ; original bulk upload of the blob
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; water reflections: bl setData_ at 0x3A6A0DC, r3=UB (base+0xED4), r5=3, r6=&plane point
; ================================================================================
capture_reflectionPlane:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r4, r6                                ; hook args: r3=UB, r4=src
bl hook_captureReflectionPlane
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl                                    ; original setData_(UB, buf, 3, src, 0, 1)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; point lights: bl setData_ in PointLightMgr::updateUBO (0x38B6948), see updateLightUniforms.cpp
; r3=UniformBlock (light+0x50), r4=buf, r5=0, r6=&staging (7 rows), r7=0, r8=7
; ================================================================================

capture_pointLightUBO:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r5, r4                                ; hook args: r3=UB, r4=staging, r5=live buf reg
mr r4, r6
lwz r6, 4(r3)                            ; r6 = *(block+4), read here in asm
bl hook_capturePointLightUBO
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl                                    ; original setData_(UB, buf, 0, staging, 0, 7)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

capture_spotLightUBO:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r5, r29
addi r6, r1, 0x30
bl hook_captureSpotLightUBO
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

capture_spotLightTex:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r4, r6
bl hook_captureSpotLightTex
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; capsule lights: the last setData_ in updateUBForLight2 (0x38B7A28)
; r3=UniformBlock (mgrEntry+80), r4=buf, r5=2, r6=&member 2 staging, r7=0, r8=4
; ================================================================================

capture_capsuleLightUBO:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r5, r4                                ; hook arg3: the game's live buf register
mr r4, r6                                ; hook args: r3=UB, r4=member2 staging
bl hook_captureCapsuleLightUBO
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl                                    ; original setData_(UB, buf, 2, mvp2, 0, 4)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; malice volumes: both member-0 setData_ calls (0x38AF5F4 scaled, 0x38AF6A0 unit)
; r3=UniformBlock (proxy+4), r4=buf, r5=0, r6=&mvp rows, r7=0, r8=4
; ================================================================================

capture_lightProxyMVP:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
stw r6, 0x18(r1)
stw r7, 0x1C(r1)
stw r8, 0x20(r1)
mr r5, r4                                ; hook args: r3=UB, r4=staging, r5=live buf half
mr r4, r6                                ;   (buf = the *(block+4) sub_38AEED8 just recomputed)
addi r6, r6, 0xA0                       ; inverse(V*M) staging, original caller stack +0x1BC
bl hook_captureLightProxyMVP
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lwz r6, 0x18(r1)
lwz r7, 0x1C(r1)
lwz r8, 0x20(r1)
lis r12, game_aglUBSetData@ha
addi r12, r12, game_aglUBSetData@l
mtctr r12
bctrl                                    ; original setData_(UB, buf, 0, mvp, 0, 4)
lwz r0, 0x34(r1)
mtlr r0
addi r1, r1, 0x30
blr

; ================================================================================
; malice stamp binds in sub_38AFB7C: VS (0x38AFCE8) and PS (0x38AFCCC), r3=slot, r4=size, r5=buffer
; ================================================================================

capture_stampBindMVP_vs:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
mr r3, r5                                ; hook arg: bound MVP buffer
bl hook_captureStampBind
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
bl import.gx2.GX2SetVertexUniformBlock
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

capture_stampBindMVP_ps:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
mr r3, r5                                ; hook arg: bound MVP buffer
bl hook_captureStampBind
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
bl import.gx2.GX2SetPixelUniformBlock
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

; ================================================================================
; agl::UniformBlock cleanup (0x3A836F0), r3 = the dying block
; blr, not bctr: Cemu's boundary tracker would walk on past the hook and crash the recompiler
; ================================================================================

0x3A836F4 = game_aglUBCleanupBody:

notify_uniformBlockDestroy:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x10(r1)
bl hook_uniformBlockDestroyed
lwz r3, 0x10(r1)
lwz r0, 0x24(r1)                         ; r0 = caller's LR, what the displaced mflr produced
addi r1, r1, 0x20
lis r12, game_aglUBCleanupBody@ha
addi r12, r12, game_aglUBCleanupBody@l
mtlr r12
blr                                      ; resume at 0x3A836F4; ends the tracker's scan
.int 0x00000000                          ; scan fence: invalid opcode stops any stray walk


0x3A22ED4 = game_cubeMapIlluminantFace:
0x39DFEA0 = game_drawDepthShadow:

; The shadow cascades are rendered from the mono record camera and both eyes sample the same ones, so their casters
; draw once in a mono section. Hook points: 0x039A9E34 and 0x039A9F54 in drawOpa_drawOpaque, both bl drawDepthShadow.
mono_drawDepthShadow:
mflr r0
stwu r1, -0x20(r1)
stw r0, 0x24(r1)
stw r3, 0x0C(r1)
stw r4, 0x10(r1)
stw r5, 0x14(r1)
li r3, 1
bl gpuWriteMonoSection
lwz r3, 0x0C(r1)
lwz r4, 0x10(r1)
lwz r5, 0x14(r1)
lis r12, game_drawDepthShadow@ha
addi r12, r12, game_drawDepthShadow@l
mtctr r12
bctrl
stw r3, 0x18(r1)
li r3, 0
bl gpuWriteMonoSection
lwz r3, 0x18(r1)
lwz r0, 0x24(r1)
mtlr r0
addi r1, r1, 0x20
blr

; one cube face of the environment illuminant capture, drawn from the record camera once for both eyes
; hook point: 0x03A23238, original instruction: bl sub_3A22ED4 (r3..r7)
mono_cubeMapFace:
mflr r0
stwu r1, -0x30(r1)
stw r0, 0x34(r1)
stw r3, 0x10(r1)
stw r4, 0x14(r1)
stw r5, 0x18(r1)
stw r6, 0x1C(r1)
stw r7, 0x20(r1)
li r3, 1
bl gpuWriteMonoSection
lwz r3, 0x10(r1)
lwz r4, 0x14(r1)
lwz r5, 0x18(r1)
lwz r6, 0x1C(r1)
lwz r7, 0x20(r1)
lis r12, game_cubeMapIlluminantFace@ha
addi r12, r12, game_cubeMapIlluminantFace@l
mtctr r12
bctrl
stw r3, 0x28(r1)
stw r4, 0x2C(r1)
li r3, 0
bl gpuWriteMonoSection
lwz r3, 0x28(r1)
lwz r4, 0x2C(r1)
lwz r0, 0x34(r1)
addi r1, r1, 0x30
mtlr r0
blr

; Patches
0x03AE9D34 = bla stereoCallDisplayList
0x039A94BC = bla captureViewFrustumCulling
0x039A94EC = bla captureViewFrustumCulling

; calcFrame_ hands calcView the raw layer->renderCamera as the cull camera when bit 11 of ModelScene+0x46E8 is set,
; and the sky and clouds read that one. Always take the getRenderCamera branch.
0x039A94D0 = calcFrame_cullFromRenderCamera:
0x039A9490 = b calcFrame_cullFromRenderCamera

; the sky and cloud projection, see hook_getStereoProjection in updateUniforms.cpp
0x03403308 = bla skyProjForEye
0x039D2034 = bla cloudProjForEye
0x3B66F38 = bla capture_particleFinalize
0x38C4E70 = bla capture_starCamera
0x39DF234 = bla capture_depthShadowProj
0x3B1C1CC = bla capture_prePassShadow
0x3A6A0DC = bla capture_reflectionPlane
; the only aglUB_setDataBulk_ call that uploads the env blob (slot 34)
0x3A0C8A0 = bla capture_envViewRows
0x38B6948 = bla capture_pointLightUBO
; sub_38BB194, the glow point-light variant, has the same updateUBO as PointLightMgr
0x38BB69C = bla capture_pointLightUBO
0x38B7A28 = bla capture_capsuleLightUBO
0x38B9310 = bla capture_capsuleLightUBO
0x3AFA3D0 = bla capture_spotLightUBO
0x3AFA4E4 = bla capture_spotLightTex
; sub_38AEED8 writes member 0 from either the scaled or the unit branch (proxy flag bit 2)
0x38AF5F4 = bla capture_lightProxyMVP
0x38AF6A0 = bla capture_lightProxyMVP
; the stamp draw's binds, which name the MVP buffer half each stamp uses this frame
0x38AFCCC = bla capture_stampBindMVP_ps
0x38AFCE8 = bla capture_stampBindMVP_vs
; agl::UniformBlock cleanup entry
0x3A836F0 = ba notify_uniformBlockDestroy

0x03A23238 = bla mono_cubeMapFace
0x039A9E34 = bla mono_drawDepthShadow
0x039A9F54 = bla mono_drawDepthShadow

; =================================================================================
; agl::UniformLocation::setUniformNVN (0x3A788F8), every GX2Set*UniformReg write goes through it
; =================================================================================
custom_agl_UniformLocation_setUniformNVN:
mflr r0
mr r7, r0                                ; callerLR -> arg5
b hook_setUniformNVNFull

0x03A788F8 = ba custom_agl_UniformLocation_setUniformNVN
