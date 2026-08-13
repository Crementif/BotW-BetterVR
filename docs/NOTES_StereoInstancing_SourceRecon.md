# Stereo Instancing — Static Source Recon Notes

*2026-08-13. Companion to `PLAN_StereoInstancing.md`. Two deep static sweeps — the BOTW v208 decompilation (`E:\BOTW_Decompile`) and the Cemu fork (`E:\Github\cemu-mcp`) — run before any oracle tooling, answering the parts of plan §6.3 that don't need a live trace and pinning the seams §7 needs. Every file:line and address below was verified against current source/decompile on 2026-08-13. Items the decompile cannot prove are explicitly marked for the live oracle or the debugger.*

*Status note: §B.8 is the pre-oracle source snapshot. The draw trace has since gained PM4-marker eye tagging, pass pairing, bounded per-mode chunk hashes, and the gameplay-gated capture described in plan §6.4. The static seam analysis and gap inventory below remain the design evidence.*

---

## 0. What this changes in the plan (read first)

1. **Plan §6.3 Q1 is answered statically, pending live confirmation: the eye difference lives in ONE shared per-view uniform block.** BOTW bakes view / viewProjection / projection / inverse-view (plus z-planes and previous-frame mirrors) into a single ~2.3 KB, 35-member `agl::UniformBlock` per view index, owned by each render context's `gsys::ShaderContext`. **No per-object MVP is ever baked on the CPU** — models upload world/bone matrices only and the GPU does the product (§A.4). Tier 2 is not just viable; it is the architecture the game was built around.
2. **BOTW has native multi-view machinery, which upgrades Tier 2.** The engine already supports multiple views per context (view 0 = main, view 2 = snapshot camera): a per-view 796-byte CPU struct array and a per-view UBO slot array, both sized by **one init-time integer** (§A.5). The strongest Tier-2 design is therefore **not** host-side block synthesis but: *provision one extra view slot at boot, have the guest bake the right-eye camera into it through the game's own code path* (concat, inverse, env inheritance from view 0, previous-frame mirrors, double-buffering all handled by existing game code), then have Cemu's right issue substitute the block **address**. Bonus: view-dependent billboard shapes (`ShpBlock` indexed by view) would get *correct* per-eye billboarding through the same mechanism — an artifact class pure host-side substitution cannot fix.
3. **§6.3 Q5 census (guest side): streamout is confined to `nw::eft` GPU particles (3 functions), compute is unused (zero `GX2DispatchCompute` call sites), predication is unused; occlusion queries are heavily used but already per-view bitmasked** (`1 << viewIndex` gating + view-offset query buffers — §A.6). The fallback table shrinks and the query strategy has a native seam.
4. **Cemu-side scope shrinkage:** no conditional rendering (dead bool), no MSAA resolve (`VK_RESOLVE_MODE_NONE` hard-coded), no mip-generation path. The non-draw pairing inventory is now **exhaustive**: 12 `IT_HLE_*` packets + special-state 5/8 draws + the surface-copy-with-format-conversion hidden drawcall (§B.9).
5. **Twin-issue trap list (would have cost debugging days):** the pipeline fast path compares a hash that **excludes the FBO** — an eye flip with `isFirst == false` silently reuses the left pipeline against the right render pass (§B.4); `s_vkUniformData` is a shared file-scope scratch; uniform-ring overflow can submit the command buffer **mid-draw**; `m_state.activeFBO` is bound in `draw_beginSequence`, not `draw_execute`.
6. **Trace-expansion worklist corrected:** UBO address/size/content-hash capture for FULL_CBANK shaders and a `bvr_get_uniform_diff` tool **already exist** in the fork. The genuine gaps: sampled textures, non-draw ops, MRT/depth identity + pass grouping, query/streamout state, uniform *values*, host pipeline identity — plus two perf fixes (hashing inside the mutex, full-vector snapshot copies) (§B.8).
7. **Shader-cache versioning is a correctness requirement:** the SPIR-V cache is keyed by *guest shader hash*, not generated GLSL — a `SET_POSITION` change without bumping `RendererShader::GeneratePrecompiledCacheId()` loads stale pre-transform SPIR-V (§B.6).
8. **Three remaining unknowns need the debugger, with exact breakpoint targets listed** (§A.8): `ASM_MTXInverse`'s dropped second argument, the scene-component vtable+44 hook after record build, and the `ModelJobQueueSceneUpdater` vtable+52 upload virtual.

---

## Part A — BOTW guest side (v208 decompilation)

### A.1 Camera bake chain

`gsys::ModelScene::calcFrame_` (`src\seg_03900000\039A8E70_gsys__ModelScene__calcFrame_.c:373-406`) fetches camera + projection from the `agl::lyr::Layer` at `ctx[130]`, then:

```c
updateRenderingMatricesUsingCamera(ctx, 0, &RenderCamera->mtx, (int)RenderProjection, &v49->mtx);
// and when snapshots are enabled:
updateRenderingMatricesUsingCamera(ctx, 2u, &snapshotTextureMgr->cameraMatrices, ..., ...);
```

**The second argument is the view index** — the seam a right eye would use. `updateRenderingMatricesUsingCamera` = **0x0399BCF0**: touches no GPU memory; writes a **796-byte (0x31C) per-view CPU struct** at `ModelRenderContext+12` (array base) indexed by view (count at `+8`):

| Offset | Contents |
|---|---|
| +0 | `Matrix34` view matrix |
| +48 | `Matrix34` inverse view (see §A.8 caveat 1) |
| +96 | `agl::cull::ViewFrustumCulling` (starts with culling-camera `Matrix34`) |
| +192 | `Matrix44` projection |
| +320/+324 | zNear / zFar |
| +328/+332/+340 | three more camera scalars |
| +516 → +688 | 6 frustum planes (source → 16-byte-stride destination) |
| +784/+788/+792 | plane count / zeroed / flag byte read back by `calcFrame_` |

### A.2 Uniform upload — three stages

The `updateGPU` bl-site 0x039A958C is a red herring (`gsys::ModelJobQueueSceneUpdater::updateGPU` 0x039A5F78 is a null-check guarding a virtual — §A.8 caveat 3). The real path:

**Stage A — struct → per-view UBO.** `gsys::ModelRenderContext::calcGPU` (0x0399C16C) loops views calling `sub_3A0ADE8` (**0x03A0ADE8**) — the **only** proj×view concat in the model path, inline paired-single, once per view:

```c
v24 = v23 + 4;                                     // per-view slot (48-byte stride) -> agl::UniformBlock
if ( !a7 ) sub_3A0ADC8(...);                       // copy members 0-3 -> 11-14 (previous-frame mirrors)
agl::UniformBlock::setData_(v24, ..., 0, view,     0, 3);   // mat3x4
agl::UniformBlock::setData_(v24, ..., 2, proj,     0, 4);   // mat4x4
agl::UniformBlock::setData_(v24, ..., 1, viewProj, 0, 4);   // mat4x4 (the ps concat result)
agl::UniformBlock::setData_(v24, ..., 3, invView,  0, 3);   // mat3x4
```

Crucial for VR: **any view index other than 0 inherits environment members from view 0** (`sub_3A83AC4` block-copies members 8-10 and 15-25 from view 0's UBO). A right eye added as view 1 gets the env/light half of the block for free. The UBO is **double-buffered** (`agl::UniformBlock::create(slot+4, heap, 2, 1)`; live index `ub+33`, sub-buffer stride `ub+36`).

**Stage B — block selection per pass.** `gsys::ModelRenderCache::setPassViewProjIndex` (**0x0399A9B8**), called from `gsys::ModelRenderContext::draw` (0x0399BE9C): caches the per-view `UniformBlock` pointer at `renderCache+28` via accessor `sub_3A0B218` (0x03A0B218: `slot = shaderCtx->array[48*viewIndex]; slot[0] |= 1; return slot+4;`) and invalidates the bind cache (`renderCache+180 = -1`).

**Stage C — the GX2 call.** `sub_399AE28` (**0x0399AE28**): binds `(size, dataPtr)` from that cached block via `GX2SetVertexUniformBlock` / `GX2SetPixelUniformBlock` / `GX2SetGeometryUniformBlock`. **The camera block's GX2 slot indices are a 3-byte triple at shaderAssign+152/153/154 (vertex/pixel/geometry; `0xFF` = unbound).** Note: `GX2Set*UniformBlock` are import symbols, absent from functions.csv — grep the decompiled sources (~119 callers of the three variants).

### A.3 Per-view block layout (35 members, declared at `agl::UniformBlock::create_0` = 0x03A0C918, misnamed by IDA)

Members 0-3 and 11-14 are proven; the rest are inferred from arithmetic/sizes.

| # | Decl | Semantic |
|---|---|---|
| 0 | vec4[3] | **view** (mat3x4) |
| 1 | vec4[4] | **viewProjection** |
| 2 | vec4[4] | **projection** |
| 3 | vec4[3] | **inverse view / camera world** |
| 4 | vec4 | `(zNear, zFar, near/far, 1-near/far)` |
| 5 | vec4 | `(1/(far-near), near/(far-near), p59, 1/p59)` |
| 6 | vec4 | `(far-near, 0, 0, 0)` |
| 7 | vec4 | `(p61*p59, p61, p58, 0)` |
| 8-10 | vec4×3 | env — **inherited from view 0** |
| 11-14 | mirror 0-3 | **previous-frame** view/viewProj/proj/invView |
| 15-25 | mixed (see report) | env/light/shadow; 17 = vec4[16] (four mat4x4); 25 = vec4[56] (light/probe array); **last inherited member** |
| 26-34 | scalars + two matrix groups | uncharacterized |

Block size is computed at runtime into `ub[2]` (~2.3 KB estimated; per-type stride table is data at 0x10362A70, not in the decompile — **read `*(ub+8)` live**). A second 35-member block (`ShaderContext::createContextUBO` 0x03A0CC94) is fog/color environment, *not* camera — don't confuse them.

**Camera-block signature for the Cemu-side matcher:** the only bound bank whose bytes are a mat3x4 at +0 followed at +48 by a mat4x4 equal to proj×view. Members 0-3 contiguous (view, viewProj, proj, invView) make this a very strong signature; combine with size and the shaderAssign+152 location triple per plan §8 Tier 2.

### A.4 Per-object baking — there is none (with one exception)

- `sub_3A24214`'s 548-byte submit records contain **pointers, not matrices**: two 26-entry pointer tables into the 796-byte view array (`record+24..127`, `+128..231`), draw-list bucket heads at `+232`, view index at `+0`, view *bitmask* `1<<viewIndex` at `+2`, view array base at `+536/540`. `UpdateModelJobQueue` (0x03A24874), `sub_3A241E8`, and `ModelJobQueue::invoke_` (0x03A24588) contain **no floating-point math at all**.
- **MtxBlock** (`SkeletonObj::CalcMtxBlock` 0x03C0191C): world×inverseBind (smooth) or plain world copy (rigid). No view dimension.
- **ShpBlock** (256 B): indexed `20 * viewIdx * shapeObj->byte9`, where `byte9` = viewDependent, set **only for billboard bones** (`ResBone.flags & 0x70000`, `ShapeObj::Init` 0x03BFA5B0). Ordinary shapes collapse to block 0 → every view binds the identical block. `nw::g3d::ModelObj::CalcView` (0x03BF85A8) early-outs unless billboarded, and even then receives a **world-space** matrix.
- No `ASM_MTXConcat`/`ASM_MTX44Concat` hits in the model draw path (only shadow light-view setup, post-FX, misc).

**Conclusion:** viewProj comes from the shared per-view block; world/bone matrices from per-object blocks; the product happens on the GPU. Billboards are the one per-object view dependency — and they already have a per-view mechanism that a real second view slot would drive correctly.

### A.5 Native multi-view machinery and the provisioning knob

`gsys::ModelRenderContext::initialize` (**0x0399BBBC**): allocates the 0x40-byte `gsys::ShaderContext` (stored at `ctx+424`), calls `ShaderContext::initialize_0(v8, *a2, a2[1], heap)`, and allocates `796 * a2[1]` bytes (align 16) for the view-struct array. **`a2[1]` — one init-arg field — sizes both the CPU view array and the per-view UBO slot array.** To provision a right-eye view: patch that value +1 at boot (find the caller filling the init-args struct — small ASM patch), then per frame call `updateRenderingMatricesUsingCamera(ctx, 1, rightCam...)` and ensure `calcGPU`'s view loop covers slot 1. The engine then produces a complete, correct, double-buffered right-eye camera UBO through its own code.

### A.6 Side-effect census (guest)

| Subsystem | Verdict | Evidence |
|---|---|---|
| Occlusion queries | **heavy use, already per-view** | ~13 callers; gsys wrapper `sub_399A700` (0x0399A700) gates on `(mask >> viewIndex) & 1` and offsets the query buffer by `viewIndex << 6`; callers include `agl::cull::OcclusionQueryDrawer::draw_` (0x03B25B04), `agl::fx::OcclusionRenderer::draw` (0x03B39EE4), the four big dispatchers `sub_39F48DC/39F4C5C/39F506C/39F53DC` |
| Conditional render | via `GX2QueryBeginConditionalRender`/`End` only (thunk 0x036CF080); `GX2SetPredication` has **zero** callers | — and Cemu ignores predication anyway (§B.7) |
| Streamout | **`nw::eft` particles only**: `sub_3B655CC` (SetStreamOutBuffer/Context), `sub_3B65650` (SaveStreamOutContext), `sub_3B592F8` (Enable 1/0) | GPU particle attributes, not skinning |
| Compute | **unused** — zero `GX2DispatchCompute` callers; `GX2SetComputeShader` once in a generic dispatcher never fed one (`sub_3C09CE4`) | — |
| Geometry shaders | present but rare: `sub_3A7A704`, `sub_3973440`, `sub_3C09CE4` (+ ring setup `sub_3A54FB4`) | the SET_POSITION GS/copy-shader variant matters |

### A.7 Uniform banks per material shader

Up to **8** bound banks per BOTW material shader (shaderAssign location triples, `0xFF`=unbound): **+152 = the per-view camera/env block (the target)**, +160 skeleton MtxBlock, +164/+168 ShpBlock (display-list vs direct), +172 env/context (fog/color) UBO, +188 scene material block, +192/196/200 three mask-gated optional blocks (`model+290`). Typical opaque materials bind fewer. The Tier-2 matcher should expect ≤8 banks and key on the §A.3 signature + size, not raw address.

### A.8 Where the decompile stops — debugger targets

1. **`ASM_MTXInverse` (0x03C6FF74)** — IDA shows one arg but it writes a second destination (proven by `agl::env::EnvObjMgr::updateView` 0x03A94484 reading `+48` nothing else wrote). Breakpoint, watch r4 — settles the identity of `viewStruct+48` and block member 3.
2. **Scene-component hook, vtable+44** — `039A8E70_...calcFrame_.c:419`, called per scene component right after the 548-byte record is built, receiving the render context. The one hook that could add view-dependent writes; breakpoint for 100% closure.
3. **The upload virtual, vtable+52** on the `ModelJobQueueSceneUpdater` at `ModelScene+0x46D8` (`calcFrame_.c:423`).
4. (Non-blocking) per-model calc dispatch via pointer-to-member tables at 0x10353EAC/0x10353EB0/0x10353E6C — resolves to the shape/material functions above; check here if runtime shows unexplained per-object writes.
5. Read the real camera-block size from `*(ub+8)` live (stride table 0x10362A70 is data, not decompiled).

---

## Part B — Cemu fork seams (E:\Github\cemu-mcp)

### B.1 Clear path and the magic-payload rewrite point

Guest → packet: `GX2_Blit.cpp:135-160` (`SubmitHLEClear`). **Design constraint: color channels are quantized to 8-bit guest-side** (`(uint32)(c*255.0f)`), so an RGBA clear carries ≤32 bits; **only the depth value crosses as a lossless f32 bit pattern** (`GX2_Blit.cpp:154`). The existing BetterVR eye protocol survives (it only needs distinguishable values), but any *new* payload rides in depth.

Dispatch `LatteCommandProcessor.cpp:1280-1284` → handler `:845-884` (23 words: mask, 8 color-surface words, 8 depth-surface words, values decoded at `:868-875`) → `LatteRenderTarget_itHLEClearColorDepthStencil` (`LatteRenderTarget.cpp:737-825`; color loops **all** matching cached textures via `LatteTC_LookupTextureByData` :775, creates mapping if none :791; depth via `LatteTC_LookupTexturesByPhysAddr` :799 / `LatteMRT_CreateDepthBuffer` :821) → `applyTextureColorClear`/`applyTextureDepthClear` (:703-735; a depth-clear on a color texture re-routes to the color clear with the depth value replicated, :730) → Vulkan `texture_clearColorSlice` (`VulkanRenderer.cpp:3287-3295`, `vkCmdClearColorImage` at :2922) / `texture_clearDepthSlice` (:3297-3336, `vkCmdClearDepthStencilImage` :3333; clears always break the render pass, :3299).

**Best interception: `LatteCommandProcessor.cpp:876-882`** — all packet data sits in plain locals; detect the magic there and pair the sibling clear by re-invoking the handler with rewritten values, or by calling the apply functions directly on the sibling texture.

### B.2 Surface copies

`IT_HLE_COPY_SURFACE_NEW`: dispatch `:1285-1289`, handler `:809-843` — **26 words entirely from the packet, zero registers** (13 src + 13 dst: physAddr, mipAddr, swizzle, format, dims, pitch, slice, dim, tilemode, AA, level). Trivially pairable. Renderer path `LatteSurfaceCopy.cpp:9-87`; **the depth↔color conversion branch is a hidden full drawcall** (`surfaceCopy_copySurfaceWithFormatConversion` → `surfaceCopy_viaDrawcall`, `VulkanSurfaceCopy.cpp:789-822`) with its own render pass/framebuffer/views; the plain branch is `vkCmdCopyImage` (`VulkanRenderer.cpp:3452`).

### B.3 Other image operations — and three absences

Special-state **8** = clear-disguised-as-draw (`VulkanRendererCore.cpp:1371-1376`); special-state **5** = depth→color transfer on the **currently bound MRT** — eye-dependent, must be paired (`:1287-1302`, invoked `:1377-1382`). Scanout copy `IT_HLE_COPY_COLORBUFFER_TO_SCANBUFFER` (`:926-941`) is presentation-side (left/mono only). **Absent from Cemu entirely: MSAA resolve (`VK_RESOLVE_MODE_NONE` hard-coded, `CachedFBOVk.cpp:111,146,155`; no `vkCmdResolveImage`), mip generation, and conditional rendering** — three pairing categories deleted.

### B.4 `draw_execute` anatomy (`VulkanRendererCore.cpp:1362-1588`) for the §7.1 refactor

| # | Lines | Step | Class |
|---|---|---|---|
| 1 | 1364-1368 | `drawSequenceSkip` early-out | once (gate) |
| 2 | 1370-1382 | special-state 8/5 early-outs | non-draw ops in disguise |
| 3 | 1384-1386 | `LatteStreamout_PrepareDrawcall` | **once — side-effecting** |
| 4 | 1388-1400 | `uniformData_updateUniformVars` ×3 stages | **hybrid — run per-eye** (fresh ring offset each call) |
| 5 | 1402-1410 | `LatteIndices_decode` | once (cached on ptr/count/mode/type — second call ~free) |
| 6 | 1412-1434 | bind index buffer (state-cached) | per-issue |
| 7 | 1436-1454 | vertex/UBO sync (`LatteBufferCache_Sync`) | once |
| 8 | 1456-1490 | pipeline lookup/create | **per-issue — see trap** |
| 9 | 1493-1508 | descriptor set lookup/create | per-issue (shared unless sibling texture sampled) |
| 10 | 1510 | `draw_setRenderPass()` | per-issue |
| 11-12 | 1512-1529 | bind pipeline, depth bias, blend constants | per-issue |
| 13 | 1531-1577 | dynamic offsets + `vkCmdBindDescriptorSets` | **per-issue — uniform injection point** |
| 14 | 1579-1583 | `vkCmdDrawIndexed`/`vkCmdDraw` | per-issue |
| 15 | 1585 | `LatteStreamout_FinishDrawcall` | **once — side-effecting** |
| 16 | 1587 | `drawCallCounter++` | once |

Suggested split: steps 1-7 + 15-16 = `PrepareDraw`; steps 8-14 = `IssuePreparedDraw(eye)`, with step 4 duplicated into the per-eye half. Prepared struct: hostIndexType/Count, indexReservation, base vertex/instance/count, three shader pointers.

**Traps:**
- **Pipeline fast path (`:1458-1476`) compares `draw_calculateMinimalGraphicsPipelineHash`, which excludes the FBO**; the full cache key includes the render pass (`:156`, `:286`). An eye flip with `isFirst == false` silently reuses the left pipeline against the right render pass. Per-eye FBO switches must force the full-lookup path.
- `float s_vkUniformData[512*4]` (`:377`) is a file-scope scratch shared across stages and draws; sibling-uniform values must be written there, not into the ring directly.
- `:460-480` can `draw_endRenderPass(); SubmitCommandBuffer();` **mid-draw** on uniform-ring overflow — never cache command-buffer-scoped handles across step 4.
- `m_state.activeFBO` is bound only via `LatteMRT::ApplyCurrentState` from `draw_beginSequence` (`:1344`) — a per-draw eye flip must rebind it itself.
- `LatteRenderTarget_GetCurrentVirtualViewportSize` (`:436`, "always call after `_updateViewport()`") and `LatteMRT::GetCurrentFragCoordScale` (`:443`) are eye-dependent whenever sibling RT effective size differs.
- Mutable `m_state` mid-function: index-buffer fields (1420-22), `activePipelineInfo` (1464/1481), the three DS pointers (1504-06), `descriptorSetsChanged`, `currentPipeline` (1516), `prevPolygonFront*` (1199-1201), `activeRenderpassFBO` (1264/1281), `hasRenderSelfDependency` (1221/1240), `currentVertexBinding` (1608). Statics: `s_syncOnNextDraw` (1212), `s_currentCollisionCheckIndex` (`CachedFBOVk.cpp:193`).

### B.5 Host-only texture routes for the RenderGraphPairRegistry (§7.2)

Precedents: **null textures** (`VulkanRenderer.cpp:1514-1590` — raw VkImage/View/Sampler, never cache-registered) and **surface-copy framebuffers** (`VulkanSurfaceCopy.cpp:450-496` — bare `VKRObjectTextureView*` straight into a `VKRObjectFramebuffer`, bypassing `LatteTextureView` entirely). The TV/DRC output textures are guest-backed (not a precedent); imgui is outside Latte.

**`CachedFBOVk`'s Vulkan half needs only view object + format + hasStencil** (`CachedFBOVk.cpp:19-21, 31, 77-99, 129/170/176`), but the `LatteCachedFBO` base class pins attachments to `LatteTextureView*` (`LatteCachedFBO.h:100-108`) and `calculateEffectiveRenderAreaSize`/`CheckForCollision` dereference `baseTexture`. Two routes:

- **(i) Real `LatteTextureVk` siblings with an `isHostOnly` flag.** Danger audit: the `LatteTexture` ctor unconditionally registers globally (`LatteTexture.cpp:1218` → `sAllTextures`; `LatteTC_RegisterTexture` → `g_allTextures`) and runs graphic-pack rules against `physAddress` (`:1239-1265`); invalidation sweeps (`LatteTextureCache.cpp:403`) and `LatteTexture_CalculateTextureDataHash` (`:41`) dereference `memory_getPointerFromPhysicalOffset` — **a fake phys address means garbage reads or faults** unless every one of those paths honors the flag. This is exactly the plan's "no fake physical addresses without an audit" clause, now with the audit list.
- **(ii) Widen `LatteCachedFBO` slots** to carry `{VKRObjectTextureView*, format, hasStencil, size}` and short-circuit size/collision for host-only attachments — smaller blast radius; `VulkanSurfaceCopy.cpp` proves the Vulkan side accepts bare views.

Either way, `LatteMRT::ApplyCurrentState` keys the FBO cache on raw view **pointer values** (`LatteRenderTarget.cpp:159`), so sibling views get distinct cached FBOs with no key changes. Sibling creation template: `LatteTextureVk` ctor (`LatteTextureVk.cpp:6-112`; memory bound lazily at `:135`); `VKRObjectTextureView(VKRObjectTexture*, VkImageView)` (`VKRBase.h:138`, impl `VulkanRenderer.cpp:3857`).

### B.6 Uniform-var ring, `uf_bvrClipTransform` slot, and cache versions

Vulkan wraps all uniform *variables* in one dynamic-offset UBO: emission `_emitUniformVariables` (`LatteDecompilerEmitGLSLHeader.hpp:5-130`, sequential `uniformCurrentOffset` with 16-byte alignment at `:98`) — **insert the mat4 right after the `uf_windowSpaceToClipSpaceTransform` block (`:58-65`)**, +64 bytes, with matching `offset_/loc_` fields. Runtime slots: `LatteDecompilerShader::uniform` struct (`LatteDecompiler.h:208-223`); decompiler mirror `LatteDecompilerOutputUniformOffsets` (`:236-263`); Vulkan location assignment `LatteShader.cpp:709-728`. Upload: `VulkanRendererCore.cpp:501-505` — ring memcpy + `dynamicOffsetInfo.uniformVarBufferOffset[stage]`, consumed by `draw_prepareDynamicOffsetsForDescriptorSet` (`:519-537`). Per-eye clip transform = two ring writes, two offsets, one descriptor set.

**Version bumps required:** `RendererShader::GeneratePrecompiledCacheId()` (`RendererShader.cpp:5-23`) — **mandatory**: the SPIR-V cache filename keys on guest shader hash, not GLSL text, so stale pre-transform SPIR-V loads otherwise (`RendererShaderVk.cpp:302, 406, 465-468`). Also bump `SHADER_CACHE_GENERIC_EXTRA_VERSION` (`LatteShaderCache.cpp:64`), the transferable `cacheFileVersion` (`:286-292`), and the pipeline-cache `cacheFileVersion` (`:294-299`).

### B.7 Predication: dead code

`IT_SET_PREDICATION` (`LatteCommandProcessor.cpp:1253-1257` → `:644-669`) sets `conditionalRenderActive` (`:642`), which is **read nowhere**. No `vkCmdBeginConditionalRendering` in the tree. Predicated draws already run unconditionally — both eyes over-render identically; nothing to pair.

### B.8 `BVRDrawTrace` — current state vs the §6.1 schema

**Already records** (`BVRDrawTrace.h:21-44`): ordinal, ppcFrame, eyePhase, draw params, index address/type, topology, three shader guest addresses, `CB_COLOR0_BASE`, a 16-register fingerprint (camera-independent by design), and — *correction to the plan's assumption* — **per-binding UBO rows for FULL_CBANK shaders** (`UniformBinding{address, size, hashedSize, stage, bank, contentHash}`, `.h:10-19`, populated `.cpp:105-134`); CFILE/REMAPPED fold into per-stage hashes (`:135-150`). Five MCP tools already registered (`MCPTools.cpp:476-504`, dispatch `:616-620`), **including `bvr_get_uniform_diff` (`:3846`)**.

**Genuine gaps for the oracle:** sampled-texture identities (nothing), non-draw operations (single `RecordDraw` call site — blind to §B.1-B.3 entirely), pass grouping / MRT slots 1-7 / depth-target identity, query/streamout state, viewport/scissor/RT size, uniform **values** (FNV hashes only — deliberate, `.cpp:61-62`), and host pipeline identity (no baseHash/auxHash join key to Vulkan objects).

**Mechanics to fix while extending:** it is not a ring — a plain vector that stops at capacity and drops newest (`:198-202`; default 200k entries); one mutex over the whole `RecordDraw` including FNV hashing of up to 64 KiB per bank per stage **inside the lock** on the CP thread (`:117, :194`); `GetSnapshot` deep-copies both vectors under the lock per MCP query (`:241-242`). Eye tag comes from the guest ABI scan (`:44-57`) with the known async-lag caveat (`MCPTools.cpp:3762`).

### B.9 Exhaustive packet inventory (main dispatch switch, `LatteCP_processCommandBuffer` `:1086`)

| Packet | Dispatch | Handler | Twin-issue policy |
|---|---|---|---|
| `IT_HLE_CLEAR_COLOR_DEPTH_STENCIL` | 1280 | 845-884 | **pair** (+ magic rewrite, §B.1) |
| `IT_HLE_COPY_SURFACE_NEW` | 1285 | 809-843 | **pair when either side has a sibling** |
| `IT_HLE_COPY_COLORBUFFER_TO_SCANBUFFER` | 1258 | 926-941 | once (presentation) |
| `IT_HLE_TRIGGER_SCANBUFFER_SWAP` | 1263 | 896-903 | once |
| `IT_HLE_WAIT_FOR_FLIP` | 1269 | 905-924 | once |
| `IT_HLE_REQUEST_SWAP_BUFFERS` | 1275 | 886-894 | once |
| `IT_HLE_SAMPLE_TIMER` | 1290 | 753-759 | once (guest memory write) |
| `IT_HLE_SPECIAL_STATE` | 1295 | 761-775 | once (drives special-state 5/8 handling) |
| `IT_HLE_BEGIN_OCCLUSION_QUERY` | 1300 | 777-783 | once; right issue must not double counts |
| `IT_HLE_END_OCCLUSION_QUERY` | 1305 | 785-791 | once |
| `IT_HLE_BOTTOM_OF_PIPE_CB` | 1310 | 793-806 | once (timestamp + `GX2NotifyEvent`) |
| `IT_HLE_SYNC_ASYNC_OPERATIONS` | 1315 | **inline** :1315-1320 | once — ⚠ no handler function; hooking by name misses it |
| `IT_DRAW_INDEX_2` / `_AUTO` / `_IMMD` | 1169/1181/1193 | 671-683 / 685-703 / 707-751 | the twin-issue entry points |
| `IT_SET_PREDICATION` | 1253 | 644-669 | inert (§B.7) |

---

## Part C — Mapping onto plan sections

| Plan section | Impact |
|---|---|
| §6.3 Q1-Q3 | Q1 provisionally answered (one shared block, layout §A.3); Q2: vertex+pixel+geometry via the +152/153/154 triple; Q3: FULL_CBANK for the camera block (register/remapped relevance still needs the live trace). The oracle's job narrows to *confirmation* + the §A.8 unknowns. |
| §6.1/6.2 trace expansion | Worklist = §B.8 gaps; skip re-implementing what exists; fix the mutex-hashing and snapshot-copy costs while in there. |
| §7.1 prepared-draw refactor | Use the §B.4 table as the split spec; the pipeline fast-path trap and mid-draw submit are the two review-blocking correctness items. |
| §7.2 RenderGraphPairRegistry | §B.5 gives both routes with the audit list; route (ii) is the smaller blast radius, route (i) needs the enumerated sweeps to honor `isHostOnly`. |
| §7.3 twin-issue | Step-4 duplication per eye; force full pipeline lookup on eye flips; rebind FBO explicitly. |
| §7.4 non-draw pairing | §B.9 is exhaustive; conversion-copy hidden drawcall included; resolve/mipgen/predication rows deleted. |
| §7.5 eligibility | Guest census (§A.6) says: streamout gate fires only around `nw::eft` particles; query gate is real but the engine's own per-view query masking (§A.6) may let the right issue simply *not* participate, matching current behavior. |
| §8 Tier 2 | Upgrade: prefer **guest-baked extra view slot** (§A.5) over host-side field synthesis — game code produces the right-eye block (env inheritance, prev-frame, double-buffering, billboards); Cemu substitutes only the block *address*. Host synthesis stays as the fallback if the view-count patch proves invasive. |
| §8 Tier 1 | `uf_bvrClipTransform` insertion point + the four cache-version bumps (§B.6). |
| §13 step 2 | Unchanged — trace expansion first — but it now starts from a corrected inventory of what exists. |
