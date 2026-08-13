# BotW BetterVR — Full Stereo Instancing Plan

*Prepared 2026-08-13. This is the implementation plan for eliminating BotW's second emulated render pass while retaining real binocular geometry. It supersedes the remaining guest-side queue-reuse experiments in `HANDOFF_RightEyeReuse.md` unless the Cemu-fork approach proves infeasible.*

## 1. Definition of done

The project is complete only when all of these are simultaneously true:

1. BotW performs one emulated `calcDraw` generation per displayed VR frame. The second eye does not rerun actor jobs, `ModelMgr::calcModel`, `UpdateModelJobQueue`, culling, model sorting, draw-list construction, or GX2 command translation.
2. Cemu consumes the resulting resolved draw state once and issues it for two eyes. Eye-invariant work (model transforms, visibility, materials, vertex/index decoding, pipeline selection, texture discovery) is shared.
3. Each eye renders into an independent target chain with its own view/projection state. The right image is not a mirror and is not depth-reprojected from the completed left image.
4. The final submitted OpenXR eye textures are distinct, show the same parallax direction and approximate disparity as the current two-pass reference, and remain free of world/UI flicker.
5. The mode fails closed. Stock Cemu, unsupported render paths, load/cutscene transitions, stale ABI data, or a fork error must select the known two-pass renderer rather than silently submit a mono image.
6. Stable gameplay reaches about 90 FPS on the current simulator rig with the capture/OBS layers disabled, while the benchmark remains gated until gameplay.

The current measured control is **48.4 FPS / 15.77 ms work** in true two-pass stereo. The one-game-pass comparison measured **94–102 FPS / 7.28–7.45 ms work** before later falling to 87 FPS, but it used image-space synthesis and reported frozen-eye anomalies. It proves the CPU budget; it is not the solution. Evidence is under `bench_out/actual_reuse_20260813/`.

## 2. Terminology and scope

There are two related deliverables:

- **Record once, issue twice:** one emulated game/GX2 draw is resolved once, then Cemu emits a left and right Vulkan issue using that resolved state. This is the minimum architecture that fulfills the CPU-reuse goal. GPU geometry cost remains close to the current two-pass renderer.
- **Vulkan multiview:** one Vulkan draw renders both array layers using `gl_ViewIndex`. This also removes the native second draw issue and render-pass switching. It is the final stereo-instancing optimization, but it requires array-aware render targets, sampled intermediate textures, and shader-decompiler work. It comes after record-once/issue-twice is correct.

The earlier `origin/LookingIntoSinglePass` commit `b5e3dca` does neither at the game-render seam. It instances only the final D3D12 fullscreen presentation of two images the game already rendered. Its array-swapchain/RTV code may be harvested later, but merging that branch does not reduce BotW or Cemu CPU work.

## 3. Why the implementation belongs in Cemu

BetterVR's Vulkan layer currently sees image creation/destruction, magic clears, queue submission, and presentation. It does not own Cemu's decoded draw state. Intercepting every raw Vulkan command in the layer would require reconstructing Cemu's pipeline, descriptor, resource-lifetime, and render-graph state from below.

The correct seam is the Cemu fork (`E:\Github\cemu-mcp`):

- `DrawPassContext::executeDraw` in `LatteCommandProcessor.cpp` sees every guest draw after command parsing.
- `VulkanRenderer::draw_execute` owns index decoding, vertex/uniform buffer synchronization, render-target selection, descriptor binding, pipeline selection, and the final `vkCmdDraw*`.
- `LatteBufferCache_syncGPUUniformBuffers` sees guest UBO addresses and sizes.
- `LatteMRT` resolves guest attachments to `LatteTextureView` objects.
- The existing `BVRDrawTrace` already tags draws using the BetterVR guest ABI.

The production implementation should be a localized Cemu feature behind `--bvr-stereo` plus a runtime ABI handshake. It must be dormant for every other title and for BotW without the BetterVR graphic pack.

### Source ownership map

| Concern | Primary seam |
|---|---|
| guest frame topology and generation latch | `resources/BreathOfTheWild_BetterVR/patch_RND_StereoRendering.asm` |
| guest ABI/counters | `patch_RND_StereoRendering_Optimizations.asm`, `src/utils/ipc_control.cpp` |
| per-eye matrix production | `src/hooking/camera.cpp` |
| magic final-image capture | `patch_RND_Find3DFrameBuffer.asm`, `src/hooking/framebuffer.cpp` |
| final-eye telemetry and dumps | `src/rendering/telemetry.cpp`, `src/rendering/renderer.cpp` |
| per-draw guest command seam | `cemu-mcp/src/Cafe/HW/Latte/Core/LatteCommandProcessor.cpp` |
| existing draw/eye oracle | `cemu-mcp/src/Cafe/HW/Latte/Core/BVRDrawTrace.cpp` |
| UBO address/size discovery | `cemu-mcp/src/Cafe/HW/Latte/Core/LatteBufferData.cpp` |
| resolved dynamic offsets and Vulkan issue | `cemu-mcp/src/Cafe/HW/Latte/Renderer/Vulkan/VulkanRenderer.cpp`, `VulkanRendererCore.cpp` |
| attachment/view/FBO mapping | `cemu-mcp/src/Cafe/HW/Latte/Core/LatteRenderTarget.cpp`, `LatteTexture*.cpp` |
| clip-transform injection | `cemu-mcp/src/Cafe/HW/Latte/LegacyShaderDecompiler/LatteDecompilerEmitGLSLHeader.hpp` |
| debugger automation | `cemu-mcp/src/Cafe/HW/Espresso/Debugger/MCPTools.cpp` |

### Existing prerequisites worth preserving

The earlier recon correctly identified that several prerequisites are already present:

| Prerequisite | Current evidence |
|---|---|
| one-game-pass frame loop | the mono tail is implemented and has run thousands of in-game frames at roughly the target CPU cost |
| right-eye visibility coverage | `patch_FixVisibilityChecks.asm` builds a union-of-both-frustums visibility result, so the one guest draw list should already contain edge geometry needed by either eye |
| eye-invariant shadows | the current stereo optimization path already renders selected shadow work once |
| one-pass HUD | the HUD has a mono capture route and is submitted as a separate OpenXR quad |
| exact per-eye camera inputs | BetterVR already computes both view/projection pairs for its camera hooks and synthesis diagnostics |
| guest/fork discovery channel | the `BVRC`/`BVR2` codecave block is already discovered by `BVRDrawTrace` |
| final-eye tagging | the graphic pack emits magic `GX2ClearBuffersEx` values and the layer decodes eye/capture/frame-slot information from the translated clear |

These reduce scope, but every item still receives a Phase-0 runtime assertion. In particular, union-frustum culling must be proven on moving/head-rotated edge cases rather than assumed from the patch's intent.

Two Cemu implementation details from the earlier recon are especially valuable:

- Guest uniform blocks are represented as dynamic UBO offsets. Changing a proven camera block can therefore be a dynamic-offset substitution with no descriptor rewrite. This is the preferred Tier-2 camera path.
- `LatteMRT::ApplyCurrentState()` keys framebuffer identity from resolved `LatteTextureView` pointers, while compatible render-pass formats participate in pipeline compatibility. Proper sibling views should naturally obtain separate cached framebuffers without recompiling shaders. This is an optimization hypothesis to verify with cache-hit counters, not a reason to bypass lifetime safety.

Fresh static recon adds a more important guest-side opportunity. BotW already allocates camera state and camera UBOs as arrays indexed by view: normal rendering populates view 0, while the snapshot camera uses view 2. `gsys::ModelRenderContext::initialize` uses the same init-time view count for both the 796-byte CPU view records and `gsys::ShaderContext`'s per-view UBO slots. `calcGPU` then writes view, view-projection, projection, inverse-view, z-plane, environment, and previous-frame members into the selected block. Ordinary object blocks remain world/bone data; billboard shape data is the notable per-view exception.

That makes a **guest-baked view 1** the preferred full-fidelity camera source: provision one additional view slot at boot, let BotW populate it from BetterVR's right-eye camera without running a second draw pass, and make Cemu select that block for the right issue. It preserves BotW's own matrix conventions, environment inheritance, history, double buffering, and billboard handling. This is still gated on live validation of the init caller, the inverse-matrix destination, two post-record virtual hooks, and the actual block size. If any of those make the view-count patch unsafe, host-side block synthesis remains the fallback.

## 4. Target architecture

```text
BotW PPC (one frame generation)
  simulation once
  union-frustum culling once
  calcDraw / GX2 command production once
  bake left + right per-view camera blocks once (no second draw pass)
                     |
                     v
Cemu Latte command processor
  parse guest packets once
  resolve each draw once
  decode indices and sync buffers once
                     |
            PreparedDraw + RenderGraphPairRegistry
                 /                         \
       issue eye 0                    issue eye 1
       left targets                   sibling targets
       identity clip transform        right clip transform
       original/shared inputs         right sibling inputs
                 \                         /
                     v
BetterVR magic clears identify left/right final images
                     |
                     v
Existing Vulkan->D3D12/OpenXR capture and UI quad composition
```

The first correct implementation twin-issues resolved draws immediately. It does **not** rewind and re-parse arbitrary PM4 buffers. Re-parsing nested indirect buffers would repeat register writes and guest-visible side effects, and guest memory could change between parses.

Once correctness is established, render-pass batching and then Vulkan multiview remove the extra target switching/native issue overhead.

### Pass-level PM4 replay as a later optimization

The other plan's rewind-and-replay design remains useful if immediate twin issue spends too much time ending and reopening left/right render passes. It is promoted only after the resolved-draw path is correct and only if profiling shows target switching is material.

A replay implementation must first provide:

- a pass capture that represents nested indirect buffers without assuming one contiguous PM4 span;
- an allowlist of idempotent register/draw packets and explicit suppression for memory writes, timers, query callbacks, streamout offset updates, swaps, and guest notifications;
- memoized resolved index/buffer offsets so replay never rereads mutable guest data;
- a bounded replay ring with overflow fallback;
- identical-image and side-effect-counter tests against immediate twin issue.

If those conditions are met, replaying the complete left pass onto right siblings can preserve Cemu's framebuffer/pipeline cache locality. If they are not, immediate twin issue remains the shipping path.

## 5. Control plane and fail-safe handshake

Extend the current versioned guest block rather than adding an unrelated IPC path. Use a new ABI version/size and the existing odd/even seqlock.

### Layer/mod to fork

- requested mode: `off`, `trace`, `twin_issue`, or `multiview`
- session nonce and requested generation
- left and right view matrices
- left and right projection/device matrices
- full left-clip-to-right-clip transform: `P_right * V_right * inverse(P_left * V_left)`
- frame slot and eye/capture-role encoding
- gameplay/fade/cutscene safety state

Matrices must have an explicitly documented byte order and row/column convention. A unit test must round-trip a known asymmetric projection through guest memory and compare C++ and Cemu results.

### Fork to layer/mod

- fork capability magic, implementation version, and feature bits
- acknowledged session nonce/generation
- current state: `idle`, `armed`, `active`, `fallback`, `faulted`
- completed frame/generation
- left issues, right issues, clears, copies, and skipped operations
- target-pair and UBO-substitution counts
- fallback reason and sticky fault flags
- moving host timing for trace, resolve, left issue, right issue, and target switching

### State machine

```text
OFF -> REQUESTED -> FORK_READY -> ACTIVE
 ^        |             |           |
 |        +--timeout----+--fault-----+
 +-------------DRAINING/FALLBACK-----+
```

BetterVR may enter the mono guest loop only after a fresh fork acknowledgement for the same session and generation. A missing/stale acknowledgement keeps two-pass stereo. Mode changes latch at the existing stereo-generation boundary. A fault completes or discards the current generation, drains host work, and returns to two-pass without leaving queue or capture state half-switched.

## 6. Reverse-engineering phase: measure before selecting the camera path

The current true two-pass renderer is the oracle. Extend `BVRDrawTrace` before implementing replay. Static source recon (2026-08-13) already answers parts of §6.3 and pins the §7 seams — see `NOTES_StereoInstancing_SourceRecon.md`; the trace phase confirms those findings live rather than discovering them.

### 6.1 Trace schema

For every draw, clear, surface copy, special-state image operation, query boundary, and streamout operation, record:

- PPC frame, eye phase, command-buffer nesting depth, pass ID, and ordinal
- shaders, topology, index/instance parameters, special-state flags
- color/depth attachment identities, format, size, mip, slice, and physical range
- all sampled texture identities and subresources
- UBO stage/bank, guest address, byte size, content hash, and uniform mode
- uniform-register/remapped-data hash
- shader-specific 16-byte uniform chunk hashes so field offsets can be ranked without retaining bulk guest data
- query/conditional-render/streamout state
- whether the operation writes guest-visible memory

Also keep assertion counters for compute dispatch, conditional rendering, MSAA resolve, and mip generation. Static recon found no active BotW/Cemu path for these categories, so a nonzero counter is a compatibility change that must fail closed rather than silently escape the inventory.

Keep large byte dumps opt-in. The normal trace stores hashes and bounded samples so a several-second capture remains usable by an LLM.

### 6.2 New debugger/MCP outputs

- `bvr_trace_render_graph`: bounded JSON/CSV render graph grouped by pass and eye
- `bvr_get_uniform_diff`: matches left/right draws and ranks differing UBO ranges/fields
- `bvr_dump_uniform_pair`: dumps one matched block pair plus known view/projection matrices
- `bvr_get_operation_diff`: unmatched clears/copies/queries/dispatches between eyes
- `bvr_get_instancing_stats`: live replay/fallback/timing counters

All draw tools must use command-stream-ordered eye identity. The live `BVR2.eyePhase` field is useful for host status, but it is not a valid Latte draw tag: the GPU thread can consume both submitted eyes after PPC has already advanced that mutable field. The implemented oracle therefore closes each pass with BetterVR's existing magic 3D `GX2ClearBuffersEx` marker and retroactively labels only the draws preceding that marker. A trace prefix/suffix without a marker remains unknown and is rejected above a bounded threshold.

### 6.3 Questions this phase must answer

1. Does the live camera-block signature match the statically recovered per-view block, and what is its exact runtime size?
2. Which shader stages and shader-assignment slots consume that block or values remapped from it?
3. Which shaders can switch a full constant-bank dynamic offset, and which require remapped/register staging from the right-eye source block?
4. What non-draw image operations are needed to preserve each eye's post-processing chain, including special-state draws and format-conversion copies?
5. How often do particle streamout, per-view occlusion queries, and Cemu special states occur in normal gameplay and transitions, and do the categories found statically absent stay absent?
6. What is the maximum draw/pass/target count per frame, and what native overhead budget remains below 11.11 ms?

**Gate:** do not implement camera UBO synthesis until matched two-pass data proves the block identity and field offsets. The geometry-only clip-transform path can proceed independently.

### 6.4 First measured oracle result (2026-08-13)

The first end-to-end, gameplay-gated oracle is implemented in `tools/capture_stereo_oracle.py` and Cemu branch `codex/bvr-stereo-instancing`. It launches the isolated debugger fork through BetterVR, waits for 90 advancing stable gameplay frames with both final eyes `NORMAL`, arms a bounded trace, and fails closed on overflow, missing PM4 markers, no matched draws, stale/non-gameplay state, or identical final OpenXR eyes.

The accepted `v4_chunk_offsets` corpus captured 123,132 draws, 303,021 uniform bindings, and 7,393,722 16-byte chunk hashes across 16 complete left/right PM4 pass pairs. It had zero dropped draws, zero truncated uniform entries, 20 boundary-unknown draws, 45,413 matched draw pairs, distinct final eyes (`meanAbsDifference=0.0469`), and no asymmetric-flicker or frozen-eye event in the accepted window. The local evidence packet is `bench_out/stereo_oracle_20260813_v4_chunk_offsets/stereo_oracle.json` and is intentionally not committed because it is generated benchmark data.

This measurement narrows the design:

- The cheap dynamic-UBO-offset path is real but not universal. Of captured vertex draws, 104,445 used remapped uniforms, 7,491 full constant-file mode, and only 9,926 full constant-bank mode. Pixel draws were similarly dominated by remapped data (75,444 versus 8,663 full constant-bank).
- Matched vertex register/remapped payloads differed on 39,585 of 40,971 comparisons, but the differences concentrate at repeatable shader-specific 16-byte offsets. Examples include offsets `48` and `96` in 160-byte payloads and offsets `256`, `288`, and `304` in a 4096-byte payload. That pattern is compatible with shared view/projection fields; it is not evidence that every byte or every per-object block must be regenerated.
- Several full constant banks also carry systematic eye differences: VS banks 1 and 6 differed in every matched comparison, VS bank 8 in about 92.6%, PS bank 1 in every comparison, and PS bank 6 in about 80.6%. Other banks changed address but not content, proving that address inequality alone cannot classify eye-varying data.
- The next camera-identification step must correlate the ranked chunks with BetterVR's exact left/right matrices and capture selected raw values. Tier 3 remapped-uniform patching is now a required first-class path, not a remote fallback. Tier 1's clip transform remains the lowest-risk way to establish geometry correctness before full shading substitution.

The 45,413 matched pairs are a strong working corpus, not proof that every draw has a twin. Unmatched draws remain to be classified as pass-boundary effects, genuine eye-specific work, or matcher ambiguity. Eligibility stays fail-closed until that classification and the non-draw operation trace are complete.

### 6.5 Static source findings adopted into the implementation

The companion source recon narrows several open-ended work items:

- The camera source is one 35-member per-view `agl::UniformBlock`; members 0–3 are view, view-projection, projection, and inverse-view, and members 11–14 mirror them for previous-frame state. A strong live signature is a `mat3x4` at byte 0 followed by the `mat4x4` view-projection at byte 48. The camera block's shader-assignment location triple is at `+152/+153/+154` for vertex/pixel/geometry stages.
- BotW already has a native per-view count and a view-indexed billboard path. The oracle therefore validates and locates the guest-baked block rather than assuming all right-eye matrices must be synthesized by Cemu.
- Guest compute is unused; streamout is confined to `nw::eft` particles; occlusion-query allocation is already indexed/gated by the view bitmask. Cemu has no active conditional-render consumption, mip-generation path, or MSAA resolve path in this renderer. Runtime counters remain mandatory so these static absences cannot become silent assumptions.
- The exhaustive Cemu packet worklist is the 12 `IT_HLE_*` operations, the special-state 5/8 image draws, and the format-conversion surface-copy draw. Presentation, waits, timers, query callbacks, guest notifications, and swap packets run once.
- A shader-epilogue change must bump `RendererShader::GeneratePrecompiledCacheId()` as well as the generic, transferable, and pipeline cache versions. The precompiled SPIR-V path is keyed from the guest shader identity, so changing generated GLSL alone would otherwise load stale shaders.

Three debugger checks remain before the guest-baked view slot is promoted from preferred design to implementation fact: inspect `ASM_MTXInverse`'s second destination, trace the scene-component vtable `+44` hook after record construction, and trace the `ModelJobQueueSceneUpdater` vtable `+52` upload call. Read the live camera-block size during the same run.

## 7. Cemu implementation work packages

### 7.1 Extract a prepared-draw seam

Refactor Vulkan `draw_execute` without changing stock behavior:

1. `PrepareDraw` performs the draw/unsupported-state gate, streamout preparation, index decoding, vertex/guest-UBO synchronization, texture discovery, and shader lookup once.
2. `IssuePreparedDraw(EyeIssueContext)` performs uniform-variable staging/ring allocation, index and descriptor binding, target/FBO selection, pipeline lookup, render-pass selection, dynamic offsets, and `vkCmdDraw*` per eye.
3. `FinishPreparedDraw` performs streamout finish and the guest draw-counter increment once.
4. Stock Cemu calls one issue. BetterVR twin-issue calls left then right from the same prepared state.

`PreparedDraw` needs stable resolved index allocation, base vertex/instance/count, active shader objects, left dynamic offsets, resolved texture bindings, and flags describing unsupported side effects. Cemu's normal performance counters continue to count one guest draw; separate BVR counters count host eye issues.

This refactor is the first build gate: stock Cemu/BotW output and performance must remain unchanged with the feature disabled.

The split has three review-blocking traps from the current Vulkan path:

- The minimal-pipeline fast path does not include FBO identity. Every eye/FBO flip must force the full render-pass-compatible pipeline lookup; carrying the left fast-path result into the right issue is invalid.
- Uniform-variable ring exhaustion can end the render pass and submit a command buffer from inside uniform staging. `PreparedDraw` may retain logical/resource state, but it must not retain command-buffer-scoped bindings or offsets across an issue.
- `m_state.activeFBO` is normally established by `draw_beginSequence`, not `draw_execute`. Each eye issue must explicitly apply/bind its attachment set before querying viewport/fragment-coordinate scale or selecting its pipeline.

The file-scope `s_vkUniformData` scratch is also not eye-stable storage. Generate and copy each eye's uniform payload into its own live ring range before it is overwritten.

### 7.2 RenderGraphPairRegistry

Create a dedicated host-only registry rather than pretending sibling textures occupy valid guest RAM.

- key: source `LatteTexture` plus compatible view/subresource description
- value: right-eye host texture and matching `LatteTextureView`
- clone format, effective resolution, sample count, mip/slice count, tiling semantics, and depth/stencil aspect
- tie lifetime to the source texture and invalidate on resize, reinterpretation, or source deletion
- exclude siblings from guest readback, physical-overlap tracking, CPU invalidation, and `isUpdatedOnGPU` bookkeeping
- expose `MapAttachment`, `MapSampledView`, `MapClear`, and `MapCopy`

The first implementation uses a renderer-owned host image/view plus a widened FBO attachment descriptor carrying `{view, format, aspect, size}`. Cemu's surface-copy path already proves the Vulkan framebuffer side can consume bare renderer views; this avoids registering fake guest textures or physical addresses. A two-layer array backing remains the multiview migration target, not the first twin-issue dependency.

If implementation evidence forces a real `LatteTextureVk` sibling, add an explicit `isHostOnly` contract and audit global texture registration, graphic-pack rules, overlap/invalidation sweeps, guest readback, and physical-memory hashing before enabling it.

No fake physical addresses ship unless an audit proves every texture-cache overlap/readback path ignores them.

### 7.3 Twin-issue draw path

For each eligible prepared draw:

1. issue left with original attachments, sampled views, and uniform offsets;
2. switch to sibling attachments;
3. replace every sampled view that has a sibling written this frame;
4. replace the camera/clip uniform for the right issue;
5. issue right with identical geometry/material state;
6. restore the left logical state before the next guest packet.

The initial version may end/begin render passes between eye issues. Measure that cost. Do not invent PM4 replay to optimize it prematurely.

### 7.4 Duplicate required non-draw image operations

Draws alone are insufficient for a complete right render graph. Add explicit twin handling for:

- `IT_HLE_CLEAR_COLOR_DEPTH_STENCIL` (including rewriting the BetterVR magic eye payload)
- `IT_HLE_COPY_SURFACE_NEW`
- special-state 8 clears and special-state 5 depth-to-color transfers
- the format-conversion surface-copy path, which internally issues a draw
- barriers/layout transitions needed by the sibling images

Run scanout/presentation copies, swaps, waits, timers, memory writes, bottom-of-pipe callbacks, query begin/end bookkeeping, and guest notifications once. Queries and streamout remain guest-authoritative left-only unless the trace proves a visual right-eye dependency; the right issue must suppress their writes/offset advancement. Cemu currently has no MSAA resolve or mip-generation path to duplicate, but assertion counters guard that assumption.

### 7.5 Eligibility/fallback table

Each unsupported draw/pass increments a reason-specific counter and selects a deliberate policy:

| Condition | Initial policy |
|---|---|
| streamout active | issue guest draw once; mark frame incomplete and request two-pass next generation |
| active occlusion query with guest-visible result | use BotW's per-view query mask where proven; never duplicate guest query side effects, and fallback if the right visual issue cannot disable them |
| Cemu special-state draw/copy | handle through its specific paired image operation or fallback |
| missing target pair | allocate before issue if safe; otherwise fallback frame |
| unsupported target format/sample layout | fallback frame and log exact format/state |
| stale matrix/ABI generation | never issue right; fallback before entering mono guest mode |

Fallback must be frame-atomic. A partly twin-issued frame is not submitted as stereo.

## 8. Camera and shader strategy

Implement in increasing fidelity, keeping all tiers measurable.

### Tier 1: full clip-to-clip transform

Inject a 4x4 `bvrClipTransform` into the final geometry position export and set it to identity for left and `P_right * V_right * inverse(P_left * V_left)` for right. This is exact for geometry position even with asymmetric or canted eye transforms, unlike a horizontal-offset approximation.

The earlier plan's cheaper parallel-eye epilogue, `gl_Position.x += C0 + C1 * w`, is retained as a diagnostic cross-check. The full 4x4 transform is the implementation target because it also covers asymmetric projections and per-eye rotation without adding meaningful per-vertex complexity relative to the rest of the emulated shader.

Cemu's legacy decompiler funnels position exports through `SET_POSITION` in `LatteDecompilerEmitGLSLHeader.hpp`/`LatteDecompilerEmitGLSL.cpp`. Add the matrix to the Vulkan uniform-variable ring, update the active eye's dynamic offset per issue, cover vertex-only and geometry/copy-shader output paths, and bump all four shader/pipeline cache versions listed in §6.5.

This tier validates real geometry/parallax early. It does not correct fragment-stage camera position, view-dependent specular/Fresnel, or game logic that selects eye-dependent shader branches.

### Tier 2: guest-baked right view plus camera UBO substitution

Prefer BotW's native per-view machinery over reconstructing its block in the host:

- patch the render-context init count so view 1 receives its own 796-byte CPU record and double-buffered UBO slot;
- populate view 1 each frame through `updateRenderingMatricesUsingCamera` using BetterVR's right camera/projection, without invoking the render loop a second time;
- let the normal `calcGPU` view loop produce view, projection, view-projection, inverse, environment inheritance, z-plane, and previous-frame fields;
- register the camera block by shader/stage/bank/size plus the matrix-content signature;
- replace the full constant-bank dynamic offset with view 1's block for eligible vertex, geometry, and fragment stages during the right issue.

Descriptors remain reusable because Cemu binds uniform blocks as dynamic UBOs; only the dynamic offset changes. Block matching must include shader/stage/bank/size and optionally a matrix-content signature, not a raw guest address that may be recycled.

This route also lets BotW calculate view-indexed billboard `ShpBlock` data correctly. If increasing the native view count or driving its update path proves unsafe, fall back to copying the left camera block into a host upload allocation and overwriting only oracle-proven fields.

### Tier 3: remapped/register uniforms

The current oracle proves this is required: remapped/register mode dominates the captured shaders. Map each ranked remapped field back to the guest-baked view-1 block, then patch the corresponding right-eye values in the Vulkan uniform staging blob before allocating the right issue's uniform-ring range. Keep left and right allocations live until command-buffer completion.

### Fidelity gate

Tier 2/3 is the shipping goal even if Tier 1 looks acceptable. Compare final images against a two-pass reference in several scenes with water, metal/specular materials, particles, sky, foliage, and interiors. Any persistent right-eye shading mismatch becomes a mapped field or an explicit documented fallback—not an unmeasured approximation.

## 9. Real Vulkan multiview phase

After twin-issue true stereo is correct and fast enough to establish the CPU win, implement actual one-command stereo instancing where supported.

1. Require Vulkan 1.1 multiview (or `VK_KHR_multiview`) and enable/query its feature explicitly.
2. Back paired render targets with two-layer images and create render passes/dynamic-rendering state with `viewMask = 0b11`.
3. Use `gl_ViewIndex` for the eye transform. Do not double guest instance counts; that corrupts guest `gl_InstanceID` and per-instance vertex fetches.
4. Teach the shader decompiler/resource mapping to sample view-dependent intermediate textures from the matching array layer. Shared textures remain ordinary 2D resources.
5. Include multiview state and array-view types in shader, descriptor, framebuffer, and pipeline cache keys.
6. Preserve a per-draw twin-issue fallback for unsupported shaders, streamout, queries, formats, and driver capabilities.

The first multiview target is opaque main-scene geometry. Post-processing passes graduate only after their sampled-texture layer flow is proven. Mixed multiview/twin-issue frames are allowed if their render graph and barriers are explicit.

The old `b5e3dca` array OpenXR swapchain work can be reconsidered here. It may reduce final compositor calls, but it is independent of the Latte multiview work and should be ported selectively, not merged wholesale.

## 10. BetterVR-side work packages

1. Extend the guest ABI and `validate_bvr_abi.ps1`; add matrix/nonce/status fields and seqlock tests.
2. Add `TrueStereoInstancing` plus advanced `StereoBackend = two_pass | twin_issue | multiview | synth_debug` settings.
3. Add IPC commands and state JSON for requested/acknowledged backend, fork version, active generation, issue counts, fallback reasons, and fork timings.
4. Enter the mono guest frame loop only while the fork handshake is fresh and active.
5. Keep two-pass stereo as the permanent safe fallback during boot, fades, cutscenes, photo/save captures, and fork faults until each transition is separately certified.
6. Keep the HUD as one separately submitted OpenXR quad. It must not be duplicated inside the 3D render graph.
7. Reject a frame if final-eye telemetry sees missing, stale, blank, or unexpectedly identical output while instancing claims to be active.

## 11. Automated validation and debugging

Every development run produces one self-contained packet:

- exact BetterVR and Cemu commit hashes and binary hashes
- settings, active OpenXR layers, simulator/runtime identity
- in-game gate timestamp and state JSON
- benchmark windows (never menus)
- fork instancing counters/timings and render-graph summary
- six or more consecutive final-eye dumps
- final-eye hash/disparity/flicker verdict
- optional source/intermediate dumps for the first failing pass
- crash dump/JIT mapping if applicable

### Acceptance ladder

| Stage | Required proof |
|---|---|
| trace oracle | matched eye topology, UBO diff report, non-draw operation inventory |
| prepared-draw refactor | disabled mode is pixel/perf equivalent to stock fork |
| independent target chains | left/right final images are byte-identical by design, but use different texture identities; no white/black frames for 10 minutes |
| clip-transform stereo | distinct final images, correct parallax direction, no near-identical fallback, no UI flicker |
| full camera substitution | image-difference tolerance against two-pass reference across material test scenes |
| performance | stable in-game work and displayed cadence around 90 FPS; one PPC draw generation; no benchmark menu samples |
| transition hardening | repeated boot/load/shrine warp/cutscene/photo/save/inventory cycles with clean backend fallback/re-entry |
| release | 30-minute mixed-content soak, no faults/frozen-eye events/ring drops/permanent recovery failures |

Use ABBA ordering for performance comparisons. Simulator `xrEndFrame` cost is reported separately from game/Cemu work. A 90 FPS number is rejected if the final eye hashes are identical or if fork acknowledgement/counters do not prove two independent eye issues.

## 12. Repository and commit strategy

- BetterVR plan/control/telemetry: `E:\Github\BotW-BetterVR`, branch `codex/right-eye-reuse-lab` until the plan checkpoint is committed.
- Cemu research implementation: branch `codex/bvr-stereo-instancing` from `E:\Github\cemu-mcp`'s current draw-trace branch. Preserve the existing dirty `dependencies/vcpkg` submodule state; do not fold it into feature commits.
- Product fork destination: `E:\Github\Cemu-BOTW-Optimized` only after the feature survives the hardening gate. It is not currently the authoritative working tree.

Commit by independently reviewable slices: trace schema/tools; prepared-draw no-op refactor; ABI handshake; host-only target pairs; clear/copy pairing; clip transform; camera UBO; fallback hardening; multiview. Never combine a correctness refactor and a performance optimization in the same commit.

## 13. Ordered execution plan

1. **Plan checkpoint:** commit this plan and its handoff link; leave experimental image reprojection disabled.
2. **Oracle tooling:** implement UBO/render-graph/non-draw tracing in `cemu-mcp`, build it, and capture a stable true two-pass gameplay corpus.
3. **Architecture decision record and debugger closure:** validate the recovered camera-block signature/size and the three guest hooks, prove the extra-view allocation/update path, then record target-pair ownership and rejected alternatives with evidence.
4. **Prepared-draw refactor:** land and validate the disabled-mode no-op split.
5. **Handshake and dormant right-view source:** add ABI v3 capability/state exchange, provision/populate guest view 1 behind a disabled feature flag, and prove fail-closed behavior on stock and forked Cemu.
6. **Independent right render graph:** host-only target pairs plus paired clear/copy handling, initially with identity eye transform.
7. **True geometry stereo:** enable the clip-to-clip shader transform and pass final-eye/parallax/flicker gates.
8. **Full shading fidelity:** select the guest-baked view-1 UBO for full-bank shaders, map it into remapped/register staging for the remaining shaders, and compare against two-pass references.
9. **Performance pass:** profile target switching, descriptors, shader/pipeline cache churn, and native issue overhead; batch only measured hotspots.
10. **Multiview:** convert eligible render-graph islands to two-layer targets and `gl_ViewIndex`, keeping twin-issue fallback.
11. **Transitions and soak:** certify fallback/re-entry across real content, then productize the fork and default the feature on only when capability handshake succeeds.

**Current checkpoint (2026-08-13):** step 1 and the uniform-diff half of step 2 are complete. The next implementation unit is the render-graph/non-draw trace plus step 3's debugger closure and matrix correlation. Arbitrary PM4 replay remains parked until immediate prepared-draw twin issue is correct and profiling justifies it.

## 14. Effort and decision checkpoints

Before multiview, the likely size is roughly **10–18 focused engineering days**: 2–3 for the oracle/ABI work, 3–6 for the prepared-draw and target-pair path, 2–4 for camera/shader fidelity, and 3–5 for transitions and soak fixes. This is a planning range, not a promise; the uniform and non-draw operation corpus is the first estimate-reset point.

Multiview is a separate **5–10 day** optimization range because array-aware intermediate sampling and mixed fallback paths are the actual work—not the final OpenXR array swapchain. It starts only if twin issue already proves correct true stereo and profiling shows worthwhile native CPU/render-pass savings.
