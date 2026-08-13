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

## 4. Target architecture

```text
BotW PPC (one frame generation)
  simulation once
  union-frustum culling once
  calcDraw / GX2 command production once
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

The current true two-pass renderer is the oracle. Extend `BVRDrawTrace` before implementing replay.

### 6.1 Trace schema

For every draw, clear, surface copy, resolve, compute dispatch, query boundary, and streamout operation, record:

- PPC frame, eye phase, command-buffer nesting depth, pass ID, and ordinal
- shaders, topology, index/instance parameters, special-state flags
- color/depth attachment identities, format, size, mip, slice, and physical range
- all sampled texture identities and subresources
- UBO stage/bank, guest address, byte size, content hash, and uniform mode
- uniform-register/remapped-data hash
- query/conditional-render/streamout state
- whether the operation writes guest-visible memory

Keep large byte dumps opt-in. The normal trace stores hashes and bounded samples so a several-second capture remains usable by an LLM.

### 6.2 New debugger/MCP outputs

- `bvr_trace_render_graph`: bounded JSON/CSV render graph grouped by pass and eye
- `bvr_get_uniform_diff`: matches left/right draws and ranks differing UBO ranges/fields
- `bvr_dump_uniform_pair`: dumps one matched block pair plus known view/projection matrices
- `bvr_get_operation_diff`: unmatched clears/copies/queries/dispatches between eyes
- `bvr_get_instancing_stats`: live replay/fallback/timing counters

All tools must report tag confidence because the Latte thread consumes GX2 packets asynchronously from the PPC eye-phase writes.

### 6.3 Questions this phase must answer

1. Do matched eye draws differ only in a small shared camera/environment block, or in per-object blocks?
2. Which shader stages consume those differences?
3. Which shaders use full constant banks versus remapped/register uniforms?
4. What non-draw image operations are needed to preserve each eye's post-processing chain?
5. How often do streamout, occlusion queries, conditional rendering, compute, and Cemu special states occur in normal gameplay and transitions?
6. What is the maximum draw/pass/target count per frame, and what native overhead budget remains below 11.11 ms?

**Gate:** do not implement camera UBO synthesis until matched two-pass data proves the block identity and field offsets. The geometry-only clip-transform path can proceed independently.

## 7. Cemu implementation work packages

### 7.1 Extract a prepared-draw seam

Refactor Vulkan `draw_execute` without changing stock behavior:

1. `PrepareDraw` performs guest-dependent work once: special-state checks, index decoding, vertex and uniform buffer synchronization, texture discovery, and shader lookup.
2. `IssuePreparedDraw(EyeIssueContext)` performs eye-dependent work: target selection, eye uniform offset, descriptor/pipeline binding, render-pass selection, and `vkCmdDraw*`.
3. Stock Cemu calls `IssuePreparedDraw(left/default)` once. BetterVR twin-issue calls it twice.

`PreparedDraw` needs stable resolved index allocation, base vertex/instance/count, active shader objects, left dynamic offsets, resolved texture bindings, and flags describing unsupported side effects. Cemu's normal performance counters continue to count one guest draw; separate BVR counters count host eye issues.

This refactor is the first build gate: stock Cemu/BotW output and performance must remain unchanged with the feature disabled.

### 7.2 RenderGraphPairRegistry

Create a dedicated host-only registry rather than pretending sibling textures occupy valid guest RAM.

- key: source `LatteTexture` plus compatible view/subresource description
- value: right-eye host texture and matching `LatteTextureView`
- clone format, effective resolution, sample count, mip/slice count, tiling semantics, and depth/stencil aspect
- tie lifetime to the source texture and invalidate on resize, reinterpretation, or source deletion
- exclude siblings from guest readback, physical-overlap tracking, CPU invalidation, and `isUpdatedOnGPU` bookkeeping
- expose `MapAttachment`, `MapSampledView`, `MapClear`, and `MapCopy`

Before choosing the factory, test two designs in a small fork-only probe:

- a renderer-owned clone created through a new explicit host-only texture API (preferred), or
- a two-layer array backing where layer 0 is the guest view and layer 1 is the sibling (closer to eventual multiview, but more invasive).

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
- resolves, mip generation, and any format-conversion copy observed by the Phase-0 trace
- barriers/layout transitions needed by the sibling images

Run timers, memory writes, bottom-of-pipe callbacks, swap requests, and guest notification packets once. Queries and streamout remain guest-authoritative left-only unless the trace proves a visual right-eye dependency; the right issue must suppress their writes/offset advancement.

### 7.5 Eligibility/fallback table

Each unsupported draw/pass increments a reason-specific counter and selects a deliberate policy:

| Condition | Initial policy |
|---|---|
| streamout active | issue guest draw once; mark frame incomplete and request two-pass next generation |
| active occlusion/conditional query with guest-visible result | do not duplicate query side effects; twin visual draw only if renderer can disable query for the right issue, otherwise fallback |
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

Cemu's legacy decompiler funnels position exports through `SET_POSITION` in `LatteDecompilerEmitGLSLHeader.hpp`/`LatteDecompilerEmitGLSL.cpp`. Add the matrix to the Vulkan uniform-variable ring, update the active eye's dynamic offset per issue, cover vertex-only and geometry/copy-shader output paths, and bump the shader-cache version.

This tier validates real geometry/parallax early. It does not correct fragment-stage camera position, view-dependent specular/Fresnel, or game logic that selects eye-dependent shader branches.

### Tier 2: identified camera UBO substitution

Using the uniform diff results:

- register the small set of camera/environment block address+size signatures;
- copy each left block once per frame into a host upload allocation;
- overwrite proven fields (view, projection, view-projection, inverses, camera position/direction, previous-view values) using the right-eye data;
- replace the dynamic UBO offset for vertex, geometry, and fragment stages during the right issue.

Descriptors remain reusable because Cemu binds uniform blocks as dynamic UBOs; only the dynamic offset changes. Block matching must include shader/stage/bank/size and optionally a matrix-content signature, not a raw guest address that may be recycled.

### Tier 3: remapped/register uniforms

If the oracle finds eye differences in uniform registers or remapped gathers, patch the corresponding right-eye values in the Vulkan uniform staging blob before allocating the right issue's uniform-ring range. Keep left and right allocations live until command-buffer completion.

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
3. **Architecture decision record:** select target-pair ownership and camera tiers from the corpus; record rejected alternatives with evidence.
4. **Prepared-draw refactor:** land and validate the disabled-mode no-op split.
5. **Handshake:** add ABI v3 capability/state exchange and prove fail-closed behavior on stock and forked Cemu.
6. **Independent right render graph:** host-only target pairs plus paired clear/copy handling, initially with identity eye transform.
7. **True geometry stereo:** enable the clip-to-clip shader transform and pass final-eye/parallax/flicker gates.
8. **Full shading fidelity:** synthesize/substitute the oracle-identified camera UBO/register data and compare to two-pass references.
9. **Performance pass:** profile target switching, descriptors, shader/pipeline cache churn, and native issue overhead; batch only measured hotspots.
10. **Multiview:** convert eligible render-graph islands to two-layer targets and `gl_ViewIndex`, keeping twin-issue fallback.
11. **Transitions and soak:** certify fallback/re-entry across real content, then productize the fork and default the feature on only when capability handshake succeeds.

The first code after this plan is therefore **trace expansion**, not replay. That is the shortest route to avoiding another visually plausible but structurally wrong shortcut.

## 14. Effort and decision checkpoints

Before multiview, the likely size is roughly **10–18 focused engineering days**: 2–3 for the oracle/ABI work, 3–6 for the prepared-draw and target-pair path, 2–4 for camera/shader fidelity, and 3–5 for transitions and soak fixes. This is a planning range, not a promise; the uniform and non-draw operation corpus is the first estimate-reset point.

Multiview is a separate **5–10 day** optimization range because array-aware intermediate sampling and mixed fallback paths are the actual work—not the final OpenXR array swapchain. It starts only if twin issue already proves correct true stereo and profiling shows worthwhile native CPU/render-pass savings.
