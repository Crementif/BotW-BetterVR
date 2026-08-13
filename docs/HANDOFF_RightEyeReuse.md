# BetterVR Right-Eye Reuse & Debug Framework — Hand-off Document

*Last updated: 2026-08-13. Covers the CPU-optimization campaign (2026-08-12 → 13): the goal, everything built, every finding, all file locations, and how to continue.*

---

## 1. The goal

BotW-BetterVR renders true stereo by running the game's render pipeline **twice per frame** (left pass, then right pass) inside Cemu. Since everything is emulated PPC code, that double render is the dominant CPU cost. The campaign has two tracks:

- **Proposal 1 (near-term, active):** reuse the left pass's CPU work for the right pass — skip right-pass calculation and redraw the left pass's draw lists with right-eye matrices.
- **Proposal 2 (long-term, not started):** a BetterVR-specific Cemu fork that translates each GX2 draw once and issues it for both eyes (record-once/issue-twice with clip-space reprojection `rightClip = P_r·V_r·(P_l·V_l)⁻¹`). Natural home: the `Cemu-BOTW-Optimized` repo (GitHub, local copy was emptied).

A separate track already **shipped**: the window-present throttle (Cemu's `vkQueuePresentKHR` blocked ~15 ms/frame on 60 Hz virtual displays, capping everything at 59.7 FPS; fixed with `VK_EXT_swapchain_maintenance1`, real presents ~30 Hz). Plus DRC-draw skip, allocation-free actor-job routing, and a mostly-working synthesized-right-eye (mono) mode that still stalls on loads/cutscenes (same root-cause family as §4).

---

## 2. Repos, paths, binaries

| What | Where |
|---|---|
| BetterVR mod source | `E:\Github\BotW-BetterVR` (all changes **uncommitted**; ~20 modified files) |
| Built launcher + layer | `E:\Github\BotW-BetterVR\Cemu\BetterVR_Launcher.exe`, `BetterVR_Layer.dll` |
| Deployed launcher | `E:\Cemu_2.6\BetterVR_Launcher.exe` (copy after every build) |
| Clean Cemu 2.6 | `E:\Cemu_2.6\Cemu.exe` (hash-verified official) |
| Game (BOTW USA) | `E:\Roms\Legend of Zelda, The - Breath of the Wild (USA) - Decrypted\code\U-King.rpx` (the old `E:\Cemu_2.6\games\...` copy is gone) |
| Update RPX v208 | `%APPDATA%\Cemu\mlc01\usr\title\0005000e\101c9400\code\U-King.rpx` |
| Decompilation | `E:\BOTW_Decompile` — `functions.csv` (155k IDA symbols, v208 addresses), `src\seg_XXXXXXXX\<addr>_<name>.c` per function, `db\BotW_150_work.i64` |
| OpenXR Simulator | `E:\Github\OpenXR-Simulator` (built to `bin\openxr_simulator.dll/json`); file IPC at `%LOCALAPPDATA%\OpenXR-Simulator` (`controller_pose_command.json`, `runtime_status.json`); its D3D12 preview blit is throttled to 30 Hz and quad readback to 10 Hz (local patches, rebuilt) |
| cemu-mcp (debugger fork) | `E:\Github\cemu-mcp`, built binary `bin\Cemu_release.exe` (`--enable-mcp` = MCP stdio server, 42 tools incl. new `jit_lookup`). Build script: scratchpad `build_cemu_mcp6.bat` (vcpkg toolset pinning saga — see the memory file / script comments before touching deps) |
| Test tooling | `E:\Github\BotW-BetterVR\tools\` (see §5); outputs in `E:\Github\BotW-BetterVR\bench_out\` |
| Crash dumps | `%LOCALAPPDATA%\CrashDumps\Cemu.exe.*.dmp` (WER LocalDumps, HKCU-registered) |
| MCP wiring | `E:\BOTW_Decompile\.mcp.json` and `E:\Github\cemu-mcp\.mcp.json` (server `cemu-debugger`; needs a Claude session restart to load) |

**Build:** `"E:\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build E:\Github\BotW-BetterVR\build --config Release --target INSTALL`, then copy the launcher to `E:\Cemu_2.6`. Never run two `BetterVR_Launcher` instances (a dying one deletes the graphic pack from under the other).

**A save exists** in the Shrine of Resurrection, so `BootDirectlyIntoGame` + title-screen A-presses reach real gameplay (~105 s from launch).

---

## 3. The debug framework (all verified working)

Built because blind iteration (edit → rebuild → reboot → squint at a preview window) cost ~10 min/experiment. Now an experiment is: flip a mask bit **live** and read numbers ~10 s later.

### In the mod (compiled into `BetterVR_Layer.dll`)

- **Eye telemetry** — `src/rendering/telemetry.h/.cpp`. Samples 5 regions (32×32: center + 4 quadrant centers) of both **final OpenXR color swapchains** every 3D frame, after presentation/reprojection, inside the frame's D3D12 command list and into a fence-tracked readback ring (no stalls). It classifies NORMAL / WHITE / BLACK and measures temporal change, stereo difference, frozen-eye behavior, and asymmetric flicker. **This is the live flicker detector**; `telemetryLevel=2` also arms a full-image incident burst when a new stereo anomaly appears.
- **Runtime IPC** — `src/utils/ipc_control.h/.cpp`. Polls `E:\Cemu_2.6\BetterVR_cmd.ini` (`key=value` lines with an exact sequence and process `session`). Commands include `skipMask`, `skipDrc`, `synthRightEye`, `rightEyeReuse`, `dumpFrames`, `traceEvents`, `telemetryLevel`, `epoch`, and `marker`. It writes versioned `BetterVR_state.json` ~2 Hz with a freshness timestamp, live PID/session, exact applied sequence/frame, desired and boundary-latched masks, eye phase/faults, process CPU counters, final-eye telemetry, capture arbitration counters, and PPC counters. The state is published atomically and the event ring drains to `BetterVR_events.csv` when tracing is enabled.
- **Eye dumps** — `dumpFrames=N` → a consecutive burst of full-resolution final color BMPs for both eyes in `E:\Cemu_2.6\BetterVR_dumps\session_<pid>\frame<N>_*.bmp`. Per-process directories prevent an LLM analyzer from mixing stale captures with the current run. Runtime-owned D32 depth swapchains are intentionally not placed into the burst: the simulator rejects their direct linear readback, while flicker review needs the exact submitted color output.
- **PPC counters + event ring** — in `patch_RND_StereoRendering_Optimizations.asm`. Lives **in the codecave** behind magic `0x42565243` ('BVRC'); the host finds it by scanning guest `0x01800000–0x01880000`. ⚠️ **Do not put data at `0x10416C00+`** — that is live game `.data` (PhysicsMemSys reads it at boot → NULL-deref crash in `initHavokSystems`; we proved this the hard way). Only `0x10416BF0–BFC` (inside an unused "Unab…" error string) are safe, and all four words are taken (logging gate, recording mode, `VR_RENDER_SKIP_MASK`, synth flag). Counter offsets from magic: `+4` frame (ticks in the swap-buffers hook), `+8` requestDraw calls, `+C` lastSwapCursor, `+10` swapCount, `+14/+18` clearQueue full/reduced, `+1C/+20` mgrClearQueue run/reduced, `+24/+28` type-2 dispatch run/skip, `+2C` event write idx, `+30` ring (64 words, `(eventId<<26)|frame`). Event IDs 1–9 are documented in the asm and in `ipc_control.cpp`.

### Tools (`E:\Github\BotW-BetterVR\tools\`)

- **`bvr_ipc.ps1`** — dot-source. `Send-BvrCommand @{skipMask=0x6}`, `Get-BvrState`, `Wait-BvrApplied`, `Wait-BvrInGame`, `Measure-BvrFlicker <sec>`, `Request-BvrEyeDump`, and `Start-BvrIncidentCapture`. It rejects stale/dead sessions, superseded acknowledgements, stopped frame/sample progress, and unconverged guest masks.
- **`bvr_harness.ps1 -ConfigName X [-SkipMask n] [-KeepAlive] [-TakeOwnership]`** — boots via launcher + simulator (`XR_RUNTIME_JSON` → sim), dismisses dialogs, minimizes Cemu (virtual-display present cost), presses A at t=75–115 s, and requires both eyes to be recently sampled NORMAL before declaring gameplay ready. `-KeepAlive` leaves an exact-PID ownership manifest. A pre-existing Cemu is never killed unless `-TakeOwnership` is explicit; `stop_bvr_session.ps1` stops only the recorded launcher/Cemu PIDs.
- **`bisect_mask.ps1 [-Boot] [-Masks @(...)] [-HoldSec n]`** — automated mask ladder over a live instance; CLEAN/MARGINAL/FLICKER verdict per mask; **aborts when the instance doesn't recover on the control mask** (permanent corruption is a real failure mode, see §4).
- **`ab_run.ps1 -TestMask n`** — baseline vs test full runs, verdict diff.
- **`analyze_dump.ps1`** — cdb-based dump analysis; classifies faulting address as module code vs anonymous memory (anonymous = Cemu's PPC JIT cache = crash is in recompiled *game* code). Note: Cemu's own crashlog (in the launcher-echoed `log.txt`) contains the **guest** context — PPC IP, r0–r31, PPC stack — usually more useful than the host dump.
- **cemu-debugger MCP** — breakpoints, memory watch, disassemble/assemble, memory & value scans, button input, and `jit_lookup(host_address)` → maps a JIT-cache crash address to the PPC function (scans `ppcRecompilerFuncTable`). The stereo branch also exposes `bvr_begin_draw_trace`, `bvr_end_draw_trace`, `bvr_get_eye_diff`, `bvr_get_uniform_diff`, and `bvr_get_queue_snapshot`. Uniform capture reports modes, bounded whole-range hashes, and shader-specific 16-byte changed offsets. Draw eye tags come from the PM4-ordered magic 3D capture clears; the asynchronous guest `eyePhase` sample is diagnostic only. Don't run a game in it and in `E:\Cemu_2.6` simultaneously (shared `mlc01`).

### Bench logging

`[bench]` lines every 600 frames in `BetterVR_log.txt`: avg frame/work/xrWait/xrEnd/vkPresent + `mask=`, `whiteL/R=`, eye states. **Caveat:** in-game on the simulator, `xrEnd` carries a ~20 ms quad-layer readback tax (sim artifact, absent on real headsets) — compare `work`, not FPS, across configs; or use `ppc.frame` rate from the state file for true game pacing.

---

## 4. Findings: the gsys pipeline and why naive skipping fails

All addresses are v208. Decompiled sources for each are in `E:\BOTW_Decompile\src\seg_039/03A...`.

### The draw-request protocol

- `gsys::Model::requestDraw` (0x039859E4): early-outs if the model's DO_DRAW flag (byte model+0x7D) is set or the scene gate (`flags3 & 0x100`) is closed; otherwise appends the model to a **staging list** (atomic index scene+0x416C, array scene+0x4174) and sets DO_DRAW.
- `gsys::ModelScene::clearQueue` (0x039A8D54): alternates per pass between a **clear branch** (`ModelJobQueue::clear` + set gate 0x100 + `swapDrawList`) and a **flag branch** (`changeRequestFlag` walks the *queued entries* clearing their DO_DRAW + clear gate + `swapDrawList`).
- `gsys::ModelJobQueue::swapDrawList` (0x03A24088): zeroes queue+196 — that is the **fill cursor of a double-buffered list**: it means "swap halves, start a fresh fill", *not* "rewind for redraw".
- `gsys::ModelMgr::clearQueue` (0x03993634): loops scenes (array Mgr+0x20, count Mgr+0x18) calling the per-scene clearQueue, then (from 0x03993690, under a critical section) **flushes the pending model-destroy list** and free-list bookkeeping.
- `gsys::ModelMgr::calcModel` (0x03994264) → `sortModelObj` (0x039A985C) per scene = `ModelMgr::invoke(scene, wm, type=2, 1)` **(worker-job dispatch)** + `sub_3A24A38` (0x03A24A38) = staging heapSort + **`UpdateModelJobQueue`** (0x03A24874) — the render-thread queue build that turns staged models into per-context draw lists.
- `gsys::ModelMgr::invoke` (0x039A9638) is the **single choke point** for all heavy parallel calc. Job types (from all call sites): **0**×12 = calcRenderView_ (per-context cull — must run), **1**×4 + **4**×3 = background draw, **2**×1 = model calc (matrices/skinning), **3**×7 = calcWorld_ → `invokeCalcWorld` switch (env/fx/decal/light/pfx/occlusion), **5**×7 = calcGPU_.
- `gsys::ModelScene::calcFrame_` (0x039A8E70), per pass per context: `ModelRenderContext::clear` (bl 0x039A9228), requestDraw pass (bl 0x039A918C), occlusion tick (bl 0x039A9194), prep `sub_3A241E8` (bl 0x039A91C4 — just zeroes the submission cursor queue+216), **bakes the per-eye camera** (`getRenderCamera` 0x03AE4AA0 / `getRenderProjection` 0x03AE4AEC → `updateRenderingMatricesUsingCamera`), submit `sub_3A24214` (bl 0x039A9524 — fills a 548-byte record per context at queue+192[cursor++], referencing the context's lists), `updateGPU` (bl 0x039A958C — uploads per-pass camera/uniforms).

### The failure catalog (each proven with framework measurements)

1. **Whole-function skip of calcModel (old bit 1)** → the staging consumer (`sub_3A24A38`/UMJQ) is inside it → draw lists starve → world vanishes (sky+fog only — see the eye dump; "white" is a fog-colored empty world, not a blank buffer).
2. **bit-13 reduced clearQueue = swapDrawList only** → each right-pass swap exposes an empty buffer half; an **odd number of extra swaps permanently desyncs the double buffer** → white forever, even after restoring the mask. Combined with the DO_DRAW deadlock (list emptied while flags stayed set; `changeRequestFlag` only walks queued entries so the flags never clear; every later requestDraw early-outs) this is also the suspected mono-mode load-stall mechanism.
3. **ModelMgr full skip** → also skips the per-scene clearQueue calls (counters showed `clearQReduced=0`).
4. **Gating prep but not submit (iteration 2)** → submission cursor (queue+216) never resets while submit keeps post-incrementing → runs off the 548-byte record array → `0xC0000005` in JIT code at load, 3/3 runs.
5. **Skipping UMJQ alone (bit 12)** → contexts are cleared by calcFrame_ but never refilled → white. But it **doubled game pacing** (22.5→45 ppc-frames/s, host work 16.7→7.9 ms): UMJQ on the right pass is ~half the render-thread frame cost. That's the prize.
6. **Type-2/3 invoke gate alone (new bits 1+2)** → **first visually clean config** (0 white frames across minutes), but ~0 render-thread win — the gated jobs run on worker threads; the render-thread cost is the queue build (UMJQ).

### The current design: "right-pass freeze, rebake, resubmit"

On the right pass: **keep** prep + submit + camera baking + updateGPU (cursor reset, re-emit records over preserved lists, right-eye matrices uploaded); **skip** context clear + requestDraw pass + occlusion tick (calcFrame_ call-site gates, keyed to bit 13) + UMJQ (bit 12) + type-2/3 dispatch (bits 1, 2) + clearQueue entirely (bit-13 do-nothing reduced body + Mgr destroy-flush deferral, so the model-list/DO_DRAW protocol sees only left passes). Target test mask: **0x3006** (+DRC bit → 0x3007 effective).

Status at hand-off: this exact configuration is built and deployed; the first live test result goes in §7.

---

## 5. How to run an experiment (the loop)

```powershell
# 1. build + deploy (after any asm/c++ change)
& "E:\Microsoft Visual Studio\18\...\cmake.exe" --build E:\Github\BotW-BetterVR\build --config Release --target INSTALL
Copy-Item E:\Github\BotW-BetterVR\Cemu\BetterVR_Launcher.exe E:\Cemu_2.6\

# 2. boot to gameplay, leave running (~2 min)
E:\Github\BotW-BetterVR\tools\bvr_harness.ps1 -ConfigName mytest -KeepAlive -SkipMask 0

# 3. flip masks live, measure (seconds per experiment)
. E:\Github\BotW-BetterVR\tools\bvr_ipc.ps1
$seq = Send-BvrCommand @{ skipMask = 0x3006; marker = "my-experiment" }
Wait-BvrApplied $seq
Measure-BvrWhiteFrames 8          # white/black/normal deltas = flicker verdict
Get-BvrState                      # counters, workMs, eye states
Request-BvrEyeDump                # BMPs if you need to see the frame

# 4. always test recovery before trusting a "clean" result
Send-BvrCommand @{ skipMask = 1 }  # if eyes stay WHITE, the state corrupted permanently
```

Rules learned: flip masks **in-game only** (load transitions with masks active hang or crash); one bad mask usually corrupts the instance permanently → reboot between failed experiments; the bisect script automates all of this.

---

## 6. Key file inventory (mod changes, all uncommitted)

- `resources/BreathOfTheWild_BetterVR/patch_RND_StereoRendering_Optimizations.asm` — **the main experiment file**: DRC skip (bit 0), rskip entry hooks (bits 3–12; bits 1/2 whole-skips disabled), the type-filtered `invoke` gate (bits 1/2), bit-13 clearQueue do-nothing + Mgr reduced loop (destroy deferral), calcFrame_ call-site gates (requestDraw/occlusion/contextClear enabled; prep/updateGPU deliberately NOT gated — comments explain why), 'BVRC' counter block + event ring, frame tick in the swap-buffers hook. Densely commented with every root cause.
- `patch_RND_StereoRendering.asm` — pipelined frame loop (`custom_sead_GameFramework_procFrame`), per-eye sections, mono mode, and `currentEyeSide`/`currentFrameIsMono` cave vars. Stereo still uses the proven GX2 drain. Mono keeps the normal one-generation `procDraw -> calcDraw` pipeline and does **not** wait on the timestamp it just submitted; that same-frame wait serialized the CPU behind the GPU and prevented 90 Hz.
- `src/rendering/telemetry.{h,cpp}`, `src/utils/ipc_control.{h,cpp}` — the framework (§3). Registered in `src/CMakeLists.txt`.
- `src/rendering/renderer.cpp` — telemetry sampling in the render lambda, IPC tick in `StartFrame`, dump handling, `[bench]` extensions. `src/rendering/d3d12.h` — fence getters.
- `src/hooking/framebuffer.cpp` — capture handling, present throttle. `src/hooking/camera.cpp` — `GetEffectiveRenderSkipMask()` (masks to bit 0 during fades/cutscenes/not-in-game; has a known 1-frame race), synth-eye gating, reprojection matrix. `src/hooking/settings.cpp` — flag pushes, actor-job routing. `src/utils/mod_settings.h` — `RightEyeCalcSkipMask` (default 0), `SkipDrcRendering` (default on), `SynthesizedRightEye` (default off; the current one-eye path is monoscopic).
- Memory files (Claude): `C:\Users\ellio\.claude\projects\E--BOTW-Decompile\memory\` — `bettervr-debug-framework.md`, `bettervr-perf-work-status.md`, `bettervr-build-and-test-setup.md`.

---

## 7. Current status, measured results, next steps

Note on co-development: late in the session a second editing stream (the user) hardened the stack in parallel — the PPC side gained a **versioned control ABI** ('BVR2' v2 after the counter block: seqlock snapshot, `_bvrActiveMask` latched at the stereo-generation boundary in procFrame, mask-pair validation for bits 12+13 with fail-open + fault flags; all hooks now key off the latched mask), the host gained a v2 state writer (`sessionId`, `stateSeq`, process CPU-time capture), telemetry gained temporal-flicker/frozen-eye/stereo-similarity detection with incident dump bursts, and the tools gained session-bound atomic commands (`bvr_ipc.ps1` v2), ownership-aware harness locking (`-TakeOwnership`), and a corruption-aware bisect. All of it is verified live (`abiValid`, `controlConverged` in the state file).

Also learned the hard way: **asm-only rebuilds can leave the launcher's embedded graphic pack stale** (no relink). Always verify after boot: `%APPDATA%\Cemu\graphicPacks\BreathOfTheWild_BetterVR\*.asm` must match the source (one early "0x3006 hangs" result was exactly this — a stale pack; it does NOT hang on the real build).

### 7.1 Sub-function gating results matrix (all measured live via IPC + telemetry)

| Mask | Meaning | Visual | Game pacing / work | Verdict |
|---|---|---|---|---|
| `0x0001` | control (DRC skip only) | clean | 22.5 ppc-fps, work ~16.7 ms | baseline |
| `0x0006` | invoke gate: skip type-2+3 worker jobs on right pass | **CLEAN — 0 white frames over minutes, stereo intact** | unchanged (~16.7 ms) | **correct but ~0 render-thread win** (jobs are worker-side) |
| `0x1006` | + skip UpdateModelJobQueue (right pass) | white (starved) | **45 ppc-fps (2×!), work 7.9 ms** | sizes the prize: UMJQ ≈ 8 ms/frame of render-thread time |
| `0x3006` | + bit-13 (clearQueue do-nothing + destroy deferral) | white 100%, runs (31.5 ppc-fps, work 7.9 ms) | no hang (the "hang" was the stale pack) | queue protocol stays left-only, but contexts cleared+unfilled |
| `0x300E` | + contextClear gate (preserve per-context lists) | **native AV in JIT code immediately** | — | preserved ctx records reference per-pass pools the left pass recycles → use-after-free |
| any failed variant → back to `0x0001` | recovery test | stays WHITE permanently | — | DO_DRAW deadlock family: reboot between failed experiments |

**The bottom line:** the *dispatch* gate (bits 1+2) is correct and shippable but saves only worker-thread CPU; the *render-thread* prize (~8 ms/frame, half the in-game frame budget) is `UpdateModelJobQueue`, and skipping it requires the per-context draw lists to survive the right pass. Preserving them by skipping `ModelRenderContext::clear` crashes because the context entries point into per-pass pools that the next left pass recycles.

### 7.2 Next steps (in order)

1. **Chase the 0x300E use-after-free precisely**: reproduce under the cemu-debugger MCP (`jit_lookup` on the fault address → guest function; memory-breakpoint the freed pool) OR statically: `sub_3A24214`'s per-range calls (`sub_399CD38`, `sub_3A16384`) and the ctx bucket arrays (`ctx+12` → 796-byte strides) — find which allocation is per-pass and who owns it.
2. Candidate designs, cheapest first:
   a. **Replay-UMJQ**: on the right pass run a cheap variant of UMJQ that re-walks the *preserved staging list* instead of a full rebuild (needs UMJQ 0x03A24874 internals decoded — what does the `stagingCount` arg drive?).
   b. **Pool pinning**: make the per-pass draw-entry pool double-buffered (defer the left pass's pool reset by one pass) so preserved ctx records stay valid — likely a small cursor/base patch like the prep/submit pair.
   c. **Ship the partial win**: bits 1+2 default-on (`RightEyeReuse` setting) — frees ~2 worker-job sets per frame on the 3 emulated cores; measure on-headset before deciding it's "no win" (the host `work` metric doesn't see worker savings, but the real rig's core contention might).
3. Wire whatever ships into `mod_settings.h` as one user-facing toggle; keep the raw mask for the lab.
4. Load/cutscene guards: the ABI latch already commits masks at generation boundaries; still gate reuse OFF during loads (fault-flag telemetry will show if the pair-validator fires).
5. Re-attack the **mono-mode load stall** with the queue-protocol knowledge (same DO_DRAW/staging family).
6. Long term: proposal 2 (record-once/issue-twice in the Cemu fork) — now tractable with the decompile + MCP debugger + `jit_lookup`. **Full implementation plan: `docs/PLAN_StereoInstancing.md` (2026-08-13)**, with file:line evidence in `docs/NOTES_StereoInstancing_SourceRecon.md` — it supersedes items 1–2 above unless instancing stalls.

---

## 8. Automated lab v2 (2026-08-13)

The framework above has been hardened so unattended runs fail closed instead of silently producing a clean result from stale state.

### Boundary-safe guest control

- `BVRC` legacy counters are followed by a versioned `BVR2` ABI (`version=2`, `size=0x170`). The host validates magic/version/size and reads changing fields with an odd/even seqlock. The last four words contain the live `clearQueue` caller-state telemetry used to diagnose mono queue poisoning.
- The fixed settings word is now only the **desired** mask. PPC hooks read `_bvrActiveMask`, which changes after the outstanding right draw and before the next left calculation generation.
- Bits `0x1000` (skip UpdateModelJobQueue) and `0x2000` (preserve the right queue) are an invariant pair. A half-enabled request is cleared and exported as a fault instead of corrupting the double-buffer protocol.
- State exports desired/active masks, mask epoch, activation frame, eye phase, faults, and `controlConverged`. IPC acknowledgement requires exact sequence equality, a matching process session, a fresh advancing state sequence, and guest convergence.
- The PPC event ring writes each slot before publishing its write index. Readers therefore cannot consume a new index with the previous slot contents.

Validate the layout without starting Cemu:

```powershell
E:\Github\BotW-BetterVR\tools\validate_bvr_abi.ps1
```

### Final OpenXR output and LLM flicker evidence

Telemetry samples the final left/right OpenXR color swapchains after the presentation/reprojection shaders. This fixes the old blind spot where synthesized-right output was never sampled and clean source textures could hide a bad final image. In addition to WHITE/BLACK it exports:

- per-eye temporal mean absolute difference, edge energy, frozen frames, sample/drop counts, and flicker events;
- inter-eye difference and near-identical fraction;
- asymmetric-flicker and frozen-eye events;
- capture arbitration counts (`current3D`, stable reuse, suppression, duplicate drops, fatal invalidations).

`dumpFrames=N` captures a consecutive final-output burst. The analyzer turns that burst into a compact review packet for an LLM or human:

```powershell
. E:\Github\BotW-BetterVR\tools\bvr_ipc.ps1
Set-BvrDir E:\Cemu_2.6
Start-BvrIncidentCapture -Frames 12 -Marker repro-flicker
E:\Github\BotW-BetterVR\tools\analyze_openxr_flicker.ps1 -DumpDir E:\Cemu_2.6\BetterVR_dumps
```

Outputs are `contact_sheet.png`, amplified `differences.png`, `motion_compensated.png`, `metrics.json`, and `LLM_REVIEW.md`. The detector uses blank flashes, robust temporal outliers, asymmetric eye changes, frozen-eye behavior, and alternating luminance. Consecutive frames also get phase-correlation `(dx,dy)` screen-space motion estimates and motion-compensated residuals; these are final-image motion vectors for separating camera motion from flashes, not game object velocities. Its clean verdict is explicitly scoped to the burst; missing/stale samples are errors, never clean evidence. Regression test:

```powershell
python E:\Github\BotW-BetterVR\tools\tests\test_flicker_analyzer.py
```

### Resumable ABBA scenario runner

`bvr_lab.ps1` loads `right_eye_scenarios.json` and runs control/candidate/candidate/control blocks. It records process cycle/time deltas, PPC progress, final-image correctness, capture counters, state snapshots, commit/diff/binary hashes, and recovery after every candidate. It resumes by trial ID and halts on stale heartbeat, a replaced process session, incomplete eye sampling, command mismatch, guest mask mismatch, or failed recovery.

```powershell
E:\Github\BotW-BetterVR\tools\bvr_lab.ps1 -TrialSec 20 -Blocks 2
# after an interrupted run
E:\Github\BotW-BetterVR\tools\bvr_lab.ps1 -Resume
```

The launcher harness no longer kills a pre-existing Cemu/BetterVR process by calling it stale. `-TakeOwnership` is required, keep-alive retains an ownership manifest, and `stop_bvr_session.ps1` stops only recorded Cemu/launcher PIDs.

### Cemu GX2 eye trace

`E:\Github\cemu-mcp` adds a bounded Latte command-processor trace and five MCP tools:

- `bvr_begin_draw_trace(max_draws, capture_uniforms, max_uniform_bytes)`
- `bvr_get_queue_snapshot(recent)`
- `bvr_get_eye_diff(max_mismatches)`
- `bvr_get_uniform_diff(max_ranges)`
- `bvr_end_draw_trace()`

Every entry records the asynchronous guest frame/eye sample, authoritative PM4 marker eye/pass, topology, instance/base/index information, shader addresses, color target, uniform modes/ranges, and a camera-independent pipeline fingerprint. The uniform diff pairs complete left/right marker passes by capture slot, ignores transient index/target addresses, and ranks shader/stage/bank 16-byte changed offsets. A trace prefix or suffix without its closing capture marker stays unknown instead of being guessed.

`tools/capture_stereo_oracle.py --out <directory>` is the automated Phase-0 runner. It uses an isolated Cemu fork directory, drives the title screen through the simulator, requires 90 advancing stable in-game frames before tracing, captures a short bounded two-pass corpus, saves the eye/uniform reports and logs, validates distinct healthy final OpenXR eyes, and terminates only its own launcher/Cemu process tree. The accepted 2026-08-13 corpus is `bench_out/stereo_oracle_20260813_v4_chunk_offsets/`; see the full interpretation in `docs/PLAN_StereoInstancing.md` section 6.4.

### Simulator-visible flicker detector

Application swapchain telemetry cannot detect a defect introduced by the desktop simulator's own layer composition or GDI paint path. `E:\Github\OpenXR-Simulator\src\flicker_detector.{h,cpp}` therefore observes the existing D3D12 CPU preview DIB after projection and quad composition, with no additional GPU readback. It also records projection-layer presence at every `xrEndFrame` and the success/fallback result of every visible `WM_PAINT`.

Live status is atomically published to `%LOCALAPPDATA%\OpenXR-Simulator\flicker_status.json`. It includes projection omissions/transitions, preview luma and temporal deltas, left/right asymmetry, paint failures, duplicate-generation paints, anomaly count, and the latest incident directory. A detected blank return, large temporal jump, asymmetric/alternating frame, missing projection after startup, or fallback-black paint creates a pre/post packet under `%LOCALAPPDATA%\OpenXR-Simulator\flicker_incidents\session_<pid>\incident_<frame>`. Those frames use the analyzer-compatible `frame<N>_color_L/R.bmp` names and also include the full composed preview.

The simulator MCP server adds `get_flicker_status(include_images, max_frames)` and `capture_flicker_window(timeout, max_frames)`. The first rejects paths outside the simulator data root, reports status freshness, and returns an in-memory JPEG contact sheet from the latest incident. The second forces a rolling pre/post composed-preview burst even when no automatic threshold fired, so an LLM can inspect subtle reported flicker instead of receiving another clean single screenshot. `analyze_openxr_flicker.py` recognizes `incident.json`, labels the source as `openxr-simulator-composed-preview`, and combines runtime continuity triggers with pixel evidence.

The D3D12 presentation path was hardened at the same time: it no longer clears the full client area to black before each image, repaints only when the composed DIB generation changes, and tracks quad writes as new generations. Letterbox bars are still explicitly cleared. This removes duplicate 90 Hz GDI paints and the intermediate black surface that can flicker even while the off-screen DIB is stable.

### UI-only flicker diagnosis and fix

The remaining UI flicker came from two independent simulator throttles. Projection readback refreshed the persistent preview DIB at about 30 Hz, replacing all prior pixels with a clean world image. Quad-layer GPU readback ran at about 10 Hz and returned immediately on the intervening frames, so it did not restore the UI after most projection refreshes. A detector-only live baseline proved the cadence mismatch: 229 refreshed projection frames with a submitted quad produced 95 fresh UI compositions and **134 `UI_NOT_RECOMPOSED_AFTER_PROJECTION` omissions**.

The simulator now caches the latest successfully read-back quad texture as a persistent premultiplied GDI bitmap. Every projection refresh receives exactly one UI composition: fresh pixels when a 10 Hz readback occurs on that refresh, otherwise the cached bitmap through `AlphaBlend`. Cached frames do not reallocate, recalculate alpha, or copy the 2320x2528 source. A fresh quad readback that occurs between projection refreshes updates the cache but is not alpha-blended over an already-composed DIB; this avoids accumulating partially transparent UI pixels.

UI-specific status is atomically published to `%LOCALAPPDATA%\OpenXR-Simulator\ui_flicker_status.json`. It reports submitted quad frames, projection refreshes, fresh readbacks/compositions, cached compositions, missing-after-projection frames, composition failures, alpha coverage, and UI-rectangle temporal metrics. Incidents under `ui_flicker_incidents\session_<pid>` contain only the projected left/right UI rectangles, excluding unrelated world motion.

The MCP tools are `get_ui_flicker_status(include_images, max_frames)` and `capture_ui_flicker_window(timeout, max_frames)`. `analyze_openxr_flicker.py` recognizes `captureSource=openxr-simulator-ui-quad`; a runtime continuity trigger yields `FLICKER_DETECTED` even if a mostly-transparent UI changes too few whole-frame pixels for image thresholds.

Post-fix optimized live acceptance: 1,176 projection refreshes, 382 fresh compositions, 794 cached recompositions (exactly one composition per refresh), **0 missing UI frames**, **0 composition failures**, and **0 automatic UI anomalies**. A forced 22-frame UI-only packet also returned `NO_FLICKER_DETECTED`; its incident count is manual evidence collection, not an anomaly. Twelve sampled BetterVR state intervals averaged about 17.0 ms host work, so persistent UI recomposition did not add the multi-megapixel conversion cost to cached frames.

---

## 9. Experimental mono throughput result (2026-08-13; not stereoscopic)

`SynthesizedRightEye=true`, `SkipDrcRendering=true`, and raw `RightEyeCalcSkipMask=0` render one left-eye game pass. The compositor currently mirrors that color/depth source into both OpenXR views: `frame.synthReprojMtx` is deliberately set to `std::nullopt` in `framebuffer.cpp`, and the acceptance dump produced byte-identical left/right final images. This is **monoscopic throughput evidence, not completion of the right-eye CPU-reuse goal**. The option remains default-off, and `rightEyeReuse=1` continues to select the two-pass queue-reuse laboratory mask.

Two fixes were required in the game-side pipeline:

1. A mono generation calls `gsys::ModelScene::clearQueue` twice, and both original calls arrived with `shouldRequest=0`. The first call destroyed the prior list before `changeRequestFlag` could clear each model's `DO_DRAW` bit, so all later `requestDraw` calls early-outed and the world stayed fog-white. The original callers provide a stable discriminator in `r5`: the pre-calc caller passes 1 and the `ModelMgr::calcFrame` caller passes 0. For mono only, the hook forces `r4=1` on the pre-calc call, restoring the required flag-walk/clear pair.
2. The mono tail called `GX2WaitTimeStamp(GX2GetLastSubmittedTimeStamp())` after every `procDraw`. This forced the next CPU `calcDraw` to wait for the whole scene render. Keeping the timestamp but removing that immediate wait restored CPU/GPU overlap while preserving the one-generation pipeline and backend fence ordering.

The automated run used the OpenXR simulator at 1280x1424 per eye with the OBS, ReShade, and FSR OpenXR API layers explicitly disabled. The in-game gate armed only after 90 stable gameplay frames. It establishes the amount of performance available if the second emulated game pass can be removed, but it does not establish stereo correctness:

- warm-up windows: **9.88 ms / 101.2 FPS** and **9.64 ms / 103.8 FPS**;
- steady runtime-paced windows: repeated **11.10-11.12 ms / 89.9-90.1 FPS**;
- steady component averages: game work about **7.1-7.2 ms**, `xrWaitFrame` about **1.7-1.9 ms**, `xrEndFrame` about **2.0-2.2 ms**, and Vulkan present about **0.10 ms**;
- more than 8,000 in-game frames in the first soak with both mirrored final eyes classified `NORMAL`, zero white frames, zero compositor omissions, and zero GPU command-buffer errors;
- simulator UI detector: **0 missing-after-projection**, **0 composition failures**, and **0 anomalies**; general simulator detector: **0 anomalies**.

The manually launched follow-up never reached gameplay because it was started without the harness's menu input, so it is not additional visual proof. The old synchronous `calcDraw -> procDraw` same-generation experiment is also invalid because it wedges Cemu's Vulkan submission path. Useful findings to carry back into the real task are the corrected mono `clearQueue` pairing and the cost of the same-frame GPU completion wait; neither substitutes for a distinct right-eye image.
