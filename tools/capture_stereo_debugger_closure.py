#!/usr/bin/env python3
"""Close BetterVR's remaining guest stereo-instancing debugger questions.

The run is gameplay-gated, then uses temporary PPC breakpoints to preserve the
arguments and before/after memory around ASM_MTXInverse and the two indirect
ModelScene calls identified by source recon. Every breakpoint is removed before
the session resumes, and only the Cemu PID created by this run is terminated.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any

from capture_stereo_oracle import (
    DEFAULT_BETTERVR_LAUNCHER,
    DEFAULT_LAUNCHER_DIR,
    DEFAULT_SIMULATOR_MANIFEST,
    McpClient,
    copy_with_retry,
    find_conflicting_processes,
    read_json,
    sha256,
    stage_if_different,
    terminate_pid_tree,
    terminate_tree,
    unlink_with_retry,
    wait_for_gameplay,
    write_settings,
)


ASM_MTX_INVERSE = 0x03C6FF74
SCENE_COMPONENT_CALL = 0x039A9578
SCENE_COMPONENT_AFTER = 0x039A957C
UPLOAD_CALL = 0x039A95B4
UPLOAD_AFTER = 0x039A95B8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launcher-dir", type=Path, default=DEFAULT_LAUNCHER_DIR)
    parser.add_argument("--cemu-source", type=Path, required=True)
    parser.add_argument("--bettervr-launcher-source", type=Path, default=DEFAULT_BETTERVR_LAUNCHER)
    parser.add_argument("--simulator-manifest", type=Path, default=DEFAULT_SIMULATOR_MANIFEST)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gameplay-timeout", type=float, default=180.0)
    parser.add_argument("--breakpoint-timeout", type=float, default=30.0)
    parser.add_argument("--stable-frames", type=int, default=90)
    parser.add_argument("--max-phase-attempts", type=int, default=32)
    parser.add_argument("--probe-seconds", type=float, default=3.0)
    return parser.parse_args()


def wait_paused(client: McpClient, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = client.call_tool("get_game_state", {}, timeout=5.0)
        if isinstance(state, dict) and state.get("paused"):
            return state
        time.sleep(0.025)
    raise TimeoutError(f"PPC breakpoint was not reached within {timeout:.1f}s")


def safe_call(client: McpClient, name: str, arguments: dict[str, Any], timeout: float = 10.0) -> Any:
    try:
        return client.call_tool(name, arguments, timeout=timeout)
    except Exception as error:
        return {"error": f"{type(error).__name__}: {error}"}


def read_range(client: McpClient, address: int, length: int) -> dict[str, Any]:
    if address <= 0:
        return {"error": "null address", "address": address, "length": length}
    result = safe_call(client, "read_memory", {"address": address, "length": length})
    return result if isinstance(result, dict) else {"error": str(result), "address": address, "length": length}


def changed_offsets(before: dict[str, Any], after: dict[str, Any]) -> list[int]:
    try:
        left = bytes.fromhex(str(before["data"]))
        right = bytes.fromhex(str(after["data"]))
    except (KeyError, TypeError, ValueError):
        return []
    return [index for index, (a, b) in enumerate(zip(left, right, strict=False)) if a != b]


def remove_breakpoint(client: McpClient, address: int) -> None:
    safe_call(client, "remove_breakpoint", {"address": address})


def recover_debugger(client: McpClient) -> None:
    for address in (ASM_MTX_INVERSE, SCENE_COMPONENT_CALL, SCENE_COMPONENT_AFTER, UPLOAD_CALL, UPLOAD_AFTER):
        remove_breakpoint(client, address)
    safe_call(client, "resume", {})


def capture_inverse(client: McpClient, timeout: float) -> dict[str, Any]:
    client.call_tool("add_breakpoint", {"address": ASM_MTX_INVERSE})
    wait_paused(client, timeout)
    registers = client.call_tool("get_registers", {})
    guest = client.call_tool("bvr_get_guest_state", {})
    callstack = safe_call(client, "get_callstack", {})
    r3 = int(registers["gpr"][3])
    r4 = int(registers["gpr"][4])
    return_address = int(registers["lr"])
    before_r3 = read_range(client, r3, 48)
    before_r4 = read_range(client, r4, 48)
    remove_breakpoint(client, ASM_MTX_INVERSE)
    client.call_tool("add_breakpoint", {"address": return_address})
    client.call_tool("resume", {})
    wait_paused(client, timeout)
    return_registers = client.call_tool("get_registers", {})
    after_r3 = read_range(client, r3, 48)
    after_r4 = read_range(client, r4, 48)
    remove_breakpoint(client, return_address)
    client.call_tool("resume", {})
    return {
        "entry": f"0x{ASM_MTX_INVERSE:08X}",
        "returnAddress": f"0x{return_address:08X}",
        "guest": guest,
        "registers": registers,
        "returnRegisters": return_registers,
        "callstack": callstack,
        "r3": f"0x{r3:08X}",
        "r4": f"0x{r4:08X}",
        "r3Before": before_r3,
        "r3After": after_r3,
        "r4Before": before_r4,
        "r4After": after_r4,
        "r3ChangedOffsets": changed_offsets(before_r3, after_r3),
        "r4ChangedOffsets": changed_offsets(before_r4, after_r4),
        "hiddenSecondDestinationWritten": bool(changed_offsets(before_r4, after_r4)),
    }


def capture_indirect_call(
    client: McpClient,
    call_address: int,
    after_address: int,
    target_register: int,
    observed_register: int,
    observed_length: int,
    timeout: float,
) -> dict[str, Any]:
    client.call_tool("add_breakpoint", {"address": call_address})
    wait_paused(client, timeout)
    registers = client.call_tool("get_registers", {})
    guest = client.call_tool("bvr_get_guest_state", {})
    target = int(registers["gpr"][target_register])
    observed = int(registers["gpr"][observed_register])
    before = read_range(client, observed, observed_length)
    target_code = safe_call(client, "disassemble", {"address": target, "count": 100})
    callstack = safe_call(client, "get_callstack", {})
    remove_breakpoint(client, call_address)
    client.call_tool("add_breakpoint", {"address": after_address})
    client.call_tool("resume", {})
    wait_paused(client, timeout)
    after_registers = client.call_tool("get_registers", {})
    after = read_range(client, observed, observed_length)
    remove_breakpoint(client, after_address)
    client.call_tool("resume", {})
    differences = changed_offsets(before, after)
    return {
        "callAddress": f"0x{call_address:08X}",
        "afterAddress": f"0x{after_address:08X}",
        "target": f"0x{target:08X}",
        "guest": guest,
        "registers": registers,
        "afterRegisters": after_registers,
        "callstack": callstack,
        "observedRegister": f"r{observed_register}",
        "observedAddress": f"0x{observed:08X}",
        "before": before,
        "after": after,
        "changedOffsets": differences,
        "changedByteCount": len(differences),
        "targetDisassembly": target_code,
    }


def capture_both_phases(
    client: McpClient,
    call_address: int,
    after_address: int,
    target_register: int,
    observed_register: int,
    observed_length: int,
    timeout: float,
    max_attempts: int,
) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    phases: set[int] = set()
    for _ in range(max_attempts):
        sample = capture_indirect_call(
            client,
            call_address,
            after_address,
            target_register,
            observed_register,
            observed_length,
            timeout,
        )
        phase = int((sample.get("guest") or {}).get("eyePhase", 2))
        if phase not in phases or phase > 1:
            samples.append(sample)
        if phase <= 1:
            phases.add(phase)
        if phases == {0, 1}:
            break
    return {
        "capturedEyePhases": sorted(phases),
        "bothEyesCaptured": phases == {0, 1},
        "attemptLimit": max_attempts,
        "samples": samples,
    }


def capture_non_pausing_probes(client: McpClient, seconds: float) -> dict[str, Any]:
    guest = client.call_tool("bvr_get_guest_state", {})
    counter_base = int(str(guest.get("counterBase", "0")), 0)
    tag_addresses = [counter_base + 4, counter_base + 0x150] if counter_base else []
    addresses = [ASM_MTX_INVERSE, SCENE_COMPONENT_CALL, UPLOAD_CALL]
    client.call_tool(
        "start_code_probe",
        {"addresses": addresses, "tag_addresses": tag_addresses, "max_hits": 8192},
    )
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        time.sleep(0.05)
    raw = client.call_tool("get_code_probe_hits", {"recent": 4096, "clear": False})
    stop = client.call_tool("stop_code_probe", {})
    hits = raw.get("hits", []) if isinstance(raw, dict) else []

    sections: dict[int, list[dict[str, Any]]] = {address: [] for address in addresses}
    for hit in hits:
        address = int(hit.get("address", 0))
        if address not in sections:
            continue
        registers = hit.get("gpr", [])
        tags = hit.get("tags", [])
        item = {
            "ordinal": hit.get("ordinal"),
            "address": f"0x{address:08X}",
            "lr": f"0x{int(hit.get('lr', 0)):08X}",
            "thread": f"0x{int(hit.get('thread', 0)):08X}",
            "ppcFrame": int(tags[0]) if len(tags) > 0 else None,
            "eyePhase": int(tags[1]) if len(tags) > 1 else None,
            "gpr": registers,
        }
        sections[address].append(item)

    def summarize(address: int, target_register: int | None, argument_registers: list[int]) -> dict[str, Any]:
        address_hits = sections[address]
        phases = sorted({int(hit["eyePhase"]) for hit in address_hits if hit.get("eyePhase") in (0, 1)})
        targets = (
            sorted({int(hit["gpr"][target_register]) for hit in address_hits if len(hit["gpr"]) > target_register})
            if target_register is not None
            else []
        )
        target_disassembly = [
            {
                "target": f"0x{target:08X}",
                "instructions": safe_call(client, "disassemble", {"address": target, "count": 100}),
            }
            for target in targets[:16]
        ]
        compact_hits = []
        for hit in address_hits[:128]:
            registers = hit.pop("gpr")
            hit["arguments"] = {
                f"r{register}": f"0x{int(registers[register]):08X}"
                for register in argument_registers
                if len(registers) > register
            }
            if target_register is not None and len(registers) > target_register:
                hit["target"] = f"0x{int(registers[target_register]):08X}"
            compact_hits.append(hit)
        return {
            "hitCount": len(address_hits),
            "capturedEyePhases": phases,
            "bothEyesCaptured": phases == [0, 1],
            "uniqueTargets": [f"0x{target:08X}" for target in targets],
            "targetDisassembly": target_disassembly,
            "hits": compact_hits,
        }

    inverse = summarize(ASM_MTX_INVERSE, None, [3, 4])
    inverse_pairs = [
        (hit["arguments"].get("r3"), hit["arguments"].get("r4"))
        for hit in inverse["hits"]
    ]
    # The routine supports in-place inversion, so some callers legitimately use
    # r3 == r4. A single out-of-place call proves the decompiler-dropped r4
    # destination; the camera calc path is one such caller.
    inverse["r4DistinctFromR3"] = any(left != right for left, right in inverse_pairs)
    inverse["hiddenSecondArgumentRole"] = (
        "r4 is the distinct output destination; static paired-single stores target 0..44(r4)"
        if inverse["r4DistinctFromR3"]
        else "not observed"
    )
    return {
        "guestAtStart": guest,
        "tagAddresses": [f"0x{address:08X}" for address in tag_addresses],
        "captureSeconds": seconds,
        "rawCapturedHits": raw.get("capturedHits", 0) if isinstance(raw, dict) else 0,
        "stop": stop,
        "asmMtxInverse": inverse,
        "sceneComponentHook": summarize(SCENE_COMPONENT_CALL, 10, [3, 4, 10]),
        "uploadVirtual": summarize(UPLOAD_CALL, 7, [3, 4, 5, 6, 7]),
    }


def main() -> int:
    args = parse_args()
    launcher_dir = args.launcher_dir.resolve()
    launcher = launcher_dir / "BetterVR_Launcher.exe"
    cemu = launcher_dir / "Cemu.exe"
    conflicts = find_conflicting_processes()
    if conflicts:
        raise RuntimeError(f"refusing to start beside an existing session: {conflicts}")
    stage_if_different(args.cemu_source.resolve(), cemu)
    stage_if_different(args.bettervr_launcher_source.resolve(), launcher)
    state_path = launcher_dir / "BetterVR_state.json"
    settings_path = launcher_dir / "BetterVR_settings.ini"
    simulator_command = Path(os.environ["LOCALAPPDATA"]) / "OpenXR-Simulator" / "controller_pose_command.json"
    write_settings(settings_path)
    for stale in (state_path, launcher_dir / "BetterVR_cmd.ini", launcher_dir / "BetterVR_events.csv"):
        unlink_with_retry(stale)
    args.out.mkdir(parents=True, exist_ok=True)

    environment = os.environ.copy()
    environment["XR_RUNTIME_JSON"] = str(args.simulator_manifest.resolve())
    process = subprocess.Popen(
        [str(launcher), "--enable-mcp"],
        cwd=launcher_dir,
        env=environment,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        bufsize=1,
        creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0),
    )
    client = McpClient(process)
    owned_cemu_pid: int | None = None
    packet: dict[str, Any] = {
        "schemaVersion": 1,
        "launcherPid": process.pid,
        "cemuSha256": sha256(cemu),
        "simulatorManifest": str(args.simulator_manifest.resolve()),
    }
    try:
        tools = client.initialize(45.0)
        required = {
            "add_breakpoint", "remove_breakpoint", "resume", "get_registers", "get_callstack",
            "read_memory", "disassemble", "get_game_state", "bvr_get_guest_state", "stop_game",
            "start_code_probe", "stop_code_probe", "get_code_probe_hits",
        }
        missing = sorted(required.difference(tools))
        if missing:
            raise RuntimeError(f"missing debugger tools: {missing}")
        gate = wait_for_gameplay(state_path, simulator_command, args.gameplay_timeout, args.stable_frames)
        packet["gameplayGate"] = gate
        owned_cemu_pid = int(gate.get("pid", 0)) or None
        probes = capture_non_pausing_probes(client, args.probe_seconds)
        packet["nonPausingProbes"] = probes
        packet["asmMtxInverse"] = probes["asmMtxInverse"]
        packet["sceneComponentHook"] = probes["sceneComponentHook"]
        packet["uploadVirtual"] = probes["uploadVirtual"]
        inverse = packet["asmMtxInverse"]
        issues: list[str] = []
        if int(inverse.get("hitCount", 0)) < 1:
            issues.append("ASM_MTXInverse did not execute during the bounded probe window")
        elif not inverse.get("r4DistinctFromR3"):
            issues.append("ASM_MTXInverse r4 was not a distinct destination in captured calls")
        for name in ("sceneComponentHook", "uploadVirtual"):
            section = packet[name]
            if int(section.get("hitCount", 0)) < 1:
                issues.append(f"{name} produced no probe hits")
            if not section.get("bothEyesCaptured"):
                issues.append(f"{name} did not capture both current two-pass eye phases")
        packet["validation"] = {"valid": not issues, "issues": issues}
        packet["success"] = not issues
    except Exception as error:
        packet["success"] = False
        packet["error"] = f"{type(error).__name__}: {error}"
    finally:
        recover_debugger(client)
        safe_call(client, "stop_code_probe", {})
        safe_call(client, "stop_game", {}, timeout=10.0)
        if owned_cemu_pid is not None:
            terminate_pid_tree(owned_cemu_pid)
        terminate_tree(process)
        packet["finishedUtc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        for name in ("BetterVR_log.txt", "BetterVR_state.json", "BetterVR_events.csv", "log.txt"):
            copy_with_retry(launcher_dir / name, args.out / name)
        for name in ("BetterVR_settings.ini", "BetterVR_cmd.ini", "BetterVR_state.json", "BetterVR_events.csv", "BetterVR_log.txt"):
            unlink_with_retry(launcher_dir / name)
        output = args.out / "stereo_debugger_closure.json"
        output.write_text(json.dumps(packet, indent=2), encoding="utf-8")
        print(f"[debugger-closure] wrote {output}", flush=True)
    return 0 if packet.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
