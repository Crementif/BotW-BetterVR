#!/usr/bin/env python3
"""Capture a gameplay-only BetterVR/Cemu stereo uniform oracle.

The BetterVR launcher forwards ``--enable-mcp`` to Cemu and preserves its
standard handles.  This tool uses that path so the Vulkan layer, graphic pack,
OpenXR Simulator, and Cemu debugger all observe the same run.  The GX2 trace is
armed only after BetterVR reports 90 advancing, stable in-game frames.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any


TITLE_ID = "00050000101c9400"
DEFAULT_LAUNCHER_DIR = Path(r"E:\Github\cemu-mcp\bin")
DEFAULT_SIMULATOR_MANIFEST = Path(r"E:\Github\OpenXR-Simulator\bin\openxr_simulator.json")
DEFAULT_BETTERVR_LAUNCHER = Path(__file__).resolve().parents[1] / "Cemu" / "BetterVR_Launcher.exe"


def atomic_json_write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="ascii", dir=path.parent, delete=False) as handle:
        json.dump(payload, handle, separators=(",", ":"))
        temp_path = Path(handle.name)
    os.replace(temp_path, path)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def sha256(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest().upper()
    except OSError:
        return None


def find_conflicting_processes() -> list[tuple[str, str]]:
    if os.name != "nt":
        return []
    result = subprocess.run(
        ["tasklist", "/FO", "CSV", "/NH"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    conflicts: list[tuple[str, str]] = []
    for row in csv.reader(io.StringIO(result.stdout)):
        if len(row) >= 2 and row[0].lower() in {"cemu.exe", "bettervr_launcher.exe"}:
            conflicts.append((row[0], row[1]))
    return conflicts


def stage_if_different(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    if sha256(source) == sha256(destination):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


class McpClient:
    def __init__(self, process: subprocess.Popen[str]) -> None:
        if process.stdin is None or process.stdout is None:
            raise RuntimeError("MCP process was not started with standard-I/O pipes")
        self.process = process
        self.stdin = process.stdin
        self.messages: queue.Queue[dict[str, Any] | str | None] = queue.Queue()
        self.next_id = 1
        self.reader = threading.Thread(target=self._read_stdout, args=(process.stdout,), daemon=True)
        self.reader.start()

    def _read_stdout(self, stdout: Any) -> None:
        for line in stdout:
            text = line.strip()
            if not text:
                continue
            try:
                self.messages.put(json.loads(text))
            except ValueError:
                self.messages.put(text)
        self.messages.put(None)

    def request(self, method: str, params: dict[str, Any], timeout: float = 30.0) -> dict[str, Any]:
        request_id = self.next_id
        self.next_id += 1
        payload = {"jsonrpc": "2.0", "id": request_id, "method": method, "params": params}
        self.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.stdin.flush()

        deadline = time.monotonic() + timeout
        diagnostics: list[str] = []
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(f"launcher/Cemu exited with code {self.process.returncode}")
            try:
                message = self.messages.get(timeout=min(0.25, max(0.01, deadline - time.monotonic())))
            except queue.Empty:
                continue
            if message is None:
                raise RuntimeError("Cemu MCP stdout closed")
            if isinstance(message, str):
                diagnostics.append(message)
                continue
            if message.get("id") != request_id:
                continue
            if "error" in message:
                raise RuntimeError(f"MCP {method} failed: {message['error']}")
            return message.get("result", {})
        detail = f"; non-JSON stdout: {diagnostics[-3:]}" if diagnostics else ""
        raise TimeoutError(f"MCP {method} timed out after {timeout:.1f}s{detail}")

    def initialize(self, timeout: float) -> list[str]:
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                self.request(
                    "initialize",
                    {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {},
                        "clientInfo": {"name": "bettervr-stereo-oracle", "version": "1"},
                    },
                    timeout=min(10.0, max(1.0, deadline - time.monotonic())),
                )
                self.stdin.write('{"jsonrpc":"2.0","method":"notifications/initialized","params":{}}\n')
                self.stdin.flush()
                result = self.request("tools/list", {}, timeout=20.0)
                return [tool.get("name", "") for tool in result.get("tools", [])]
            except (BrokenPipeError, RuntimeError, TimeoutError) as error:
                last_error = error
                if self.process.poll() is not None:
                    break
                time.sleep(0.5)
        raise RuntimeError(f"Cemu MCP did not become ready: {last_error}")

    def call_tool(self, name: str, arguments: dict[str, Any], timeout: float = 60.0) -> Any:
        result = self.request("tools/call", {"name": name, "arguments": arguments}, timeout=timeout)
        if result.get("isError"):
            raise RuntimeError(f"MCP tool {name} failed: {result.get('content')}")
        content = result.get("content", [])
        if not content:
            return None
        text = content[0].get("text", "")
        try:
            return json.loads(text)
        except ValueError:
            return text


def write_settings(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "[BetterVR][Settings]",
                "BootDirectlyIntoGame=true",
                f"BootDirectlyTitleId={TITLE_ID}",
                "SynthesizedRightEye=false",
                "SkipDrcRendering=true",
                "TutorialPromptShown=true",
                "PerformanceOverlay=DISABLE",
                "LogRendering=false",
                "RightEyeCalcSkipMask=0",
                "",
            ]
        ),
        encoding="ascii",
    )


def is_stable_gameplay(state: dict[str, Any]) -> bool:
    try:
        frame = int(state["frame"])
        left = state["eyeL"]
        right = state["eyeR"]
        return (
            int(state.get("inGame", 0)) == 1
            and int(state.get("fadeVisible", 1)) == 0
            and left.get("state") == "NORMAL"
            and right.get("state") == "NORMAL"
            and int(left.get("lastSampleFrame", -1000)) >= frame - 12
            and int(right.get("lastSampleFrame", -1000)) >= frame - 12
        )
    except (KeyError, TypeError, ValueError):
        return False


def wait_for_gameplay(
    state_path: Path,
    simulator_command: Path,
    timeout: float,
    stable_frames: int,
) -> dict[str, Any]:
    started = time.monotonic()
    press_schedule = [75.0, 85.0, 95.0, 105.0, 115.0]
    pressed: set[float] = set()
    stable_start: int | None = None
    last_report = -1

    while time.monotonic() - started < timeout:
        elapsed = time.monotonic() - started
        for press_at in press_schedule:
            if elapsed >= press_at and press_at not in pressed:
                atomic_json_write(simulator_command, {"hand": 1, "trigger": 1.0})
                time.sleep(0.7)
                atomic_json_write(simulator_command, {"hand": 1, "trigger": 0.0})
                pressed.add(press_at)
                print(f"[oracle] pressed A through simulator at t={elapsed:.0f}s", flush=True)

        state = read_json(state_path)
        if state and is_stable_gameplay(state):
            frame = int(state["frame"])
            if stable_start is None or frame < stable_start:
                stable_start = frame
            if frame - stable_start >= stable_frames:
                print(f"[oracle] gameplay gate passed at frame {frame} after {frame - stable_start} stable frames", flush=True)
                return state
        else:
            stable_start = None

        report_second = int(elapsed) // 15
        if report_second != last_report:
            last_report = report_second
            status = "stable" if state and is_stable_gameplay(state) else "waiting"
            frame = state.get("frame") if state else None
            print(f"[oracle] {status} for gameplay t={elapsed:.0f}s frame={frame}", flush=True)
        time.sleep(0.25)
    raise TimeoutError(f"stable gameplay was not reached within {timeout:.0f}s")


def terminate_tree(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    else:
        process.terminate()
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.kill()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launcher-dir", type=Path, default=DEFAULT_LAUNCHER_DIR)
    parser.add_argument("--cemu-source", type=Path, default=None)
    parser.add_argument("--bettervr-launcher-source", type=Path, default=DEFAULT_BETTERVR_LAUNCHER)
    parser.add_argument("--simulator-manifest", type=Path, default=DEFAULT_SIMULATOR_MANIFEST)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gameplay-timeout", type=float, default=180.0)
    parser.add_argument("--mcp-timeout", type=float, default=45.0)
    parser.add_argument("--stable-frames", type=int, default=90)
    parser.add_argument("--capture-seconds", type=float, default=0.5)
    parser.add_argument("--max-draws", type=int, default=250_000)
    parser.add_argument("--max-uniform-bytes", type=int, default=65_536)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    launcher_dir = args.launcher_dir.resolve()
    launcher = launcher_dir / "BetterVR_Launcher.exe"
    cemu = launcher_dir / "Cemu.exe"
    conflicts = find_conflicting_processes()
    if conflicts:
        description = ", ".join(f"{name} pid={pid}" for name, pid in conflicts)
        raise RuntimeError(f"refusing to start beside an existing Cemu/BetterVR session: {description}")

    cemu_source = args.cemu_source.resolve() if args.cemu_source else launcher_dir / "Cemu_release.exe"
    stage_if_different(cemu_source, cemu)
    stage_if_different(args.bettervr_launcher_source.resolve(), launcher)
    state_path = launcher_dir / "BetterVR_state.json"
    settings_path = launcher_dir / "BetterVR_settings.ini"
    simulator_command = Path(os.environ.get("LOCALAPPDATA", "")) / "OpenXR-Simulator" / "controller_pose_command.json"

    for required in (launcher, cemu, args.simulator_manifest):
        if not required.is_file():
            raise FileNotFoundError(required)
    if not os.environ.get("LOCALAPPDATA"):
        raise RuntimeError("LOCALAPPDATA is not set")

    args.out.mkdir(parents=True, exist_ok=True)
    write_settings(settings_path)
    for stale in (state_path, launcher_dir / "BetterVR_cmd.ini", launcher_dir / "BetterVR_events.csv"):
        stale.unlink(missing_ok=True)

    environment = os.environ.copy()
    environment["XR_RUNTIME_JSON"] = str(args.simulator_manifest.resolve())
    creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
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
        creationflags=creation_flags,
    )
    client = McpClient(process)
    trace_started = False
    packet: dict[str, Any] = {
        "schemaVersion": 1,
        "launcherDir": str(launcher_dir),
        "launcherPid": process.pid,
        "cemuSha256": sha256(cemu),
        "launcherSha256": sha256(launcher),
        "simulatorManifest": str(args.simulator_manifest.resolve()),
        "simulatorManifestSha256": sha256(args.simulator_manifest),
        "traceConfig": {
            "maxDraws": args.max_draws,
            "maxUniformBytes": args.max_uniform_bytes,
            "captureSeconds": args.capture_seconds,
            "stableFramesRequired": args.stable_frames,
        },
    }

    try:
        tools = client.initialize(args.mcp_timeout)
        required_tools = {"bvr_begin_draw_trace", "bvr_end_draw_trace", "bvr_get_eye_diff", "bvr_get_uniform_diff"}
        missing = sorted(required_tools.difference(tools))
        if missing:
            raise RuntimeError(f"Cemu does not expose required stereo-oracle tools: {missing}")
        packet["mcpToolCount"] = len(tools)

        gate_state = wait_for_gameplay(state_path, simulator_command, args.gameplay_timeout, args.stable_frames)
        packet["gameplayGate"] = gate_state

        packet["traceBegin"] = client.call_tool(
            "bvr_begin_draw_trace",
            {
                "max_draws": args.max_draws,
                "capture_uniforms": True,
                "max_uniform_bytes": args.max_uniform_bytes,
            },
        )
        trace_started = True
        capture_deadline = time.monotonic() + args.capture_seconds
        while time.monotonic() < capture_deadline:
            state = read_json(state_path)
            if not state or not is_stable_gameplay(state):
                raise RuntimeError("gameplay gate became invalid during the trace window")
            time.sleep(0.1)

        packet["traceEnd"] = client.call_tool("bvr_end_draw_trace", {})
        trace_started = False
        packet["eyeDiff"] = client.call_tool("bvr_get_eye_diff", {"max_mismatches": 64}, timeout=120.0)
        packet["uniformDiff"] = client.call_tool("bvr_get_uniform_diff", {"max_ranges": 64}, timeout=120.0)
        packet["queueTail"] = client.call_tool("bvr_get_queue_snapshot", {"recent": 64}, timeout=120.0)
        packet["finalState"] = read_json(state_path)

        # Fail closed: an output file is not a valid oracle merely because the tool
        # calls returned. Require a gameplay-only, balanced PM4 marker corpus and
        # healthy, distinct final OpenXR eyes.
        issues: list[str] = []
        eye_diff = packet.get("eyeDiff", {})
        uniform_diff = packet.get("uniformDiff", {})
        trace_end = packet.get("traceEnd", {})
        final_state = packet.get("finalState") or {}
        captured = max(1, int(trace_end.get("captured", 0)))
        if int(trace_end.get("dropped", 0)) != 0:
            issues.append("trace overflowed; shorten the capture or raise max_draws")
        if int(eye_diff.get("eyeMarkers", 0)) < 2:
            issues.append("fewer than two PM4 eye markers were captured")
        if int(uniform_diff.get("pairedEyePasses", 0)) < 1:
            issues.append("no left/right PM4 pass pair was formed")
        if int(uniform_diff.get("pairedDraws", 0)) < 1:
            issues.append("no left/right draw pair was formed")
        if int(eye_diff.get("unknownPhaseDraws", 0)) / captured > 0.02:
            issues.append("more than 2% of captured draws lack a PM4 eye tag")
        if not is_stable_gameplay(final_state):
            issues.append("final state is not stable gameplay with two NORMAL eyes")
        try:
            if float(final_state["stereo"]["meanAbsDifference"]) < 0.002:
                issues.append("final OpenXR eyes are effectively identical")
        except (KeyError, TypeError, ValueError):
            issues.append("final stereo disparity metric is missing")
        packet["validation"] = {
            "valid": not issues,
            "issues": issues,
            "requirements": {
                "gameplayOnly": True,
                "noTraceOverflow": True,
                "pm4EyeMarkers": True,
                "pairedDraws": True,
                "normalDistinctFinalEyes": True,
            },
        }
        if issues:
            raise RuntimeError("oracle validation failed: " + "; ".join(issues))
        packet["success"] = True
    except Exception as error:
        packet["success"] = False
        packet["error"] = f"{type(error).__name__}: {error}"
        if trace_started:
            try:
                packet["traceEndAfterError"] = client.call_tool("bvr_end_draw_trace", {}, timeout=10.0)
            except Exception as stop_error:
                packet["traceStopError"] = f"{type(stop_error).__name__}: {stop_error}"
    finally:
        packet["finishedUtc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        output_path = args.out / "stereo_oracle.json"
        output_path.write_text(json.dumps(packet, indent=2), encoding="utf-8")
        for name in ("BetterVR_log.txt", "BetterVR_state.json", "BetterVR_events.csv", "log.txt"):
            source = launcher_dir / name
            if source.is_file():
                shutil.copy2(source, args.out / name)
        try:
            client.call_tool("stop_game", {}, timeout=10.0)
        except Exception:
            pass
        terminate_tree(process)
        for runtime_name in ("BetterVR_settings.ini", "BetterVR_cmd.ini", "BetterVR_state.json", "BetterVR_events.csv", "BetterVR_log.txt"):
            (launcher_dir / runtime_name).unlink(missing_ok=True)
        print(f"[oracle] wrote {output_path}", flush=True)

    if packet.get("success"):
        uniform = packet.get("uniformDiff", {})
        print(
            "[oracle] success: "
            f"paired={uniform.get('pairedDraws')} "
            f"uniform-different={uniform.get('pairsWithUniformContentDifference')} "
            f"unpaired-L/R={uniform.get('unpairedLeft')}/{uniform.get('unpairedRight')}",
            flush=True,
        )
        return 0
    print(f"[oracle] failed: {packet.get('error')}", file=sys.stderr, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
