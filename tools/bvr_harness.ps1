# BetterVR automated test harness (OpenXR Simulator).
# Boots Cemu through the BetterVR launcher, drives the title menu into gameplay via
# simulated controller input, and produces a machine-readable verdict JSON.
#
# Examples:
#   .\bvr_harness.ps1 -ConfigName baseline
#   .\bvr_harness.ps1 -ConfigName skiptest -SkipMask 8262 -DurationSec 420
#   .\bvr_harness.ps1 -ConfigName live -KeepAlive        # boot to gameplay, leave running
param(
    [Parameter(Mandatory=$true)][string]$ConfigName,
    [string]$Synth = "false",
    [string]$SkipDrc = "true",
    [int]$DurationSec = 420,
    [string]$LogRendering = "false",
    [string]$SkipMask = "0",
    [switch]$KeepAlive,           # boot to gameplay then exit, leaving Cemu running
    [switch]$NoBoot,              # skip launch; only produce a verdict from an existing log
    [switch]$TakeOwnership,       # explicitly stop a pre-existing Cemu/launcher session
    [string]$LauncherDir = "E:\Cemu_2.6",
    [string]$OutDir = "$PSScriptRoot\..\bench_out"
)

$ErrorActionPreference = "Continue"
New-Item -ItemType Directory -Force $OutDir | Out-Null
$OutDir = (Resolve-Path $OutDir).Path
$simDir = "$env:LOCALAPPDATA\OpenXR-Simulator"
. "$PSScriptRoot\bvr_ipc.ps1"
Set-BvrDir $LauncherDir

# ---------------------------------------------------------------- single instance
$lockFile = "$LauncherDir\bvr_harness.lock"
if (-not $NoBoot) {
    if (Test-Path $lockFile) {
        $lockRaw = Get-Content $lockFile -Raw -ErrorAction SilentlyContinue
        $lockPids = @()
        try {
            $lock = $lockRaw | ConvertFrom-Json -ErrorAction Stop
            $lockPids = @($lock.harnessPid, $lock.launcherPid) + @($lock.cemuPids) | Where-Object { $_ }
        } catch {
            if ($lockRaw -match '^\s*\d+\s*$') { $lockPids = @([int]$lockRaw) }
        }
        $liveLockPids = @($lockPids | Where-Object { Get-Process -Id $_ -ErrorAction SilentlyContinue })
        if ($liveLockPids.Count -gt 0 -and -not $TakeOwnership) {
            Write-Output "ABORT: another owned session (pid(s) $($liveLockPids -join ', ')) is active. Use -TakeOwnership only if safe."
            exit 2
        }
    }
    # A dying launcher deletes the graphic pack from under a new run. Existing work is
    # never called "stale" and killed implicitly; takeover must be explicit.
    $existing = @(Get-Process -Name Cemu,BetterVR_Launcher -ErrorAction SilentlyContinue)
    if ($existing.Count -gt 0 -and -not $TakeOwnership) {
        Write-Output "ABORT: an existing Cemu/BetterVR session owns pid(s) $($existing.Id -join ', '). Use -TakeOwnership only if it is safe to stop."
        exit 2
    }
    if ($existing.Count -gt 0) {
        Write-Output "Taking ownership and stopping pid(s): $($existing.Id -join ', ')"
        $existing | Stop-Process -Force
        Start-Sleep -Seconds 5
    }
    [ordered]@{ schemaVersion=2; harnessPid=$PID; launcherPid=$null; cemuPids=@(); keepAlive=$false; created=(Get-Date).ToString('o') } |
        ConvertTo-Json | Set-Content $lockFile -Encoding ascii
}

function Get-VerdictFromLog {
    param([string]$logPath, [bool]$ingameReached, [array]$simStats)

    $verdict = [ordered]@{
        config = $ConfigName
        timestamp = (Get-Date).ToString("s")
        skipMask = $SkipMask
        synth = $Synth
        skipDrc = $SkipDrc
        ingameReached = $ingameReached
        crashed = $false
        exitCode = $null
        benchWindows = 0
        lastBench = $null
        telemetryTransitions = @()
        white = $null
        simFps = @($simStats | Select-Object -Last 20)
        logPath = $logPath
    }

    if (Test-Path $logPath) {
        $exitLine = Select-String -Path $logPath -Pattern "Cemu exited with code (\d+)" | Select-Object -First 1
        if ($exitLine) {
            $code = [int64]$exitLine.Matches[0].Groups[1].Value
            $verdict.exitCode = "0x{0:X}" -f $code
            $verdict.crashed = ($code -ne 0)
        }

        $benchLines = @(Select-String -Path $logPath -Pattern "\[bench\]" | ForEach-Object { $_.Line })
        $verdict.benchWindows = $benchLines.Count
        if ($benchLines.Count -gt 0) {
            $last = $benchLines[-1]
            if ($last -match "avg frame ([\d.]+) ms \(([\d.]+) FPS\), avg work ([\d.]+) ms.*whiteL=(\d+) whiteR=(\d+)") {
                $verdict.lastBench = [ordered]@{
                    frameMs = [double]$Matches[1]; fps = [double]$Matches[2]; workMs = [double]$Matches[3]
                }
                $verdict.white = [ordered]@{ L = [int]$Matches[4]; R = [int]$Matches[5] }
            }
            elseif ($last -match "avg frame ([\d.]+) ms \(([\d.]+) FPS\), avg work ([\d.]+) ms") {
                $verdict.lastBench = [ordered]@{
                    frameMs = [double]$Matches[1]; fps = [double]$Matches[2]; workMs = [double]$Matches[3]
                }
            }
        }

        $verdict.telemetryTransitions = @(Select-String -Path $logPath -Pattern "\[telemetry\] eye=" |
            ForEach-Object { $_.Line -replace "^.*\[telemetry\] ", "" } | Select-Object -Last 30)
    }

    return $verdict
}

if ($NoBoot) {
    $log = "$OutDir\bench_$ConfigName.log"
    $verdict = Get-VerdictFromLog $log $false @()
    $verdict | ConvertTo-Json -Depth 5 | Set-Content "$OutDir\verdict_$ConfigName.json" -Encoding ascii
    Write-Output ($verdict | ConvertTo-Json -Depth 5)
    exit 0
}

# ---------------------------------------------------------------- seed settings
@"
[BetterVR][Settings]
BootDirectlyIntoGame=true
BootDirectlyTitleId=00050000101c9400
SynthesizedRightEye=$Synth
SkipDrcRendering=$SkipDrc
TutorialPromptShown=true
PerformanceOverlay=DISABLE
LogRendering=$LogRendering
RightEyeCalcSkipMask=$SkipMask
"@ | Set-Content "$LauncherDir\BetterVR_settings.ini" -Encoding ascii

Remove-Item "$LauncherDir\BetterVR_log.txt","$LauncherDir\BetterVR_cmd.ini","$LauncherDir\BetterVR_state.json","$LauncherDir\BetterVR_events.csv" -Force -ErrorAction SilentlyContinue

$env:XR_RUNTIME_JSON = "E:\Github\OpenXR-Simulator\bin\openxr_simulator.json"

$proc = Start-Process -FilePath "$LauncherDir\BetterVR_Launcher.exe" -WorkingDirectory $LauncherDir -PassThru
[ordered]@{ schemaVersion=2; harnessPid=$PID; launcherPid=$proc.Id; cemuPids=@(); keepAlive=$false; created=(Get-Date).ToString('o') } |
    ConvertTo-Json | Set-Content $lockFile -Encoding ascii
Write-Output "Launched (pid $($proc.Id)) config=$ConfigName mask=$SkipMask synth=$Synth keepAlive=$KeepAlive"

Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class Win32b {
    [DllImport("user32.dll", SetLastError=true)] public static extern IntPtr FindWindowA(string cls, string title);
    [DllImport("user32.dll")] public static extern bool PostMessageA(IntPtr hWnd, uint msg, IntPtr w, IntPtr l);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
}
'@

function Press-A {
    '{"hand": 1, "trigger": 1.0}' | Set-Content "$simDir\controller_pose_command.json" -Encoding ascii
    Start-Sleep -Milliseconds 700
    '{"hand": 1, "trigger": 0.0}' | Set-Content "$simDir\controller_pose_command.json" -Encoding ascii
}

$dialogTitles = @("BetterVR Update Available", "BetterVR Launcher - Outdated Cemu", "BetterVR Launcher - FPS++ Required", "Launch SteamVR?", "Graphic pack error")
$start = Get-Date
$minimized = $false
$pressTimes = @(75, 85, 95, 105, 115)
$pressed = @{}
$simStats = @()
$lastSimSample = $null
$ingameReached = $false
$ingameMarked = $false

while (((Get-Date) - $start).TotalSeconds -lt $DurationSec) {
    $elapsed = ((Get-Date) - $start).TotalSeconds

    if ($proc.HasExited -and $elapsed -gt 30) {
        Write-Output ("Launcher exited early (t={0:n0}s) - aborting run" -f $elapsed)
        break
    }

    foreach ($title in $dialogTitles) {
        $hwnd = [Win32b]::FindWindowA($null, $title)
        if ($hwnd -ne [IntPtr]::Zero) {
            Write-Output "Dismissing dialog: $title"
            [Win32b]::PostMessageA($hwnd, 0x0010, [IntPtr]::Zero, [IntPtr]::Zero) | Out-Null
        }
    }

    if (-not $minimized -and $elapsed -gt 35) {
        Get-Process Cemu -ErrorAction SilentlyContinue | ForEach-Object {
            if ($_.MainWindowHandle -ne [IntPtr]::Zero) {
                [Win32b]::ShowWindow($_.MainWindowHandle, 6) | Out-Null  # SW_MINIMIZE
                Write-Output "Minimized Cemu window"
                $script:minimized = $true
            }
        }
    }

    foreach ($t in $pressTimes) {
        if ($elapsed -gt $t -and -not $pressed[$t]) {
            $pressed[$t] = $true
            Write-Output ("Pressing A (t={0:n0}s)" -f $elapsed)
            Press-A
        }
    }

    # positive in-game detection via the mod's state file
    if (-not $ingameMarked -and $elapsed -gt 100) {
        $state = Get-BvrState
        if ($state -and $state.inGame -eq 1 -and $state.fadeVisible -eq 0 -and
            $state.eyeL.state -eq "NORMAL" -and $state.eyeR.state -eq "NORMAL" -and
            $state.eyeL.lastSampleFrame -ge ($state.frame - 12) -and $state.eyeR.lastSampleFrame -ge ($state.frame - 12)) {
            $ingameReached = $true
            $ingameMarked = $true
            $epochId = [int][DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
            Send-BvrCommand @{ marker = "harness-ingame-t$([int]$elapsed)"; epoch = $epochId } | Out-Null
            Write-Output ("IN-GAME confirmed by state file (t={0:n0}s) eyeL={1} eyeR={2}" -f $elapsed, $state.eyeL.state, $state.eyeR.state)
            if ($KeepAlive) { break }
        }
    }

    try {
        $status = Get-Content "$simDir\runtime_status.json" -Raw -ErrorAction Stop | ConvertFrom-Json
        $now = Get-Date
        if ($lastSimSample -ne $null -and $status.frame_count -gt $lastSimSample.count) {
            $dt = ($now - $lastSimSample.time).TotalSeconds
            if ($dt -gt 2) {
                $fps = ($status.frame_count - $lastSimSample.count) / $dt
                $simStats += [pscustomobject]@{ t = [int]$elapsed; fps = [math]::Round($fps, 1) }
                $script:lastSimSample = @{ count = $status.frame_count; time = $now }
            }
        }
        else {
            $script:lastSimSample = @{ count = $status.frame_count; time = $now }
        }
    } catch {}

    Start-Sleep -Seconds 3
}

if ($KeepAlive -and $ingameReached) {
    $state = Get-BvrState
    $cemuPids = @(Get-Process -Name Cemu -ErrorAction SilentlyContinue | ForEach-Object Id)
    [ordered]@{ schemaVersion=2; harnessPid=$null; launcherPid=$proc.Id; cemuPids=$cemuPids; keepAlive=$true; sessionId=$state.sessionId; created=$start.ToString('o') } |
        ConvertTo-Json | Set-Content $lockFile -Encoding ascii
    Write-Output "KEEP-ALIVE: owned session left in-game. Use bvr_ipc.ps1 to interact; stop with tools\stop_bvr_session.ps1."
    if ($state) { Write-Output ("state: frame={0} mask={1} eyeL={2} eyeR={3}" -f $state.frame, $state.effectiveMask, $state.eyeL.state, $state.eyeR.state) }
    exit 0
}

if ($KeepAlive -and -not $ingameReached) {
    Write-Output "KEEP-ALIVE requested but in-game was never confirmed; killing and producing a verdict instead"
}

$ownedCemu = @(Get-Process -Name Cemu -ErrorAction SilentlyContinue | Where-Object { $_.StartTime -ge $start.AddSeconds(-2) })
$ownedCemu | Stop-Process -Force -ErrorAction SilentlyContinue
Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 3
Remove-Item $lockFile -Force -ErrorAction SilentlyContinue

$log = "$OutDir\bench_$ConfigName.log"
Copy-Item "$LauncherDir\BetterVR_log.txt" $log -Force -ErrorAction SilentlyContinue
Copy-Item "$LauncherDir\BetterVR_events.csv" "$OutDir\events_$ConfigName.csv" -Force -ErrorAction SilentlyContinue

$verdict = Get-VerdictFromLog $log $ingameReached $simStats
$verdict | ConvertTo-Json -Depth 5 | Set-Content "$OutDir\verdict_$ConfigName.json" -Encoding ascii

Write-Output "=== verdict ($ConfigName) ==="
Write-Output ($verdict | ConvertTo-Json -Depth 5)
