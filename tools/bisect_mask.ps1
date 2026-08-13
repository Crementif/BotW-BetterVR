# Automated live bisect of the right-eye calc-skip mask.
# Requires a RUNNING in-game instance (start one with: .\bvr_harness.ps1 -ConfigName live -KeepAlive)
# or pass -Boot to launch one first. Flips mask bits via runtime IPC (no reboots) and
# measures white/black frames per mask via the mod's eye telemetry.
#
#   .\bisect_mask.ps1                       # default ladder: each skip bit alone + combos
#   .\bisect_mask.ps1 -Masks 0x2002,0x2046  # explicit list
param(
    [string[]]$Masks = @(),
    [int]$HoldSec = 8,
    [switch]$Boot,
    [string]$LauncherDir = "E:\Cemu_2.6",
    [string]$OutDir = "$PSScriptRoot\..\bench_out"
)

$ErrorActionPreference = "Continue"
New-Item -ItemType Directory -Force $OutDir | Out-Null
. "$PSScriptRoot\bvr_ipc.ps1"
Set-BvrDir $LauncherDir

if ($Boot) {
    & "$PSScriptRoot\bvr_harness.ps1" -ConfigName "bisect-boot" -KeepAlive -SkipMask 0 -LauncherDir $LauncherDir
    if ($LASTEXITCODE -ne 0) { Write-Output "boot failed"; exit 1 }
}

$state = Get-BvrState
if (-not $state) { Write-Output "ABORT: no BetterVR_state.json - is a BetterVR instance running?"; exit 1 }
if ($state.inGame -ne 1) { Write-Output "WARNING: state file says not in-game; results may be meaningless" }

if ($Masks.Count -eq 0) {
    # bit 0 = DRC skip (always on), bit 13 = queue-preserve companion.
    # Ladder: control, queue-preserve alone, then each calc-skip bit with the companion.
    $Masks = @("0x0001", "0x2001") + (1..12 | ForEach-Object { "0x{0:X4}" -f ((1 -shl $_) -bor 0x2001) })
}

$results = @()
foreach ($maskText in $Masks) {
    $mask = [Convert]::ToUInt32($maskText, 16)

    # settle on control mask between candidates so effects don't bleed across
    $seq = Send-BvrCommand @{ skipMask = 1; marker = "bisect-settle" }
    if (-not (Wait-BvrApplied $seq 20)) { Write-Output "ABORT: mod stopped acking commands (crashed?)"; break }
    Start-Sleep -Seconds 3

    # known failure mode: a bad mask can corrupt the draw-list state PERMANENTLY (the
    # screen stays white on the control mask). Later measurements would be garbage.
    $settleCheck = Measure-BvrWhiteFrames 3
    if (-not $settleCheck -or $settleCheck.stateL -ne 'NORMAL' -or $settleCheck.stateR -ne 'NORMAL' -or
        $settleCheck.badPct -gt 1 -or $settleCheck.flickerEvents -gt 0) {
        Write-Output "ABORT: instance did not recover on the control mask (permanent draw-list corruption) - remaining masks need a fresh boot"
        $results += [pscustomobject]@{ mask = $maskText; verdict = "NOT-RUN (instance corrupted)" }
        break
    }

    $seq = Send-BvrCommand @{ skipMask = $mask; marker = "bisect-$maskText" }
    if (-not (Wait-BvrApplied $seq 20)) { Write-Output "ABORT: mod stopped acking commands at $maskText"; break }
    Start-Sleep -Seconds 2   # let the new mask propagate through the pipelined frame loop

    $measurement = Measure-BvrWhiteFrames $HoldSec
    if (-not $measurement) {
        $results += [pscustomobject]@{ mask = $maskText; verdict = "NO-STATE (crash?)" }
        Write-Output ("{0}  NO STATE - instance gone?" -f $maskText)
        if (-not (Get-Process -Name Cemu -ErrorAction SilentlyContinue)) { Write-Output "Cemu is dead - stopping bisect"; break }
        continue
    }

    $badFrames = $measurement.whiteL + $measurement.whiteR + $measurement.blackL + $measurement.blackR
    # Both eyes contribute samples, so the denominator must be both-eye samples too.
    # The old one-frame denominator could report an impossible 200% bad rate.
    $totalFrames = [math]::Max(1, $measurement.sampledL + $measurement.sampledR)
    $badPercent = [math]::Round(100.0 * $badFrames / $totalFrames, 1)
    $verdict = if ($badPercent -gt 10) { "FLICKER" } elseif ($badPercent -gt 1) { "MARGINAL" } else { "CLEAN" }

    $results += [pscustomobject]@{
        mask = $maskText; verdict = $verdict; badPct = $badPercent
        whiteL = $measurement.whiteL; whiteR = $measurement.whiteR
        blackL = $measurement.blackL; blackR = $measurement.blackR
        frames = $measurement.framesElapsed
    }
    Write-Output ("{0}  {1,-8} bad={2,5}%  whiteL={3} whiteR={4} blackL={5} blackR={6} ({7} frames)" -f `
        $maskText, $verdict, $badPercent, $measurement.whiteL, $measurement.whiteR, $measurement.blackL, $measurement.blackR, $measurement.framesElapsed)
}

# restore control mask
Send-BvrCommand @{ skipMask = 1; marker = "bisect-done" } | Out-Null

$results | ConvertTo-Json | Set-Content "$OutDir\bisect_$((Get-Date).ToString('HHmmss')).json" -Encoding ascii
Write-Output ""
Write-Output "=== bisect summary ==="
$results | Format-Table -AutoSize | Out-String | Write-Output
$flickering = @($results | Where-Object { $_.verdict -eq "FLICKER" } | ForEach-Object { $_.mask })
Write-Output ("Flickering masks: {0}" -f ($(if ($flickering.Count) { $flickering -join ", " } else { "none" })))
