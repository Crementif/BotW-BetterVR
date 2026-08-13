param(
    [string]$LauncherDir = 'E:\Cemu_2.6',
    [string]$ScenarioFile = "$PSScriptRoot\right_eye_scenarios.json",
    [string]$OutDir = "$PSScriptRoot\..\bench_out\right_eye_lab",
    [int]$Blocks = 2,
    [int]$TrialSec = 20,
    [int]$SettleSec = 4,
    [switch]$Resume,
    [switch]$NoIncidentCapture
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\bvr_ipc.ps1"
Set-BvrDir $LauncherDir

function Write-JsonAtomic([object]$Value, [string]$Path) {
    $temp = "$Path.$PID.tmp"
    $Value | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $temp -Encoding utf8
    Move-Item -LiteralPath $temp -Destination $Path -Force
}

function Get-Sha256Text([string]$Text) {
    $bytes = [Text.Encoding]::UTF8.GetBytes($Text)
    $hash = [Security.Cryptography.SHA256]::Create().ComputeHash($bytes)
    return -join ($hash | ForEach-Object { $_.ToString('x2') })
}

function Invoke-BvrConfiguration([object]$Config, [int]$Epoch, [string]$Marker) {
    $command = @{ epoch = $Epoch; marker = $Marker; traceEvents = $true; telemetryLevel = 2 }
    foreach ($key in @('skipMask','rightEyeReuse','synthRightEye','skipDrc')) {
        if ($null -ne $Config.$key) { $command[$key] = $Config.$key }
    }
    $seq = Send-BvrCommand $command
    if (-not (Wait-BvrApplied $seq 20)) { throw "Command $seq failed equality/session/convergence acknowledgement for $Marker" }
}

function Test-BvrRecovery([object]$Control, [int]$Epoch, [string]$Marker) {
    Invoke-BvrConfiguration $Control $Epoch $Marker
    Start-Sleep -Seconds $SettleSec
    $check = Measure-BvrFlicker 4
    if (-not $check) { throw 'Recovery produced no fresh progressing samples' }
    if ($check.stateL -ne 'NORMAL' -or $check.stateR -ne 'NORMAL' -or $check.badPct -gt 1 -or $check.flickerEvents -gt 0) {
        throw "Recovery invariant failed: L=$($check.stateL) R=$($check.stateR) bad=$($check.badPct)% flicker=$($check.flickerEvents)"
    }
    return $check
}

$initial = Get-BvrState
if (-not $initial) { throw "No fresh BetterVR v2 session in $LauncherDir. Boot with bvr_harness.ps1 -KeepAlive first." }
if ($initial.inGame -ne 1 -or $initial.eyeL.state -ne 'NORMAL' -or $initial.eyeR.state -ne 'NORMAL') {
    throw "Lab requires stable gameplay and two NORMAL final outputs (got inGame=$($initial.inGame), L=$($initial.eyeL.state), R=$($initial.eyeR.state))"
}
if (-not $initial.ppc.abiValid) { throw 'The running build does not expose the BVR2 guest ABI' }

$scenario = Get-Content -LiteralPath $ScenarioFile -Raw | ConvertFrom-Json
$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$runRoot = if ($Resume) {
    Get-ChildItem -LiteralPath $OutDir -Directory -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
} else { $null }
if (-not $runRoot) { $runRoot = Join-Path $OutDir ("run_$stamp") }
New-Item -ItemType Directory -Path $runRoot -Force | Out-Null
$resultsPath = Join-Path $runRoot 'trials.json'
$results = @()
if ($Resume -and (Test-Path $resultsPath)) { $results = @(Get-Content $resultsPath -Raw | ConvertFrom-Json) }
$completed = @{}; foreach ($result in $results) { $completed[$result.trialId] = $true }

$repo = Split-Path $PSScriptRoot -Parent
$diff = (& git -C $repo diff --binary | Out-String)
$manifest = [ordered]@{
    schemaVersion = 1; runId = Split-Path $runRoot -Leaf; started = (Get-Date).ToString('o')
    sessionId = $initial.sessionId; pid = $initial.pid; launcherDir = [IO.Path]::GetFullPath($LauncherDir)
    scenarioFile = [IO.Path]::GetFullPath($ScenarioFile); blocks = $Blocks; trialSec = $TrialSec; settleSec = $SettleSec
    gitCommit = (& git -C $repo rev-parse HEAD).Trim(); gitDiffSha256 = Get-Sha256Text $diff
    layerSha256 = if (Test-Path "$LauncherDir\BetterVR_Layer.dll") { (Get-FileHash "$LauncherDir\BetterVR_Layer.dll" -Algorithm SHA256).Hash } else { $null }
    cemuSha256 = if (Test-Path "$LauncherDir\Cemu.exe") { (Get-FileHash "$LauncherDir\Cemu.exe" -Algorithm SHA256).Hash } else { $null }
    simulatorManifestSha256 = if (Test-Path 'E:\Github\OpenXR-Simulator\bin\openxr_simulator.json') { (Get-FileHash 'E:\Github\OpenXR-Simulator\bin\openxr_simulator.json').Hash } else { $null }
    initialState = $initial
}
Write-JsonAtomic $manifest (Join-Path $runRoot 'manifest.json')

$epoch = [int]($initial.epoch.id + 1)
$halted = $false
try {
    foreach ($candidate in $scenario.candidates) {
        for ($block = 1; $block -le $Blocks; $block++) {
            # ABBA counters slow drift and simulator warm-up without requiring restarts.
            $order = @($scenario.control, $candidate, $candidate, $scenario.control)
            for ($position = 0; $position -lt $order.Count; $position++) {
                $config = $order[$position]
                $trialId = "$($candidate.name)-b$block-p$($position + 1)-$($config.name)"
                if ($completed[$trialId]) { Write-Output "RESUME skip $trialId"; continue }
                $current = Get-BvrState
                if (-not $current -or $current.sessionId -ne $initial.sessionId) { throw 'BetterVR session changed or heartbeat stopped' }
                Invoke-BvrConfiguration $config $epoch $trialId
                Start-Sleep -Seconds $SettleSec
                $measurement = Measure-BvrFlicker $TrialSec
                if (-not $measurement) { throw "$trialId returned no fresh/progressing final-output samples" }
                $before, $after = $measurement.before, $measurement.after
                $result = [ordered]@{
                    trialId = $trialId; candidate = $candidate.name; config = $config.name; block = $block; position = $position + 1; epoch = $epoch
                    verdict = if ($measurement.badPct -gt 1 -or $measurement.flickerEvents -gt 0 -or $measurement.asymmetricFlickerEvents -gt 0 -or $measurement.frozenEyeEvents -gt 0) { 'FLICKER' } else { 'CLEAN' }
                    measurement = $measurement
                    ppcFrames = [int64]$after.ppc.frame - [int64]$before.ppc.frame
                    processCycles = [int64]$after.process.cycles - [int64]$before.process.cycles
                    processCpu100ns = ([int64]$after.process.user100ns - [int64]$before.process.user100ns) + ([int64]$after.process.kernel100ns - [int64]$before.process.kernel100ns)
                    captureDelta = [ordered]@{
                        current3D = [int64]$after.capture.current3D - [int64]$before.capture.current3D
                        stableReused = [int64]$after.capture.stableReused - [int64]$before.capture.stableReused
                        suppressed3D = [int64]$after.capture.suppressed3D - [int64]$before.capture.suppressed3D
                        duplicateDrops = [int64]$after.capture.duplicateDrops - [int64]$before.capture.duplicateDrops
                        fatalInvalidations = [int64]$after.capture.fatalInvalidations - [int64]$before.capture.fatalInvalidations
                    }
                }
                $results += [pscustomobject]$result
                Write-JsonAtomic $results $resultsPath
                Write-Output ("{0}: {1} bad={2}% flicker={3} ppcFrames={4} cycles={5}" -f $trialId,$result.verdict,$measurement.badPct,$measurement.flickerEvents,$result.ppcFrames,$result.processCycles)
                if ($result.verdict -eq 'FLICKER' -and -not $NoIncidentCapture) {
                    Start-BvrIncidentCapture 12 "incident-$trialId" | Out-Null
                    $deadline = (Get-Date).AddSeconds(45)
                    do { Start-Sleep -Milliseconds 500; $dumpState = Get-BvrState } while ($dumpState -and $dumpState.dumpFramesPending -gt 0 -and (Get-Date) -lt $deadline)
                }
                $epoch++
                if ($config.name -ne $scenario.control.name) {
                    Test-BvrRecovery $scenario.control $epoch "recovery-$trialId" | Out-Null
                    $epoch++
                }
            }
        }
    }
} catch {
    $halted = $true
    $_ | Out-String | Set-Content -LiteralPath (Join-Path $runRoot 'HALTED.txt') -Encoding utf8
    throw
} finally {
    try { Invoke-BvrConfiguration $scenario.control $epoch 'lab-final-control' } catch {}
    foreach ($artifact in @('BetterVR_state.json','BetterVR_events.csv','BetterVR_log.txt')) {
        if (Test-Path "$LauncherDir\$artifact") { Copy-Item "$LauncherDir\$artifact" $runRoot -Force }
    }
}

$csv = foreach ($result in $results) {
    [pscustomobject]@{ trialId=$result.trialId; candidate=$result.candidate; config=$result.config; block=$result.block; position=$result.position; verdict=$result.verdict; badPct=$result.measurement.badPct; flicker=$result.measurement.flickerEvents; ppcFrames=$result.ppcFrames; processCycles=$result.processCycles; processCpu100ns=$result.processCpu100ns }
}
$csv | Export-Csv -LiteralPath (Join-Path $runRoot 'trials.csv') -NoTypeInformation
$groups = $csv | Group-Object candidate,config
$lines = @('# BetterVR right-eye lab report','',"Run: `$(Split-Path $runRoot -Leaf)`  ","Session: `$($initial.sessionId)`  ",'', '## ABBA summary','', '| Candidate/config | Trials | Clean | Mean PPC frames | Mean process cycles |', '|---|---:|---:|---:|---:|')
foreach ($group in $groups) {
    $meanPpc = [Math]::Round(($group.Group | Measure-Object ppcFrames -Average).Average,1)
    $meanCycles = [Math]::Round(($group.Group | Measure-Object processCycles -Average).Average,0)
    $clean = @($group.Group | Where-Object verdict -eq 'CLEAN').Count
    $lines += "| $($group.Name) | $($group.Count) | $clean | $meanPpc | $meanCycles |"
}
$lines += @('', 'A `CLEAN` result requires fresh advancing samples from both final OpenXR eyes. Recovery is checked after every non-control trial; the run halts on stale state, session replacement, control mismatch, or failed recovery.')
$lines | Set-Content -LiteralPath (Join-Path $runRoot 'REPORT.md') -Encoding utf8

$dumpDir = Join-Path $LauncherDir 'BetterVR_dumps'
$dumpSessionDir = Get-ChildItem -LiteralPath $dumpDir -Directory -Filter 'session_*' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
if ($dumpSessionDir -and @(Get-ChildItem $dumpSessionDir -Filter 'frame*_color_L.bmp').Count -ge 2) {
    & python "$PSScriptRoot\analyze_openxr_flicker.py" $dumpSessionDir --output (Join-Path $runRoot 'flicker_packet') --max-frames 24
}
Write-Output "Lab complete: $runRoot"
