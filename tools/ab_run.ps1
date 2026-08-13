# A/B comparison: run the harness twice (baseline vs test config) and diff the verdicts.
#   .\ab_run.ps1 -TestMask 8262 -DurationSec 420
param(
    [string]$BaselineMask = "1",
    [Parameter(Mandatory=$true)][string]$TestMask,
    [string]$Synth = "false",
    [int]$DurationSec = 420,
    [string]$OutDir = "$PSScriptRoot\..\bench_out"
)

$ErrorActionPreference = "Continue"
$stamp = (Get-Date).ToString("HHmmss")

& "$PSScriptRoot\bvr_harness.ps1" -ConfigName "ab${stamp}_base" -SkipMask $BaselineMask -Synth $Synth -DurationSec $DurationSec -OutDir $OutDir
& "$PSScriptRoot\bvr_harness.ps1" -ConfigName "ab${stamp}_test" -SkipMask $TestMask -Synth $Synth -DurationSec $DurationSec -OutDir $OutDir

$base = Get-Content "$OutDir\verdict_ab${stamp}_base.json" -Raw | ConvertFrom-Json
$test = Get-Content "$OutDir\verdict_ab${stamp}_test.json" -Raw | ConvertFrom-Json

Write-Output ""
Write-Output "=== A/B comparison (baseline mask=$BaselineMask vs test mask=$TestMask) ==="
$rows = @()
$rows += [pscustomobject]@{ metric = "crashed";       baseline = $base.crashed;              test = $test.crashed }
$rows += [pscustomobject]@{ metric = "ingameReached"; baseline = $base.ingameReached;        test = $test.ingameReached }
$rows += [pscustomobject]@{ metric = "benchWindows";  baseline = $base.benchWindows;         test = $test.benchWindows }
$rows += [pscustomobject]@{ metric = "lastFps";       baseline = $base.lastBench.fps;        test = $test.lastBench.fps }
$rows += [pscustomobject]@{ metric = "lastWorkMs";    baseline = $base.lastBench.workMs;     test = $test.lastBench.workMs }
$rows += [pscustomobject]@{ metric = "whiteL";        baseline = $base.white.L;              test = $test.white.L }
$rows += [pscustomobject]@{ metric = "whiteR";        baseline = $base.white.R;              test = $test.white.R }
$rows | Format-Table -AutoSize | Out-String | Write-Output

if ($test.crashed) { Write-Output "TEST VERDICT: CRASHED ($($test.exitCode))" }
elseif ($test.white.L + $test.white.R -gt ($base.white.L + $base.white.R + 20)) { Write-Output "TEST VERDICT: FLICKER (white frames regressed)" }
elseif ($base.lastBench -and $test.lastBench) {
    $speedup = [math]::Round(100.0 * ($base.lastBench.workMs - $test.lastBench.workMs) / [math]::Max(0.01, $base.lastBench.workMs), 1)
    Write-Output "TEST VERDICT: OK - work time ${speedup}% vs baseline"
}
