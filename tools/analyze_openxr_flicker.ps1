param(
    [string]$DumpDir = "E:\Cemu_2.6\BetterVR_dumps",
    [string]$OutputDir = "",
    [int]$MaxFrames = 24
)
$ErrorActionPreference = 'Stop'
if (-not $OutputDir) { $OutputDir = Join-Path $DumpDir ("flicker_analysis_" + (Get-Date -Format 'yyyyMMdd_HHmmss')) }
& python "$PSScriptRoot\analyze_openxr_flicker.py" $DumpDir --output $OutputDir --max-frames $MaxFrames
if ($LASTEXITCODE -ne 0) { throw "Flicker analyzer failed with exit code $LASTEXITCODE" }
Get-Content -LiteralPath (Join-Path $OutputDir 'LLM_REVIEW.md') -Raw
