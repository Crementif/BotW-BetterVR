param([string]$Repo = (Split-Path $PSScriptRoot -Parent))
$ErrorActionPreference = 'Stop'
$asm = Join-Path $Repo 'resources\BreathOfTheWild_BetterVR\patch_RND_StereoRendering_Optimizations.asm'
$text = Get-Content -LiteralPath $asm
$labels = @('_bvrCounters:','_bvrEvtRing:','_bvrAbiMagic:','_bvrSnapshotSeq:','_bvrActiveMask:','_bvrDesiredMask:','_bvrMaskEpoch:','_bvrActivationFrame:','_bvrEyePhase:','_bvrFaultFlags:','_bvrTelemetryLevel:','_bvrLastControlEvent:','_bvrClearShouldRequest:','_bvrClearFlagsBefore:','_bvrClearStagingBefore:','_bvrClearFillBefore:')
foreach ($label in $labels) { if ($text -notcontains $label) { throw "Missing ABI label $label" } }
$ringStart = [Array]::IndexOf($text, '_bvrEvtRing:')
$abiStart = [Array]::IndexOf($text, '_bvrAbiMagic:')
$ringInts = @($text[($ringStart + 1)..($abiStart - 1)] | Where-Object { $_ -match '^\.int\s' }).Count
if ($ringInts -ne 64) { throw "Event ring ABI drift: expected 64 words, found $ringInts" }
$magicLine = $text[$abiStart + 1]
if ($magicLine -notmatch '0x42565232') { throw "BVR2 magic drift: $magicLine" }
$sizeLine = ($text | Select-String -Pattern '^\.int 0x00000170$').Line
if (-not $sizeLine) { throw 'ABI size must remain 0x170' }
$optimizationRefs = @(Select-String -LiteralPath $asm -Pattern 'VR_RENDER_SKIP_MASK@')
if ($optimizationRefs.Count -ne 0) { throw "Optimization hook bypasses boundary latch at line $($optimizationRefs[0].LineNumber)" }
$publishLines = @(Select-String -LiteralPath $asm -Pattern '^stw r12, _cEvtWriteIdx@l\(r11\)$')
if ($publishLines.Count -ne 10) { throw "Expected 10 event publishers, found $($publishLines.Count)" }
foreach ($publisher in $publishLines) {
    $index = $publisher.LineNumber - 1
    $window = $text[[Math]::Max(0,$index-5)..($index-1)] -join "`n"
    if ($window -notmatch 'stw r(?:9|11), 0\(r12\)') { throw "Event index published before its ring slot near line $($publisher.LineNumber)" }
}
Write-Output "BVR2 ABI validation: PASS (ring=$ringInts words, size=0x170, 10 slot-before-index event publishers, all hooks use activeMask)"
