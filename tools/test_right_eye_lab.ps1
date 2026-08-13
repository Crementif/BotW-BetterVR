$ErrorActionPreference = 'Stop'
$scripts = @(
    'bvr_ipc.ps1','bvr_harness.ps1','bvr_lab.ps1','bisect_mask.ps1',
    'analyze_openxr_flicker.ps1','validate_bvr_abi.ps1','stop_bvr_session.ps1',
    'tests\test_bvr_ipc.ps1'
)
foreach ($script in $scripts) {
    $tokens = $null; $errors = $null
    [Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $script), [ref]$tokens, [ref]$errors) | Out-Null
    if ($errors) { throw "PowerShell parse failed for $script`: $($errors[0].Message)" }
}
& "$PSScriptRoot\validate_bvr_abi.ps1"
& "$PSScriptRoot\tests\test_bvr_ipc.ps1"
& python "$PSScriptRoot\tests\test_flicker_analyzer.py"
if ($LASTEXITCODE -ne 0) { throw "Flicker analyzer regression failed with exit code $LASTEXITCODE" }
Write-Output 'BetterVR right-eye lab test suite: PASS'
