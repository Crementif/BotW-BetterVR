param([string]$LauncherDir = 'E:\Cemu_2.6')
$ErrorActionPreference = 'Stop'
$lockPath = Join-Path $LauncherDir 'bvr_harness.lock'
if (-not (Test-Path $lockPath)) { throw "No owned BetterVR session manifest at $lockPath" }
$lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json
$targets = @($lock.launcherPid) + @($lock.cemuPids) | Where-Object { $_ } | Select-Object -Unique
foreach ($target in $targets) {
    $process = Get-Process -Id ([int]$target) -ErrorAction SilentlyContinue
    if ($process -and ($process.ProcessName -eq 'Cemu' -or $process.ProcessName -eq 'BetterVR_Launcher')) {
        Stop-Process -Id $process.Id -Force
        Write-Output "Stopped owned $($process.ProcessName) pid $($process.Id)"
    }
}
Remove-Item -LiteralPath $lockPath -Force
