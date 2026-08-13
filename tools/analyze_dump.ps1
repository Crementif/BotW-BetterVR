# Analyze a WER crash dump with cdb (Windows SDK debugger).
#   .\analyze_dump.ps1                     # newest Cemu* dump
#   .\analyze_dump.ps1 -Pattern "Cemu*"    # explicit pattern
#   .\analyze_dump.ps1 -DumpFile <path>
# Cemu note: a fault address in anonymous (non-module) executable memory is the PPC
# recompiler's code cache - the crash is in recompiled GAME code, not host code. Use
# the cemu-debugger MCP jit_lookup / breakpoints to map it back to a PPC address.
param(
    [string]$DumpFile = "",
    [string]$Pattern = "Cemu*",
    [switch]$Full   # also print !analyze -v (slow)
)

$ErrorActionPreference = "Continue"
$cdb = "C:\Program Files (x86)\Windows Kits\10\Debuggers\x64\cdb.exe"

if (-not $DumpFile) {
    $searchDirs = @("$env:LOCALAPPDATA\CrashDumps", "E:\CrashDumps")
    $candidates = foreach ($dir in $searchDirs) {
        Get-ChildItem $dir -Filter "$Pattern.dmp" -ErrorAction SilentlyContinue
    }
    $newest = $candidates | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $newest) { Write-Output "No dumps matching '$Pattern.dmp' in $($searchDirs -join ', ')"; exit 1 }
    $DumpFile = $newest.FullName
}

Write-Output "=== analyzing: $DumpFile ==="

if (Test-Path $cdb) {
    $commands = if ($Full) { "!analyze -v; .ecxr; kb 20; lm m *; q" } else { ".ecxr; kb 20; lm m *; q" }
    $raw = & $cdb -z $DumpFile -c $commands 2>&1 | Out-String

    # exception line, register context, and top frames
    $lines = $raw -split "`r?`n"
    $interesting = $lines | Where-Object {
        $_ -match "ExceptionCode|ExceptionAddress|Access violation|The stored exception|rip=|rsp=|^[0-9a-f]{2} [0-9a-f`` ]{17}" -or
        $_ -match "^\s*\d+\s+[0-9a-f]" -or $_ -match "Child-?SP|RetAddr|Call Site"
    }
    $interesting | Select-Object -First 50 | ForEach-Object { $_ }

    # classify the faulting address: module code vs anonymous (JIT) memory
    $ripLine = $lines | Where-Object { $_ -match "rip=([0-9a-f]+)" } | Select-Object -First 1
    if ($ripLine -and $ripLine -match "rip=([0-9a-f]+)") {
        $rip = [Convert]::ToUInt64($Matches[1], 16)
        $inModule = $false
        foreach ($moduleLine in $lines) {
            if ($moduleLine -match "^([0-9a-f``]{8,})\s+([0-9a-f``]{8,})\s+(\w[\w.]*)\s") {
                $startText = $Matches[1].Replace("``", "")
                $endText = $Matches[2].Replace("``", "")
                $moduleName = $Matches[3]
                try {
                    $modStart = [Convert]::ToUInt64($startText, 16)
                    $modEnd = [Convert]::ToUInt64($endText, 16)
                } catch { continue }
                if ($rip -ge $modStart -and $rip -lt $modEnd) {
                    Write-Output ("`nFAULT LOCATION: inside module {0} (rip=0x{1:X})" -f $moduleName, $rip)
                    $inModule = $true
                    break
                }
            }
        }
        if (-not $inModule) {
            Write-Output ("`nFAULT LOCATION: anonymous executable memory (rip=0x{0:X})" -f $rip)
            Write-Output "For Cemu this is the PPC recompiler cache -> the crash is in recompiled GAME code."
            Write-Output "Next step: cemu-debugger MCP (breakpoints/memory watch) or compare against hook addresses."
        }
    }
}
else {
    Write-Output "cdb.exe not found - install the Windows SDK Debugging Tools. Raw dump: $DumpFile"
}
