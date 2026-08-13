$ErrorActionPreference = 'Stop'
$root = Join-Path ([IO.Path]::GetTempPath()) ("bvr-ipc-test-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $root | Out-Null
try {
    . "$PSScriptRoot\..\bvr_ipc.ps1"
    Set-BvrDir $root
    function Write-State([long]$AppliedSeq, [long]$StateSeq, [long]$Timestamp, [string]$Session = 'test-session') {
        [ordered]@{
            schemaVersion=2; sessionId=$Session; pid=$PID; stateSeq=$StateSeq; timestampUnixMs=$Timestamp
            frame=100+$StateSeq; appliedSeq=$AppliedSeq
            ppc=[ordered]@{ frame=200+$StateSeq; abiValid=$true; snapshotStable=$true; controlConverged=$true }
        } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $root 'BetterVR_state.json') -Encoding utf8
    }
    $now = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    Write-State 4 1 $now
    $state = Get-BvrState
    if (-not $state -or $state.sessionId -ne 'test-session') { throw 'Fresh state was not accepted' }

    Write-State 4 2 ($now - 60000)
    if (Get-BvrState) { throw 'Stale state was accepted' }

    Write-State 4 3 ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())
    $seq = Send-BvrCommand @{ marker='atomic-test'; traceEvents=$true }
    if ($seq -ne 5) { throw "Expected exact next seq 5, got $seq" }
    $command = Get-Content -LiteralPath (Join-Path $root 'BetterVR_cmd.ini') -Raw
    if ($command -notmatch '(?m)^seq=5\r?$' -or $command -notmatch '(?m)^session=test-session\r?$' -or $command -notmatch '(?m)^traceEvents=1\r?$') {
        throw "Atomic command content incorrect: $command"
    }
    if (Get-ChildItem -LiteralPath $root -Filter '*.tmp') { throw 'Atomic publisher leaked a temporary file' }

    Write-State 5 4 ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())
    if (-not (Wait-BvrApplied $seq 1)) { throw 'Exact/session acknowledgement failed' }

    $seq2 = Send-BvrCommand @{ marker='supersede-test' }
    Write-State ($seq2 + 1) 5 ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())
    if (Wait-BvrApplied $seq2 1) { throw 'A superseding sequence was falsely acknowledged' }
    Write-Output 'BetterVR IPC v2 regression: PASS'
} finally {
    Remove-Item -LiteralPath $root -Recurse -Force -ErrorAction SilentlyContinue
}
