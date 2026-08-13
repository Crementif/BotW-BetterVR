# BetterVR runtime control protocol v2. Dot-source this file from lab scripts.
# Commands are published atomically and tied to the active process session so a
# stale state/command file can never produce a false acknowledgement.

$script:BvrDir = "E:\Cemu_2.6"
$script:BvrSeq = 0L
$script:BvrPending = @{}

function Set-BvrDir([Parameter(Mandatory=$true)][string]$Path) {
    $script:BvrDir = [IO.Path]::GetFullPath($Path)
}

function Get-BvrState([switch]$AllowStale, [int]$MaxAgeMs = 5000) {
    $path = Join-Path $script:BvrDir "BetterVR_state.json"
    for ($attempt = 0; $attempt -lt 4; $attempt++) {
        try {
            $state = Get-Content -LiteralPath $path -Raw -ErrorAction Stop | ConvertFrom-Json -ErrorAction Stop
            if (-not $state.sessionId -or -not $state.stateSeq -or -not $state.timestampUnixMs) { return $null }
            $nowMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
            $state | Add-Member -NotePropertyName stateAgeMs -NotePropertyValue ($nowMs - [int64]$state.timestampUnixMs) -Force
            if (-not $AllowStale -and ($state.stateAgeMs -lt -2000 -or $state.stateAgeMs -gt $MaxAgeMs)) { return $null }
            if (-not (Get-Process -Id ([int]$state.pid) -ErrorAction SilentlyContinue)) { return $null }
            return $state
        } catch {
            Start-Sleep -Milliseconds 35
        }
    }
    return $null
}

function Publish-BvrAtomic([string[]]$Lines) {
    $target = Join-Path $script:BvrDir "BetterVR_cmd.ini"
    $temp = "$target.$PID.$([Guid]::NewGuid().ToString('N')).tmp"
    [IO.File]::WriteAllLines($temp, $Lines, [Text.Encoding]::ASCII)
    try {
        if ([IO.File]::Exists($target)) {
            [IO.File]::Replace($temp, $target, $null, $true)
        } else {
            [IO.File]::Move($temp, $target)
        }
    } catch {
        Move-Item -LiteralPath $temp -Destination $target -Force
    } finally {
        Remove-Item -LiteralPath $temp -Force -ErrorAction SilentlyContinue
    }
}

function Send-BvrCommand([Parameter(Mandatory=$true)][hashtable]$Values) {
    $state = Get-BvrState
    if (-not $state) { throw "No fresh, live BetterVR v2 state in $script:BvrDir" }
    $script:BvrSeq = [Math]::Max([int64]$script:BvrSeq, [int64]$state.appliedSeq) + 1
    $lines = @("seq=$script:BvrSeq", "session=$($state.sessionId)")
    foreach ($key in ($Values.Keys | Sort-Object)) {
        $value = $Values[$key]
        if ($value -is [bool]) { $value = if ($value) { 1 } else { 0 } }
        $lines += "$key=$value"
    }
    Publish-BvrAtomic $lines
    $script:BvrPending[[string]$script:BvrSeq] = [pscustomobject]@{ session = $state.sessionId; values = $Values }
    return $script:BvrSeq
}

function Wait-BvrApplied([long]$Seq, [int]$TimeoutSec = 15) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSec)
    $pending = $script:BvrPending[[string]$Seq]
    $lastStateSeq = -1L
    $lastFrame = -1L
    while ([DateTime]::UtcNow -lt $deadline) {
        $state = Get-BvrState
        if ($state) {
            if ($pending -and $state.sessionId -ne $pending.session) { return $false }
            if ([int64]$state.stateSeq -gt $lastStateSeq -and [int64]$state.frame -ge $lastFrame) {
                $lastStateSeq = [int64]$state.stateSeq
                $lastFrame = [int64]$state.frame
                if ([int64]$state.appliedSeq -eq $Seq) {
                    $needsConvergence = $pending -and ($pending.values.ContainsKey('skipMask') -or $pending.values.ContainsKey('rightEyeReuse'))
                    if (-not $needsConvergence -or ($state.ppc.abiValid -and $state.ppc.snapshotStable -and $state.ppc.controlConverged)) {
                        $script:BvrPending.Remove([string]$Seq)
                        return $true
                    }
                }
                if ([int64]$state.appliedSeq -gt $Seq) { return $false }
            }
        }
        Start-Sleep -Milliseconds 150
    }
    return $false
}

function Wait-BvrInGame([int]$TimeoutSec = 300) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSec)
    $lastPpcFrame = -1L
    while ([DateTime]::UtcNow -lt $deadline) {
        $state = Get-BvrState
        if ($state) {
            $advancing = $lastPpcFrame -ge 0 -and [int64]$state.ppc.frame -gt $lastPpcFrame
            $lastPpcFrame = [int64]$state.ppc.frame
            if ($state.inGame -eq 1 -and $state.fadeVisible -eq 0 -and $advancing -and
                $state.eyeL.state -eq 'NORMAL' -and $state.eyeR.state -eq 'NORMAL' -and
                $state.eyeL.lastSampleFrame -ge ($state.frame - 12) -and $state.eyeR.lastSampleFrame -ge ($state.frame - 12)) { return $true }
        }
        Start-Sleep -Seconds 1
    }
    return $false
}

function Measure-BvrFlicker([int]$HoldSec = 6) {
    $before = Get-BvrState
    if (-not $before) { return $null }
    $deadline = [DateTime]::UtcNow.AddSeconds($HoldSec)
    $after = $null
    do {
        Start-Sleep -Milliseconds 250
        $candidate = Get-BvrState
        if ($candidate -and $candidate.sessionId -eq $before.sessionId -and $candidate.stateSeq -gt $before.stateSeq) { $after = $candidate }
    } while ([DateTime]::UtcNow -lt $deadline)
    if (-not $after -or $after.frame -le $before.frame -or $after.ppc.frame -le $before.ppc.frame) { return $null }
    $sampleL = [int64]$after.eyeL.sampled - [int64]$before.eyeL.sampled
    $sampleR = [int64]$after.eyeR.sampled - [int64]$before.eyeR.sampled
    if ($sampleL -le 0 -or $sampleR -le 0) { return $null }
    $bad = ([int64]$after.eyeL.white - $before.eyeL.white) + ([int64]$after.eyeR.white - $before.eyeR.white) +
           ([int64]$after.eyeL.black - $before.eyeL.black) + ([int64]$after.eyeR.black - $before.eyeR.black)
    $flicker = ([int64]$after.eyeL.flickerEvents - $before.eyeL.flickerEvents) + ([int64]$after.eyeR.flickerEvents - $before.eyeR.flickerEvents)
    return [pscustomobject]@{
        sessionId = $after.sessionId; framesElapsed = [int64]$after.frame - $before.frame
        sampledL = $sampleL; sampledR = $sampleR; badFrames = $bad
        whiteL = [int64]$after.eyeL.white - $before.eyeL.white; whiteR = [int64]$after.eyeR.white - $before.eyeR.white
        blackL = [int64]$after.eyeL.black - $before.eyeL.black; blackR = [int64]$after.eyeR.black - $before.eyeR.black
        normalL = [int64]$after.eyeL.normal - $before.eyeL.normal; normalR = [int64]$after.eyeR.normal - $before.eyeR.normal
        badPct = [Math]::Round(100.0 * $bad / [Math]::Max(1, $sampleL + $sampleR), 3)
        flickerEvents = $flicker
        asymmetricFlickerEvents = [int64]$after.stereo.asymmetricFlickerEvents - $before.stereo.asymmetricFlickerEvents
        frozenEyeEvents = [int64]$after.stereo.frozenEyeEvents - $before.stereo.frozenEyeEvents
        nearIdenticalFrames = [int64]$after.stereo.nearIdenticalFrames - $before.stereo.nearIdenticalFrames
        ringDrops = [int64]$after.stereo.ringDrops - $before.stereo.ringDrops
        stateL = $after.eyeL.state; stateR = $after.eyeR.state
        before = $before; after = $after
    }
}

function Measure-BvrWhiteFrames([int]$HoldSec = 6) { Measure-BvrFlicker $HoldSec }

function Request-BvrEyeDump([int]$Frames = 1) {
    $seq = Send-BvrCommand @{ dumpFrames = [Math]::Min(120, [Math]::Max(1, $Frames)); marker = 'manual-eye-dump' }
    return Wait-BvrApplied $seq
}

function Start-BvrIncidentCapture([int]$Frames = 12, [string]$Marker = 'auto-flicker-incident') {
    $seq = Send-BvrCommand @{ dumpFrames = $Frames; telemetryLevel = 3; traceEvents = 1; marker = $Marker }
    return Wait-BvrApplied $seq
}
