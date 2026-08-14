#pragma once

#include "pch.h"

// File-based control channel so scripts can flip settings, request eye dumps, and read
// live telemetry without restarting Cemu. All paths are relative to Cemu's working
// directory (same as BetterVR_log.txt):
//   BetterVR_cmd.ini    (script -> mod)  key=value lines; applied when "seq" changes
//   BetterVR_state.json (mod -> script)  frame counter, masks, telemetry, PPC counters
//   BetterVR_events.csv (mod -> script)  PPC event-ring drain, when traceEvents=1
class IpcControl {
public:
    static IpcControl& instance();

    // called once per frame from the render thread (renderer StartFrame, before the
    // frame's flag push so a new mask takes effect on the same frame)
    void Tick(uint32_t frameNo);

    // True only after the fork has acknowledged this process's exact request
    // generation as active. All stale/missing/unsupported states fail closed.
    bool IsStereoInstancingActive() const { return m_stereoInstancingActive; }

    // true while an incident burst still has frames to dump; caller consumes one
    // frame at a safe point so consecutive compositor outputs are preserved.
    bool ConsumeDumpRequest() {
        if (m_dumpFramesPending == 0) return false;
        --m_dumpFramesPending;
        return true;
    }

private:
    void PollCommandFile(uint32_t frameNo);
    void ApplyCommand(uint32_t frameNo);
    void WriteStateFile(uint32_t frameNo);
    void DrainEventRing();
    void PublishStereoInstancingRequest(uint32_t frameNo);

    // parsed command values (only applied when seq changes)
    struct Command {
        uint32_t seq = 0;
        std::optional<uint32_t> skipMask;
        std::optional<bool> skipDrc;
        std::optional<bool> synthRightEye;
        std::optional<bool> stereoInstancing;
        std::optional<bool> rightEyeReuse;
        uint32_t dumpFrames = 0;
        std::optional<bool> traceEvents;
        std::optional<uint32_t> telemetryLevel;
        std::optional<uint32_t> epoch;
        std::string marker;
        std::string session;
    };

    Command m_pendingCommand;
    uint32_t m_lastAppliedSeq = 0;
    uint64_t m_lastCommandFileWriteTime = 0;
    bool m_traceEvents = false;
    uint32_t m_dumpFramesPending = 0;
    uint32_t m_lastEventReadIdx = 0;
    bool m_eventCsvHeaderWritten = false;
    uint32_t m_stateSeq = 0;
    uint32_t m_appliedHostFrame = 0;
    uint32_t m_epoch = 0;
    uint32_t m_epochStartFrame = 0;
    uint64_t m_epochStartCycles = 0;
    uint64_t m_epochStartKernel100ns = 0;
    uint64_t m_epochStartUser100ns = 0;
    uint32_t m_telemetryLevel = 1;
    uint32_t m_lastAutoIncidentSourceFrame = 0;
    uint32_t m_lastAutoIncidentHostFrame = 0;
    std::string m_marker;
    std::string m_sessionId;
    uint32_t m_stereoRequestNonce = 0;
    uint32_t m_stereoRequestGeneration = 0;
    uint32_t m_lastStereoRequestMode = UINT32_MAX;
    uint32_t m_stereoMatrixSeq = 0;
    bool m_stereoInstancingActive = false;
};
