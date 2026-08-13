#include "pch.h"

#include "ipc_control.h"
#include "hooking/cemu_hooks.h"
#include "instance.h"
#include "mod_settings.h"
#include "rendering/telemetry.h"

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
    constexpr const char* kCommandFile = "BetterVR_cmd.ini";
    constexpr const char* kStateFile = "BetterVR_state.json";
    constexpr const char* kStateFileTmp = "BetterVR_state.json.tmp";
    constexpr const char* kEventsFile = "BetterVR_events.csv";

    constexpr uint32_t kPollInterval = 15;  // frames between command-file stats
    constexpr uint32_t kStateInterval = 45; // frames between state-file writes

    // PPC-side counter block (see patch_RND_StereoRendering_Optimizations.asm). It
    // lives in the graphic pack's codecave (the only game-free memory), so its address
    // is discovered by scanning the cave region for the 'BVRC' magic once. Offsets
    // below match the block layout in the asm.
    constexpr uint32_t kCounterMagic = 0x42565243; // 'BVRC'
    constexpr uint64_t kCaveScanStart = 0x01800000;
    constexpr uint64_t kCaveScanEnd = 0x01880000;
    constexpr uint64_t kOffFrame = 0x04;
    constexpr uint64_t kOffRequestDraw = 0x08;
    constexpr uint64_t kOffLastSwapCursor = 0x0C;
    constexpr uint64_t kOffSwapCount = 0x10;
    constexpr uint64_t kOffClearQFull = 0x14;
    constexpr uint64_t kOffClearQReduced = 0x18;
    constexpr uint64_t kOffMgrRun = 0x1C;
    constexpr uint64_t kOffMgrSkip = 0x20;
    constexpr uint64_t kOffCalcModelRun = 0x24;
    constexpr uint64_t kOffCalcModelSkip = 0x28;
    constexpr uint64_t kOffEventWriteIdx = 0x2C;
    constexpr uint64_t kOffEventRing = 0x30;
    constexpr uint32_t kPpcEventRingSize = 64;
    constexpr uint64_t kOffAbiMagic = 0x130;
    constexpr uint64_t kOffAbiVersion = 0x134;
    constexpr uint64_t kOffAbiSize = 0x138;
    constexpr uint64_t kOffSnapshotSeq = 0x13C;
    constexpr uint64_t kOffActiveMask = 0x140;
    constexpr uint64_t kOffDesiredMask = 0x144;
    constexpr uint64_t kOffMaskEpoch = 0x148;
    constexpr uint64_t kOffActivationFrame = 0x14C;
    constexpr uint64_t kOffEyePhase = 0x150;
    constexpr uint64_t kOffFaultFlags = 0x154;
    constexpr uint64_t kOffTelemetryLevel = 0x158;
    constexpr uint64_t kOffLastControlEvent = 0x15C;
    constexpr uint64_t kOffClearShouldRequest = 0x160;
    constexpr uint64_t kOffClearFlagsBefore = 0x164;
    constexpr uint64_t kOffClearStagingBefore = 0x168;
    constexpr uint64_t kOffClearFillBefore = 0x16C;
    constexpr uint32_t kAbiMagic = 0x42565232; // 'BVR2'
    constexpr uint32_t kAbiVersion = 2;
    constexpr uint32_t kAbiSize = 0x170;

    struct PpcSnapshot {
        bool abiValid = false;
        bool stable = false;
        uint32_t version = 0;
        uint32_t size = 0;
        uint32_t snapshotSeq = 0;
        uint32_t activeMask = 0;
        uint32_t desiredMask = 0;
        uint32_t maskEpoch = 0;
        uint32_t activationFrame = 0;
        uint32_t eyePhase = 2;
        uint32_t faultFlags = 0;
        uint32_t telemetryLevel = 0;
        uint32_t lastControlEvent = 0;
        uint32_t clearShouldRequest = 0;
        uint32_t clearFlagsBefore = 0;
        uint32_t clearStagingBefore = 0;
        uint32_t clearFillBefore = 0;
    };

    uint32_t ReadPpcU32(uint64_t address) {
        return CemuHooks::getMemory<BEType<uint32_t>>(address).getLE();
    }

    // 0 until found; scanned lazily and cached
    uint64_t s_counterBlockBase = 0;
    bool s_counterScanDone = false;

    uint64_t GetCounterBlockBase() {
        if (s_counterBlockBase == 0) {
            for (uint64_t address = kCaveScanStart; address < kCaveScanEnd; address += 4) {
                if (ReadPpcU32(address) == kCounterMagic) {
                    s_counterBlockBase = address;
                    Log::print<INFO>("[ipc] PPC counter block found at 0x{:08X}", address);
                    break;
                }
            }
            // not found: retry on a later call (patches may not be applied yet), warn once
            if (s_counterBlockBase == 0 && !s_counterScanDone) {
                s_counterScanDone = true;
                Log::print<WARNING>("[ipc] PPC counter block magic not found in cave region yet; counters read 0 until it appears");
            }
        }
        return s_counterBlockBase;
    }

    uint32_t ReadCounter(uint64_t offset) {
        const uint64_t base = GetCounterBlockBase();
        return base != 0 ? ReadPpcU32(base + offset) : 0;
    }

    PpcSnapshot ReadPpcSnapshot() {
        PpcSnapshot result;
        const uint64_t base = GetCounterBlockBase();
        if (base == 0) return result;
        result.version = ReadPpcU32(base + kOffAbiVersion);
        result.size = ReadPpcU32(base + kOffAbiSize);
        result.abiValid = ReadPpcU32(base + kOffAbiMagic) == kAbiMagic &&
            result.version == kAbiVersion && result.size >= kAbiSize;
        if (!result.abiValid) return result;

        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint32_t before = ReadPpcU32(base + kOffSnapshotSeq);
            if ((before & 1u) != 0) continue;
            result.activeMask = ReadPpcU32(base + kOffActiveMask);
            result.desiredMask = ReadPpcU32(base + kOffDesiredMask);
            result.maskEpoch = ReadPpcU32(base + kOffMaskEpoch);
            result.activationFrame = ReadPpcU32(base + kOffActivationFrame);
            result.eyePhase = ReadPpcU32(base + kOffEyePhase);
            result.faultFlags = ReadPpcU32(base + kOffFaultFlags);
            result.telemetryLevel = ReadPpcU32(base + kOffTelemetryLevel);
            result.lastControlEvent = ReadPpcU32(base + kOffLastControlEvent);
            result.clearShouldRequest = ReadPpcU32(base + kOffClearShouldRequest);
            result.clearFlagsBefore = ReadPpcU32(base + kOffClearFlagsBefore);
            result.clearStagingBefore = ReadPpcU32(base + kOffClearStagingBefore);
            result.clearFillBefore = ReadPpcU32(base + kOffClearFillBefore);
            const uint32_t after = ReadPpcU32(base + kOffSnapshotSeq);
            if (before == after && (after & 1u) == 0) {
                result.snapshotSeq = after;
                result.stable = true;
                break;
            }
        }
        return result;
    }

    uint64_t FileTimeToU64(const FILETIME& value) {
        return (uint64_t(value.dwHighDateTime) << 32) | value.dwLowDateTime;
    }

    uint64_t UnixTimeMs() {
        FILETIME now = {};
        GetSystemTimeAsFileTime(&now);
        constexpr uint64_t kWindowsToUnix100ns = 116444736000000000ull;
        return (FileTimeToU64(now) - kWindowsToUnix100ns) / 10000ull;
    }

    std::string JsonEscape(std::string_view value) {
        std::string result;
        result.reserve(value.size());
        for (const char ch : value) {
            switch (ch) {
                case '\\': result += "\\\\"; break;
                case '"': result += "\\\""; break;
                case '\n': result += "\\n"; break;
                case '\r': result += "\\r"; break;
                case '\t': result += "\\t"; break;
                default: if ((unsigned char)ch >= 0x20) result += ch; break;
            }
        }
        return result;
    }

    const char* PpcEventName(uint32_t eventId) {
        switch (eventId) {
            case 1: return "clearQueue-full";
            case 2: return "clearQueue-reduced";
            case 3: return "mgrClearQueue-run";
            case 4: return "mgrClearQueue-skip";
            case 5: return "swapDrawList";
            case 6: return "calcModel-run";
            case 7: return "calcModel-skip";
            case 8: return "procFrame";
            case 9: return "invokeType3-skip";
            case 10: return "mono-clearQueue-state";
            default: return "?";
        }
    }

    uint64_t GetFileWriteTime(const char* path) {
        WIN32_FILE_ATTRIBUTE_DATA attributes = {};
        if (!GetFileAttributesExA(path, GetFileExInfoStandard, &attributes)) {
            return 0;
        }
        return ((uint64_t)attributes.ftLastWriteTime.dwHighDateTime << 32) | attributes.ftLastWriteTime.dwLowDateTime;
    }
}

IpcControl& IpcControl::instance() {
    static IpcControl s_instance;
    return s_instance;
}

void IpcControl::Tick(uint32_t frameNo) {
    if (m_sessionId.empty()) {
        LARGE_INTEGER counter = {};
        QueryPerformanceCounter(&counter);
        char id[64];
        snprintf(id, sizeof(id), "%08X-%08X-%016llX", GetCurrentProcessId(), GetCurrentThreadId(),
            (unsigned long long)counter.QuadPart);
        m_sessionId = id;
        m_epochStartFrame = frameNo;
        QueryProcessCycleTime(GetCurrentProcess(), &m_epochStartCycles);
        FILETIME creation = {}, exit = {}, kernel = {}, user = {};
        if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
            m_epochStartKernel100ns = FileTimeToU64(kernel);
            m_epochStartUser100ns = FileTimeToU64(user);
        }
    }
    if (frameNo % kPollInterval == 0) {
        PollCommandFile(frameNo);
    }
    if (m_traceEvents) {
        DrainEventRing();
    }
    if (m_telemetryLevel >= 2) {
        const auto stereo = EyeTelemetry::instance().GetStereoStats();
        if (stereo.lastAnomalyFrame != 0 && stereo.lastAnomalyFrame != m_lastAutoIncidentSourceFrame) {
            m_lastAutoIncidentSourceFrame = stereo.lastAnomalyFrame;
            constexpr uint32_t kAutoIncidentCooldownFrames = 900;
            const bool cooldownElapsed = m_lastAutoIncidentHostFrame == 0 ||
                frameNo - m_lastAutoIncidentHostFrame >= kAutoIncidentCooldownFrames;
            // A blocking incident dump can itself make one eye's next temporal sample
            // arrive later than the other. Never extend an active burst from that
            // feedback, and rate-limit later captures while still consuming the new
            // anomaly id so an old event cannot arm after the cooldown expires.
            if (m_dumpFramesPending == 0 && cooldownElapsed) {
                m_lastAutoIncidentHostFrame = frameNo;
                m_dumpFramesPending = 12;
                Log::print<WARNING>("[ipc] auto incident burst armed for telemetry anomaly '{}' at frame {}",
                    stereo.lastAnomaly, stereo.lastAnomalyFrame);
            }
        }
    }
    if (frameNo % kStateInterval == 0) {
        WriteStateFile(frameNo);
    }
}

void IpcControl::PollCommandFile(uint32_t frameNo) {
    const uint64_t writeTime = GetFileWriteTime(kCommandFile);
    if (writeTime == 0 || writeTime == m_lastCommandFileWriteTime) {
        return;
    }
    m_lastCommandFileWriteTime = writeTime;

    std::ifstream file(kCommandFile);
    if (!file.is_open()) {
        return;
    }

    Command command;
    std::string line;
    while (std::getline(file, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        while (!value.empty() && (value.back() == '\r' || value.back() == '\n' || value.back() == ' ')) {
            value.pop_back();
        }

        const uint32_t number = (uint32_t)strtoul(value.c_str(), nullptr, 0);
        if (key == "seq") command.seq = number;
        else if (key == "skipMask") command.skipMask = number;
        else if (key == "skipDrc") command.skipDrc = number != 0;
        else if (key == "synthRightEye") command.synthRightEye = number != 0;
        else if (key == "rightEyeReuse") command.rightEyeReuse = number != 0;
        else if (key == "dumpEyes" && number != 0) command.dumpFrames = std::max(command.dumpFrames, 1u);
        else if (key == "dumpFrames") command.dumpFrames = std::min(number, 120u);
        else if (key == "traceEvents") command.traceEvents = number != 0;
        else if (key == "telemetryLevel") command.telemetryLevel = std::min(number, 3u);
        else if (key == "epoch") command.epoch = number;
        else if (key == "marker") command.marker = value;
        else if (key == "session") command.session = value;
    }

    if (!command.session.empty() && command.session != m_sessionId) {
        Log::print<WARNING>("[ipc] rejected seq={} for stale session '{}' (active '{}')", command.seq, command.session, m_sessionId);
        return;
    }
    if (command.seq != 0 && command.seq != m_lastAppliedSeq) {
        m_pendingCommand = command;
        ApplyCommand(frameNo);
    }
}

void IpcControl::ApplyCommand(uint32_t frameNo) {
    const Command& command = m_pendingCommand;
    m_lastAppliedSeq = command.seq;

    if (command.skipMask.has_value()) {
        GetSettings().rightEyeCalcSkipMask = *command.skipMask & 0x3FFEu;
    }
    if (command.skipDrc.has_value()) {
        GetSettings().skipDrcRendering = *command.skipDrc;
    }
    if (command.synthRightEye.has_value()) {
        GetSettings().synthesizedRightEye = *command.synthRightEye;
    }
    if (command.rightEyeReuse.has_value()) {
        // The model-job update and queue preservation hooks are one invariant.
        // 0x0002/0x0004 are the two proven calc reuse candidates. This remains
        // laboratory-only until the preserved right-eye draw list is correct.
        GetSettings().synthesizedRightEye = false;
        GetSettings().rightEyeCalcSkipMask = *command.rightEyeReuse ? 0x3006u : 0u;
    }
    if (command.traceEvents.has_value()) {
        m_traceEvents = *command.traceEvents;
    }
    if (command.dumpFrames != 0) {
        m_dumpFramesPending = std::max(m_dumpFramesPending, command.dumpFrames);
    }
    if (command.telemetryLevel.has_value()) {
        m_telemetryLevel = *command.telemetryLevel;
        const uint64_t base = GetCounterBlockBase();
        if (base != 0 && ReadPpcU32(base + kOffAbiMagic) == kAbiMagic) {
            CemuHooks::setMemory<uint32_t>(base + kOffTelemetryLevel, m_telemetryLevel);
        }
    }
    if (command.epoch.has_value()) {
        m_epoch = *command.epoch;
        m_epochStartFrame = frameNo;
        QueryProcessCycleTime(GetCurrentProcess(), &m_epochStartCycles);
        FILETIME creation = {}, exit = {}, kernel = {}, user = {};
        if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
            m_epochStartKernel100ns = FileTimeToU64(kernel);
            m_epochStartUser100ns = FileTimeToU64(user);
        }
    }
    if (!command.marker.empty()) m_marker = command.marker;
    m_appliedHostFrame = frameNo;

    Log::print<INFO>("[ipc] applied seq={} at frame {}: skipMask={} reuse={} skipDrc={} synth={} dumpFrames={} traceEvents={} epoch={} marker='{}'",
        command.seq, frameNo,
        command.skipMask.has_value() ? std::to_string(*command.skipMask) : "-",
        command.rightEyeReuse.has_value() ? (*command.rightEyeReuse ? "1" : "0") : "-",
        command.skipDrc.has_value() ? (*command.skipDrc ? "1" : "0") : "-",
        command.synthRightEye.has_value() ? (*command.synthRightEye ? "1" : "0") : "-",
        command.dumpFrames,
        command.traceEvents.has_value() ? (*command.traceEvents ? "1" : "0") : "-",
        command.epoch.has_value() ? std::to_string(*command.epoch) : "-", command.marker);
}

void IpcControl::WriteStateFile(uint32_t frameNo) {
    const auto statsL = EyeTelemetry::instance().GetStats(0);
    const auto statsR = EyeTelemetry::instance().GetStats(1);
    const auto stereo = EyeTelemetry::instance().GetStereoStats();
    const PpcSnapshot ppc = ReadPpcSnapshot();

    RND_Renderer* renderer = nullptr;
    if (auto* xr = VRManager::instance().XR.get(); xr != nullptr) {
        renderer = xr->GetRenderer();
    }
    const double frameMs = renderer != nullptr ? renderer->GetLastFrameTimeMs() : 0.0;
    const double workMs = renderer != nullptr ? renderer->GetLastFrameWorkTimeMs() : 0.0;

    uint64_t processCycles = 0;
    QueryProcessCycleTime(GetCurrentProcess(), &processCycles);
    FILETIME creation = {}, exit = {}, kernel = {}, user = {};
    uint64_t kernel100ns = 0, user100ns = 0;
    if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
        kernel100ns = FileTimeToU64(kernel);
        user100ns = FileTimeToU64(user);
    }
    const uint32_t effectiveMask = CemuHooks::GetEffectiveRenderSkipMask();
    const bool controlConverged = ppc.stable && ppc.activeMask == ppc.desiredMask && ppc.desiredMask == effectiveMask;

    std::ostringstream json;
    json << std::fixed << std::setprecision(4);
    const auto writeEye = [&json](const char* name, const EyeTelemetry::EyeStats& s) {
        json << "  \"" << name << "\": { \"state\": \"" << EyeTelemetry::StateName(s.state)
             << "\", \"luma\": " << s.meanLuma << ", \"minLuma\": " << s.minRegionLuma << ", \"maxLuma\": " << s.maxRegionLuma
             << ", \"white\": " << s.whiteFrames << ", \"black\": " << s.blackFrames << ", \"normal\": " << s.normalFrames
             << ", \"transitions\": " << s.transitions << ", \"sampled\": " << s.sampledFrames << ", \"dropped\": " << s.droppedFrames
             << ", \"temporalDelta\": " << s.temporalDelta << ", \"edgeEnergy\": " << s.edgeEnergy
             << ", \"flickerEvents\": " << s.flickerEvents << ", \"frozenFrames\": " << s.frozenFrames
             << ", \"lastTransitionFrame\": " << s.lastTransitionFrame << ", \"lastSampleFrame\": " << s.lastSampleFrame << " }";
    };
    json << "{\n  \"schemaVersion\": 2,\n  \"sessionId\": \"" << JsonEscape(m_sessionId) << "\",\n"
         << "  \"pid\": " << GetCurrentProcessId() << ",\n  \"stateSeq\": " << ++m_stateSeq << ",\n"
         << "  \"timestampUnixMs\": " << UnixTimeMs() << ",\n  \"frame\": " << frameNo << ",\n"
         << "  \"appliedSeq\": " << m_lastAppliedSeq << ",\n  \"appliedAtHostFrame\": " << m_appliedHostFrame << ",\n"
         << "  \"marker\": \"" << JsonEscape(m_marker) << "\",\n  \"settingMask\": " << (uint32_t)GetSettings().GetRenderSkipMask() << ",\n"
         << "  \"effectiveMask\": " << effectiveMask << ",\n  \"synthRightEye\": " << (CemuHooks::ShouldUseSynthesizedRightEye() ? 1 : 0) << ",\n"
         << "  \"inGame\": " << (CemuHooks::IsInGame() ? 1 : 0) << ",\n  \"fadeVisible\": " << (CemuHooks::IsAnyFadeScreenVisible() ? 1 : 0) << ",\n"
         << "  \"frameMs\": " << frameMs << ",\n  \"workMs\": " << workMs << ",\n"
         << "  \"dumpFramesPending\": " << m_dumpFramesPending << ",\n  \"traceEvents\": " << (m_traceEvents ? "true" : "false") << ",\n";
    writeEye("eyeL", statsL); json << ",\n";
    writeEye("eyeR", statsR); json << ",\n";
    json << "  \"stereo\": { \"sampled\": " << stereo.sampledFrames << ", \"incomplete\": " << stereo.incompleteFrames
         << ", \"meanAbsDifference\": " << stereo.meanAbsDifference << ", \"nearIdenticalFraction\": " << stereo.nearIdenticalFraction
         << ", \"nearIdenticalFrames\": " << stereo.nearIdenticalFrames << ", \"asymmetricFlickerEvents\": " << stereo.asymmetricFlickerEvents
         << ", \"frozenEyeEvents\": " << stereo.frozenEyeEvents << ", \"lastAnomalyFrame\": " << stereo.lastAnomalyFrame
         << ", \"lastAnomaly\": \"" << stereo.lastAnomaly << "\", \"ringDrops\": " << EyeTelemetry::instance().GetDroppedRingFrames() << " },\n"
         << "  \"capture\": { \"current3D\": " << (renderer ? renderer->GetCurrent3DPresentedCount() : 0)
         << ", \"stableReused\": " << (renderer ? renderer->GetStable3DReusedCount() : 0)
         << ", \"suppressed3D\": " << (renderer ? renderer->Get3DSuppressedCount() : 0)
         << ", \"duplicateDrops\": " << (renderer ? renderer->GetDuplicateCaptureDrops() : 0)
         << ", \"fatalInvalidations\": " << (renderer ? renderer->GetFatalSlotInvalidations() : 0) << " },\n"
         << "  \"compositor\": { \"frames\": " << (renderer ? renderer->GetCompositorFrameCount() : 0)
         << ", \"projectionSubmitted\": " << (renderer ? renderer->GetProjectionSubmittedCount() : 0)
         << ", \"cachedProjectionReuse\": " << (renderer ? renderer->GetCachedProjectionReuseCount() : 0)
         << ", \"omittedWhenExpected\": " << (renderer ? renderer->GetProjectionOmittedWhenExpectedCount() : 0)
         << ", \"presenceTransitions\": " << (renderer ? renderer->GetProjectionPresenceTransitions() : 0)
         << ", \"emptyFrames\": " << (renderer ? renderer->GetEmptyCompositionFrameCount() : 0)
         << ", \"twoDOnlyFrames\": " << (renderer ? renderer->GetTwoDOnlyCompositionFrameCount() : 0)
         << ", \"consecutiveOmissions\": " << (renderer ? renderer->GetConsecutiveProjectionOmissions() : 0)
         << ", \"maxConsecutiveOmissions\": " << (renderer ? renderer->GetMaxConsecutiveProjectionOmissions() : 0)
         << ", \"lastLayerCount\": " << (renderer ? renderer->GetLastCompositionLayerCount() : 0)
         << ", \"projectionLastFrame\": " << (renderer && renderer->WasProjectionSubmittedLastFrame() ? "true" : "false") << " },\n"
         << "  \"process\": { \"cycles\": " << processCycles << ", \"kernel100ns\": " << kernel100ns << ", \"user100ns\": " << user100ns << " },\n"
         << "  \"epoch\": { \"id\": " << m_epoch << ", \"startFrame\": " << m_epochStartFrame << ", \"frames\": " << (frameNo - m_epochStartFrame)
         << ", \"cycles\": " << (processCycles - m_epochStartCycles) << ", \"kernel100ns\": " << (kernel100ns - m_epochStartKernel100ns)
         << ", \"user100ns\": " << (user100ns - m_epochStartUser100ns) << " },\n"
         << "  \"ppc\": { \"abiValid\": " << (ppc.abiValid ? "true" : "false") << ", \"snapshotStable\": " << (ppc.stable ? "true" : "false")
         << ", \"abiVersion\": " << ppc.version << ", \"abiSize\": " << ppc.size << ", \"snapshotSeq\": " << ppc.snapshotSeq
         << ", \"activeMask\": " << ppc.activeMask << ", \"desiredMask\": " << ppc.desiredMask << ", \"maskEpoch\": " << ppc.maskEpoch
         << ", \"activationFrame\": " << ppc.activationFrame << ", \"eyePhase\": " << ppc.eyePhase << ", \"faultFlags\": " << ppc.faultFlags
         << ", \"telemetryLevel\": " << ppc.telemetryLevel << ", \"lastControlEvent\": " << ppc.lastControlEvent
         << ", \"clearShouldRequest\": " << ppc.clearShouldRequest << ", \"clearFlagsBefore\": " << ppc.clearFlagsBefore
         << ", \"clearStagingBefore\": " << ppc.clearStagingBefore << ", \"clearFillBefore\": " << ppc.clearFillBefore
         << ", \"controlConverged\": " << (controlConverged ? "true" : "false")
         << ", \"frame\": " << ReadCounter(kOffFrame) << ", \"requestDraw\": " << ReadCounter(kOffRequestDraw)
         << ", \"swapCursor\": " << ReadCounter(kOffLastSwapCursor) << ", \"swapCount\": " << ReadCounter(kOffSwapCount)
         << ", \"clearQFull\": " << ReadCounter(kOffClearQFull) << ", \"clearQReduced\": " << ReadCounter(kOffClearQReduced)
         << ", \"mgrClearRun\": " << ReadCounter(kOffMgrRun) << ", \"mgrClearSkip\": " << ReadCounter(kOffMgrSkip)
         << ", \"calcModelRun\": " << ReadCounter(kOffCalcModelRun) << ", \"calcModelSkip\": " << ReadCounter(kOffCalcModelSkip)
         << ", \"eventWriteIdx\": " << ReadCounter(kOffEventWriteIdx) << " }\n}\n";
    const std::string payload = json.str();

    FILE* file = nullptr;
    if (fopen_s(&file, kStateFileTmp, "wb") != 0 || file == nullptr) {
        return;
    }
    fwrite(payload.data(), 1, payload.size(), file);
    fflush(file);
    fclose(file);
    if (!MoveFileExA(kStateFileTmp, kStateFile, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        Log::print<WARNING>("[ipc] failed to publish state file: {}", GetLastError());
    }
}

void IpcControl::DrainEventRing() {
    const uint64_t base = GetCounterBlockBase();
    if (base == 0) {
        return;
    }
    const uint32_t writeIdx = ReadPpcU32(base + kOffEventWriteIdx);
    if (writeIdx == m_lastEventReadIdx) {
        return;
    }

    // if we fell more than a full ring behind, resync to the oldest still-valid entry
    uint32_t readIdx = m_lastEventReadIdx;
    if (writeIdx - readIdx > kPpcEventRingSize) {
        readIdx = writeIdx - kPpcEventRingSize;
    }

    FILE* file = nullptr;
    if (fopen_s(&file, kEventsFile, "ab") != 0 || file == nullptr) {
        return;
    }
    if (!m_eventCsvHeaderWritten) {
        // may append to an existing file from a previous run; the harness deletes it at start
        fprintf(file, "ppcFrame,eventId,eventName\n");
        m_eventCsvHeaderWritten = true;
    }
    for (; readIdx != writeIdx; ++readIdx) {
        const uint32_t word = ReadPpcU32(base + kOffEventRing + (uint64_t)(readIdx % kPpcEventRingSize) * 4);
        const uint32_t eventId = word >> 26;
        const uint32_t ppcFrame = word & 0x03FFFFFF;
        fprintf(file, "%u,%u,%s\n", ppcFrame, eventId, PpcEventName(eventId));
    }
    fclose(file);
    m_lastEventReadIdx = writeIdx;
}
