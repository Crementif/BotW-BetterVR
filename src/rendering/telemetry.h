#pragma once

#include "pch.h"

class RND_D3D12;

// Frame-content telemetry for the *final OpenXR swapchain images*. Small regions are
// copied through a non-stalling readback ring and classified for blank frames,
// temporal flicker, frozen-eye output, and suspiciously identical stereo output.
// Full-resolution incident bursts are exported as BMPs for offline/LLM inspection.
class EyeTelemetry {
public:
    static constexpr uint32_t kSlots = 4;      // readback ring depth (frames in flight)
    static constexpr uint32_t kRegions = 5;    // center + 4 quadrant centers
    static constexpr uint32_t kRegionSize = 32;
    static constexpr uint32_t kRowPitch = 256; // D3D12_TEXTURE_DATA_PITCH_ALIGNMENT
    static constexpr uint64_t kEyeBufferSize = kRegions * kRegionSize * kRowPitch;

    enum class FrameState : uint8_t { UNKNOWN, NORMAL, WHITE, BLACK };

    struct EyeStats {
        FrameState state = FrameState::UNKNOWN;
        float meanLuma = 0.0f;
        float minRegionLuma = 0.0f;
        float maxRegionLuma = 0.0f;
        uint64_t whiteFrames = 0;
        uint64_t blackFrames = 0;
        uint64_t normalFrames = 0;
        uint64_t transitions = 0;
        uint32_t lastTransitionFrame = 0;
        uint64_t sampledFrames = 0;
        uint64_t droppedFrames = 0;
        uint64_t flickerEvents = 0;
        uint64_t frozenFrames = 0;
        float temporalDelta = 0.0f;
        float edgeEnergy = 0.0f;
        uint32_t lastSampleFrame = 0;
        uint32_t lastFlickerFrame = 0;
    };

    struct StereoStats {
        uint64_t sampledFrames = 0;
        uint64_t incompleteFrames = 0;
        uint64_t nearIdenticalFrames = 0;
        uint64_t asymmetricFlickerEvents = 0;
        uint64_t frozenEyeEvents = 0;
        float meanAbsDifference = 0.0f;
        float nearIdenticalFraction = 0.0f;
        uint32_t lastAnomalyFrame = 0;
        const char* lastAnomaly = "NONE";
    };

    static EyeTelemetry& instance();

    // records sample copies for one eye into the current ring slot; called inside the
    // frame's render command list, after the texture waits are already queued
    void RecordSample(ID3D12Device* device, ID3D12GraphicsCommandList* cmdList, int side, ID3D12Resource* texture);

    // stamps the current slot with the fence value that covers this frame's queue work
    // and advances the ring; no-op if RecordSample was not called this frame
    void OnFrameSubmitted(uint64_t fenceValue, uint32_t frameNo, uint32_t skipMask);

    // decodes any ring slots whose fence value has completed and updates stats/logs
    void Collect(uint64_t completedFenceValue);

    EyeStats GetStats(int side) const { return m_stats[side]; }
    StereoStats GetStereoStats() const { return m_stereoStats; }
    uint64_t GetDroppedRingFrames() const { return m_droppedRingFrames; }
    static const char* StateName(FrameState s) {
        switch (s) {
            case FrameState::NORMAL: return "NORMAL";
            case FrameState::WHITE: return "WHITE";
            case FrameState::BLACK: return "BLACK";
            default: return "UNKNOWN";
        }
    }

    // blocking full-texture readback of the final eye color swapchains, written as
    // BMPs to BetterVR_dumps\. Depth is deliberately excluded: D3D12 cannot copy the
    // runtime-owned D32 swapchains to a linear readback buffer on all runtimes, while
    // flicker review needs the exact submitted color images. Returns files written.
    int DumpEyeTextures(RND_D3D12* d3d12, ID3D12Resource* colorL, ID3D12Resource* colorR,
        uint32_t frameNo, D3D12_RESOURCE_STATES initialState = D3D12_RESOURCE_STATE_RENDER_TARGET,
        const char* namePrefix = "color");

private:
    struct Slot {
        ComPtr<ID3D12Resource> readback[2];
        uint8_t* mapped[2] = { nullptr, nullptr };
        DXGI_FORMAT format[2] = { DXGI_FORMAT_UNKNOWN, DXGI_FORMAT_UNKNOWN };
        bool sampled[2] = { false, false };
        bool pending = false;
        uint64_t fenceValue = 0;
        uint32_t frameNo = 0;
        uint32_t skipMask = 0;
    };

    void EnsureSlotBuffer(ID3D12Device* device, Slot& slot, int side);
    void DecodeSlot(const Slot& slot);

    std::array<Slot, kSlots> m_slots = {};
    uint32_t m_writeSlot = 0;
    std::array<EyeStats, 2> m_stats = {};
    StereoStats m_stereoStats = {};
    std::array<std::array<float, kRegions * kRegionSize * kRegionSize>, 2> m_previousLuma = {};
    std::array<bool, 2> m_hasPrevious = { false, false };
    uint64_t m_droppedRingFrames = 0;
};
