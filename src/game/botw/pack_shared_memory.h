#pragma once

// has to match shared.h in the pack
#pragma pack(push, 1)
struct VRSharedMemory {
    BEType<uint32_t> dllActive;
    BEMatrix34 leftView;
    BEMatrix44 leftProj;
    BEMatrix34 rightView;
    BEMatrix44 rightProj;
    BEType<float> leftNear;
    BEType<float> leftFar;
    BEType<float> leftFovy;
    BEType<float> leftAspect;
    BEType<float> leftStuff5;
    BEType<float> rightNear;
    BEType<float> rightFar;
    BEType<float> rightFovy;
    BEType<float> rightAspect;
    BEType<float> rightStuff5;
    BEType<uint32_t> vrActive;
};

struct VRGpuTimelineMemory {
    BEType<uint32_t> currentEye;
    BEType<uint32_t> frameBufferIdx;
    BEType<uint32_t> currentFrameCounter;
    BEType<uint32_t> monoCapture;
};

struct VRSharedMemoryRoot {
    VRSharedMemory shared;
    VRGpuTimelineMemory gpuTimeline;
};
#pragma pack(pop)

// has to match shared.h in the pack
enum class PackHostEvent : uint32_t {
    Capture3DColor = 1,
    Capture3DDepth = 2,
    Capture2D = 3,
};
