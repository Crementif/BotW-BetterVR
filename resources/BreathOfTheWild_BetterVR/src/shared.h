#pragma once


typedef unsigned int uint32;
typedef unsigned short uint16;
typedef unsigned char uint8;
typedef signed int sint32;
typedef signed short sint16;
typedef signed char sint8;
typedef unsigned int size_t;

#define ATTR_KEEP __attribute__((used))

#ifndef offsetof
#define offsetof(s, m) __builtin_offsetof(s, m)
#endif

struct VRSharedMemory {
    uint32 dllActive;
    float leftView[12];
    float leftProj[16];
    float rightView[12];
    float rightProj[16];
    float leftNear;
    float leftFar;
    float leftFovy;
    float leftAspect;
    float leftStuff5;
    float rightNear;
    float rightFar;
    float rightFovy;
    float rightAspect;
    float rightStuff5;
    uint32 vrActive;
};

struct VRGpuTimelineMemory {
    uint32 currentEye;
    uint32 frameBufferIdx;
    uint32 currentFrameCounter;
    uint32 monoCapture;
};

struct VRSharedMemoryRoot {
    VRSharedMemory shared;
    VRGpuTimelineMemory gpuTimeline;
};

// has to match PackHostEvent in pack_shared_memory.h
#define HOST_EVENT_CAPTURE_3D_COLOR 1
#define HOST_EVENT_CAPTURE_3D_DEPTH 2
#define HOST_EVENT_CAPTURE_2D 3

extern "C" {
extern VRSharedMemoryRoot gVRSharedMemory;
extern int _IMPORT_COREINIT___os_snprintf(char* buf, size_t n, const char* format, ...);
extern void _IMPORT_COREINIT_OSConsoleWrite(const char* str, size_t length);
extern void _IMPORT_GX2_GX2CopyDisplayList(void* addr, uint32 size);
extern uint32 _IMPORT_COREINIT_OSEffectiveToPhysical(uint32 effectiveAddr);
extern uint32 _IMPORT_COREINIT_OSGetCoreId(void);
extern void printToLog(const char* format, ...);
extern void memset(void* Dst, int Val, size_t Size);
extern uint32 sFrameCounter;
}

#define snprintf _IMPORT_COREINIT___os_snprintf
#define OSConsoleWrite _IMPORT_COREINIT_OSConsoleWrite

// no math library, tanf and __builtin_sqrtf fail to load and Cemu skips the fsqrts that sqrtf compiles to

// DEBUG_LOG fires per call, PROBE_LOG on a frame interval
// #define BETTERVR_DEBUG_LOGGING
// #define BETTERVR_STEREO_PROBES

#ifdef BETTERVR_DEBUG_LOGGING
#define DEBUG_LOG(...) printToLog(__VA_ARGS__)
#else
#define DEBUG_LOG(...) ((void)0)
#endif

#ifdef BETTERVR_STEREO_PROBES
#define PROBE_LOG(...) printToLog(__VA_ARGS__)
#else
#define PROBE_LOG(...) ((void)0)
#endif

#define PROBE_INTERVAL 120

static inline uint8* ptrAdd(void* p, uint32 offset) {
    return ((uint8*)p) + offset;
}

static inline uint32 readU32(void* base, uint32 offset) {
    return *(uint32*)ptrAdd(base, offset);
}

static inline uint16 readU16(void* base, uint32 offset) {
    return *(uint16*)ptrAdd(base, offset);
}

static inline uint8 readU8(void* base, uint32 offset) {
    return *(uint8*)ptrAdd(base, offset);
}

static inline float readF32(void* base, uint32 offset) {
    return *(float*)ptrAdd(base, offset);
}

static inline bool StringsEqual(const char* lhs, const char* rhs) {
    if (lhs == rhs) {
        return true;
    }
    if (lhs == nullptr || rhs == nullptr) {
        return false;
    }

    while (*lhs != '\0' && *rhs != '\0') {
        if (*lhs != *rhs) {
            return false;
        }

        lhs++;
        rhs++;
    }

    return *lhs == *rhs;
}

typedef void (*MTXInverse_t)(float* src, float* dst);
extern "C" MTXInverse_t gameMTXInverse;

typedef void* (*getRenderCamera_t)(void* layer);
extern "C" getRenderCamera_t gameGetRenderCamera;

typedef void (*Matrix44Inverse_t)(float* dst, const float* src);
extern "C" Matrix44Inverse_t gameMatrix44Inverse;

typedef void (*MTX44Concat_t)(const float* lhs, const float* rhs, float* dst);
extern "C" MTX44Concat_t gameMTX44Concat;

// eyeOffset only says which eye this is, the actual offset comes from the tracked views
static inline float getEyeViewOffsetX(float eyeOffset) {
    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1) return 0.0f;

    // both eyes face the same way and the camera sits halfway between them
    float halfSeparation = 0.5f * (sharedMem->leftView[3] - sharedMem->rightView[3]);
    return (eyeOffset < 0.0f) ? halfSeparation : -halfSeparation;
}

#define IT_MEM_WRITE 0x3D
#define IT_SURFACE_SYNC 0x43
#define pm4HeaderType3(op, count) (((op) << 8) | (((count) - 1) << 16) | 0xC0000000)

static inline void writeWordByteReversed(uint32* dst, uint32 value) {
    __asm__ volatile("stwbrx %0, 0, %1" : : "r"(value), "r"(dst) : "memory");
}

extern "C" {
void gpuWriteWordsLE(uint32 dstVA, const void* src, uint32 count);
void gpuWriteWordsBE(uint32 dstVA, const void* src, uint32 count);
// picks which view the next IT_HLE_VIEW_ALU_CONST packets load from
void gpuWriteSetView(uint32 view);
// Cemu draws everything inside a stereo scope for both eyes
void gpuWriteViewScope(uint32 isStereo);
// draws inside a mono section only happen once
void gpuWriteMonoSection(uint32 isMonoSection);
// while set, writes only apply to the right eye's draws
void gpuWriteSetRightView(uint32 isRightView);
void gpuWriteFlush(void);
}

static inline void gpuWriteFloatLE(uint32 dstVA, float value) {
    gpuWriteWordsLE(dstVA, &value, 1);
}

static inline void gpuWriteWordBE(uint32 dstVA, uint32 value) {
    gpuWriteWordsBE(dstVA, &value, 1);
}

static inline void resolveSlot(uint32 uniformBlock, uint32 slot, uint32* outDstVA, uint32* outWordsPerRow) {
    uint32 ptrA = readU32((void*)uniformBlock, 0);
    uint32 descTableBase = readU32((void*)ptrA, 0);
    uint32 descriptor = descTableBase + slot * 12;
    uint16 descOffset = readU16((void*)descriptor, 4);
    uint8 typeIdx = readU8((void*)descriptor, 8);
    *outWordsPerRow = *(uint8*)(0x10362A70 + typeIdx * 4);
    uint32 baseOffset = readU32((void*)uniformBlock, 4);
    *outDstVA = baseOffset + descOffset;
}

static inline uint32 getStrideBytes(uint32 uniformBlock, uint32 typeIdx) {
    uint32 callbackObj = readU32((void*)uniformBlock, 0x28);
    uint32 fnAddr = readU32((void*)callbackObj, 0x1C);
    typedef uint32 (*StrideFn)(uint32 self, uint32 typeIdx);
    StrideFn fn = (StrideFn)fnAddr;
    uint32 strideWords = fn(uniformBlock, typeIdx);
    return strideWords * 4;
}

static inline void gpuWriteRows(uint32 dstVA, uint32 strideBytes, const float* src, uint32 rowCount) {
    if (strideBytes == 16) {
        gpuWriteWordsLE(dstVA, src, rowCount * 4);
        return;
    }
    for (uint32 r = 0; r < rowCount; r++) {
        gpuWriteWordsLE(dstVA + r * strideBytes, src + r * 4, 4);
    }
}

static inline void writeUniformSlot(uint32 uniformBlock, uint32 slot, const float* src, uint32 rowCount) {
    uint32 dstVA;
    uint32 wordsPerRow;
    resolveSlot(uniformBlock, slot, &dstVA, &wordsPerRow);

    uint32 ptrA = readU32((void*)uniformBlock, 0);
    uint32 descTableBase = readU32((void*)ptrA, 0);
    uint32 descriptor = descTableBase + slot * 12;
    uint8 typeIdx = readU8((void*)descriptor, 8);

    uint32 strideBytes = getStrideBytes(uniformBlock, typeIdx);
    // agl uniform rows are at least a vec4
    if (strideBytes < 16) strideBytes = 16;
    gpuWriteRows(dstVA, strideBytes, src, rowCount);
}

static inline void mulProjView(const float* proj, const float* view, float* result) {
    for (int r = 0; r < 4; r++) {
        float p0 = proj[r * 4 + 0];
        float p1 = proj[r * 4 + 1];
        float p2 = proj[r * 4 + 2];
        float p3 = proj[r * 4 + 3];

        result[r * 4 + 0] = p0 * view[0] + p1 * view[4] + p2 * view[8];
        result[r * 4 + 1] = p0 * view[1] + p1 * view[5] + p2 * view[9];
        result[r * 4 + 2] = p0 * view[2] + p1 * view[6] + p2 * view[10];
        result[r * 4 + 3] = p0 * view[3] + p1 * view[7] + p2 * view[11] + p3;
    }
}

static inline void writeAllSlots(uint32 uniformBlock, const float* viewMtx, const float* viewProjMtx, const float* projMtx, const float* invViewMtx, float nearZ, float farZ, float fovy, float aspect, float stuff5) {
    if (uniformBlock == 0) return;

    writeUniformSlot(uniformBlock, 0, viewMtx, 3);
    writeUniformSlot(uniformBlock, 1, viewProjMtx, 4);
    writeUniformSlot(uniformBlock, 2, projMtx, 4);
    writeUniformSlot(uniformBlock, 3, invViewMtx, 3);

    // slot 4: {near, far, near/far, 1-(near/far)}
    float slot4[4];
    float nearOverFar = nearZ / farZ;
    slot4[0] = nearZ;
    slot4[1] = farZ;
    slot4[2] = nearOverFar;
    slot4[3] = 1.0f - nearOverFar;
    writeUniformSlot(uniformBlock, 4, slot4, 1);

    // slot 5: {1/(far-near), near*slot5[0], aspect, 1/aspect}
    float slot5[4];
    float rangeDiff = farZ - nearZ;
    float invRange = 1.0f / rangeDiff;
    slot5[0] = invRange;
    slot5[1] = nearZ * invRange;
    slot5[2] = aspect;
    slot5[3] = 1.0f / aspect;
    writeUniformSlot(uniformBlock, 5, slot5, 1);

    // slot 6: {far-near, 0, 0, 0}
    float slot6[4];
    slot6[0] = rangeDiff;
    slot6[1] = 0.0f;
    slot6[2] = 0.0f;
    slot6[3] = 0.0f;
    writeUniformSlot(uniformBlock, 6, slot6, 1);

    // slot 7: {stuff5*aspect, stuff5, fovy, 0}
    float slot7[4];
    slot7[0] = stuff5 * aspect;
    slot7[1] = stuff5;
    slot7[2] = fovy;
    slot7[3] = 0.0f;
    writeUniformSlot(uniformBlock, 7, slot7, 1);
}
