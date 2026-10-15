#include "shared.h"

// shadow cascades have the camera baked in, written by calcGPUDrawBackGround (0x39DEE9C, slot 17) and the ShadowPrePass builder (0x3B1BFD4, slot 0)

#define MAX_SHADOW_MATS 24

struct ShadowMat4Entry {
    uint32 rowAddr[4];  // GPU VA of each row's first word (col 0)
    float  m[16];       // mono source matrix, row-major
};

extern "C" { ShadowMat4Entry s_shadowMats[MAX_SHADOW_MATS]; }
static volatile uint32 s_shadowMatCount = 0;
static uint32 s_shadowProbeFrame = 0;

// the two writers record on different cores
static inline uint32 atomicFetchAdd(volatile uint32* p, uint32 v) {
    uint32 old, tmp;
    __asm__ volatile(
        "1: lwarx  %0, 0, %2\n"
        "   add    %1, %0, %3\n"
        "   stwcx. %1, 0, %2\n"
        "   bne-   1b\n"
        : "=&r"(old), "=&r"(tmp)
        : "r"(p), "r"(v)
        : "cr0", "memory");
    return old;
}

// the camera at record time, getRenderCamera would give the live pose
static inline bool getRecordCameraView(void* modelRenderContext, float* out12) {
    if (modelRenderContext == nullptr) return false;
    uint32 cullArraySrc = readU32(modelRenderContext, 0x0C);
    if (cullArraySrc == 0) return false;
    const float* bakedView = (const float*)cullArraySrc;
    for (int i = 0; i < 12; i++) out12[i] = bakedView[i];
    return true;
}

static inline void mtx34Mul(const float* a, const float* b, float* out) {
    for (int r = 0; r < 3; r++) {
        float a0 = a[r * 4 + 0];
        float a1 = a[r * 4 + 1];
        float a2 = a[r * 4 + 2];
        out[r * 4 + 0] = a0 * b[0] + a1 * b[4] + a2 * b[8];
        out[r * 4 + 1] = a0 * b[1] + a1 * b[5] + a2 * b[9];
        out[r * 4 + 2] = a0 * b[2] + a1 * b[6] + a2 * b[10];
        out[r * 4 + 3] = a0 * b[3] + a1 * b[7] + a2 * b[11] + a[r * 4 + 3];
    }
}

static void captureShadowMat4(uint32 ub, uint32 slot, uint32 elemBase, const float* src) {
    uint32 dstVA;
    uint32 wordsPerRow;
    resolveSlot(ub, slot, &dstVA, &wordsPerRow);
    if (dstVA == 0 || wordsPerRow != 4) return;  // expect vec4 elements

    uint32 ptrA = readU32((void*)ub, 0);
    uint32 descTableBase = readU32((void*)ptrA, 0);
    uint8 typeIdx = readU8((void*)(descTableBase + slot * 12), 8);
    uint32 strideBytes = getStrideBytes(ub, typeIdx);
    if (strideBytes == 0) return;

    uint32 idx = atomicFetchAdd(&s_shadowMatCount, 1);
    if (idx >= MAX_SHADOW_MATS) return;
    ShadowMat4Entry* e = &s_shadowMats[idx];
    for (uint32 r = 0; r < 4; r++)
        e->rowAddr[r] = dstVA + (elemBase + r) * strideBytes;
    for (uint32 k = 0; k < 16; k++)
        e->m[k] = src[k];
}

// cShadowBuffer slot 17, only the view space branch (bl 0x39DF234)
extern "C" ATTR_KEEP void hook_captureDepthShadowProj(uint32 shaderCtx, uint32 cascade, const float* src) {
    if (shaderCtx == 0 || src == 0 || cascade >= 4) return;
    uint32 field4 = readU32((void*)shaderCtx, 4);
    if (field4 == 0) return;
    captureShadowMat4(field4 + 4, 17, cascade * 4, src);  // UB = *(shaderCtx+4)+4
}

// ShadowPrePass block slot 0, 3 matrices at once (bl 0x3B1C1CC)
extern "C" ATTR_KEEP void hook_capturePrePassShadow(uint32 ub, const float* src) {
    if (ub == 0 || src == 0) return;
    for (uint32 m = 0; m < 3; m++)
        captureShadowMat4(ub, 0, m * 4, src + m * 16);
}

extern "C" ATTR_KEEP void hook_resetShadowEntries(void) {
    if ((s_shadowProbeFrame++ % PROBE_INTERVAL) == 0) {
        PROBE_LOG("STEREO shadows=%u max=%u", s_shadowMatCount, (uint32)MAX_SHADOW_MATS);
    }
    s_shadowMatCount = 0;
}

extern "C" ATTR_KEEP void recalcShadowUniforms(void* modelRenderContext, float eyeOffset, const float* renderEyeView) {
    uint32 count = s_shadowMatCount;
    if (count > MAX_SHADOW_MATS) count = MAX_SHADOW_MATS;
    if (count == 0) return;
    if (renderEyeView == nullptr) return;

    float monoView[12];
    if (!getRecordCameraView(modelRenderContext, monoView)) return;

    float eyeView[12];
    for (int i = 0; i < 12; i++) eyeView[i] = renderEyeView[i];
    // an empty view can't be inverted
    if (eyeView[0] == 0.0f && eyeView[5] == 0.0f && eyeView[10] == 0.0f) return;

    float invEye[12];
    gameMTXInverse(eyeView, invEye);
    float C[12];
    mtx34Mul(monoView, invEye, C);

    for (uint32 i = 0; i < count; i++) {
        ShadowMat4Entry* e = &s_shadowMats[i];
        const float* sm = e->m;
        for (uint32 row = 0; row < 4; row++) {
            float s0 = sm[row * 4 + 0];
            float s1 = sm[row * 4 + 1];
            float s2 = sm[row * 4 + 2];
            float s3 = sm[row * 4 + 3];
            float o[4];
            o[0] = s0 * C[0] + s1 * C[4] + s2 * C[8];
            o[1] = s0 * C[1] + s1 * C[5] + s2 * C[9];
            o[2] = s0 * C[2] + s1 * C[6] + s2 * C[10];
            o[3] = s0 * C[3] + s1 * C[7] + s2 * C[11] + s3;
            uint32 base = e->rowAddr[row];
            if (base == 0) continue;
            gpuWriteWordsLE(base, o, 4);
        }
    }
}
