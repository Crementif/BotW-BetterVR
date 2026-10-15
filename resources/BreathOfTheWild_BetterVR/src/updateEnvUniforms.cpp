#include "shared.h"

// the env block has the inverse view for distant shadows somewhere in its data, so it's found by its shape

// three env UniformBlocks exist (ShaderContext+52, 80 bytes each)
#define MAX_ENV_VIEW_BLOCKS 8

#define ENV_BLOB_SLOT 34
#define ENV_BLOB_BYTES 0x1000

// the block and not the buffer, since beginFrameUBOs (0x3A0A5A0) swaps *(ub+4) every frame
struct EnvViewEntry {
    uint32 blockPtr;  // agl::UniformBlock object, *(blockPtr+4) = the buffer the DL binds
    uint32 rowOff;    // byte offset of the inverse-view rows inside that buffer
    float  m[12];
};

extern "C" { EnvViewEntry s_envViewBlocks[MAX_ENV_VIEW_BLOCKS]; }
static volatile uint32 s_envViewCount = 0;
static uint32 s_envProbeFrame = 0;

static inline uint32 atomicFetchAddEnv(volatile uint32* p, uint32 v) {
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

static bool isViewToWorld(const float* m) {
    float l0 = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
    float l1 = m[4] * m[4] + m[5] * m[5] + m[6] * m[6];
    float l2 = m[8] * m[8] + m[9] * m[9] + m[10] * m[10];
    if (l0 < 0.99f || l0 > 1.01f) return false;
    if (l1 < 0.99f || l1 > 1.01f) return false;
    if (l2 < 0.99f || l2 > 1.01f) return false;
    float d01 = m[0] * m[4] + m[1] * m[5] + m[2] * m[6];
    float d02 = m[0] * m[8] + m[1] * m[9] + m[2] * m[10];
    float d12 = m[4] * m[8] + m[5] * m[9] + m[6] * m[10];
    if (d01 < -0.01f || d01 > 0.01f) return false;
    if (d02 < -0.01f || d02 > 0.01f) return false;
    if (d12 < -0.01f || d12 > 0.01f) return false;
    return true;
}

static bool isTransposeRotation(const float* a, const float* b) {
    for (uint32 r = 0; r < 3; r++) {
        for (uint32 c = 0; c < 3; c++) {
            float d = a[r * 4 + c] - b[c * 4 + r];
            if (d < -0.002f || d > 0.002f) return false;
        }
    }
    return true;
}

// the setDataBulk_ call in gsys_ShaderContext_updateEnvUBO (0x3A0C8A0)
extern "C" ATTR_KEEP void hook_captureEnvViewRows(uint32 uniformBlock, uint32 bufferVA, uint32 slot, const float* blob) {
    if (slot != ENV_BLOB_SLOT || uniformBlock == 0 || bufferVA == 0 || blob == 0) return;

    // the inverse view, followed 0x30 later by the view
    uint32 found = 0xFFFFFFFF;
    for (uint32 off = 0; off + 0x60 <= ENV_BLOB_BYTES; off += 16) {
        const float* a = (const float*)((uint32)blob + off);
        if (!isViewToWorld(a)) continue;
        const float* b = (const float*)((uint32)blob + off + 0x30);
        if (!isTransposeRotation(a, b)) continue;
        found = off;
        break;
    }

    if (found == 0xFFFFFFFF) return;

    const float* src = (const float*)((uint32)blob + found);

    uint32 ptrA = readU32((void*)uniformBlock, 0);
    if (ptrA == 0) return;
    uint32 descTableBase = readU32((void*)ptrA, 0);
    if (descTableBase == 0) return;
    // a halfword, as resolveSlot reads it
    uint32 slotOffset = readU16((void*)(descTableBase + slot * 12), 4);
    uint32 rowOff = slotOffset + found;

    uint32 count = s_envViewCount;
    for (uint32 i = 0; i < count && i < MAX_ENV_VIEW_BLOCKS; i++) {
        if (s_envViewBlocks[i].blockPtr == uniformBlock) {
            EnvViewEntry* prev = &s_envViewBlocks[i];
            prev->rowOff = rowOff;
            for (uint32 k = 0; k < 12; k++) prev->m[k] = src[k];
            return;
        }
    }

    uint32 idx = atomicFetchAddEnv(&s_envViewCount, 1);
    if (idx >= MAX_ENV_VIEW_BLOCKS) return;
    EnvViewEntry* e = &s_envViewBlocks[idx];
    e->blockPtr = uniformBlock;
    e->rowOff = rowOff;
    for (uint32 k = 0; k < 12; k++) e->m[k] = src[k];
}

extern "C" ATTR_KEEP void envInvalidateBlock(uint32 uniformBlock) {
    uint32 count = s_envViewCount;
    if (count > MAX_ENV_VIEW_BLOCKS) count = MAX_ENV_VIEW_BLOCKS;
    for (uint32 i = 0; i < count; i++) {
        if (s_envViewBlocks[i].blockPtr == uniformBlock) s_envViewBlocks[i].blockPtr = 0;
    }
}

extern "C" ATTR_KEEP void hook_resetEnvViewEntries(void) {
    if ((s_envProbeFrame++ % PROBE_INTERVAL) == 0) {
        PROBE_LOG("STEREO env=%u blk=%08X", s_envViewCount, s_envViewCount ? s_envViewBlocks[0].blockPtr : 0);
    }
    s_envViewCount = 0;
}

extern "C" ATTR_KEEP void recalcEnvViewUniforms(void* modelRenderContext, float eyeOffset, const float* renderEyeView) {
    (void)modelRenderContext;
    uint32 count = s_envViewCount;
    if (count > MAX_ENV_VIEW_BLOCKS) count = MAX_ENV_VIEW_BLOCKS;
    if (count == 0) return;
    if (renderEyeView == nullptr) return;

    float eyeView[12];
    for (int i = 0; i < 12; i++) eyeView[i] = renderEyeView[i];
    if (eyeView[0] == 0.0f && eyeView[5] == 0.0f && eyeView[10] == 0.0f) return;

    float invEye[12];
    gameMTXInverse(eyeView, invEye);

    for (uint32 i = 0; i < count; i++) {
        EnvViewEntry* e = &s_envViewBlocks[i];
        if (e->blockPtr == 0) continue;
        uint32 liveBuf = readU32((void*)e->blockPtr, 4);
        if (liveBuf == 0) continue;
        gpuWriteWordsLE(liveBuf + e->rowOff, invEye, 12);
    }
}
