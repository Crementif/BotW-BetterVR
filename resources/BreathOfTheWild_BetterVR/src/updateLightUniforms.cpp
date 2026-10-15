#include "shared.h"

extern "C" {
extern uint32 capturedXLow;
extern uint32 sModelRenderContextPtr;
}

extern "C" ATTR_KEEP void hook_recalcLightUniforms(float eyeOffset) {
    if (capturedXLow == 0) return;

    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1) return;

    uint32 eye = (eyeOffset < 0.0f) ? 0u : 1u;
    const float* viewMtx = eye == 0 ? sharedMem->leftView : sharedMem->rightView;
    const float* projMtx = eye == 0 ? sharedMem->leftProj : sharedMem->rightProj;
    float nearZ = eye == 0 ? sharedMem->leftNear : sharedMem->rightNear;
    float farZ = eye == 0 ? sharedMem->leftFar : sharedMem->rightFar;

    float proj00 = projMtx[0];
    float proj11 = projMtx[5];
    if (proj00 == 0.0f || proj11 == 0.0f) return;

    float projScaleX = 1.0f / proj00;
    float projScaleY = 1.0f / proj11;

    float viewProjMtx[16];
    mulProjView(projMtx, viewMtx, viewProjMtx);
    writeUniformSlot(capturedXLow, 0, viewProjMtx, 4);

    writeUniformSlot(capturedXLow, 1, viewMtx, 3);

    float invViewMtx[12];
    gameMTXInverse((float*)viewMtx, invViewMtx);
    writeUniformSlot(capturedXLow, 2, invViewMtx, 3);

    float slot4[8];
    slot4[0] = nearZ;
    slot4[1] = farZ - nearZ;
    slot4[2] = 0.0f;
    slot4[3] = 0.0f;
    slot4[4] = projScaleX;
    slot4[5] = -projScaleY;
    slot4[6] = projMtx[2] * projScaleX;
    slot4[7] = projMtx[6] * projScaleY;
    writeUniformSlot(capturedXLow, 4, slot4, 2);
}

// point lights (PointLightMgr::updateUBO, 0x38B63E0), rows 5 and 6 get rebuilt per eye from the world position

// usually ~10 per frame
#define MAX_LIGHT_BLOCKS 48

struct LightBlockEntry {
    uint32 blockPtr;  // agl::UniformBlock (light+0x50), *(blockPtr+4) is the buffer the display list binds
    uint32 descOff;   // member-0 offset inside that buffer (descriptor u16)
    float  worldX;    // staging +0x40, world position
    float  worldY;    // staging +0x44
    float  worldZ;    // staging +0x48
    float  mono60;    // staging +0x60: mono projected center x
    float  mono64;    // staging +0x64: mono projected center y
    float  mono68;    // staging +0x68: mono viewZ^2/r^2 (falloff scale)
};

extern "C" { LightBlockEntry s_lightBlocks[MAX_LIGHT_BLOCKS]; }
static volatile uint32 s_lightBlockCount = 0;

// malice volumes (GrudgeRenderer, 0x038AEED8)
struct ProxyMVPEntry {
    uint32 blockToken;
    uint32 bufA;
    uint32 bufB;
    float monoMVP[16];
    float monoInverse[12];
    float model[12];
    uint32 ok;
    uint32 lastSeenFrame;
};

// hook_uniformBlockDestroyed removes freed ones
#define MAX_PROXY_BLOCKS 128
#define PROXY_TOKEN_CLAIMED 0xFFFFFFFF

extern "C" { ProxyMVPEntry s_proxyBlocks[MAX_PROXY_BLOCKS]; }

// stamp buffers bound this frame (sub_38AFB7C), so only those get written per eye
struct StampBindEntry {
    uint32 buf;      // bound MVP buffer half, straight from the bind's r5
    uint32 entryIdx; // matching s_proxyBlocks index + 1, resolved by eye 0
};

#define MAX_STAMP_BINDS 128

extern "C" { StampBindEntry s_stampBinds[MAX_STAMP_BINDS]; }
static volatile uint32 s_stampBindCount = 0;

// capsule lights (updateUBForLight2), like the bright core of a fire
#define MAX_CAPSULE_BLOCKS 16

struct CapsuleEntry {
    uint32 blockPtr;  // agl::UniformBlock object (mgrEntry+80)
    float  w1x, w1y, w1z;  // mgrEntry+124..132, world position
    float  c1x, c1y, c1z;  // buf +0x40/44/48
};

extern "C" { CapsuleEntry s_capsuleBlocks[MAX_CAPSULE_BLOCKS]; }
static volatile uint32 s_capsuleBlockCount = 0;

#define MAX_SPOT_LIGHT_BLOCKS 8

struct SpotLightEntry {
    uint32 blockPtr;
    uint32 buf;
    uint32 mvpOff;
    uint32 mvpStride;
    uint32 posOff;
    uint32 dirOff;
    uint32 texOff;
    uint32 texStride;
    float  model[12];
    float  worldPos[3];
    float  worldDir[3];
    float  tex[16];
};

extern "C" { SpotLightEntry s_spotLightBlocks[MAX_SPOT_LIGHT_BLOCKS]; }
static volatile uint32 s_spotLightBlockCount = 0;

// setData_ stores little endian, and a shift swap would compile into lwbrx which Cemu's assembler doesn't support
static inline float readF32LE(uint32 va) {
    uint32 raw = readU32((void*)va, 0);
    uint32 tmp;
    writeWordByteReversed(&tmp, raw);
    return *(float*)&tmp;
}

// the camera this frame's display list was recorded with
static inline bool getRecordCameraView(float* out12) {
    uint32 ctx = sModelRenderContextPtr;
    if (ctx == 0) return false;
    void* layer = (void*)readU32((void*)ctx, 0x208);
    if (layer == nullptr) return false;
    float* cam = (float*)gameGetRenderCamera(layer);
    if (cam == nullptr) return false;
    for (int i = 0; i < 12; i++) out12[i] = cam[i];
    return true;
}

// half addresses are unique per proxy
static inline bool stampBufMatches(const ProxyMVPEntry* e, uint32 buf) {
    return buf == e->bufA || (e->bufB != 0 && buf == e->bufB);
}

static inline void mtx34MulVec(const float* m, float wx, float wy, float wz, float* out3) {
    out3[0] = m[0] * wx + m[1] * wy + m[2] * wz + m[3];
    out3[1] = m[4] * wx + m[5] * wy + m[6] * wz + m[7];
    out3[2] = m[8] * wx + m[9] * wy + m[10] * wz + m[11];
}

static inline uint32 atomicFetchAddLight(volatile uint32* p, uint32 v) {
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

static inline uint32 atomicCompareSwapLight(volatile uint32* p, uint32 expected, uint32 desired) {
    uint32 old;
    __asm__ volatile(
        "1: lwarx  %0, 0, %1\n"
        "   cmpw   %0, %2\n"
        "   bne-   2f\n"
        "   stwcx. %3, 0, %1\n"
        "   bne-   1b\n"
        "2:\n"
        : "=&r"(old)
        : "r"(p), "r"(expected), "r"(desired)
        : "cr0", "memory");
    return old;
}

// r3 = UniformBlock (light+0x50), r4 = staging (big endian), r5 = buf since *(block+4) isn't set yet
extern "C" ATTR_KEEP void hook_capturePointLightUBO(uint32 uniformBlock, uint32 staging, uint32 buf, uint32 asmB4) {
    if (uniformBlock == 0 || staging == 0 || buf == 0) return;

    uint32 ptrA = readU32((void*)uniformBlock, 0);
    uint32 descTableBase = readU32((void*)ptrA, 0);
    uint16 descOffset = readU16((void*)descTableBase, 4);

    uint32 idx = atomicFetchAddLight(&s_lightBlockCount, 1);
    if (idx >= MAX_LIGHT_BLOCKS) return;

    LightBlockEntry* e = &s_lightBlocks[idx];
    e->blockPtr = uniformBlock;
    e->descOff  = descOffset;
    e->worldX   = readF32((void*)staging, 0x40);
    e->worldY   = readF32((void*)staging, 0x44);
    e->worldZ   = readF32((void*)staging, 0x48);
    e->mono60   = readF32((void*)staging, 0x60);
    e->mono64   = readF32((void*)staging, 0x64);
    e->mono68   = readF32((void*)staging, 0x68);
    (void)asmB4;
}

// the last setData_ in updateUBForLight2/3, r3 = UniformBlock (mgrEntry+80), r4 = member 2 staging, r5 = buf
extern "C" ATTR_KEEP void hook_captureCapsuleLightUBO(uint32 uniformBlock, uint32 staging, uint32 buf) {
    if (uniformBlock == 0 || staging == 0 || buf == 0) return;
    uint32 mgr = uniformBlock - 80;

    uint32 idx = atomicFetchAddLight(&s_capsuleBlockCount, 1);
    if (idx >= MAX_CAPSULE_BLOCKS) return;

    CapsuleEntry* e = &s_capsuleBlocks[idx];
    e->blockPtr = uniformBlock;
    e->w1x = readF32((void*)mgr, 124);
    e->w1y = readF32((void*)mgr, 128);
    e->w1z = readF32((void*)mgr, 132);
    e->c1x = readF32LE(buf + 0x40);
    e->c1y = readF32LE(buf + 0x44);
    e->c1z = readF32LE(buf + 0x48);
}

static inline uint32 getMemberOffset(uint32 uniformBlock, uint32 slot, uint32 elem, uint32* outStride) {
    uint32 desc = readU32((void*)readU32((void*)uniformBlock, 0), 0) + slot * 12;
    uint32 stride = getStrideBytes(uniformBlock, readU8((void*)desc, 8));
    if (outStride != nullptr) *outStride = stride;
    return readU16((void*)desc, 4) + stride * elem;
}

extern "C" ATTR_KEEP void hook_captureSpotLightUBO(uint32 uniformBlock, uint32 buf, uint32 light, uint32 frame) {
    if (uniformBlock == 0 || buf == 0 || light == 0 || frame == 0) return;

    float v[12];
    if (!getRecordCameraView(v)) return;
    const float* worldPos = (const float*)(light + 0xC);
    const float* viewPos = (const float*)(frame + 0x6C);
    const float* viewDir = (const float*)(frame + 0x78);
    float check[3];
    mtx34MulVec(v, worldPos[0], worldPos[1], worldPos[2], check);
    float drift = 0.0f;
    for (uint32 i = 0; i < 3; i++) {
        float d = check[i] - viewPos[i];
        drift += d < 0.0f ? -d : d;
    }
    if (drift > 0.01f) return;

    uint32 idx = atomicFetchAddLight(&s_spotLightBlockCount, 1);
    if (idx >= MAX_SPOT_LIGHT_BLOCKS) return;

    SpotLightEntry* e = &s_spotLightBlocks[idx];
    e->blockPtr = uniformBlock;
    e->buf = buf;
    e->mvpOff = getMemberOffset(uniformBlock, 0, 0, &e->mvpStride);
    e->posOff = getMemberOffset(uniformBlock, 2, 2, nullptr);
    e->dirOff = getMemberOffset(uniformBlock, 2, 3, nullptr);
    e->texStride = 0;

    const float* model = (const float*)(frame + 0x84);
    for (uint32 i = 0; i < 12; i++) e->model[i] = model[i];
    for (uint32 i = 0; i < 3; i++) {
        e->worldPos[i] = worldPos[i];
        e->worldDir[i] = v[i] * viewDir[0] + v[4 + i] * viewDir[1] + v[8 + i] * viewDir[2];
    }
}

extern "C" ATTR_KEEP void hook_captureSpotLightTex(uint32 uniformBlock, uint32 src) {
    if (uniformBlock == 0 || src == 0) return;

    uint32 count = s_spotLightBlockCount;
    if (count > MAX_SPOT_LIGHT_BLOCKS) count = MAX_SPOT_LIGHT_BLOCKS;
    SpotLightEntry* e = nullptr;
    for (uint32 i = count; i > 0; i--) {
        if (s_spotLightBlocks[i - 1].blockPtr == uniformBlock) {
            e = &s_spotLightBlocks[i - 1];
            break;
        }
    }
    if (e == nullptr) return;

    float v[12];
    if (!getRecordCameraView(v)) return;
    const float* m = (const float*)src;
    for (uint32 r = 0; r < 4; r++) {
        for (uint32 c = 0; c < 4; c++) {
            e->tex[r * 4 + c] = m[r * 4] * v[c] + m[r * 4 + 1] * v[4 + c] + m[r * 4 + 2] * v[8 + c];
        }
        e->tex[r * 4 + 3] += m[r * 4 + 3];
    }
    e->texOff = getMemberOffset(uniformBlock, 1, 0, &e->texStride);
    if (e->texStride < 16) e->texStride = 16;
}

// both member 0 setData_ calls in sub_38AEED8, member 1's staging is at +0xA0
// the proxy's world matrix is at UniformBlock+0x58
extern "C" ATTR_KEEP void hook_captureLightProxyMVP(uint32 uniformBlock, uint32 staging, uint32 buf, uint32 inverseStaging) {
    if (uniformBlock == 0 || staging == 0 || buf == 0 || inverseStaging == 0) return;

    // only the 4+3 vec4 layout the malice shaders use
    uint32 desc = readU32((void*)readU32((void*)uniformBlock, 0), 0);
    uint32 valid = readU16((void*)desc, 4) == 0 && readU16((void*)desc, 16) == 0x40;
    valid = valid && readU32((void*)uniformBlock, 8) >= 0x70;
    valid = valid && getStrideBytes(uniformBlock, readU8((void*)desc, 8)) == 16;
    valid = valid && getStrideBytes(uniformBlock, readU8((void*)desc, 20)) == 16;

    for (uint32 i = 0; i < MAX_PROXY_BLOCKS; i++) {
        ProxyMVPEntry* e = &s_proxyBlocks[i];
        if (e->blockToken != uniformBlock) continue;
        if (buf != e->bufA && buf != e->bufB) {
            if (e->bufB == 0) {
                e->bufB = buf;
            } else {
                e->bufA = buf;
                e->bufB = 0;
            }
        }
        for (uint32 j = 0; j < 16; j++) e->monoMVP[j] = ((const float*)staging)[j];
        for (uint32 j = 0; j < 12; j++) {
            e->monoInverse[j] = ((const float*)inverseStaging)[j];
            e->model[j] = ((const float*)(uniformBlock + 0x58))[j];
        }
        e->ok = valid;
        e->lastSeenFrame = sFrameCounter;
        return;
    }

    // no free slot, so reuse the oldest one and write the token last so a half written entry never matches
    ProxyMVPEntry* slot = nullptr;
    for (uint32 i = 0; i < MAX_PROXY_BLOCKS; i++) {
        ProxyMVPEntry* e = &s_proxyBlocks[i];
        if (e->blockToken != 0) continue;
        if (atomicCompareSwapLight((volatile uint32*)&e->blockToken, 0, PROXY_TOKEN_CLAIMED) == 0) {
            slot = e;
            break;
        }
    }
    if (slot == nullptr) {
        for (uint32 i = 0; i < MAX_PROXY_BLOCKS; i++) {
            ProxyMVPEntry* e = &s_proxyBlocks[i];
            if (e->blockToken == PROXY_TOKEN_CLAIMED) continue;
            if (slot == nullptr || e->lastSeenFrame < slot->lastSeenFrame) slot = e;
        }
        if (slot == nullptr) return;
        slot->blockToken = PROXY_TOKEN_CLAIMED;
    }
    slot->bufA = buf;
    slot->bufB = 0;
    for (uint32 j = 0; j < 16; j++) slot->monoMVP[j] = ((const float*)staging)[j];
    for (uint32 j = 0; j < 12; j++) {
        slot->monoInverse[j] = ((const float*)inverseStaging)[j];
        slot->model[j] = ((const float*)(uniformBlock + 0x58))[j];
    }
    slot->ok = valid;
    slot->lastSeenFrame = sFrameCounter;
    slot->blockToken = uniformBlock;
}

// r3 = the bound stamp buffer, from the binds in sub_38AFB7C
extern "C" ATTR_KEEP void hook_captureStampBind(uint32 buf) {
    if (buf == 0) return;
    uint32 count = s_stampBindCount;
    if (count > MAX_STAMP_BINDS) count = MAX_STAMP_BINDS;
    for (uint32 i = 0; i < count; i++) {
        if (s_stampBinds[i].buf == buf) return;
    }
    uint32 idx = atomicFetchAddLight(&s_stampBindCount, 1);
    if (idx >= MAX_STAMP_BINDS) return;
    s_stampBinds[idx].buf = buf;
    s_stampBinds[idx].entryIdx = 0;
}

static bool getReplayCamera(float eyeOffset, const float** outView, const float** outProj) {
    VRSharedMemory* shared = &gVRSharedMemory.shared;
    if (shared->dllActive != 1) return false;
    *outView = eyeOffset < 0.0f ? shared->leftView : shared->rightView;
    *outProj = eyeOffset < 0.0f ? shared->leftProj : shared->rightProj;
    return (*outProj)[0] != 0.0f && (*outProj)[5] != 0.0f;
}

static void RecalcSpotLightUniforms(float eyeOffset) {
    uint32 count = s_spotLightBlockCount;
    if (count > MAX_SPOT_LIGHT_BLOCKS) count = MAX_SPOT_LIGHT_BLOCKS;
    if (count == 0) return;

    const float* view;
    const float* proj;
    if (!getReplayCamera(eyeOffset, &view, &proj)) return;

    for (uint32 i = 0; i < count; i++) {
        SpotLightEntry* e = &s_spotLightBlocks[i];
        if (e->blockPtr == 0) continue;

        float viewModel[12];
        for (uint32 r = 0; r < 3; r++) {
            for (uint32 c = 0; c < 4; c++) {
                viewModel[r * 4 + c] = view[r * 4] * e->model[c] + view[r * 4 + 1] * e->model[4 + c] + view[r * 4 + 2] * e->model[8 + c];
                if (c == 3) viewModel[r * 4 + c] += view[r * 4 + 3];
            }
        }
        float mvp[16];
        mulProjView(proj, viewModel, mvp);
        gpuWriteRows(e->buf + e->mvpOff, e->mvpStride < 16 ? 16 : e->mvpStride, mvp, 4);

        float viewPos[3];
        float viewDir[3];
        mtx34MulVec(view, e->worldPos[0], e->worldPos[1], e->worldPos[2], viewPos);
        for (uint32 r = 0; r < 3; r++) {
            viewDir[r] = view[r * 4] * e->worldDir[0] + view[r * 4 + 1] * e->worldDir[1] + view[r * 4 + 2] * e->worldDir[2];
        }
        gpuWriteWordsLE(e->buf + e->posOff, viewPos, 3);
        gpuWriteWordsLE(e->buf + e->dirOff, viewDir, 3);

        if (e->texStride == 0) continue;
        float eyePos[3];
        for (uint32 k = 0; k < 3; k++) {
            eyePos[k] = -(view[k] * view[3] + view[4 + k] * view[7] + view[8 + k] * view[11]);
        }
        float tex[16];
        for (uint32 r = 0; r < 4; r++) {
            const float* t = &e->tex[r * 4];
            for (uint32 c = 0; c < 3; c++) {
                tex[r * 4 + c] = t[0] * view[c * 4] + t[1] * view[c * 4 + 1] + t[2] * view[c * 4 + 2];
            }
            tex[r * 4 + 3] = t[0] * eyePos[0] + t[1] * eyePos[1] + t[2] * eyePos[2] + t[3];
        }
        gpuWriteRows(e->buf + e->texOff, e->texStride, tex, 4);
    }
}

// the malice shader expects a centered projection, so the eye's off-center part goes into the inverse view instead
static void RecalcGrudgeUniforms(float eyeOffset) {
    uint32 count = s_stampBindCount;
    if (count > MAX_STAMP_BINDS) count = MAX_STAMP_BINDS;
    if (count == 0) return;

    const float* view;
    const float* proj;
    if (!getReplayCamera(eyeOffset, &view, &proj)) return;

    for (uint32 b = 0; b < count; b++) {
        StampBindEntry* bind = &s_stampBinds[b];
        if (eyeOffset < 0.0f) {
            bind->entryIdx = 0;
            for (uint32 i = 0; i < MAX_PROXY_BLOCKS; i++) {
                ProxyMVPEntry* e = &s_proxyBlocks[i];
                if (e->blockToken == 0 || e->blockToken == PROXY_TOKEN_CLAIMED || !e->ok) continue;
                if (!stampBufMatches(e, bind->buf)) continue;
                e->lastSeenFrame = sFrameCounter;
                bind->entryIdx = i + 1;
                break;
            }
        }
        if (bind->buf == 0 || bind->entryIdx == 0) continue;
        ProxyMVPEntry* e = &s_proxyBlocks[bind->entryIdx - 1];
        if (e->blockToken == 0 || !e->ok) continue;

        float viewModel[12];
        for (uint32 r = 0; r < 3; r++) {
            for (uint32 c = 0; c < 4; c++) {
                viewModel[r * 4 + c] = view[r * 4] * e->model[c] + view[r * 4 + 1] * e->model[4 + c] + view[r * 4 + 2] * e->model[8 + c];
                if (c == 3) viewModel[r * 4 + c] += view[r * 4 + 3];
            }
        }
        float inverse[12];
        gameMTXInverse(viewModel, inverse);
        for (uint32 r = 0; r < 3; r++) {
            inverse[r * 4 + 2] -= inverse[r * 4] * (proj[2] / proj[0]) + inverse[r * 4 + 1] * (proj[6] / proj[5]);
            for (uint32 c = 0; c < 3; c++) viewModel[r * 4 + c] *= 1.4142135623730951f;
        }
        float mvp[16];
        mulProjView(proj, viewModel, mvp);
        gpuWriteWordsLE(bind->buf, mvp, 16);
        gpuWriteWordsLE(bind->buf + 0x40, inverse, 12);
    }
}

extern "C" ATTR_KEEP void hook_recalcPointLightUniforms(float eyeOffset) {
    // these also fix the projection, so they have to run even at zero IPD
    RecalcGrudgeUniforms(eyeOffset);
    RecalcSpotLightUniforms(eyeOffset);
    uint32 count = s_lightBlockCount;
    uint32 bindCount = s_stampBindCount;
    if (count == 0 && bindCount == 0) return;
    if (count > MAX_LIGHT_BLOCKS) count = MAX_LIGHT_BLOCKS;
    if (bindCount > MAX_STAMP_BINDS) bindCount = MAX_STAMP_BINDS;

    float d = getEyeViewOffsetX(eyeOffset);
    if (d == 0.0f) return;

    float monoView[12];
    if (!getRecordCameraView(monoView)) return;

    VRSharedMemory* sharedMemL = &gVRSharedMemory.shared;
    const float* eyeView = (eyeOffset < 0.0f) ? sharedMemL->leftView : sharedMemL->rightView;

    for (uint32 i = 0; i < count; i++) {
        LightBlockEntry* e = &s_lightBlocks[i];
        if (e->blockPtr == 0) continue;
        uint32 liveBuf = readU32((void*)e->blockPtr, 4);
        if (liveBuf == 0) continue;
        uint32 dst = liveBuf + e->descOff;

        float v[3];
        float m[3];
        mtx34MulVec(eyeView, e->worldX, e->worldY, e->worldZ, v);
        mtx34MulVec(monoView, e->worldX, e->worldY, e->worldZ, m);

        // row 5: view-space position
        gpuWriteWordsLE(dst + 0x50, v, 3);

        // row 6, applied as a difference on top of the original values
        if (v[2] != 0.0f && m[2] != 0.0f) {
            float rayMy = m[1] / m[2];
            float dPos = e->mono64 - rayMy;
            float dNeg = e->mono64 + rayMy;
            if (dPos < 0.0f) dPos = -dPos;
            if (dNeg < 0.0f) dNeg = -dNeg;
            float sy = (dPos <= dNeg) ? 1.0f : -1.0f;
            float row6[3];
            row6[0] = e->mono60 + (v[0] / v[2] - m[0] / m[2]);
            row6[1] = e->mono64 + sy * (v[1] / v[2] - rayMy);
            row6[2] = e->mono68 * (v[2] * v[2]) / (m[2] * m[2]);
            gpuWriteWordsLE(dst + 0x60, row6, 3);
        }
    }

    uint32 capsuleCount = s_capsuleBlockCount;
    if (capsuleCount > MAX_CAPSULE_BLOCKS) capsuleCount = MAX_CAPSULE_BLOCKS;
    if (capsuleCount == 0) return;

    for (uint32 i = 0; i < capsuleCount; i++) {
        CapsuleEntry* e = &s_capsuleBlocks[i];
        if (e->blockPtr == 0) continue;
        uint32 liveBuf = readU32((void*)e->blockPtr, 4);
        if (liveBuf == 0) continue;

        // make sure this is still the block we captured
        float chk = readF32LE(liveBuf + 0x40) - e->c1x;
        if (chk < 0.0f) chk = -chk;
        if (chk > 0.25f) continue;

        float v1[3];
        float m1[3];
        mtx34MulVec(eyeView, e->w1x, e->w1y, e->w1z, v1);
        mtx34MulVec(monoView, e->w1x, e->w1y, e->w1z, m1);

        // +0x00, view space light position
        gpuWriteWordsLE(liveBuf + 0x00, v1, 3);

        // +0x40, same as row 6
        if (v1[2] != 0.0f && m1[2] != 0.0f) {
            float rayMy = m1[1] / m1[2];
            float dPos = e->c1y - rayMy;
            float dNeg = e->c1y + rayMy;
            if (dPos < 0.0f) dPos = -dPos;
            if (dNeg < 0.0f) dNeg = -dNeg;
            float sy = (dPos <= dNeg) ? 1.0f : -1.0f;
            float center[3];
            center[0] = e->c1x + (v1[0] / v1[2] - m1[0] / m1[2]);
            center[1] = e->c1y + sy * (v1[1] / v1[2] - rayMy);
            center[2] = e->c1z * (v1[2] * v1[2]) / (m1[2] * m1[2]);
            gpuWriteWordsLE(liveBuf + 0x40, center, 3);
        }
    }
}

extern "C" ATTR_KEEP void hook_resetPointLightUBOs(void) {
    s_lightBlockCount = 0;
    s_capsuleBlockCount = 0;
    s_spotLightBlockCount = 0;

    // restore the original values so the next frame doesn't start with an eye's values
    uint32 bindCount = s_stampBindCount;
    if (bindCount > MAX_STAMP_BINDS) bindCount = MAX_STAMP_BINDS;
    for (uint32 b = 0; b < bindCount; b++) {
        StampBindEntry* bind = &s_stampBinds[b];
        uint32 idx = bind->entryIdx;
        if (bind->buf == 0 || idx == 0) continue;
        ProxyMVPEntry* e = &s_proxyBlocks[idx - 1];
        if (!e->ok) continue;
        if (e->blockToken == 0) continue;
        gpuWriteWordsLE(bind->buf, e->monoMVP, 16);
        gpuWriteWordsLE(bind->buf + 0x40, e->monoInverse, 12);
    }
    s_stampBindCount = 0;
}

extern "C" void envInvalidateBlock(uint32 uniformBlock);

// every agl::UniformBlock gets destroyed through 0x3A836F0, so forget it before the memory gets reused
extern "C" ATTR_KEEP void hook_uniformBlockDestroyed(uint32 uniformBlock) {
    if (uniformBlock == 0) return;
    if (capturedXLow == uniformBlock) capturedXLow = 0;

    uint32 lc = s_lightBlockCount;
    if (lc > MAX_LIGHT_BLOCKS) lc = MAX_LIGHT_BLOCKS;
    for (uint32 i = 0; i < lc; i++) {
        if (s_lightBlocks[i].blockPtr == uniformBlock) s_lightBlocks[i].blockPtr = 0;
    }

    uint32 cc = s_capsuleBlockCount;
    if (cc > MAX_CAPSULE_BLOCKS) cc = MAX_CAPSULE_BLOCKS;
    for (uint32 i = 0; i < cc; i++) {
        if (s_capsuleBlocks[i].blockPtr == uniformBlock) s_capsuleBlocks[i].blockPtr = 0;
    }

    uint32 sc = s_spotLightBlockCount;
    if (sc > MAX_SPOT_LIGHT_BLOCKS) sc = MAX_SPOT_LIGHT_BLOCKS;
    for (uint32 i = 0; i < sc; i++) {
        if (s_spotLightBlocks[i].blockPtr == uniformBlock) s_spotLightBlocks[i].blockPtr = 0;
    }

    for (uint32 i = 0; i < MAX_PROXY_BLOCKS; i++) {
        if (s_proxyBlocks[i].blockToken == uniformBlock) s_proxyBlocks[i].blockToken = 0;
    }

    envInvalidateBlock(uniformBlock);
}
