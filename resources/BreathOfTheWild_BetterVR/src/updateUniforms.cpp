#include "shared.h"

extern "C" {
    MTXInverse_t gameMTXInverse = (MTXInverse_t)0x03C6FF74;
    getRenderCamera_t gameGetRenderCamera = (getRenderCamera_t)0x03AE4AA0;
    void recalcShadowUniforms(void* modelRenderContext, float eyeOffset, const float* renderEyeView);
    void recalcEnvViewUniforms(void* modelRenderContext, float eyeOffset, const float* renderEyeView);

    void memset(void *Dst, int Val, size_t Size) {
        uint8* p = (uint8*)Dst;
        for (size_t i = 0; i < Size; i++) {
            p[i] = (uint8)Val;
        }
    }
}


// stuff5 is VFC+0x154, tan(fovy/2) of the projection that's actually used

static inline float deriveTanHalfFovY(const float* projMtx, float fallback) {
    if (projMtx == nullptr || projMtx[5] == 0.0f) return fallback;
    return 1.0f / projMtx[5];
}

static inline float deriveAspect(const float* projMtx, float fallback) {
    if (projMtx == nullptr || projMtx[0] == 0.0f || projMtx[5] == 0.0f) return fallback;
    return projMtx[5] / projMtx[0];
}

// the sky and clouds would use the combined frustum of both eyes (VFC[view]+0xC0), so they get the average instead

static float s_stereoProj[16];

// replaces the bl to the sky/cloud uniform writers at 0x03403308 and 0x039D2034, r3 = VFC[view]+0xC0
extern "C" ATTR_KEEP const float* hook_getStereoProjection(const float* gameProj) {
    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1 || gameProj == nullptr) return gameProj;
    if (sharedMem->leftProj[0] == 0.0f || sharedMem->rightProj[0] == 0.0f) return gameProj;
    if (gameProj[14] > -0.5f) return gameProj;

    for (uint32 i = 0; i < 16; i++) {
        s_stereoProj[i] = 0.5f * (sharedMem->leftProj[i] + sharedMem->rightProj[i]);
    }

    return s_stereoProj;
}

// per core, other cores can be drawing cube map faces with the same sky function
extern "C" { ATTR_KEEP uint32 s_inVisibleSkyDraw[3] = { 0, 0, 0 }; }

// marks the visible sky draw so it isn't mixed up with the cube map faces
extern "C" ATTR_KEEP const float* hook_getVisibleSkyProjection(const float* gameProj) {
    uint32 core = _IMPORT_COREINIT_OSGetCoreId();
    if (core < 3) s_inVisibleSkyDraw[core] = 1;
    return hook_getStereoProjection(gameProj);
}

extern "C" ATTR_KEEP void hook_endVisibleSkyDraw(void) {
    uint32 core = _IMPORT_COREINIT_OSGetCoreId();
    if (core < 3) s_inVisibleSkyDraw[core] = 0;
}

// cloud layers (sub_3A59CF4), a layer that doesn't match the camera stays the same for both eyes

#define CLOUD_LAYERS 3

struct CloudLayerCapture {
    uint32 uniformBlock;
    uint32 valid;
    float mvp[16];
    float viewM[16];
    float proj[16];
};

extern "C" {
CloudLayerCapture s_cloudLayers[CLOUD_LAYERS];
float s_cloudMonoView[12];
}

static uint32 s_cloudLayerCount = 0;
static uint32 s_cloudProbeFrame = 0;

// setData_ stores little endian
static inline float readCloudF32LE(uint32 va) {
    uint32 raw = readU32((void*)va, 0);
    uint32 tmp;
    writeWordByteReversed(&tmp, raw);
    return *(float*)&tmp;
}

static inline void readUniformSlot(uint32 uniformBlock, uint32 slot, float* dst, uint32 rowCount) {
    uint32 dstVA;
    uint32 wordsPerRow;
    resolveSlot(uniformBlock, slot, &dstVA, &wordsPerRow);

    uint32 ptrA = readU32((void*)uniformBlock, 0);
    uint32 descTableBase = readU32((void*)ptrA, 0);
    uint32 descriptor = descTableBase + slot * 12;
    uint8 typeIdx = readU8((void*)descriptor, 8);

    uint32 strideBytes = getStrideBytes(uniformBlock, typeIdx);
    if (strideBytes < 16) strideBytes = 16;

    for (uint32 r = 0; r < rowCount; r++) {
        for (uint32 c = 0; c < 4; c++) {
            dst[r * 4 + c] = readCloudF32LE(dstVA + r * strideBytes + c * 4);
        }
    }
}

static inline void mtx44Mul(const float* a, const float* b, float* out) {
    for (uint32 r = 0; r < 4; r++) {
        for (uint32 c = 0; c < 4; c++) {
            out[r * 4 + c] = a[r * 4 + 0] * b[c] + a[r * 4 + 1] * b[4 + c] + a[r * 4 + 2] * b[8 + c] + a[r * 4 + 3] * b[12 + c];
        }
    }
}

static inline void mtx34To44(const float* m34, float* out44) {
    for (uint32 i = 0; i < 12; i++) out44[i] = m34[i];
    out44[12] = 0.0f;
    out44[13] = 0.0f;
    out44[14] = 0.0f;
    out44[15] = 1.0f;
}

// returning the accumulator compiles to bdzlr, which Cemu's assembler rejects
static inline void cloudCompareMatrices(const float* a, const float* b, float* outMaxDiff, float* outMaxAbs) {
    float worstDiff = 0.0f;
    float worstAbs = 0.0f;
    for (uint32 i = 0; i < 16; i++) {
        float d = a[i] - b[i];
        if (d < 0.0f) d = -d;
        if (d > worstDiff) worstDiff = d;
        float m = b[i];
        if (m < 0.0f) m = -m;
        if (m > worstAbs) worstAbs = m;
    }
    *outMaxDiff = worstDiff;
    *outMaxAbs = worstAbs;
}

// right after sub_3A59CF4, r3 = cloud manager, r4 = view index, r5 = layer base, r6 = VFC[view]+0x00
extern "C" ATTR_KEEP void hook_captureCloudUniforms(uint32 cloudMgr, uint32 viewIdx, uint32 layerBase, uint32 monoView) {
    s_cloudLayerCount = 0;
    if (cloudMgr == 0 || monoView == 0) return;

    uint32 entry = readU32((void*)cloudMgr, 0x4B8);
    if (entry == 0) return;
    if (viewIdx < readU32((void*)cloudMgr, 0x4B4)) entry += 2168 * viewIdx;
    if (readU8((void*)entry, 2164) == 0) return;

    const float* mono = (const float*)monoView;
    for (uint32 i = 0; i < 12; i++) s_cloudMonoView[i] = mono[i];

    uint32 validCount = 0;
    for (uint32 L = 0; L < CLOUD_LAYERS; L++) {
        CloudLayerCapture* c = &s_cloudLayers[L];
        c->valid = 0;
        c->uniformBlock = entry + 44 * layerBase + 352 * L;

        readUniformSlot(c->uniformBlock, 0, c->mvp, 4);
        readUniformSlot(c->uniformBlock, 1, c->viewM, 4);
        readUniformSlot(c->uniformBlock, 2, c->proj, 4);

        float check[16];
        mtx44Mul(c->proj, c->viewM, check);
        float diff;
        float scale;
        cloudCompareMatrices(check, c->mvp, &diff, &scale);
        if (scale < 1.0f) scale = 1.0f;
        if (diff <= scale * 0.01f) c->valid = 1;
        validCount += c->valid;
    }

    if ((s_cloudProbeFrame++ % PROBE_INTERVAL) == 0) {
        PROBE_LOG("STEREO clouds=%u valid=%u ub=%08X", (uint32)CLOUD_LAYERS, validCount, s_cloudLayers[0].uniformBlock);
    }

    s_cloudLayerCount = CLOUD_LAYERS;
}

extern "C" ATTR_KEEP void hook_recalcCloudUniforms(float eyeOffset) {
    if (s_cloudLayerCount == 0) return;

    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1) return;

    const float* eyeView = (eyeOffset < 0.0f) ? sharedMem->leftView : sharedMem->rightView;
    const float* eyeProj = (eyeOffset < 0.0f) ? sharedMem->leftProj : sharedMem->rightProj;
    if (eyeProj[0] == 0.0f) return;

    float invMono[12];
    gameMTXInverse(s_cloudMonoView, invMono);

    float eyeView44[16];
    float invMono44[16];
    mtx34To44(eyeView, eyeView44);
    mtx34To44(invMono, invMono44);

    float corr[16];
    mtx44Mul(eyeView44, invMono44, corr);

    for (uint32 L = 0; L < s_cloudLayerCount; L++) {
        CloudLayerCapture* c = &s_cloudLayers[L];
        if (c->valid == 0 || c->uniformBlock == 0) continue;

        float viewM[16];
        mtx44Mul(corr, c->viewM, viewM);

        float proj[16];
        for (uint32 i = 0; i < 16; i++) proj[i] = c->proj[i];
        proj[0] = eyeProj[0];
        proj[2] = eyeProj[2];
        proj[5] = eyeProj[5];
        proj[6] = eyeProj[6];

        float mvp[16];
        mtx44Mul(proj, viewM, mvp);

        writeUniformSlot(c->uniformBlock, 0, mvp, 4);
        writeUniformSlot(c->uniformBlock, 1, viewM, 4);
        writeUniformSlot(c->uniformBlock, 2, proj, 4);
    }
}

// ViewFrustumCullingArray entries are 0x31C apart, [0] is the main camera and [3..6] the shadow cascades
// view at +0x00, the culling copy at +0x60 and near/far/fovy/aspect/tan(fovy/2) at +0xE0..+0xF4

// calcGpuViewFlush (0x3A0ADE8) copies slots 0-3 to the previous frame slots 11-14, so each eye keeps its own copy
struct PreviousViewSlots {
    uint32 valid;
    float view[12];
    float viewProj[16];
    float proj[16];
    float invView[12];
};

static PreviousViewSlots s_previousViewSlots[2];

static void writePreviousViewSlots(uint32 uniformBlock, uint32 eye, const float* viewMtx, const float* viewProjMtx, const float* projMtx, const float* invViewMtx) {
    PreviousViewSlots* previous = &s_previousViewSlots[eye];
    if (previous->valid != 0) {
        writeUniformSlot(uniformBlock, 11, previous->view, 3);
        writeUniformSlot(uniformBlock, 12, previous->viewProj, 4);
        writeUniformSlot(uniformBlock, 13, previous->proj, 4);
        writeUniformSlot(uniformBlock, 14, previous->invView, 3);
    }
    for (uint32 i = 0; i < 12; i++) {
        previous->view[i] = viewMtx[i];
        previous->invView[i] = invViewMtx[i];
    }
    for (uint32 i = 0; i < 16; i++) {
        previous->viewProj[i] = viewProjMtx[i];
        previous->proj[i] = projMtx[i];
    }
    previous->valid = 1;
}

extern "C" ATTR_KEEP void hook_recalcGpuView(void* modelRenderContext, float eyeOffset) {
    uint32 ctxPtr = (uint32)modelRenderContext;

    uint32 shaderCtx = readU32((void*)ctxPtr, 0x1A8);
    if (shaderCtx == 0) return;

    uint32 cullArraySize = readU32((void*)ctxPtr, 0x08);
    uint32 cullArraySrc = readU32((void*)ctxPtr, 0x0C);
    if (cullArraySize == 0 || cullArraySrc == 0) return;

    // getRenderCamera re-enters the module, which refreshes leftView/rightView from the live pose
    void* layer = *(void**)ptrAdd(modelRenderContext, 0x208);
    if (layer != nullptr) {
        void* camera = gameGetRenderCamera(layer);
        if (camera != nullptr) {
            const float cameraPosX = readF32(camera, 0x34);
            const float cameraAtZ = readF32(camera, 0x48);
            if (cameraPosX == 0.0f || (cameraAtZ < 1.1920929e-07f && cameraAtZ > -1.1920929e-07f)) {
                return;
            }
        }
    }

    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1) return;

    const bool left = eyeOffset < 0.0f;
    float* viewMtx = left ? sharedMem->leftView : sharedMem->rightView;
    float* projMtx = left ? sharedMem->leftProj : sharedMem->rightProj;
    float nearZ = left ? sharedMem->leftNear : sharedMem->rightNear;
    float farZ = left ? sharedMem->leftFar : sharedMem->rightFar;
    float fovy = left ? sharedMem->leftFovy : sharedMem->rightFovy;
    float stuff5 = deriveTanHalfFovY(projMtx, left ? sharedMem->leftStuff5 : sharedMem->rightStuff5);
    float aspect = deriveAspect(projMtx, left ? sharedMem->leftAspect : sharedMem->rightAspect);

    recalcShadowUniforms(modelRenderContext, eyeOffset, viewMtx);
    recalcEnvViewUniforms(modelRenderContext, eyeOffset, viewMtx);

    // shaderCtx+4 is the per view array, 48 bytes each
    uint32 shaderCtxField4 = readU32((void*)shaderCtx, 4);
    uint32 uniformBlock = shaderCtxField4 + 4;

    float invViewMtx[12];
    gameMTXInverse(viewMtx, invViewMtx);

    float viewProjMtx[16];
    mulProjView(projMtx, viewMtx, viewProjMtx);

    writeAllSlots(uniformBlock, viewMtx, viewProjMtx, projMtx, invViewMtx, nearZ, farZ, fovy, aspect, stuff5);
    writePreviousViewSlots(uniformBlock, eyeOffset < 0.0f ? 0 : 1, viewMtx, viewProjMtx, projMtx, invViewMtx);
}
