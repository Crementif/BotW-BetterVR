#include "shared.h"

// nw::eft particles have a view (0x00), proj (0x40) and viewProj (0x80) block that's made for one eye

// the peak measured in-game is 8 blocks per frame
#define MAX_PARTICLE_BUFS 108
#define PARTICLE_BLOCK_SIZE 320

struct ParticleBufEntry {
    uint32 va;
    float  proj[16];
};

static ParticleBufEntry s_bufs[MAX_PARTICLE_BUFS];
static volatile uint32  s_bufCount = 0;

// drawList records on multiple cores
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

// stars (Pa_Star_00) have their own camera block, uploaded through setData_ at 0x038C4E70
#define MAX_STAR_BLOCKS 4

struct StarBlockEntry {
    uint32 va;
    float  m[16];
    float  v[12];          // inverse(P) * M, rows 0-2
    float  recordRot[9];   // the recorded view's rotation
};

extern "C" { extern uint32 sModelRenderContextPtr; }

static StarBlockEntry  s_stars[MAX_STAR_BLOCKS];
static volatile uint32 s_starCount = 0;

extern "C" ATTR_KEEP void hook_captureStarCamera(uint32 buf, uint32 src) {
    if (buf == 0 || src == 0 || sModelRenderContextPtr == 0) return;
    uint32 vfc = *(uint32*)(sModelRenderContextPtr + 0x0C);
    if (vfc == 0) return;

    const float* m = (const float*)src;
    const float* proj = (const float*)(vfc + 0xC0);
    const float* view = (const float*)(vfc + 0x60);
    if (proj[0] == 0.0f || proj[5] == 0.0f) return;

    float v[12];
    for (uint32 k = 0; k < 4; k++) {
        v[0 * 4 + k] = (m[0 * 4 + k] + proj[2] * m[3 * 4 + k]) / proj[0];
        v[1 * 4 + k] = (m[1 * 4 + k] + proj[6] * m[3 * 4 + k]) / proj[5];
        v[2 * 4 + k] = -m[3 * 4 + k];
    }
    float len0 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    float len1 = v[4] * v[4] + v[5] * v[5] + v[6] * v[6];
    float len2 = v[8] * v[8] + v[9] * v[9] + v[10] * v[10];
    // cube map faces and other cameras don't match
    if (len2 == 0.0f || len0 < len2 * 0.99f || len0 > len2 * 1.01f || len1 < len2 * 0.99f || len1 > len2 * 1.01f) return;

    uint32 idx = atomicFetchAdd(&s_starCount, 1);
    if (idx >= MAX_STAR_BLOCKS) return;
    StarBlockEntry* entry = &s_stars[idx];
    entry->va = buf;
    for (uint32 k = 0; k < 16; k++) entry->m[k] = m[k];
    for (uint32 k = 0; k < 12; k++) entry->v[k] = v[k];
    for (uint32 r = 0; r < 3; r++) {
        for (uint32 k = 0; k < 3; k++) entry->recordRot[r * 3 + k] = view[r * 4 + k];
    }
}

static void recalcStarCamera(const StarBlockEntry* entry, const float* eyeView, const float* eyeProj) {
    const float* m = entry->m;
    const float* v = entry->v;

    float q[9];
    for (uint32 i = 0; i < 3; i++) {
        for (uint32 j = 0; j < 3; j++) {
            q[i * 3 + j] = eyeView[i * 4 + 0] * entry->recordRot[j * 3 + 0] + eyeView[i * 4 + 1] * entry->recordRot[j * 3 + 1] + eyeView[i * 4 + 2] * entry->recordRot[j * 3 + 2];
        }
    }
    float vn[12];
    for (uint32 i = 0; i < 3; i++) {
        for (uint32 k = 0; k < 4; k++) {
            vn[i * 4 + k] = q[i * 3 + 0] * v[0 * 4 + k] + q[i * 3 + 1] * v[1 * 4 + k] + q[i * 3 + 2] * v[2 * 4 + k];
        }
    }

    float len3 = m[12] * m[12] + m[13] * m[13] + m[14] * m[14];
    float depthScale = -(m[8] * m[12] + m[9] * m[13] + m[10] * m[14]) / len3;

    float out[16];
    for (uint32 r = 0; r < 2; r++) {
        for (uint32 k = 0; k < 4; k++) {
            out[r * 4 + k] = eyeProj[r * 4 + 0] * vn[0 * 4 + k] + eyeProj[r * 4 + 1] * vn[1 * 4 + k] + eyeProj[r * 4 + 2] * vn[2 * 4 + k];
        }
        out[r * 4 + 3] += eyeProj[r * 4 + 3];
    }
    for (uint32 k = 0; k < 4; k++) {
        out[2 * 4 + k] = m[2 * 4 + k] + depthScale * (vn[2 * 4 + k] - v[2 * 4 + k]);
        out[3 * 4 + k] = -vn[2 * 4 + k];
    }
    gpuWriteWordsLE(entry->va, out, 16);
}

// handle = {VA, size, flag} from sub_3B684D8, still big endian here
extern "C" ATTR_KEEP void hook_captureParticleBuf(uint32 handle) {
    if (handle == 0) return;
    uint32 va   = *(uint32*)(handle + 0);
    uint32 size = *(uint32*)(handle + 4);
    if (va == 0 || size != PARTICLE_BLOCK_SIZE) return;

    uint32 idx = atomicFetchAdd(&s_bufCount, 1);
    if (idx >= MAX_PARTICLE_BUFS) return;

    ParticleBufEntry* e = &s_bufs[idx];
    e->va = va;
    for (uint32 k = 0; k < 16; k++)
        e->proj[k] = *(float*)(va + 0x40 + k * 4);
}

extern "C" ATTR_KEEP void hook_resetParticleBufs(void) {
    s_bufCount = 0;
    s_starCount = 0;
}

// false until the module has published a usable projection
static bool getEyeCamera(float eyeOffset, const float** outView, const float** outProj) {
    VRSharedMemory* sharedMem = &gVRSharedMemory.shared;
    if (sharedMem->dllActive != 1) return false;

    const float* view = (eyeOffset < 0.0f) ? sharedMem->leftView : sharedMem->rightView;
    const float* proj = (eyeOffset < 0.0f) ? sharedMem->leftProj : sharedMem->rightProj;
    if (proj[0] == 0.0f || proj[5] == 0.0f) return false;

    *outView = view;
    *outProj = proj;
    return true;
}

// the buffer is little endian by now
extern "C" ATTR_KEEP void hook_recalcParticleUniforms(float eyeOffset) {
    uint32 count = s_bufCount;
    if (count > MAX_PARTICLE_BUFS) count = MAX_PARTICLE_BUFS;

    const float* eyeView = nullptr;
    const float* eyeProj = nullptr;
    if (!getEyeCamera(eyeOffset, &eyeView, &eyeProj)) return;

    for (uint32 i = 0; i < count; i++) {
        ParticleBufEntry* e = &s_bufs[i];
        if (e->va == 0) continue;

        float proj[16];
        for (uint32 k = 0; k < 8; k++)  proj[k] = eyeProj[k];  // per-eye FOV + frustum offset
        for (uint32 k = 8; k < 16; k++) proj[k] = e->proj[k];  // block's own depth rows

        float vp[16];
        mulProjView(proj, eyeView, vp);

        gpuWriteWordsLE(e->va + 0x00, eyeView, 12);
        gpuWriteWordsLE(e->va + 0x40, proj, 8);
        gpuWriteWordsLE(e->va + 0x80, vp, 16);
    }

    uint32 starCount = s_starCount;
    if (starCount > MAX_STAR_BLOCKS) starCount = MAX_STAR_BLOCKS;
    for (uint32 i = 0; i < starCount; i++) {
        recalcStarCamera(&s_stars[i], eyeView, eyeProj);
    }
}
