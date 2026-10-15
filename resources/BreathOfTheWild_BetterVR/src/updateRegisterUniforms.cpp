#include "shared.h"

// replaces agl::UniformLocation::setUniformNVN (0x3A788F8), the GX2 functions are read from its bl instructions at 0x3A78930 and 0x3A78948

typedef void (*gx2SetUniformReg_t)(uint32 offset, uint32 count, const void* data);
static gx2SetUniformReg_t s_gx2SetVertexUniformReg = nullptr;
static gx2SetUniformReg_t s_gx2SetPixelUniformReg = nullptr;

static gx2SetUniformReg_t decodeBlTarget(uint32 blAddr) {
    uint32 insn = readU32((void*)blAddr, 0);
    if ((insn & 0xFC000001) != 0x48000001) return nullptr;
    uint32 disp = insn & 0x03FFFFFC;
    if (disp & 0x02000000) disp |= 0xFC000000;
    if (insn & 0x2) return (gx2SetUniformReg_t)disp;
    return (gx2SetUniformReg_t)(blAddr + disp);
}

extern "C" { extern uint32 sModelRenderContextPtr; }

// camera uploads are found by caller (s_bakedCallSites), and an IT_HLE_VIEW_ALU_CONST after each makes Cemu load the eye's values instead

#define BAKED_KIND_VIEW34 0
#define BAKED_KIND_PROJ44 1
#define BAKED_KIND_INVVIEW34 2
#define BAKED_KIND_INVPROJ44 3
#define BAKED_KIND_YAWVIEW34 4
#define BAKED_KIND_COUNT 5

// matches camera uploads by content for callers missing from s_bakedCallSites
#define BETTERVR_BAKED_CONTENT_GATES

// logs every content-gate capture once per caller, kind and row
// #define BETTERVR_BAKED_DISCOVERY

// the sky function also draws the CubeMapMgr illuminant faces, the terrain pass uploads straight from the main VFC
#define BAKED_GATE_VISIBLE_SKY 1
#define BAKED_GATE_MAIN_VFC 2

struct BakedCallSite {
    uint32 lr;
    uint32 kind;
    uint32 words;
    uint32 gate;
};

static const BakedCallSite s_bakedCallSites[] = {
    { 0x033F7684, BAKED_KIND_INVPROJ44, 16, BAKED_GATE_VISIBLE_SKY }, // aglPfxSky_drawSky_regUniforms
    { 0x033F76B8, BAKED_KIND_INVVIEW34, 12, BAKED_GATE_VISIBLE_SKY },
    { 0x036CFB8C, BAKED_KIND_PROJ44, 16, BAKED_GATE_MAIN_VFC },       // sub_36CFAD8, the terrain half-res mask pass
    { 0x036CFBBC, BAKED_KIND_VIEW34, 12, BAKED_GATE_MAIN_VFC },
};

// one escape-free format string, tag 1 = discovery (lr, kind, row, count)
#ifdef BETTERVR_BAKED_DISCOVERY
#define BAKED_PROBE(tag, a, b, c, d) printToLog("BAKEDPROBE %u %08X %u %u %u", (uint32)(tag), (uint32)(a), (uint32)(b), (uint32)(c), (uint32)(d))
#endif

#ifdef BETTERVR_BAKED_DISCOVERY
#define MAX_BAKED_DISCOVERED 64
static uint32 s_bakedDiscovered[MAX_BAKED_DISCOVERED];
static uint32 s_bakedDiscoveredCount = 0;

static void bakedDiscover(uint32 callerLR, uint32 kind, uint32 row, uint32 count) {
    uint32 key = callerLR ^ (kind << 28) ^ (row << 20);
    for (uint32 i = 0; i < s_bakedDiscoveredCount; i++) {
        if (s_bakedDiscovered[i] == key) return;
    }
    if (s_bakedDiscoveredCount >= MAX_BAKED_DISCOVERED) return;
    s_bakedDiscovered[s_bakedDiscoveredCount++] = key;
    BAKED_PROBE(1, callerLR, kind, row, count);
}
#else
#define bakedDiscover(callerLR, kind, row, count) ((void)0)
#endif

uint32 g_bakedCaptured = 0;
uint32 g_bakedPatched = 0;
static uint32 s_bakedLogN = 0;

typedef void (*gx2GetCurDL_t)(uint32* outAddr, uint32* outSize);
static gx2GetCurDL_t s_gx2GetCurrentDisplayList = nullptr;

// false while no display list is being recorded
static bool bakedCurrentDisplayList(uint32* dlBase, uint32* dlSize) {
    if (s_gx2GetCurrentDisplayList == nullptr) {
        s_gx2GetCurrentDisplayList = (gx2GetCurDL_t)decodeBlTarget(0x03A74918);
        if (s_gx2GetCurrentDisplayList == nullptr) return false;
    }
    *dlBase = 0;
    *dlSize = 0;
    s_gx2GetCurrentDisplayList(dlBase, dlSize);
    return *dlBase != 0 && *dlSize >= 64 && *dlSize <= 0x1000000;
}

#define IT_HLE_VIEW_ALU_CONST 0xE2
#define ALU_CONST_VERTEX_BASE 0x400

static const uint32 s_bakedKindWords[BAKED_KIND_COUNT] = { 12, 16, 12, 16, 12 };
static uint32 s_bakedViewSlots[BAKED_KIND_COUNT][2][16];
// empty until the first fill
static uint32 s_bakedViewSlotsFilled = 0;

static void bakedEmitViewPacket(uint32 kind, uint32 registerOffset) {
    uint32 packet[5];
    packet[0] = pm4HeaderType3(IT_HLE_VIEW_ALU_CONST, 4);
    packet[1] = registerOffset;
    packet[2] = s_bakedKindWords[kind];
    packet[3] = _IMPORT_COREINIT_OSEffectiveToPhysical((uint32)s_bakedViewSlots[kind][0]);
    packet[4] = _IMPORT_COREINIT_OSEffectiveToPhysical((uint32)s_bakedViewSlots[kind][1]);
    _IMPORT_GX2_GX2CopyDisplayList(packet, sizeof(packet));
    g_bakedPatched++;
}

static void bakedFound(uint32 kind, uint32 dataAddr, uint32 uploadAddr, sint16 vsLoc, sint16 psLoc) {
    // only uploads recorded into a display list are replayed per eye
    uint32 dlBase = 0;
    uint32 dlSize = 0;
    if (s_bakedViewSlotsFilled == 0 || gVRSharedMemory.shared.dllActive != 1 || !bakedCurrentDisplayList(&dlBase, &dlSize)) return;

    uint32 wordOffset = (dataAddr - uploadAddr) >> 2;
    if (vsLoc != -1) bakedEmitViewPacket(kind, ALU_CONST_VERTEX_BASE + (uint32)(sint32)vsLoc + wordOffset);
    if (psLoc != -1) bakedEmitViewPacket(kind, (uint32)(sint32)psLoc + wordOffset);
    g_bakedCaptured++;
}

// each caller calculates its own inverse, so they can differ slightly
static bool bakedLooksLikeInvMainView(const float* rows, uint32 vfcView) {
    float n0 = rows[0] * rows[0] + rows[1] * rows[1] + rows[2] * rows[2];
    if (n0 < 0.98f || n0 > 1.02f) return false;
    float invMain[12];
    gameMTXInverse((float*)vfcView, invMain);
    float dot = rows[0] * invMain[0] + rows[1] * invMain[1] + rows[2] * invMain[2];
    if (dot < 0.999f) return false;
    float dx = rows[3] - invMain[3];
    float dy = rows[7] - invMain[7];
    float dz = rows[11] - invMain[11];
    if (dx < 0.0f) dx = -dx;
    if (dy < 0.0f) dy = -dy;
    if (dz < 0.0f) dz = -dz;
    return dx < 1.0f && dy < 1.0f && dz < 1.0f;
}

static inline float bakedRSqrt(float v) {
    if (v <= 0.0f) return 0.0f;
    union { float f; uint32 u; } c;
    c.f = v;
    c.u = 0x5F3759DF - (c.u >> 1);
    float x = c.f;
    x = x * (1.5f - 0.5f * v * x * x);
    x = x * (1.5f - 0.5f * v * x * x);
    return x;
}

// the cloud shadows use the camera's inverse with only the yaw left
static void bakedBuildYawFrame(const float* invView, float* out) {
    float bx = invView[2];
    float bz = invView[10];
    float n = bakedRSqrt(bx * bx + bz * bz);
    bx *= n;
    bz *= n;
    out[0] = bz;   out[1] = 0.0f; out[2] = bx;   out[3] = invView[3];
    out[4] = 0.0f; out[5] = 1.0f; out[6] = 0.0f; out[7] = invView[7];
    out[8] = -bx;  out[9] = 0.0f; out[10] = bz;  out[11] = invView[11];
}

// the env block is a frame behind the camera, so this only checks the shape and that it faces the same way
static bool bakedLooksLikeYawFrame(const float* rows, uint32 vfcView) {
    float ay = rows[5] - 1.0f;
    if (ay < 0.0f) ay = -ay;
    if (ay > 0.001f) return false;
    float z1 = rows[1] < 0.0f ? -rows[1] : rows[1];
    float z4 = rows[4] < 0.0f ? -rows[4] : rows[4];
    float z6 = rows[6] < 0.0f ? -rows[6] : rows[6];
    float z9 = rows[9] < 0.0f ? -rows[9] : rows[9];
    if (z1 > 0.001f || z4 > 0.001f || z6 > 0.001f || z9 > 0.001f) return false;

    // the game's values can be ~1% off
    float dDiag = rows[0] - rows[10];
    if (dDiag < 0.0f) dDiag = -dDiag;
    if (dDiag > 0.005f) return false;
    float dSkew = rows[2] + rows[8];
    if (dSkew < 0.0f) dSkew = -dSkew;
    if (dSkew > 0.05f) return false;
    float n0 = rows[0] * rows[0] + rows[1] * rows[1] + rows[2] * rows[2];
    if (n0 < 0.90f || n0 > 1.10f) return false;

    float invMain[12];
    gameMTXInverse((float*)vfcView, invMain);
    float expect[12];
    bakedBuildYawFrame(invMain, expect);
    for (uint32 i = 0; i < 12; i += 4) {
        float d0 = rows[i] - expect[i];
        float d2 = rows[i + 2] - expect[i + 2];
        float d3 = rows[i + 3] - expect[i + 3];
        if (d0 < 0.0f) d0 = -d0;
        if (d2 < 0.0f) d2 = -d2;
        if (d3 < 0.0f) d3 = -d3;
        if (d0 > 1.2f || d2 > 1.2f || d3 > 3.0f) return false;
    }
    return true;
}

static bool bakedLooksLikeInvMainProj(const float* rows, const float* proj) {
    if (proj[0] <= 0.0f || proj[5] <= 0.0f || rows[0] <= 0.0f || rows[5] <= 0.0f) return false;
    if (rows[1] != 0.0f || rows[4] != 0.0f) return false;
    float e0 = 1.0f / proj[0];
    float d0 = rows[0] - e0;
    if (d0 < 0.0f) d0 = -d0;
    if (d0 > e0 * 0.01f) return false;
    float e5 = 1.0f / proj[5];
    float d5 = rows[5] - e5;
    if (d5 < 0.0f) d5 = -d5;
    return d5 < e5 * 0.01f;
}

extern "C" { extern uint32 s_inVisibleSkyDraw[3]; }

static inline bool bakedIsSkyDrawLR(uint32 callerLR) {
    return callerLR >= 0x033F6A08 && callerLR < 0x033F7A24;
}

static inline uint32 bakedMainVfc(void) {
    uint32 ctx = sModelRenderContextPtr;
    if (ctx == 0) return 0;
    return readU32((void*)ctx, 0x0C);
}

static bool bakedCallSiteGateOpen(const BakedCallSite* site, uint32 dataAddr) {
    if (site->gate == BAKED_GATE_VISIBLE_SKY) {
        uint32 core = _IMPORT_COREINIT_OSGetCoreId();
        return core < 3 && s_inVisibleSkyDraw[core] != 0;
    }
    if (site->gate == BAKED_GATE_MAIN_VFC) {
        uint32 vfc = bakedMainVfc();
        if (vfc == 0) return false;
        return dataAddr == vfc + (site->kind == BAKED_KIND_PROJ44 ? 0xC0 : 0x60);
    }
    return true;
}

static void bakedRegScan(uint32 count, uint32 dataAddr, uint32 callerLR, sint16 vsLoc, sint16 psLoc) {
    if (dataAddr == 0 || count < 12) return;

    // a closed main-VFC gate falls through to the content gates
    for (uint32 i = 0; i < sizeof(s_bakedCallSites) / sizeof(s_bakedCallSites[0]); i++) {
        const BakedCallSite* site = &s_bakedCallSites[i];
        if (site->lr != callerLR) continue;
        if (count == site->words && bakedCallSiteGateOpen(site, dataAddr)) {
            bakedFound(site->kind, dataAddr, dataAddr, vsLoc, psLoc);
            return;
        }
        break;
    }

    // the sky function's other uploads draw the CubeMapMgr illuminant faces, which must stay world-aligned
    if (bakedIsSkyDrawLR(callerLR)) return;

#ifdef BETTERVR_BAKED_CONTENT_GATES
    uint32 vfc = bakedMainVfc();
    if (vfc == 0) return;

    const uint32* viewBits = (const uint32*)(vfc + 0x60);
    const uint32* projBits = (const uint32*)(vfc + 0xC0);
    const uint32* d = (const uint32*)dataAddr;
    uint32 rows = count >> 2;

    for (uint32 r = 0; r < rows; r++) {
        const uint32* p = d + r * 4;
        uint32 rem = (rows - r) * 4;
        if (rem >= 12 && p[0] == viewBits[0] && p[1] == viewBits[1]) {
            bool all = true;
            for (uint32 i = 2; i < 12; i++) {
                if (p[i] != viewBits[i]) {
                    all = false;
                    break;
                }
            }
            if (all) {
                bakedDiscover(callerLR, BAKED_KIND_VIEW34, r, count);
                bakedFound(BAKED_KIND_VIEW34, (uint32)p, dataAddr, vsLoc, psLoc);
                r += 2;
                continue;
            }
        }
        if (rem >= 16 && p[0] == projBits[0] && p[5] == projBits[5]) {
            bool all = true;
            for (uint32 i = 1; i < 16; i++) {
                if (p[i] != projBits[i]) {
                    all = false;
                    break;
                }
            }
            if (all) {
                bakedDiscover(callerLR, BAKED_KIND_PROJ44, r, count);
                bakedFound(BAKED_KIND_PROJ44, (uint32)p, dataAddr, vsLoc, psLoc);
                r += 3;
                continue;
            }
        }
        if (rem >= 12 && bakedLooksLikeYawFrame((const float*)p, vfc)) {
            bakedDiscover(callerLR, BAKED_KIND_YAWVIEW34, r, count);
            bakedFound(BAKED_KIND_YAWVIEW34, (uint32)p, dataAddr, vsLoc, psLoc);
            r += 2;
            continue;
        }
        if (rem >= 12 && bakedLooksLikeInvMainView((const float*)p, vfc)) {
            bakedDiscover(callerLR, BAKED_KIND_INVVIEW34, r, count);
            bakedFound(BAKED_KIND_INVVIEW34, (uint32)p, dataAddr, vsLoc, psLoc);
            r += 2;
            continue;
        }
        if (rem >= 16 && bakedLooksLikeInvMainProj((const float*)p, (const float*)(vfc + 0xC0))) {
            bakedDiscover(callerLR, BAKED_KIND_INVPROJ44, r, count);
            bakedFound(BAKED_KIND_INVPROJ44, (uint32)p, dataAddr, vsLoc, psLoc);
            r += 3;
            continue;
        }
    }
#endif
}

static bool buildInvProj44(const float* p, float* out) {
    if (p[0] == 0.0f || p[5] == 0.0f || p[11] == 0.0f) return false;
    for (uint32 i = 0; i < 16; i++) out[i] = 0.0f;
    out[0] = 1.0f / p[0];
    out[3] = p[2] / p[0];
    out[5] = 1.0f / p[5];
    out[7] = p[6] / p[5];
    out[11] = -1.0f;
    out[14] = 1.0f / p[11];
    out[15] = p[10] / p[11];
    return true;
}

// false while that eye's camera isn't known yet
static bool bakedBuildEyeValues(const float* view, const float* proj, float out[BAKED_KIND_COUNT][16]) {
    if (proj[0] == 0.0f) return false;
    if (view[0] == 0.0f && view[5] == 0.0f && view[10] == 0.0f) return false;
    if (!buildInvProj44(proj, out[BAKED_KIND_INVPROJ44])) return false;
    for (uint32 i = 0; i < 12; i++) out[BAKED_KIND_VIEW34][i] = view[i];
    for (uint32 i = 0; i < 16; i++) out[BAKED_KIND_PROJ44][i] = proj[i];
    gameMTXInverse((float*)view, out[BAKED_KIND_INVVIEW34]);
    bakedBuildYawFrame(out[BAKED_KIND_INVVIEW34], out[BAKED_KIND_YAWVIEW34]);
    return true;
}

// both eyes' slots are filled by the left prologue
static void bakedFillViewSlots(const VRSharedMemory* sm) {
    float values[2][BAKED_KIND_COUNT][16];
    if (!bakedBuildEyeValues(sm->leftView, sm->leftProj, values[0]) || !bakedBuildEyeValues(sm->rightView, sm->rightProj, values[1])) return;
    for (uint32 eye = 0; eye < 2; eye++) {
        for (uint32 kind = 0; kind < BAKED_KIND_COUNT; kind++) {
            gpuWriteWordsBE((uint32)s_bakedViewSlots[kind][eye], values[eye][kind], s_bakedKindWords[kind]);
        }
    }
    s_bakedViewSlotsFilled = 1;
}

extern "C" ATTR_KEEP void hook_recalcBakedRegUniforms(uint32 renderDisplay, float eyeOffset) {
    (void)renderDisplay;
    VRSharedMemory* sm = &gVRSharedMemory.shared;
    if (sm->dllActive != 1) return;
    const bool isLeftEye = eyeOffset < 0.0f;

    if (isLeftEye) bakedFillViewSlots(sm);
    gpuWriteSetView(isLeftEye ? 0 : 1);
}

extern "C" ATTR_KEEP void hook_resetBakedRegEntries(void) {
    if ((s_bakedLogN++ % PROBE_INTERVAL) == 0) {
        PROBE_LOG("STEREO bakedCaptured=%u bakedPatched=%u", g_bakedCaptured, g_bakedPatched);
    }
}

extern "C" ATTR_KEEP void hook_setUniformNVNFull(uint32 location, uint32 drawContext, uint32 count, uint32 data, uint32 callerLR) {
    (void)drawContext;
    if (s_gx2SetVertexUniformReg == nullptr) {
        s_gx2SetVertexUniformReg = decodeBlTarget(0x03A78930);
        s_gx2SetPixelUniformReg = decodeBlTarget(0x03A78948);
        // without these every register uniform in the game silently stops being written
        if (s_gx2SetVertexUniformReg == nullptr || s_gx2SetPixelUniformReg == nullptr) {
            printToLog("SETUNIFORMNVN decodeBlTarget FAILED vs=%08X ps=%08X", readU32((void*)0x03A78930, 0), readU32((void*)0x03A78948, 0));
            return;
        }
        PROBE_LOG("SETUNIFORMNVN live gx2 targets vs=%08X ps=%08X", (uint32)s_gx2SetVertexUniformReg, (uint32)s_gx2SetPixelUniformReg);
    }

    sint16 vsLoc = (sint16)readU16((void*)location, 0);
    if (vsLoc != -1) s_gx2SetVertexUniformReg((uint32)(sint32)vsLoc, count, (const void*)data);
    sint16 psLoc = (sint16)readU16((void*)location, 2);
    if (psLoc != -1) s_gx2SetPixelUniformReg((uint32)(sint32)psLoc, count, (const void*)data);

    // the view packets have to come after the upload
    if (vsLoc != -1 || psLoc != -1) {
        bakedRegScan(count, data, callerLR, vsLoc, psLoc);
    }
}
