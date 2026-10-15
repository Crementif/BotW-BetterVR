#include "shared.h"

// water reflections (updateReflectionUniforms, 0x3A697F4) only need the view space x of the water plane moved per eye

#define MAX_REFLECTION_ENTRIES 8

struct ReflectionPlaneEntry {
    uint32 addr;  // GPU VA of block B slot 3 word 0 (view-space plane point x)
    float  x;     // mono x value
};

static ReflectionPlaneEntry s_refl[MAX_REFLECTION_ENTRIES];
static volatile uint32 s_reflCount = 0;

// record jobs run on other cores
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

// once per reflection view each frame (0x3A6A0DC)
extern "C" ATTR_KEEP void hook_captureReflectionPlane(uint32 ub, const float* src) {
    if (ub == 0 || src == 0) return;

    uint32 dstVA;
    uint32 wordsPerRow;
    resolveSlot(ub, 3, &dstVA, &wordsPerRow);
    if (dstVA == 0 || wordsPerRow != 4) return;  // expect a vec4 slot

    uint32 idx = atomicFetchAdd(&s_reflCount, 1);
    if (idx >= MAX_REFLECTION_ENTRIES) return;
    s_refl[idx].addr = dstVA;
    s_refl[idx].x    = src[0];
}

extern "C" ATTR_KEEP void hook_resetReflectionEntries(void) {
    s_reflCount = 0;
}

extern "C" ATTR_KEEP void hook_recalcReflectionUniforms(float eyeOffset) {
    uint32 count = s_reflCount;
    if (count > MAX_REFLECTION_ENTRIES) count = MAX_REFLECTION_ENTRIES;

    float viewOffsetX = getEyeViewOffsetX(eyeOffset);

    for (uint32 i = 0; i < count; i++) {
        ReflectionPlaneEntry* e = &s_refl[i];
        if (e->addr == 0) continue;
        gpuWriteFloatLE(e->addr, e->x + viewOffsetX);
    }
}
