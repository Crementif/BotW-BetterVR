#include "shared.h"

#define MEM_WRITE_U32 0x40000
#define MEM_WRITE_U64_BE 0x00000
#define MEM_WRITE_U64_LE 0x20000

#define GPU_WRITE_PACKET_WORDS 5
#define GPU_WRITE_BATCH_WORDS 480

static uint32 s_batch[GPU_WRITE_BATCH_WORDS];
static uint32 s_batchWords = 0;

// s_syncEnd == 0 means there's no sync pending
static uint32 s_syncBegin = 0;
static uint32 s_syncEnd = 0;

static void copyBatchToDisplayList(void) {
    if (s_batchWords == 0) return;
    _IMPORT_GX2_GX2CopyDisplayList(s_batch, s_batchWords * 4);
    s_batchWords = 0;
}

// a sync that ends up in a later batch still runs before the replay
static inline void reservePacket(void) {
    if (s_batchWords + GPU_WRITE_PACKET_WORDS > GPU_WRITE_BATCH_WORDS) copyBatchToDisplayList();
}

static void emitPendingSync(void) {
    if (s_syncEnd == 0) return;
    reservePacket();
    uint32* p = s_batch + s_batchWords;
    p[0] = pm4HeaderType3(IT_SURFACE_SYNC, 4);
    p[1] = 0x8800000;
    p[2] = (s_syncEnd - s_syncBegin) >> 8;
    p[3] = s_syncBegin >> 8;
    p[4] = 0x00000004;
    s_batchWords += GPU_WRITE_PACKET_WORDS;
    s_syncEnd = 0;
}

extern "C" ATTR_KEEP void gpuWriteFlush(void) {
    emitPendingSync();
    copyBatchToDisplayList();
}

static void addSyncRange(uint32 phys, uint32 bytes) {
    uint32 begin = phys & ~0xFFu;
    uint32 end = (phys + bytes + 0xFF) & ~0xFFu;
    if (s_syncEnd != 0 && begin <= s_syncEnd && end >= s_syncBegin) {
        if (begin < s_syncBegin) s_syncBegin = begin;
        if (end > s_syncEnd) s_syncEnd = end;
        return;
    }
    emitPendingSync();
    s_syncBegin = begin;
    s_syncEnd = end;
}

#define IT_HLE_VIEW_SCOPE 0xE3
#define VIEW_SCOPE_MONO_SECTION_BEGIN 2
#define VIEW_SCOPE_MONO_SECTION_END 3
#define IT_HLE_VIEW_MEM_WRITE 0xE4
#define VIEW_MEM_WRITE_MAX_WORDS 64

static uint32 s_rightView = 0;

extern "C" ATTR_KEEP void gpuWriteSetRightView(uint32 isRightView) {
    s_rightView = isRightView;
}

// Cemu keeps these for the right eye's draws without touching memory, so they need no sync
static void writeRightViewRun(uint32 phys, const uint32* src, uint32 count, bool littleEndian) {
    while (count > 0) {
        uint32 words = count < VIEW_MEM_WRITE_MAX_WORDS ? count : VIEW_MEM_WRITE_MAX_WORDS;
        if (s_batchWords + 2 + words > GPU_WRITE_BATCH_WORDS) copyBatchToDisplayList();
        uint32* p = s_batch + s_batchWords;
        p[0] = pm4HeaderType3(IT_HLE_VIEW_MEM_WRITE, 1 + words);
        p[1] = phys;
        for (uint32 i = 0; i < words; i++) {
            if (littleEndian) {
                writeWordByteReversed(&p[2 + i], src[i]);
            } else {
                p[2 + i] = src[i];
            }
        }
        s_batchWords += 2 + words;
        phys += words * 4;
        src += words;
        count -= words;
    }
}

extern "C" ATTR_KEEP void gpuWriteViewScope(uint32 isStereo) {
    emitPendingSync();
    reservePacket();
    uint32* p = s_batch + s_batchWords;
    p[0] = pm4HeaderType3(IT_HLE_VIEW_SCOPE, 1);
    p[1] = isStereo;
    s_batchWords += 2;
}

// record jobs run on other cores, so this can't use the shared batch
extern "C" ATTR_KEEP void gpuWriteMonoSection(uint32 isMonoSection) {
    uint32 packet[2];
    packet[0] = pm4HeaderType3(IT_HLE_VIEW_SCOPE, 1);
    packet[1] = isMonoSection != 0 ? VIEW_SCOPE_MONO_SECTION_BEGIN : VIEW_SCOPE_MONO_SECTION_END;
    _IMPORT_GX2_GX2CopyDisplayList(packet, sizeof(packet));
}

// Cemu writes the 64-bit modes with a 64-bit atomic, so only aligned pairs can use them
static void writeRun(uint32 dstVA, const uint32* src, uint32 count, bool littleEndian) {
    if (count == 0) return;
    uint32 phys = _IMPORT_COREINIT_OSEffectiveToPhysical(dstVA);
    if (s_rightView != 0) {
        writeRightViewRun(phys, src, count, littleEndian);
        return;
    }
    uint32 addr = phys;
    uint32 i = 0;
    while (i < count) {
        reservePacket();
        uint32* p = s_batch + s_batchWords;
        p[0] = pm4HeaderType3(IT_MEM_WRITE, 4);
        p[1] = addr | 0x2;
        if ((addr & 7) == 0 && i + 1 < count) {
            p[2] = littleEndian ? MEM_WRITE_U64_LE : MEM_WRITE_U64_BE;
            p[3] = src[i];
            p[4] = src[i + 1];
            addr += 8;
            i += 2;
        } else {
            p[2] = MEM_WRITE_U32;
            if (littleEndian) {
                writeWordByteReversed(&p[3], src[i]);
            } else {
                p[3] = src[i];
            }
            p[4] = 0;
            addr += 4;
            i += 1;
        }
        s_batchWords += GPU_WRITE_PACKET_WORDS;
    }
    if (littleEndian) addSyncRange(phys, count * 4);
}

extern "C" ATTR_KEEP void gpuWriteWordsLE(uint32 dstVA, const void* src, uint32 count) {
    writeRun(dstVA, (const uint32*)src, count, true);
}

extern "C" ATTR_KEEP void gpuWriteWordsBE(uint32 dstVA, const void* src, uint32 count) {
    writeRun(dstVA, (const uint32*)src, count, false);
}

#define IT_HLE_SET_VIEW 0xE1

extern "C" ATTR_KEEP void gpuWriteSetView(uint32 view) {
    reservePacket();
    uint32* p = s_batch + s_batchWords;
    p[0] = pm4HeaderType3(IT_HLE_SET_VIEW, 1);
    p[1] = view;
    s_batchWords += 2;
}
