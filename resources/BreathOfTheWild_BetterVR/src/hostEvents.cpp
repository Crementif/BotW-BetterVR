#include "shared.h"

#define IT_HLE_HOST_EVENT 0xE0
#define HOST_EVENT_SURFACE_COLOR 1
#define HOST_EVENT_SURFACE_DEPTH 2

// GX2Surface / GX2ColorBuffer / GX2DepthBuffer field offsets
#define GX2_SURFACE_WIDTH 0x04
#define GX2_SURFACE_HEIGHT 0x08
#define GX2_SURFACE_FORMAT 0x14
#define GX2_SURFACE_IMAGE_PTR 0x24
#define GX2_SURFACE_TILE_MODE 0x30
#define GX2_SURFACE_PITCH 0x3C
#define GX2_BUFFER_VIEW_FIRST_SLICE 0x78

// agl::RenderBuffer: mColorBuffer[0] at +0x1C, mDepthTarget at +0x3C, each holding its GX2 buffer at +0xBC
#define AGL_RENDER_BUFFER_COLOR 0x1C
#define AGL_RENDER_BUFFER_DEPTH 0x3C
#define AGL_RENDER_TARGET_GX2_BUFFER 0xBC

static void emitHostEvent(uint32 eventId, uint32 surfaceKind, uint32 gx2Buffer) {
    uint32 packet[14];
    packet[0] = pm4HeaderType3(IT_HLE_HOST_EVENT, 13);
    packet[1] = eventId;
    packet[2] = 0;
    packet[3] = 0;
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = surfaceKind;
    packet[7] = _IMPORT_COREINIT_OSEffectiveToPhysical(readU32((void*)gx2Buffer, GX2_SURFACE_IMAGE_PTR));
    packet[8] = readU32((void*)gx2Buffer, GX2_SURFACE_FORMAT);
    packet[9] = readU32((void*)gx2Buffer, GX2_SURFACE_TILE_MODE);
    packet[10] = readU32((void*)gx2Buffer, GX2_SURFACE_WIDTH);
    packet[11] = readU32((void*)gx2Buffer, GX2_SURFACE_HEIGHT);
    packet[12] = readU32((void*)gx2Buffer, GX2_SURFACE_PITCH);
    packet[13] = readU32((void*)gx2Buffer, GX2_BUFFER_VIEW_FIRST_SLICE);
    _IMPORT_GX2_GX2CopyDisplayList(packet, sizeof(packet));
}

extern "C" uint32 sRecordedScene3D;

extern "C" ATTR_KEEP void hook_emitCaptureEvents3D(uint32 renderBuffer) {
    if (renderBuffer == 0) return;
    uint32 colorTarget = readU32((void*)renderBuffer, AGL_RENDER_BUFFER_COLOR);
    uint32 depthTarget = readU32((void*)renderBuffer, AGL_RENDER_BUFFER_DEPTH);
    if (colorTarget == 0 || depthTarget == 0) return;
    sRecordedScene3D = 1;
    emitHostEvent(HOST_EVENT_CAPTURE_3D_COLOR, HOST_EVENT_SURFACE_COLOR, colorTarget + AGL_RENDER_TARGET_GX2_BUFFER);
    emitHostEvent(HOST_EVENT_CAPTURE_3D_DEPTH, HOST_EVENT_SURFACE_DEPTH, depthTarget + AGL_RENDER_TARGET_GX2_BUFFER);
    // the HUD after this only needs to be drawn once, and without foveation
    gpuWriteMonoSection(1);
}

extern "C" ATTR_KEEP void hook_emitCaptureEvent2D(uint32 renderBuffer) {
    if (renderBuffer == 0) return;
    uint32 colorTarget = readU32((void*)renderBuffer, AGL_RENDER_BUFFER_COLOR);
    if (colorTarget == 0) return;
    emitHostEvent(HOST_EVENT_CAPTURE_2D, HOST_EVENT_SURFACE_COLOR, colorTarget + AGL_RENDER_TARGET_GX2_BUFFER);
}
