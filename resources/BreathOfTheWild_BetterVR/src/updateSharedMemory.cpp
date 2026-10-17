#include "shared.h"

extern "C" {
    ATTR_KEEP VRSharedMemoryRoot gVRSharedMemory;

    // frames left to draw a single image for camera rune photos and save file pictures
    ATTR_KEEP uint32 sMonoCaptureFrames = 0;
    // set each frame by the left eye's prologue
    ATTR_KEEP uint32 sStereoIssueThisFrame = 0;
    // no VR session, so draw once from the normal camera
    ATTR_KEEP uint32 sMonoWithoutSession = 1;
    // set while recording the 3D scene, menus and loading screens don't have one
    ATTR_KEEP uint32 sRecordedScene3D = 0;

    // two frames so a capture raised after this frame's prologues is still covered
    ATTR_KEEP void hook_signalMonoCapture() {
        sMonoCaptureFrames = 2;
    }

    ATTR_KEEP void hook_updateVRSharedMemory(float eyeOffset) {
        uint32 timelineMemAddr = (uint32)&gVRSharedMemory.gpuTimeline;
        uint32 currentEye = (eyeOffset < 0.0f) ? 0 : 1;
        uint32 frameCounter = sFrameCounter;
        uint32 frameIdx = frameCounter % 2u;
        if (currentEye == 0) {
            sMonoWithoutSession = gVRSharedMemory.shared.vrActive != 1 ? 1u : 0u;
            // frames without the 3D scene only need to be drawn once
            sStereoIssueThisFrame = sMonoWithoutSession == 0 && sRecordedScene3D != 0 ? 1u : 0u;
            sRecordedScene3D = 0;
        }

        DEBUG_LOG("hook_updateVRSharedMemory: eyeOffset=%f -> currentEye=%u, frameCounter=%u, frameIdx=%u",
                  eyeOffset, currentEye, frameCounter, frameIdx);

        uint32 frameState[2];
        frameState[0] = frameIdx;
        frameState[1] = sMonoCaptureFrames != 0 ? 1u : 0u;
        gpuWriteWordBE(timelineMemAddr + offsetof(VRGpuTimelineMemory, currentEye), currentEye);
        gpuWriteWordsBE(timelineMemAddr + offsetof(VRGpuTimelineMemory, currentFrameCounter), frameState, 2);

        if (sMonoCaptureFrames > 0) {
            sMonoCaptureFrames--;
        }
    }

    void hook_recalcGpuView(void* modelRenderContext, float eyeOffset);
    void hook_recalcReflectionUniforms(float eyeOffset);
    void hook_recalcLightUniforms(float eyeOffset);
    void hook_recalcCloudUniforms(float eyeOffset);
    void hook_recalcParticleUniforms(float eyeOffset);
    void hook_recalcPointLightUniforms(float eyeOffset);

    // the right eye's prologue runs inside the stereo scope so its writes only apply to the right eye
    ATTR_KEEP void hook_beginStereoIssue(void* modelRenderContext, float eyeOffset) {
        if (sStereoIssueThisFrame == 0) return;
        gpuWriteViewScope(1);
        gpuWriteSetRightView(1);
        if (modelRenderContext != nullptr) hook_recalcGpuView(modelRenderContext, eyeOffset);
        hook_recalcReflectionUniforms(eyeOffset);
        hook_recalcLightUniforms(eyeOffset);
        hook_recalcCloudUniforms(eyeOffset);
        hook_recalcParticleUniforms(eyeOffset);
        hook_recalcPointLightUniforms(eyeOffset);
        gpuWriteSetRightView(0);
    }

    // the HUD and everything after it only gets drawn once
    ATTR_KEEP void hook_endStereoIssue() {
        if (sStereoIssueThisFrame == 0) return;
        gpuWriteViewScope(0);
        gpuWriteFlush();
    }
}
