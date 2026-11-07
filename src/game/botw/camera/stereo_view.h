#pragma once

namespace StereoView {
    void RegisterHooks();

    uint32_t GetSharedMemoryAddress();
    void ResetFlatCameraFov();
    // the game's own camera, before the headset moves it
    std::optional<glm::mat4> GetGameCameraWorld();
}
