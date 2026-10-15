#pragma once

namespace Bow {
    void RegisterHooks();
    void OnGameFrame();
    void QueueAimingArcPreview(long frameIdx = -1);
}
