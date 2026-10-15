#pragma once

#include "rendering/openxr.h"

namespace WeaponAttackDebugger {
    void DrawOverlays();
    bool IsOverlayVisible(OpenXR::EyeSide side);
    void SetOverlayVisible(OpenXR::EyeSide side, bool visible);
}
