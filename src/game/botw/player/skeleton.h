#pragma once

namespace Skeleton {
    void RegisterHooks();

    bool IsBowDrawEngaged();
    bool IsArrowNearBowStart();
    // arrow speed scale from how far the arrow hand is pulled back from where the draw started
    float GetBowPullPower();
    float ConsumeReleasedBowPullPower();

    bool IsTwoHandGripActive();
    bool TwoHandGripConsumesGrabInput();
    void ReleaseTwoHandGripGrabInput();
}
