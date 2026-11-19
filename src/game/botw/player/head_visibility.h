#pragma once

namespace HeadVisibility {
    bool IsFaceBone(std::string_view boneName);

    // run on Link's root bone, hides the head from the camera passes but keeps it in the shadow pass
    void Apply(bool hideHead);
}
