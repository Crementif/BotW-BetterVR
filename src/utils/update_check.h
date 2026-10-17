#pragma once

#include <functional>
#include <optional>
#include <string>

namespace UpdateCheck {
    inline constexpr const char* kReleasesUrl = "https://github.com/Crementif/BotW-BetterVR/releases";

    // onNewerRelease runs on the check's thread with the newer release's tag
    void Start(std::function<void(const std::string&)> onNewerRelease);
    std::optional<std::string> GetNewerRelease();
}
