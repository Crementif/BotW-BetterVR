#pragma once

#include <optional>
#include <string>

namespace UpdateCheck {
    void Start();
    std::optional<std::string> GetNewerRelease();
}
