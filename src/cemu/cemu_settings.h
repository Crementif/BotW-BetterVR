#pragma once

#include <filesystem>

namespace CemuSettings {
    // copies the game paths, mlc, account and game list over from Cemu's settings
    void Sync(const std::filesystem::path& cemuSettingsFile, const std::filesystem::path& settingsFile);
    // and copies them back on exit
    void ShareBack(const std::filesystem::path& settingsFile, const std::filesystem::path& cemuSettingsFile);
    // forces Vulkan, no accurate barriers and a present mode that doesn't block
    void Enforce(const std::filesystem::path& settingsFile);
}
