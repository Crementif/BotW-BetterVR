#include "pch.h"

#include "imgui_internal.h"
#include "instance.h"
#include "utils/mod_settings.h"

#include <fstream>

ModSettings g_settings = {};

bool ModSettings::IsDebuggingToolsEnabled() const {
    return VRManager::instance().XR->GetRenderer() != nullptr && VRManager::instance().Debugger && GetSettings().enableDebuggerTools;
}

ModSettings& GetSettings() {
    return g_settings;
}

void Settings_LogSavedSettings() {
    Log::print<INFO>("VR Settings Saved:\n{}", GetSettings().ToString());
}

static void* Settings_ReadOpen(ImGuiContext*, ImGuiSettingsHandler*, const char* name) {
    if (strcmp(name, "Settings") != 0)
        return nullptr;
    return &GetSettings();
}

static constexpr std::array kRetiredOptionKeys = { "PlayMode" };

static void Settings_ReadLine(ImGuiContext*, ImGuiSettingsHandler*, void* entry, const char* line) {
    auto* s = (ModSettings*)entry;
    std::string_view lineView = line;
    if (lineView.empty()) return;
    // Remove leading whitespaces
    lineView.remove_prefix(std::min(lineView.find_first_not_of(" \t"), lineView.size()));
    if (line[0] == '#' || line[0] == ';') return; //ignore comments
    size_t sepIndex = lineView.find_first_of('=');
    if (sepIndex == std::string_view::npos) { //invalid string
        Log::print<ERROR>("Failed to parse illegal option line \"{}\": Missing key-value separator \"=\"", line);
        return;
    }
    std::string_view nameView = lineView.substr(0, sepIndex);
    // Remove trailing whitespaces
    nameView.remove_suffix(std::min(nameView.size() - nameView.find_last_not_of(" \t") - 1, nameView.size()));
    if (nameView.empty()) {
        Log::print<ERROR>("Failed to parse option line \"{}\": missing option key", line);
        return;
    }
    for (const char* retiredKey : kRetiredOptionKeys) {
        if (IEquals(retiredKey, nameView)) {
            Log::print<INFO>("Ignoring retired option \"{}\", it will be removed from the settings file on the next save", nameView);
            return;
        }
    }
    auto options = s->GetOptions();
    for (ModSettingBase* option : options) {
        if (IEquals(option->name, nameView)) {
            std::string_view valueView = lineView.substr(sepIndex + 1, lineView.size() - sepIndex - 1);
            // Remove leading whitespaces
            valueView.remove_prefix(std::min(valueView.find_first_not_of(" \t"), valueView.size()));
            // Remove trailing whitespaces
            valueView.remove_suffix(std::min(valueView.size() - valueView.find_last_not_of(" \t") - 1, valueView.size()));
            if (valueView.empty()) {
                Log::print<ERROR>("Failed to parse option line \"{}\": missing value", line);
                return;
            }
            option->Deserialize(valueView);
            return;
        }
    }
    Log::print<ERROR>("Failed to parse option line \"{}\": Unknown option key \"{}\"", line, nameView);
}

static void Settings_WriteAll(ImGuiContext* ctx, ImGuiSettingsHandler* handler, ImGuiTextBuffer* buf) {
    auto& s = GetSettings();
    buf->reserve(buf->size() + 1024);
    buf->appendf("[%s][Settings]\n", handler->TypeName);
    auto options = s.GetOptions();
    for (ModSettingBase* option : options) {
        std::string serialized = option->Serialize();
        buf->appendf("%s=%s\n", option->name, serialized.c_str());
    }
    buf->appendf("\n");
}

static void Settings_ReadFinish(ImGuiContext* ctx, ImGuiSettingsHandler* handler) {
    auto& s = GetSettings();
    Log::print<INFO>("VR Settings Loaded:\n{}", s.ToString());
}

void Settings_ReadFile(const std::filesystem::path& iniPath) {
    std::ifstream ini(iniPath);
    bool inSettings = false;
    for (std::string line; std::getline(ini, line);) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.starts_with('[')) {
            inSettings = line == "[BetterVR][Settings]";
        }
        else if (inSettings) {
            Settings_ReadLine(nullptr, nullptr, &GetSettings(), line.c_str());
        }
    }
}

void InitSettings() {
    ImGuiSettingsHandler ini_handler;
    ini_handler.TypeName = "BetterVR";
    ini_handler.TypeHash = ImHashStr("BetterVR");
    ini_handler.ReadOpenFn = Settings_ReadOpen;
    ini_handler.ReadLineFn = Settings_ReadLine;
    ini_handler.ApplyAllFn = Settings_ReadFinish;
    ini_handler.WriteAllFn = Settings_WriteAll;
    ImGui::AddSettingsHandler(&ini_handler);
}
