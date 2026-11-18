#include "pch.h"
#include "cemu_settings.h"

#include <pugixml.hpp>
#include <sstream>

namespace fs = std::filesystem;

static constexpr std::array<const char*, 6> SharedNodes = { "mlc_path", "console_language", "GamePaths", "GameCache", "Account", "AccountService" };

static pugi::xml_node Child(pugi::xml_node parent, const char* name) {
    pugi::xml_node child = parent.child(name);
    return child ? child : parent.append_child(name);
}

static void SetText(pugi::xml_node parent, const char* name, const char* value) {
    Child(parent, name).text().set(value);
}

static std::string Print(pugi::xml_node node) {
    std::ostringstream stream;
    node.print(stream, "", pugi::format_raw);
    return stream.str();
}

static bool CopyShared(const pugi::xml_document& from, pugi::xml_document& to) {
    pugi::xml_node source = from.child("content");
    pugi::xml_node content = Child(to, "content");
    bool changed = false;
    for (const char* name : SharedNodes) {
        if (Print(source.child(name)) == Print(content.child(name))) {
            continue;
        }
        changed = true;
        content.remove_child(name);
        if (pugi::xml_node shared = source.child(name)) {
            content.append_copy(shared);
        }
    }
    return changed;
}

void CemuSettings::Sync(const fs::path& cemuSettingsFile, const fs::path& settingsFile) {
    pugi::xml_document cemu;
    if (!cemu.load_file(cemuSettingsFile.c_str())) {
        return;
    }
    std::error_code error;
    if (!fs::exists(settingsFile, error)) {
        Log::print<INFO>("Starting from the settings in {}", cemuSettingsFile.string());
        fs::copy_file(cemuSettingsFile, settingsFile, error);
        if (error) {
            Log::print<ERROR>("Couldn't copy {} to {}: {}", cemuSettingsFile.string(), settingsFile.string(), error.message());
        }
        return;
    }
    // ShareBack doesn't run when Cemu crashes or gets killed, so whichever file changed last wins
    if (fs::last_write_time(settingsFile, error) > fs::last_write_time(cemuSettingsFile, error)) {
        ShareBack(settingsFile, cemuSettingsFile);
        return;
    }

    pugi::xml_document document;
    if (!document.load_file(settingsFile.c_str())) {
        return;
    }
    if (CopyShared(cemu, document) && !document.save_file(settingsFile.c_str(), "  ")) {
        Log::print<WARNING>("Couldn't write the settings shared with Cemu to {}", settingsFile.string());
    }
}

void CemuSettings::ShareBack(const fs::path& settingsFile, const fs::path& cemuSettingsFile) {
    pugi::xml_document document;
    pugi::xml_document cemu;
    if (!document.load_file(settingsFile.c_str()) || !cemu.load_file(cemuSettingsFile.c_str())) {
        return;
    }
    if (CopyShared(document, cemu)) {
        cemu.save_file(cemuSettingsFile.c_str(), "  ");
    }
}

void CemuSettings::Enforce(const fs::path& settingsFile) {
    pugi::xml_document document;
    if (!document.load_file(settingsFile.c_str())) {
        return;
    }
    pugi::xml_node graphic = Child(Child(document, "content"), "Graphic");
    SetText(graphic, "api", "1");
    SetText(graphic, "vkAccurateBarriers", "false");
    // the FIFO modes block presenting, mailbox and immediate don't
    if (int vsync = Child(graphic, "VSync").text().as_int(0); vsync == 1 || vsync == 3) {
        SetText(graphic, "VSync", "0");
    }
    if (!document.save_file(settingsFile.c_str(), "  ")) {
        Log::print<WARNING>("Couldn't write the settings VR needs to {}", settingsFile.string());
    }
}
