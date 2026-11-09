#include "pch.h"

#include "imgui_menus.h"
#include "bettervr_version.h"

#include "cemu/cemu_host.h"
#include "game/game.h"
#include "game/botw/debug/weapon_attack_debugger.h"
#include "imgui_internal.h"
#include "instance.h"
#include "rendering/renderer.h"
#include "rendering/openxr.h"
#include "utils/profiler.h"
#include "utils/mod_settings.h"
#include "utils/update_check.h"
#include "supporters_generated.h"
#include "font_material_symbols.h"

static constexpr ImVec4 kDescriptionColor = ImVec4(0.63f, 0.71f, 0.78f, 1.0f);
static constexpr ImVec4 kSectionCaptionColor = ImVec4(0.40f, 0.66f, 1.0f, 1.0f);
static constexpr ImVec4 kHighTierSupporterColor = ImVec4(0.78f, 0.53f, 1.0f, 1.0f);
static constexpr ImVec4 kSupporterColor = ImVec4(0.29f, 0.80f, 0.90f, 1.0f);
static constexpr ImVec4 kCreditNameColor = ImVec4(1.0f, 0.80f, 0.42f, 1.0f);
static constexpr ImVec4 kLinkColor = ImVec4(0.26f, 0.59f, 0.98f, 1.0f);
static constexpr ImVec4 kLinkHoveredColor = ImVec4(0.45f, 0.72f, 1.0f, 1.0f);

static constexpr ImVec4 kCardColor = ImVec4(0.19f, 0.23f, 0.28f, 1.0f);
static constexpr ImVec4 kAccentColor = ImVec4(0.28f, 0.56f, 1.0f, 1.0f);
static constexpr ImVec4 kAccentHoveredColor = ImVec4(0.38f, 0.64f, 1.0f, 1.0f);
static constexpr ImVec4 kRowIconColor = ImVec4(0.56f, 0.73f, 0.93f, 1.0f);
static constexpr ImVec4 kControlColor = ImVec4(0.27f, 0.33f, 0.39f, 1.0f);
static constexpr ImVec4 kControlHoveredColor = ImVec4(0.34f, 0.41f, 0.48f, 1.0f);
static constexpr float kCardRounding = 8.0f;
static constexpr float kRowIconWidth = 24.0f;

// text and tables can't be focused, so pages that end with them need an invisible focus stop for the gamepad to scroll down
static void DrawNavScrollStop(const char* id) {
    ImGui::PushStyleColor(ImGuiCol_NavHighlight, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
    ImGui::PushItemFlag(ImGuiItemFlags_NoNavDefaultFocus, true);
    ImGui::InvisibleButton(id, ImVec2(std::max(1.0f, ImGui::GetContentRegionAvail().x), 1.0f));
    ImGui::PopItemFlag();
    ImGui::PopStyleColor();
}

static void DrawPageHeader(const char* icon, const char* title) {
    DrawNavScrollStop("##PageTop");
    ImGui::Spacing();
    if (ImGuiMenus::g_titleFont != nullptr) {
        ImGui::PushFont(ImGuiMenus::g_titleFont);
        ImGui::Text("%s  %s", icon, title);
        ImGui::PopFont();
    }
    else {
        ImGui::Text("%s  %s", icon, title);
    }
    ImGui::Dummy(ImVec2(0.0f, 2.0f));
}

static void DrawSectionCaption(const char* label) {
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, kSectionCaptionColor);
    ImGui::TextUnformatted(label);
    ImGui::PopStyleColor();
}

static void DrawPageNote(const char* text) {
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}

struct CardLayout {
    ImGuiID id = 0;
    ImVec2 min = {};
    bool icons = true;
};
static CardLayout s_card;

// the rows aren't laid out yet when the background has to go down, so it uses the height the card had last frame
static bool BeginCard(const char* id, const char* caption = nullptr, bool icons = true) {
    if (caption != nullptr) {
        DrawSectionCaption(caption);
    }
    else {
        ImGui::Dummy(ImVec2(0.0f, 4.0f));
    }
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    s_card.id = window->GetID(id);
    s_card.min = ImGui::GetCursorScreenPos();
    s_card.icons = icons;
    const float width = ImGui::GetContentRegionAvail().x;
    const float height = window->StateStorage.GetFloat(s_card.id, 0.0f);
    if (height > 0.0f) {
        window->DrawList->AddRectFilled(s_card.min, ImVec2(s_card.min.x + width, s_card.min.y + height), ImGui::GetColorU32(kCardColor), kCardRounding);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(8.0f, 9.0f));
    if (!ImGui::BeginTable(id, icons ? 3 : 2, ImGuiTableFlags_NoSavedSettings | ImGuiTableFlags_PadOuterX)) {
        ImGui::PopStyleVar();
        return false;
    }
    if (icons) {
        ImGui::TableSetupColumn("##icon", ImGuiTableColumnFlags_WidthFixed, kRowIconWidth);
    }
    ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("##widget", ImGuiTableColumnFlags_WidthFixed, std::clamp(width * 0.42f, 210.0f, 360.0f));
    return true;
}

static void EndCard() {
    ImGui::EndTable();
    ImGui::PopStyleVar();
    const float bottom = ImGui::GetCursorScreenPos().y - ImGui::GetStyle().ItemSpacing.y;
    ImGui::GetCurrentWindow()->StateStorage.SetFloat(s_card.id, bottom - s_card.min.y);
}

static void DrawRowIcon(const char* icon) {
    ImFont* font = ImGuiMenus::g_titleFont != nullptr ? ImGuiMenus::g_titleFont : ImGui::GetFont();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImVec2 size(kRowIconWidth, ImGui::GetFrameHeight());
    ImGui::Dummy(size);
    const float fontSize = font->FontSize * ImGui::GetIO().FontGlobalScale;
    const ImVec2 glyphSize = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, icon);
    ImGui::GetWindowDrawList()->AddText(font, fontSize, ImVec2(std::floor(pos.x + (size.x - glyphSize.x) * 0.5f), std::floor(pos.y + (size.y - glyphSize.y) * 0.5f)), ImGui::GetColorU32(kRowIconColor), icon);
}

template <typename DrawWidget>
static void DrawSetting(const char* icon, const char* label, const char* description, DrawWidget&& drawWidget) {
    ImGui::TableNextRow();
    int column = 0;
    if (s_card.icons) {
        ImGui::TableSetColumnIndex(column++);
        DrawRowIcon(icon);
    }
    ImGui::TableSetColumnIndex(column++);
    if (description != nullptr && description[0] != '\0') {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 4.0f);
    }
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    if (description != nullptr && description[0] != '\0') {
        ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 5.0f);
        ImGui::TextWrapped("%s", description);
        ImGui::PopStyleColor();
    }

    ImGui::TableSetColumnIndex(column);
    ImGui::PushID(label);
    ImGui::PushItemWidth(-FLT_MIN);
    drawWidget();
    ImGui::PopItemWidth();
    ImGui::PopID();
}

template <typename DrawWidget>
static void DrawSetting(const char* icon, const char* label, DrawWidget&& drawWidget) {
    DrawSetting(icon, label, nullptr, std::forward<DrawWidget>(drawWidget));
}

static void DrawSummaryRow(const char* icon, const char* text) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    DrawRowIcon(icon);
    ImGui::TableSetColumnIndex(1);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(text);
}

static bool DrawAdvancedHeader(const char* id) {
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::PushStyleColor(ImGuiCol_Text, kSectionCaptionColor);
    const bool open = ImGui::TreeNodeEx(id, ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen, "Advanced");
    ImGui::PopStyleColor();
    return open;
}

static ImVec4 Mix(const ImVec4& from, const ImVec4& to, float t) {
    return ImVec4(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t, from.w + (to.w - from.w) * t);
}

static bool ToggleSwitch(const char* id, bool* value) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) {
        return false;
    }

    const float height = ImGui::GetFrameHeight();
    const float width = height * 1.9f;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, ImGui::GetContentRegionAvail().x - width));
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImRect bb(pos, ImVec2(pos.x + width, pos.y + height));
    const ImGuiID itemId = window->GetID(id);
    ImGui::ItemSize(bb, ImGui::GetStyle().FramePadding.y);
    if (!ImGui::ItemAdd(bb, itemId)) {
        return false;
    }

    bool hovered, held;
    const bool pressed = ImGui::ButtonBehavior(bb, itemId, &hovered, &held);
    if (pressed) {
        *value = !*value;
        ImGui::MarkItemEdited(itemId);
    }

    float& knob = *window->StateStorage.GetFloatRef(itemId, *value ? 1.0f : 0.0f);
    const float target = *value ? 1.0f : 0.0f;
    knob += (target - knob) * std::min(1.0f, ImGui::GetIO().DeltaTime * 14.0f);
    if (std::fabs(target - knob) < 0.01f) {
        knob = target;
    }

    ImGui::RenderNavHighlight(bb, itemId);
    const ImVec4 offColor = hovered ? kControlHoveredColor : kControlColor;
    const ImVec4 onColor = hovered ? kAccentHoveredColor : kAccentColor;
    window->DrawList->AddRectFilled(bb.Min, bb.Max, ImGui::GetColorU32(Mix(offColor, onColor, knob)), height * 0.5f);
    const float radius = height * 0.5f - 3.0f;
    const float knobX = bb.Min.x + radius + 3.0f + (width - (radius + 3.0f) * 2.0f) * knob;
    window->DrawList->AddCircleFilled(ImVec2(knobX, bb.Min.y + height * 0.5f), radius, IM_COL32(245, 248, 252, 255), 24);
    return pressed;
}

static void DrawToggle(BoolSetting& setting, bool* changed) {
    bool value = setting;
    if (ToggleSwitch(setting.name, &value)) {
        setting = value;
        *changed = true;
    }
}

// a row of buttons that fills the column, the chosen one filled in
static bool Segmented(const char* id, std::span<const char* const> labels, int* selected) {
    ImGui::PushID(id);
    const float spacing = 3.0f;
    const float width = (ImGui::GetContentRegionAvail().x - spacing * (float)(labels.size() - 1)) / (float)labels.size();
    bool edited = false;
    for (int i = 0; i < (int)labels.size(); i++) {
        if (i > 0) {
            ImGui::SameLine(0.0f, spacing);
        }
        const bool isSelected = i == *selected;
        ImGui::PushStyleColor(ImGuiCol_Button, isSelected ? kAccentColor : kControlColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, isSelected ? kAccentHoveredColor : kControlHoveredColor);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, kAccentHoveredColor);
        ImGui::PushStyleColor(ImGuiCol_NavHighlight, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
        if (ImGui::Button(labels[i], ImVec2(width, 0.0f)) && !isSelected) {
            *selected = i;
            edited = true;
        }
        ImGui::PopStyleColor(4);
        if (ImGui::IsItemFocused() && !ImGui::GetCurrentContext()->NavDisableHighlight) {
            const ImVec2 min = ImGui::GetItemRectMin();
            const ImVec2 max = ImGui::GetItemRectMax();
            const ImU32 outline = isSelected ? ImGui::GetColorU32(ImVec4(0.86f, 0.92f, 1.0f, 0.85f)) : ImGui::GetColorU32(ImGuiCol_NavHighlight);
            ImGui::GetWindowDrawList()->AddRect(ImVec2(min.x + 1.0f, min.y + 1.0f), ImVec2(max.x - 1.0f, max.y - 1.0f), outline, ImGui::GetStyle().FrameRounding, 0, 2.0f);
        }
    }
    ImGui::PopID();
    return edited;
}

template <typename T>
struct EnumChoice {
    T value;
    const char* label;
};

template <typename T, size_t N>
static void DrawEnumSegments(EnumSetting<T>& setting, const EnumChoice<T> (&choices)[N], bool* changed) {
    std::array<const char*, N> labels;
    int selected = -1;
    for (size_t i = 0; i < N; i++) {
        labels[i] = choices[i].label;
        if (choices[i].value == setting.load()) {
            selected = (int)i;
        }
    }
    if (Segmented(setting.name, labels, &selected)) {
        setting = choices[selected].value;
        *changed = true;
    }
}

template <typename T, size_t N>
static void DrawEnumCombo(EnumSetting<T>& setting, const EnumChoice<T> (&choices)[N], bool* changed) {
    const char* preview = "";
    for (const EnumChoice<T>& choice : choices) {
        if (choice.value == setting.load()) {
            preview = choice.label;
        }
    }
    ImGui::PushID(setting.name);
    if (ImGui::BeginCombo("##value", preview)) {
        for (const EnumChoice<T>& choice : choices) {
            const bool isSelected = choice.value == setting.load();
            if (ImGui::Selectable(choice.label, isSelected)) {
                setting = choice.value;
                *changed = true;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::PopID();
}

// the reset button only shows once the value moved away from its default, but its space is always kept so the slider doesn't jump
static bool DrawResetButton(bool show) {
    ImGui::SameLine(0.0f, 6.0f);
    const ImVec2 size(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
    if (!show) {
        ImGui::Dummy(size);
        return false;
    }
    ImGui::PushStyleColor(ImGuiCol_Button, kControlColor);
    const bool pressed = ImGui::Button(ICON_MS_UNDO, size);
    ImGui::PopStyleColor();
    return pressed;
}

static bool WideButton(const char* label) {
    ImGui::PushStyleColor(ImGuiCol_Button, kControlColor);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kControlHoveredColor);
    const bool pressed = ImGui::Button(label, ImVec2(-FLT_MIN, 0.0f));
    ImGui::PopStyleColor(2);
    return pressed;
}

static float ResetButtonReserve() {
    return ImGui::GetFrameHeight() + 6.0f;
}

static void DrawPercentSlider(FloatSetting& setting, float minPercent, float maxPercent, bool* changed) {
    ImGui::PushID(setting.name);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ResetButtonReserve());
    float percent = setting * 100.0f;
    if (ImGui::SliderFloat("##value", &percent, minPercent, maxPercent, "%.0f%%")) {
        setting = percent / 100.0f;
        *changed = true;
    }
    if (DrawResetButton(std::fabs(setting - setting.defaultValue) > 0.001f)) {
        setting.Reset();
        *changed = true;
    }
    ImGui::PopID();
}

static void DrawBowArcSlider(FloatSetting& setting, bool* changed) {
    static constexpr const char* kStepLabels[] = { "Off", "Dot Only", "10%%", "20%%", "30%%", "40%%", "50%%", "60%%", "70%%", "80%%", "90%%", "100%%" };
    ImGui::PushID(setting.name);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ResetButtonReserve());
    const float value = setting;
    int step = value <= 0.0f ? 0 : value < 0.05f ? 1 : std::clamp((int)std::lround(value * 10.0f) + 1, 2, 11);
    if (ImGui::SliderInt("##value", &step, 0, 11, kStepLabels[step], ImGuiSliderFlags_NoInput)) {
        setting = step == 0 ? 0.0f : step == 1 ? ModSettings::BOW_ARC_DOT_ONLY : (float)(step - 1) / 10.0f;
        *changed = true;
    }
    if (DrawResetButton(std::fabs(setting - setting.defaultValue) > 0.001f)) {
        setting.Reset();
        *changed = true;
    }
    ImGui::PopID();
}

template <typename Format>
static void DrawFloatSlider(FloatSetting& setting, float minValue, float maxValue, Format&& format, bool* changed) {
    ImGui::PushID(setting.name);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - ResetButtonReserve());
    float value = setting;
    if (ImGui::SliderFloat("##value", &value, minValue, maxValue, format(value).c_str())) {
        setting = std::clamp(value, minValue, maxValue);
        *changed = true;
    }
    if (DrawResetButton(std::fabs(setting - setting.defaultValue) > 0.001f)) {
        setting.Reset();
        *changed = true;
    }
    ImGui::PopID();
}

static const char* s_restartNotice = nullptr;
static const char* s_restartNoticeBody = nullptr;

static void RequestRestartNotice(const char* title, const char* body) {
    s_restartNotice = title;
    s_restartNoticeBody = body;
}

static void DrawRestartNotice() {
    constexpr const char* PopupId = "##RestartNotice";
    if (s_restartNotice != nullptr && !ImGui::IsPopupOpen(PopupId)) {
        ImGui::OpenPopup(PopupId);
    }
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22.0f, 18.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_PopupRounding, 10.0f);
    ImGui::PushStyleColor(ImGuiCol_PopupBg, kCardColor);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSizeConstraints(ImVec2(420.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
    if (ImGui::BeginPopupModal(PopupId, nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings)) {
        if (ImGuiMenus::g_titleFont != nullptr) {
            ImGui::PushFont(ImGuiMenus::g_titleFont);
        }
        ImGui::TextColored(kAccentColor, ICON_MS_REFRESH);
        ImGui::SameLine();
        ImGui::TextUnformatted(s_restartNotice != nullptr ? s_restartNotice : "");
        if (ImGuiMenus::g_titleFont != nullptr) {
            ImGui::PopFont();
        }
        ImGui::Dummy(ImVec2(0.0f, 2.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
        ImGui::TextUnformatted(s_restartNoticeBody != nullptr ? s_restartNoticeBody : "");
        ImGui::PopStyleColor();
        ImGui::Dummy(ImVec2(0.0f, 12.0f));
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - 120.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, kAccentColor);
        if (ImGui::IsWindowAppearing()) {
            ImGui::SetKeyboardFocusHere();
        }
        if (ImGui::Button("Got It", ImVec2(120.0f, 0.0f))) {
            s_restartNotice = nullptr;
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

struct MenuPageEntry {
    uint8_t page;
    const char* label;
};

static constexpr MenuPageEntry kSettingsPages[] = {
    { ImGuiMenus::PLAYSTYLE_PAGE, ICON_MS_ACCESSIBILITY_NEW "  Play Style" },
    { ImGuiMenus::COMFORT_PAGE, ICON_MS_SPA "  Comfort" },
    { ImGuiMenus::CONTROLS_PAGE, ICON_MS_STADIA_CONTROLLER "  Controls" },
    { ImGuiMenus::DISPLAY_PAGE, ICON_MS_DESKTOP_WINDOWS "  Display" },
    { ImGuiMenus::SYSTEM_PAGE, ICON_MS_SETTINGS "  System" },
};

static constexpr MenuPageEntry kHowToPlayPages[] = {
    { ImGuiMenus::GUIDE_CONTROLS_PAGE, ICON_MS_MENU_BOOK "  The Controls" },
    { ImGuiMenus::GUIDE_EQUIPMENT_PAGE, ICON_MS_MENU_BOOK "  Weapons & Runes" },
    { ImGuiMenus::GUIDE_SWING_PAGE, ICON_MS_MENU_BOOK "  Swinging & Blocking" },
    { ImGuiMenus::GUIDE_WHISTLE_PAGE, ICON_MS_MENU_BOOK "  Whistle & Magnesis" },
};

static constexpr MenuPageEntry kMorePages[] = {
    { ImGuiMenus::CREDITS_PAGE, ICON_MS_FAVORITE "  Credits" },
    { ImGuiMenus::DEBUG_PAGE, ICON_MS_BUG_REPORT "  Debug" },
};

struct SidebarNavTarget {
    ImGuiID itemId = 0;
    ImGuiID focusScopeId = 0;
    ImRect rectRel = {};
    bool enterContentRequested = false;
};

static void DrawSidebarGroup(const char* caption, std::span<const MenuPageEntry> entries, std::atomic_uint8_t& selectedPage, SidebarNavTarget& navTarget) {
    ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
    ImGui::TextUnformatted(caption);
    ImGui::PopStyleColor();
    for (const MenuPageEntry& entry : entries) {
        if (!ImGuiMenus::IsPageAvailable(entry.page)) {
            continue;
        }
        const bool isSelectedPage = selectedPage == entry.page;
        if (ImGui::Selectable(entry.label, isSelectedPage, ImGuiSelectableFlags_None, ImVec2(0.0f, 30.0f))) {
            // confirming the page you are already on is the only way it can mean "take me into this page"
            navTarget.enterContentRequested = isSelectedPage;
            selectedPage = entry.page;
        }
        if (selectedPage == entry.page) {
            const ImGuiContext& g = *ImGui::GetCurrentContext();
            navTarget.itemId = g.LastItemData.ID;
            navTarget.focusScopeId = g.CurrentFocusScopeId;
            navTarget.rectRel = ImGui::WindowRectAbsToRel(ImGui::GetCurrentWindow(), g.LastItemData.NavRect);
        }
    }
}

static SidebarNavTarget DrawSidebar(std::atomic_uint8_t& selectedPage) {
    SidebarNavTarget navTarget;
    ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
    DrawSidebarGroup("SETTINGS", kSettingsPages, selectedPage, navTarget);
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    DrawSidebarGroup("HOW TO PLAY", kHowToPlayPages, selectedPage, navTarget);
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    DrawSidebarGroup("MORE", kMorePages, selectedPage, navTarget);
    ImGui::PopStyleVar();
    return navTarget;
}

// imgui only links items sideways when they overlap vertically, which the sidebar and settings rarely do
static bool WantsToEnterContentPane(ImGuiWindow* sidebarWindow, const SidebarNavTarget& sidebarNavTarget) {
    if (sidebarWindow == nullptr) {
        return false;
    }
    const ImGuiContext& g = *ImGui::GetCurrentContext();
    if (g.NavWindow != sidebarWindow) {
        return false;
    }
    return sidebarNavTarget.enterContentRequested || (g.NavMoveSubmitted && g.NavMoveDir == ImGuiDir_Right);
}

// imgui only picks up a nav init request from items after it, so this has to run in the content window before its widgets
static void EnterContentPane(ImGuiWindow* contentWindow, bool startAtFirstItem) {
    ImGui::SetNavWindow(contentWindow); // also cancels the pending move request, so nothing else can claim it
    ImGui::NavInitWindow(contentWindow, startAtFirstItem);
    ImGui::GetCurrentContext()->NavDisableHighlight = false;
}

// every page is its own content window, so a trigger page switch leaves focus on a window that's gone or a stale sidebar entry
static void FollowPageSwitch(ImGuiWindow* sidebarWindow, const SidebarNavTarget& sidebarNavTarget, uint8_t page) {
    static uint8_t s_lastPage = 0xFF;
    const bool switched = s_lastPage != page;
    s_lastPage = page;
    ImGuiContext& g = *ImGui::GetCurrentContext();
    if (!switched || sidebarWindow == nullptr || sidebarNavTarget.itemId == 0 || g.NavWindow == nullptr) {
        return;
    }
    ImGui::SetNavWindow(sidebarWindow);
    ImGui::SetNavID(sidebarNavTarget.itemId, ImGuiNavLayer_Main, sidebarNavTarget.focusScopeId, sidebarNavTarget.rectRel);
}

// imgui finds no target for a left move out of the content pane, so the sidebar has to take it
static void ApplyPaneNavFallback(ImGuiWindow* sidebarWindow, ImGuiWindow* contentWindow, const SidebarNavTarget& sidebarNavTarget) {
    ImGuiContext& g = *ImGui::GetCurrentContext();
    if (!g.NavMoveSubmitted || g.NavMoveDir != ImGuiDir_Left || contentWindow == nullptr || g.NavWindow != contentWindow) {
        return;
    }
    if (g.NavMoveResultLocal.ID != 0 || g.NavMoveResultOther.ID != 0) {
        return;
    }

    if (sidebarWindow != nullptr && sidebarNavTarget.itemId != 0) {
        ImGui::SetNavWindow(sidebarWindow);
        ImGui::SetNavID(sidebarNavTarget.itemId, ImGuiNavLayer_Main, sidebarNavTarget.focusScopeId, sidebarNavTarget.rectRel);
        g.NavDisableHighlight = false;
    }
}

void ImGuiMenus::OpenLinkInBrowser(const char* url) {
    const std::string_view link = url != nullptr ? url : "";
    if (!link.starts_with("https://") && !link.starts_with("http://")) {
        Log::print<WARNING>("Refusing to open link that isn't a http(s) URL: {}", url != nullptr ? url : "(null)");
        return;
    }

    // opening the browser can block for a while, so keep it off the render thread
    std::thread([link = std::string(link)]() {
        if (Platform::OpenUrl(link.c_str())) {
            Log::print<INFO>("Opened {} in the default browser, it's behind the game window", link);
        }
        else {
            Log::print<WARNING>("Failed to open {} in the default browser", link);
        }
    }).detach();
}

// underlined text that the gamepad cursor can land on
static bool LinkButton(const char* label) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) {
        return false;
    }

    const ImGuiID id = window->GetID(label);
    const ImVec2 pos(window->DC.CursorPos.x, window->DC.CursorPos.y + window->DC.CurrLineTextBaseOffset);
    const ImRect bb(pos, pos + ImGui::CalcTextSize(label, nullptr, true));
    ImGui::ItemSize(bb.GetSize(), 0.0f);
    if (!ImGui::ItemAdd(bb, id)) {
        return false;
    }

    bool hovered, held;
    const bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held);
    ImGui::RenderNavHighlight(bb, id);
    if (hovered) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

    const ImU32 color = ImGui::GetColorU32(hovered || held ? kLinkHoveredColor : kLinkColor);
    window->DrawList->AddLine(ImVec2(bb.Min.x, bb.Max.y), ImVec2(bb.Max.x, bb.Max.y), color);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::RenderText(bb.Min, label);
    ImGui::PopStyleColor();
    return pressed;
}

static void DrawLink(const char* label, const char* url) {
    if (LinkButton(label)) {
        Log::print<INFO>("Mod menu link pressed: {}", url);
        ImGuiMenus::OpenLinkInBrowser(url);
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
        ImGui::SetTooltip("Open %s", url);
    }
}

struct CreditEntry {
    const char* icon;
    const char* names;
    const char* role;
};

static constexpr CreditEntry kCredits[] = {
    { ICON_KI_WRENCH, "Crementif", "Main developer" },
    { ICON_KI_GAMEPAD, "Holydh", "Inputs and gestures" },
    { ICON_MS_SWORDS, "Acudofy", "Sword and stab analysis system" },
    { ICON_KI_SEARCH, "leoetlino", "Made the BotW Decomp project, which was useful while reverse engineering" },
    { ICON_MS_SPEED, "Exzap", "Technical support and optimization help" },
    { ICON_KI_PAINT_BRUSH, "Mako Marci", "Made the logo, controller guide and trailer" },
    { ICON_KI_USERS, "Tim, Mako Marci, Solarwolf07, Elliott Tate and Derra", "Helped with QA testing, recording, feedback and support" },
    { ICON_KI_MOVIE, "WunderbarVR", "Made cool trailers" },
};

static void DrawCreditsList() {
    int index = 0;
    for (const auto& credit : kCredits) {
        ImGui::PushStyleColor(ImGuiCol_Text, kCreditNameColor);
        ImGui::Text("%s %s:", credit.icon, credit.names);
        ImGui::PopStyleColor();

        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
        ImGui::TextWrapped("%s", credit.role);
        ImGui::PopStyleColor();

        index++;
        if (index % 2 == 0 && index < (int)std::size(kCredits)) {
            ImGui::PushID(index);
            DrawNavScrollStop("##CreditStop");
            ImGui::PopID();
        }
    }
}

static void DrawSupporterWall() {
    if (Supporters::List.empty()) {
        return;
    }

    ImGui::SeparatorText("Supporters");
    ImGui::TextWrapped("These people are funding the development of the mod. Thank you!");
    DrawNavScrollStop("##SupportersTop");
    ImGui::Dummy(ImVec2(0.0f, 6.0f));

    int columns = std::clamp((int)(ImGui::GetContentRegionAvail().x / 260.0f), 1, 4);
    if (ImGui::BeginTable("##Supporters", columns, ImGuiTableFlags_NoSavedSettings)) {
        for (const auto& supporter : Supporters::List) {
            const char* icon = ICON_KI_STAR;
            if (supporter.platform == Supporters::Platform::GITHUB) {
                icon = ICON_KI_GITHUB;
            }
            else if (supporter.platform == Supporters::Platform::PATREON) {
                icon = ICON_KI_PATREON;
            }

            ImGui::TableNextColumn();
            ImGui::TextColored(supporter.tier == Supporters::Tier::HIGH ? kHighTierSupporterColor : kSupporterColor, "%s %s", icon, supporter.name);
        }
        ImGui::EndTable();
    }
}

static void DrawCompactProfilerSummary() {
    auto snapshots = BetterVRProfiler::GetSnapshots();
    std::sort(snapshots.begin(), snapshots.end(), [](const BetterVRProfiler::Snapshot& lhs, const BetterVRProfiler::Snapshot& rhs) {
        return lhs.averageFrameMs > rhs.averageFrameMs;
    });

    ImGui::Separator();
    int rowsDrawn = 0;
    for (const auto& snapshot : snapshots) {
        if (std::max(snapshot.lastFrameMs, snapshot.averageFrameMs) < 0.01 && snapshot.maxFrameMs < 0.01) {
            continue;
        }

        ImGui::Text("%s %.2f/%.2f ms x%u", snapshot.name, snapshot.lastFrameMs, snapshot.averageFrameMs, snapshot.lastFrameCalls);
        rowsDrawn++;
        if (rowsDrawn >= 3) {
            break;
        }
    }

    if (rowsDrawn == 0) {
        ImGui::TextDisabled("Profiler warming up...");
    }
}

static void DrawDetailedProfilerSummary() {
    auto snapshots = BetterVRProfiler::GetSnapshots();
    std::sort(snapshots.begin(), snapshots.end(), [](const BetterVRProfiler::Snapshot& lhs, const BetterVRProfiler::Snapshot& rhs) {
        return std::max(lhs.lastFrameMs, lhs.averageFrameMs) > std::max(rhs.lastFrameMs, rhs.averageFrameMs);
    });

    DrawSectionCaption("Developer CPU Profiling");
    if (ImGui::Button("Reset Profiling Times")) {
        BetterVRProfiler::Reset();
    }

    constexpr ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
    if (!ImGui::BeginTable("##BetterVRProfiling", 7, tableFlags)) {
        return;
    }

    ImGui::TableSetupColumn("Section");
    ImGui::TableSetupColumn("Last Frame");
    ImGui::TableSetupColumn("Avg Frame");
    ImGui::TableSetupColumn("Max Frame");
    ImGui::TableSetupColumn("Calls");
    ImGui::TableSetupColumn("Last Call");
    ImGui::TableSetupColumn("Max Call");
    ImGui::TableHeadersRow();

    const bool showPpcSections = BetterVRProfiler::HasPpcSamples();
    for (const auto& snapshot : snapshots) {
        if (snapshot.isPpcSection && !showPpcSections) {
            continue;
        }

        ImGui::TableNextRow();

        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted(snapshot.name);

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.3f ms", snapshot.lastFrameMs);

        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f ms", snapshot.averageFrameMs);

        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f ms", snapshot.maxFrameMs);

        ImGui::TableSetColumnIndex(4);
        ImGui::Text("%u", snapshot.lastFrameCalls);

        ImGui::TableSetColumnIndex(5);
        ImGui::Text("%.3f ms", snapshot.lastCallMs);

        ImGui::TableSetColumnIndex(6);
        ImGui::Text("%.3f ms", snapshot.maxCallMs);
    }

    ImGui::EndTable();
}

static void DrawBetterVRProfiler(bool detailed) {
    if (detailed) {
        DrawDetailedProfilerSummary();
    }
    else {
        DrawCompactProfilerSummary();
    }
}

enum class OverlayProfilerMode : uint8_t {
    NONE = 0,
    COMPACT = 1,
    DETAILED = 2,
};

static void DrawFPSOverlayContent(RND_Renderer* renderer, bool renderText, OverlayProfilerMode profilerMode) {
    const float predictedHz = renderer->GetDisplayRefreshRate();

    // the interval between the predicted display times of the frames that were submitted, averaged over half a second
    const float appMs = (float)renderer->GetAverageFrameTimeMs();
    const float appFps = appMs > 0.0000001f ? (1000.0f / appMs) : 0.0f;

    if (renderText) {
        ImGui::Text("Your headset runs at %.0f Hz", predictedHz);
        ImGui::Text("Currently Running At %.1f FPS", appFps);
        ImGui::Text("");
    }

    if (predictedHz > 0.0f) {
        auto rateForDivisor = [predictedHz](int divisor) -> double {
            return divisor > 0 ? (predictedHz / (double)divisor) : 0.0;
        };

        // the refresh divisor (1x, 1/2x, 1/3x, 1/4x) the current frame rate holds
        auto chooseBestDivisor = [&](double fps) -> int {
            for (int div = 1; div <= 4; ++div) {
                if (fps >= rateForDivisor(div) * 0.98) {
                    return div;
                }
            }
            return 4;
        };

        const int currentDiv = chooseBestDivisor(appFps);
        const double currentTarget = rateForDivisor(currentDiv);
        const int nextDiv = std::max(1, currentDiv - 1);
        const double nextTarget = rateForDivisor(nextDiv);

        if (renderText) {
            if (currentDiv == 1) {
                ImGui::Text("Its reaching the full refresh rate you've set (%.0f hz)", currentTarget);
            }
            else {
                ImGui::Text("It is reaching %.0f FPS (%.0f Hz / %d)", currentTarget, predictedHz, currentDiv);
                ImGui::Text("The next step up is %.0f FPS", nextTarget);
            }
        }
    }

    static float history_app_fps[60] = {};
    static int offset = 0;

    history_app_fps[offset] = appFps;
    offset = (offset + 1) % 60;

    const double targetFps = predictedHz;
    const double halfFps = predictedHz / 2.0;
    const double thirdFps = predictedHz / 3.0;

    if (ImPlot::BeginPlot("##Frametime", ImVec2(300, 150), ImPlotFlags_NoFrame | ImPlotFlags_NoTitle | ImPlotFlags_NoMouseText | ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect | ImPlotFlags_NoInputs)) {
        ImPlot::SetupAxes(nullptr, "##FPS", ImPlotAxisFlags_NoDecorations, ImPlotAxisFlags_NoInitialFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, 60, ImPlotCond_Always);

        if (targetFps >= 0.000000001f) {
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, predictedHz * 1.5f, ImPlotCond_Always);

            ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 0.5f));
            ImPlot::PlotInfLines("##Target", &targetFps, 1, ImPlotInfLinesFlags_Horizontal);
            ImPlot::TagY(targetFps, ImVec4(0, 1, 0, 0.5f), "%.0f Hz", targetFps);

            ImPlot::SetNextLineStyle(ImVec4(1, 1, 0, 0.5f));
            ImPlot::PlotInfLines("##1/2 Rate", &halfFps, 1, ImPlotInfLinesFlags_Horizontal);
            ImPlot::TagY(halfFps, ImVec4(1, 1, 0, 0.5f), "%.0f Hz", halfFps);

            ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 0.5f));
            ImPlot::PlotInfLines("##1/3 Rate", &thirdFps, 1, ImPlotInfLinesFlags_Horizontal);
        }
        else {
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, 144.0, ImPlotCond_Always);
        }

        ImPlot::SetNextFillStyle(ImVec4(0.4f, 0.4f, 1.0f, 0.50f));
        ImPlot::SetNextLineStyle(ImVec4(0.4f, 0.4f, 1.0f, 1.0f));
        ImPlot::PlotShaded("Actual", history_app_fps, 60, 0.0, 1.0, 0.0, 0, offset);
        ImPlot::PlotLine("##TotalLine", history_app_fps, 60, 1.0, 0.0, 0, offset);

        if (appFps > 0.0f) {
            const double currentFps = appFps;
            ImPlot::SetNextLineStyle(ImVec4(1, 1, 1, 0.65f));
            ImPlot::PlotInfLines("##Current", &currentFps, 1, ImPlotInfLinesFlags_Horizontal);
            ImPlot::TagY(currentFps, ImVec4(1, 1, 1, 0.8f), "%.1f FPS", currentFps);
        }

        ImPlot::EndPlot();
    }

    if (profilerMode != OverlayProfilerMode::NONE) {
        DrawBetterVRProfiler(profilerMode == OverlayProfilerMode::DETAILED);
    }
}

namespace ImGuiMenus {
    ImFont* g_titleFont = nullptr;

    bool IsPageAvailable(uint8_t page) {
        if (page == DEBUG_PAGE) {
            return GetSettings().IsDebuggingToolsEnabled();
        }
        return page < PAGE_COUNT;
    }

    void DrawFPSOverlay(RND_Renderer* renderer) {
        ImGui::SetNextWindowBgAlpha(0.4f);

        ImVec2 windowSize = ImGui::GetIO().DisplaySize;

        const ImVec2 pad(10.0f, 10.0f);
        ImGui::SetNextWindowPos(ImVec2(windowSize.x - pad.x, pad.y), ImGuiCond_Always, ImVec2(1.0f, 0.0f));

        OverlayProfilerMode profilerMode = OverlayProfilerMode::NONE;
        if (GetSettings().performanceOverlay == PerformanceOverlayMode::WINDOW_AND_VR_WITH_PROFILER) {
            profilerMode = OverlayProfilerMode::COMPACT;
        }

        if (ImGui::Begin("AppMS Overlay", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoMove)) {
            DrawFPSOverlayContent(renderer, false, profilerMode);
        }
        ImGui::End();
    }
}

static std::string FormatHeightOffset(float height) {
    if (std::fabs(height) <= 0.01f) {
        return "0.00 m";
    }
    const float inches = std::fabs(height) * 39.3700787f;
    const int feet = (int)std::floor(inches / 12.0f);
    return fmt::format("{}{:.2f} m / {}'{:.1f}\"", height < 0.0f ? "-" : "+", std::fabs(height), feet, inches - (float)feet * 12.0f);
}

void RND_Renderer::ImGuiOverlay::DrawPlaystylePage(bool* changed) {
    auto& settings = GetSettings();
    const bool firstPerson = settings.cameraMode == CameraMode::FIRST_PERSON;

    DrawPageHeader(ICON_MS_ACCESSIBILITY_NEW, "Play Style");

    if (BeginCard("##Basics")) {
        static constexpr EnumChoice<CameraMode> cameraChoices[] = { { CameraMode::FIRST_PERSON, "First Person" }, { CameraMode::THIRD_PERSON, "Third Person" } };
        DrawSetting(firstPerson ? ICON_MS_EYE_TRACKING : ICON_MS_FRAME_PERSON, "Camera", [&]() {
            DrawEnumSegments(settings.cameraMode, cameraChoices, changed);
        });
        EndCard();
    }

    if (!firstPerson) {
        if (BeginCard("##ThirdPerson", "Third-Person Camera")) {
            DrawSetting(ICON_MS_ZOOM_OUT_MAP, "Camera Distance", [&]() {
                DrawFloatSlider(settings.thirdPlayerDistance, 0.5f, 0.65f, [](float value) { return fmt::format("{:.2f}", value); }, changed);
            });
            EndCard();
        }
        DrawPageNote("Set up your controller in Cemu's toolbar under Options > Input Settings.");
        return;
    }

    if (BeginCard("##Body", "Your Body")) {
        const bool seated = settings.GetPlayMode() == PlayMode::SEATED;
        static constexpr EnumChoice<PlayMode> positionChoices[] = { { PlayMode::STANDING, "Standing" }, { PlayMode::SEATED, "Seated" } };
        DrawSetting(seated ? ICON_MS_AIRLINE_SEAT_RECLINE_NORMAL : ICON_MS_MAN, "Play Position", [&]() {
            DrawEnumSegments(settings.playMode, positionChoices, changed);
        });
        if (seated) {
            const std::optional<float> eyeHeight = VRManager::instance().XR->GetCalibratedSeatedEyeHeight();
            const std::string measured = eyeHeight ? fmt::format("Measured your eyes at {:.2f} m. Sit how you play, then measure again.", *eyeHeight) : std::string("Measures when the game loads.");
            DrawSetting(ICON_MS_HEIGHT, "Seated Height", measured.c_str(), [&]() {
                if (WideButton("Measure Again")) {
                    VRManager::instance().XR->RequestSeatedHeightCalibration();
                }
            });
        }
        DrawSetting(ICON_MS_EXPAND, "Height Adjustment", [&]() {
            DrawFloatSlider(settings.playerHeightOffset, -0.5f, 1.0f, FormatHeightOffset, changed);
        });
        EndCard();
    }
}

struct ComfortPreset {
    const char* icon;
    const char* name;
    const char* subtitle;
    TurnMode turnMode;
    bool preventFalling;
    EventMode cutscenes;
    bool theatre;
    bool stillCamera;
    float bowArc;
};

static constexpr ComfortPreset kComfortPresets[] = {
    { ICON_MS_ECO, "Gentle", "New to VR", TurnMode::SNAP_45, true, EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS, true, true, 0.7f },
    { ICON_MS_BALANCE, "Balanced", "Recommended", TurnMode::SMOOTH_NORMAL, true, EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS, false, false, 0.7f },
    { ICON_MS_LOCAL_FIRE_DEPARTMENT, "Immersive", "VR veterans", TurnMode::SMOOTH_NORMAL, false, EventMode::ALWAYS_FIRST_PERSON, false, false, ModSettings::BOW_ARC_DOT_ONLY },
};

static int FindComfortPreset(const ModSettings& settings) {
    for (int i = 0; i < (int)std::size(kComfortPresets); i++) {
        const ComfortPreset& preset = kComfortPresets[i];
        if (settings.turnMode == preset.turnMode && settings.preventFirstPersonRagdoll == preset.preventFalling && settings.cutsceneCameraMode == preset.cutscenes && settings.useBlackBarsForCutscenes == preset.theatre && settings.alwaysPreventFirstPersonCutsceneCameraMovement == preset.stillCamera && std::fabs(settings.bowArcOpacity - preset.bowArc) < 0.005f) {
            return i;
        }
    }
    return -1;
}

static void ApplyComfortPreset(ModSettings& settings, const ComfortPreset& preset) {
    settings.turnMode = preset.turnMode;
    settings.preventFirstPersonRagdoll = preset.preventFalling;
    settings.cutsceneCameraMode = preset.cutscenes;
    settings.useBlackBarsForCutscenes = preset.theatre;
    settings.alwaysPreventFirstPersonCutsceneCameraMovement = preset.stillCamera;
    settings.bowArcOpacity = preset.bowArc;
}

static bool DrawPresetCard(const ComfortPreset& preset, bool selected, float width) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    const float height = 96.0f;
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const ImRect bb(pos, ImVec2(pos.x + width, pos.y + height));
    const ImGuiID itemId = window->GetID(preset.name);
    ImGui::ItemSize(bb);
    if (!ImGui::ItemAdd(bb, itemId)) {
        return false;
    }
    bool hovered, held;
    const bool pressed = ImGui::ButtonBehavior(bb, itemId, &hovered, &held);
    ImGui::RenderNavHighlight(bb, itemId);

    window->DrawList->AddRectFilled(bb.Min, bb.Max, ImGui::GetColorU32(hovered ? kControlColor : kCardColor), kCardRounding);
    if (selected) {
        window->DrawList->AddRect(bb.Min, bb.Max, ImGui::GetColorU32(kAccentColor), kCardRounding, 0, 2.5f);
    }
    const ImVec2 textPos(bb.Min.x + 16.0f, bb.Min.y + 14.0f);
    if (ImGuiMenus::g_titleFont != nullptr) {
        const float titleSize = ImGuiMenus::g_titleFont->FontSize * ImGui::GetIO().FontGlobalScale;
        window->DrawList->AddText(ImGuiMenus::g_titleFont, titleSize, textPos, ImGui::GetColorU32(selected ? kAccentColor : kRowIconColor), preset.icon);
        window->DrawList->AddText(ImGuiMenus::g_titleFont, titleSize, ImVec2(textPos.x, textPos.y + 30.0f), ImGui::GetColorU32(ImGuiCol_Text), preset.name);
    }
    window->DrawList->AddText(ImVec2(textPos.x, textPos.y + 56.0f), ImGui::GetColorU32(kDescriptionColor), preset.subtitle);
    return pressed;
}

static bool IsSnapTurning(TurnMode mode) {
    return mode == TurnMode::SNAP_30 || mode == TurnMode::SNAP_45 || mode == TurnMode::SNAP_60;
}

static const char* DescribeTurnMode(TurnMode mode) {
    switch (mode) {
        case TurnMode::SMOOTH_SLOW: return "Slow smooth turning";
        case TurnMode::SMOOTH_NORMAL: return "Smooth turning";
        case TurnMode::SMOOTH_FAST: return "Fast smooth turning";
        case TurnMode::SNAP_30: return "Snap turns of 30 degrees";
        case TurnMode::SNAP_45: return "Snap turns of 45 degrees";
        case TurnMode::SNAP_60: return "Snap turns of 60 degrees";
    }
    return "";
}

static const char* DescribeCutscenes(const ModSettings& settings) {
    if (settings.useBlackBarsForCutscenes) {
        return settings.alwaysPreventFirstPersonCutsceneCameraMovement ? "Cutscenes on a still 2D screen" : "Cutscenes on a 2D screen";
    }
    switch (settings.cutsceneCameraMode.load()) {
        case EventMode::ALWAYS_FIRST_PERSON: return "Cutscenes always in first person";
        case EventMode::ALWAYS_THIRD_PERSON: return "Cutscenes always in third person";
        default: return "Cutscenes in first or third person";
    }
}

void RND_Renderer::ImGuiOverlay::DrawComfortPage(bool* changed) {
    auto& settings = GetSettings();
    const bool firstPerson = settings.cameraMode == CameraMode::FIRST_PERSON;

    DrawPageHeader(ICON_MS_SPA, "Comfort");
    ImGui::Dummy(ImVec2(0.0f, 4.0f));

    const int activePreset = FindComfortPreset(settings);
    const float spacing = 10.0f;
    const float cardWidth = (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f;
    for (int i = 0; i < (int)std::size(kComfortPresets); i++) {
        if (i > 0) {
            ImGui::SameLine(0.0f, spacing);
        }
        if (DrawPresetCard(kComfortPresets[i], i == activePreset, cardWidth)) {
            ApplyComfortPreset(settings, kComfortPresets[i]);
            *changed = true;
        }
    }

    if (BeginCard("##PresetSummary", activePreset >= 0 ? "What This Preset Does" : "Your Own Mix")) {
        DrawSummaryRow(IsSnapTurning(settings.turnMode) ? ICON_MS_ROTATE_RIGHT : ICON_MS_360, DescribeTurnMode(settings.turnMode));
        DrawSummaryRow(settings.preventFirstPersonRagdoll ? ICON_MS_SHIELD : ICON_MS_FALLING, settings.preventFirstPersonRagdoll ? "Link never falls over" : "Link can fall over");
        DrawSummaryRow(settings.useBlackBarsForCutscenes ? ICON_MS_TV : ICON_MS_MOVIE, DescribeCutscenes(settings));
        DrawSummaryRow(ICON_MS_BOW_ARC, settings.IsBowArcDotOnly() ? "Show only the aiming dot" : settings.bowArcOpacity > 0.0f ? "Show arrow arc while aiming" : "No arrow arc or aiming dot");
        EndCard();
    }

    if (DrawAdvancedHeader("##ComfortAdvanced")) {
        if (BeginCard("##ComfortDetails", nullptr, false)) {
            if (firstPerson) {
                static constexpr EnumChoice<TurnMode> turnChoices[] = {
                    { TurnMode::SMOOTH_SLOW, "Smooth, slow" },
                    { TurnMode::SMOOTH_NORMAL, "Smooth" },
                    { TurnMode::SMOOTH_FAST, "Smooth, fast" },
                    { TurnMode::SNAP_30, "Snap 30 degrees" },
                    { TurnMode::SNAP_45, "Snap 45 degrees" },
                    { TurnMode::SNAP_60, "Snap 60 degrees" },
                };
                DrawSetting(nullptr, "Turning", [&]() {
                    DrawEnumCombo(settings.turnMode, turnChoices, changed);
                });
                DrawSetting(nullptr, "Prevent Ragdolling", [&]() {
                    DrawToggle(settings.preventFirstPersonRagdoll, changed);
                });
                static constexpr EnumChoice<EventMode> cutsceneChoices[] = { { EventMode::ALWAYS_FIRST_PERSON, "First" }, { EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS, "Mixed" }, { EventMode::ALWAYS_THIRD_PERSON, "Third" } };
                DrawSetting(nullptr, "Cutscene Camera", [&]() {
                    DrawEnumSegments(settings.cutsceneCameraMode, cutsceneChoices, changed);
                });
            }
            DrawSetting(nullptr, "2D Cutscene Screen", "For third-person cutscenes.", [&]() {
                DrawToggle(settings.useBlackBarsForCutscenes, changed);
            });
            DrawSetting(nullptr, "Bow Aiming Arc", [&]() {
                DrawBowArcSlider(settings.bowArcOpacity, changed);
            });
            EndCard();
        }
    }
}

void RND_Renderer::ImGuiOverlay::DrawControlsPage(bool* changed) {
    auto& settings = GetSettings();
    const bool firstPerson = settings.cameraMode == CameraMode::FIRST_PERSON;
    const bool questLink = VRManager::instance().XR->m_capabilities.isOculusLinkRuntime;
    if (!questLink) {
        settings.buggyAngularVelocity = AngularVelocityFixerMode::AUTO;
    }

    DrawPageHeader(ICON_MS_STADIA_CONTROLLER, "Controls");

    if (firstPerson && BeginCard("##Buttons")) {
        static constexpr EnumChoice<WalkingDirection> walkChoices[] = { { WalkingDirection::CAMERA, "Head" }, { WalkingDirection::CONTROLLER, "Hand" } };
        DrawSetting(settings.walkingDirection == WalkingDirection::CONTROLLER ? ICON_MS_PAN_TOOL_ALT : ICON_MS_HEAD_MOUNTED_DEVICE, "Walk Direction", [&]() {
            DrawEnumSegments(settings.walkingDirection, walkChoices, changed);
        });
        EndCard();
    }

    if (firstPerson) {
        if (BeginCard("##Motion", "Motion Controls")) {
            static constexpr EnumChoice<SwingSensitivity> swingChoices[] = { { SwingSensitivity::SWING_NORMAL, "Normal" }, { SwingSensitivity::SWING_EASY, "Relaxed" } };
            DrawSetting(ICON_MS_SWORDS, "Sword Swings", [&]() {
                DrawEnumSegments(settings.swingSensitivity, swingChoices, changed);
            });
            EndCard();
        }
    }
    else {
        DrawPageNote("Third person is played with a regular controller, which you set up in Cemu's toolbar under Options > Input Settings.");
    }

    if (DrawAdvancedHeader("##ControlsAdvanced")) {
        if (BeginCard("##Sticks", "Thumbsticks", false)) {
            DrawSetting(nullptr, "Stick Deadzone", [&]() {
                DrawPercentSlider(settings.stickDeadzone, 0.0f, 50.0f, changed);
            });
            DrawSetting(nullptr, "Direction Threshold", [&]() {
                DrawPercentSlider(settings.axisThreshold, 10.0f, 90.0f, changed);
            });
            EndCard();
        }

        if (BeginCard("##Combat", "Combat", false)) {
            if (firstPerson) {
                DrawSetting(nullptr, "One Hit Per Swing", "Off lets a swing hit again, for less damage each time.", [&]() {
                    DrawToggle(settings.oneHitPerSwing, changed);
                });
            }
            else {
                DrawSetting(nullptr, "Camera Bow Aiming", [&]() {
                    DrawToggle(settings.thirdPersonBowCameraAim, changed);
                });
            }
            EndCard();
        }

        if (questLink) {
            if (BeginCard("##Link", "Quest Link", false)) {
                static constexpr EnumChoice<AngularVelocityFixerMode> fixChoices[] = { { AngularVelocityFixerMode::AUTO, "Auto" }, { AngularVelocityFixerMode::FORCED_ON, "On" }, { AngularVelocityFixerMode::FORCED_OFF, "Off" } };
                DrawSetting(nullptr, "Fix Controller Rotation", "Only change this if your sword swings don't come out.", [&]() {
                    DrawEnumSegments(settings.buggyAngularVelocity, fixChoices, changed);
                });
                EndCard();
            }
        }
    }
}

// the Graphics pack's Recording Mode presets, in the order of the pack's $recordingOutput values
static constexpr const char* kRecordingModeCategory = "Recording Mode";
static constexpr const char* kRecordingModeLabels[] = { "Punch In", "Wider", "No HUD" };
static constexpr const char* kRecordingModeNames[] = { "Punch In", "Wider View", "Wider View Without HUD" };
static constexpr const char* kResolutionMultiplierCategory = "VR Resolution Multiplier";

// the normal and wider presets are named after their resolutions, so match them by the " (1x, ...)" part
static void SelectRecordingMode(const std::string& preset) {
    const std::string multiplier = CemuHost::GetGraphicsPreset(kResolutionMultiplierCategory);
    CemuHost::SetGraphicsPreset(kRecordingModeCategory, preset);
    const size_t stepStart = multiplier.find(" (");
    if (stepStart == std::string::npos) {
        return;
    }
    for (const std::string& candidate : CemuHost::GetGraphicsPresets(kResolutionMultiplierCategory)) {
        if (candidate.ends_with(std::string_view(multiplier).substr(stepStart))) {
            CemuHost::SetGraphicsPreset(kResolutionMultiplierCategory, candidate);
            return;
        }
    }
}

static void DrawRecordingModeSetting() {
    const std::vector<std::string> presets = CemuHost::GetGraphicsPresets(kRecordingModeCategory);
    if (presets.size() != std::size(kRecordingModeLabels)) {
        return;
    }
    int selected = (int)(std::ranges::find(presets, CemuHost::GetGraphicsPreset(kRecordingModeCategory)) - presets.begin());
    DrawSetting(ICON_MS_SCREEN_RECORD, "Recording View", "Wider runs slower.", [&]() {
        if (Segmented("##recording", kRecordingModeLabels, &selected)) {
            SelectRecordingMode(presets[selected]);
            if (selected != (int)Game::GetPresentation().recordingOutputMode) {
                static std::string s_title;
                s_title = fmt::format("Restart Cemu to switch to {}", kRecordingModeNames[selected]);
                RequestRestartNotice(s_title.c_str(), "It's saved and applies the next time you start Cemu.");
            }
        }
    });
}

void RND_Renderer::ImGuiOverlay::DrawDisplayPage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_MS_DESKTOP_WINDOWS, "Display");

    if (BeginCard("##Hud", "HUD & Menus")) {
        DrawSetting(settings.uiFollowsGaze ? ICON_MS_HEAD_MOUNTED_DEVICE : ICON_MS_PUSH_PIN, "Follows Your Head", [&]() {
            DrawToggle(settings.uiFollowsGaze, changed);
        });
        DrawSetting(ICON_MS_OPEN_IN_FULL, "Distance & Size", [&]() {
            DrawFloatSlider(settings.hudDistance, settings.hudDistance.min, settings.hudDistance.max, FormatDistance, changed);
            DrawFloatSlider(settings.hudSize, settings.hudSize.min, settings.hudSize.max, [](float size) { return fmt::format("{:.2f}x size", size); }, changed);
        });
        EndCard();
    }

    if (BeginCard("##Window", "Cemu's Window")) {
        DrawRecordingModeSetting();
        EndCard();
    }
}

void RND_Renderer::ImGuiOverlay::DrawSystemPage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_MS_SETTINGS, "System");

    if (const std::optional<std::string> newerRelease = UpdateCheck::GetNewerRelease()) {
        if (BeginCard("##Update")) {
            const std::string text = fmt::format("BetterVR {} is out, you're on {}.", *newerRelease, BETTERVR_VERSION_STRING);
            DrawSetting(ICON_MS_DOWNLOAD, text.c_str(), UpdateCheck::kReleasesUrl, []() {});
            EndCard();
        }
    }

    if (BeginCard("##General")) {
        DrawSetting(ICON_MS_ROCKET_LAUNCH, "Boot Straight Into BotW", [&]() {
            DrawToggle(settings.bootDirectlyIntoGame, changed);
        });
        DrawSetting(ICON_MS_EXTENSION, "Graphic Packs", [&]() {
            if (WideButton("Open On Desktop")) {
                CemuHost::OpenGraphicPacksWindow();
                RequestRestartNotice("Restart Cemu after changing graphic packs", "Changes in that window apply the next time you start Cemu.");
            }
        });
        EndCard();
    }

    if (BeginCard("##Performance", "Performance")) {
        static constexpr EnumChoice<FoveatedRenderingMode> foveationChoices[] = { { FoveatedRenderingMode::OFF, "Off" }, { FoveatedRenderingMode::ON, "On" }, { FoveatedRenderingMode::EXAGGERATED, "Test" } };
        DrawSetting(ICON_MS_CENTER_FOCUS_STRONG, "Foveated Rendering", "Less detail at the edges, so it runs faster.", [&]() {
            DrawEnumSegments(settings.foveatedRendering, foveationChoices, changed);
        });
        static constexpr EnumChoice<PerformanceOverlayMode> overlayChoices[] = {
            { PerformanceOverlayMode::DISABLE, "Off" },
            { PerformanceOverlayMode::WINDOW_ONLY, "In Cemu's window" },
            { PerformanceOverlayMode::WINDOW_AND_VR, "In Cemu and VR" },
            { PerformanceOverlayMode::WINDOW_AND_VR_WITH_PROFILER, "In Cemu and VR, with profiler" },
        };
        DrawSetting(ICON_MS_SPEED, "FPS Overlay", [&]() {
            DrawEnumCombo(settings.performanceOverlay, overlayChoices, changed);
        });
        EndCard();
    }

    if (BeginCard("##Reset")) {
        DrawSetting(ICON_MS_RESTART_ALT, "Reset All Settings", [&]() {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.45f, 0.12f, 0.12f, 1.0f));
            if (ImGui::Button("Reset Everything", ImVec2(-FLT_MIN, 0.0f))) {
                for (ModSettingBase* option : settings.GetOptions()) {
                    option->Reset();
                }
                *changed = true;
            }
            ImGui::PopStyleColor(3);
        });
        EndCard();
    }

    if (DrawAdvancedHeader("##SystemAdvanced")) {
        if (BeginCard("##Debugging", nullptr, false)) {
            DrawSetting(nullptr, "Debugging Console", [&]() {
                DrawToggle(settings.showDebugConsole, changed);
            });
#ifdef _DEBUG
            DrawSetting(nullptr, "Debugger Tools", "Lowers performance and can cause crashes.", [&]() {
                DrawToggle(settings.enableDebuggerTools, changed);
            });
#endif
            EndCard();
        }
    }

    // the read-only info goes last so it doesn't interrupt the gamepad cursor
    if (settings.performanceOverlay != PerformanceOverlayMode::DISABLE) {
        DrawSectionCaption("Live Frame Timings");
        DrawFPSOverlayContent(VRManager::instance().XR->GetRenderer(), true, OverlayProfilerMode::NONE);
        DrawNavScrollStop("##AfterFrameTimings");
    }

#ifdef _DEBUG
    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    DrawBetterVRProfiler(true);
#endif
    DrawNavScrollStop("##SystemBottom");
}

static void DrawLogToggle(const char* label, BoolSetting& setting, bool* changed) {
    bool enabled = setting;
    if (ImGui::Checkbox(label, &enabled)) {
        setting = enabled;
        *changed = true;
    }
}

void RND_Renderer::ImGuiOverlay::DrawDebugPage(bool* changed) {
    DrawPageHeader(ICON_MS_BUG_REPORT, "Debug");

    auto* entityDebugger = VRManager::instance().Debugger.get();
    if (entityDebugger != nullptr) {
        ImGui::Dummy(ImVec2(0.0f, 10.0f));
        ImGui::SeparatorText("World-Space Overlay");
        entityDebugger->DrawWorldSpaceOverlaySettings(changed);
    }

    if (entityDebugger != nullptr) {
        ImGui::Dummy(ImVec2(0.0f, 10.0f));
        ImGui::SeparatorText("Entity Debugger");
        entityDebugger->DrawEntityInspectorContent();
    }

    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    ImGui::SeparatorText("Attack Debugger");
    bool showLeftOverlay = WeaponAttackDebugger::IsOverlayVisible(EyeSide::LEFT);
    if (ImGui::Checkbox("Left Hand", &showLeftOverlay)) {
        WeaponAttackDebugger::SetOverlayVisible(EyeSide::LEFT, showLeftOverlay);
    }
    ImGui::SameLine();
    bool showRightOverlay = WeaponAttackDebugger::IsOverlayVisible(EyeSide::RIGHT);
    if (ImGui::Checkbox("Right Hand", &showRightOverlay)) {
        WeaponAttackDebugger::SetOverlayVisible(EyeSide::RIGHT, showRightOverlay);
    }

    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    ImGui::SeparatorText("Log Categories");

    auto& settings = GetSettings();
    DrawLogToggle("Rendering", settings.logRendering, changed);
    DrawLogToggle("Interop", settings.logInterop, changed);
    DrawLogToggle("Controls", settings.logControls, changed);
    DrawLogToggle("PPC Hooks", settings.logPpc, changed);
    DrawLogToggle("OpenXR Debug Utils", settings.logXrDebugUtils, changed);
    DrawLogToggle("Arrow Shot Capture", settings.logArrowShotCapture, changed);
    DrawLogToggle("Verbose", settings.logVerbose, changed);

    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    ImGui::SeparatorText("Log Line Prefix");
    DrawLogToggle("Timestamps", settings.logTimestamps, changed);
    DrawLogToggle("Process & Thread IDs", settings.logThreadIds, changed);
}

void RND_Renderer::ImGuiOverlay::DrawGuidePage(size_t helpImageIndex) {
    DrawNavScrollStop("##PageTop");

    if (helpImageIndex >= m_helpImages.size()) {
        return;
    }
    const auto& image = m_helpImages[helpImageIndex];

    if (ImGui::BeginChild("HelpImage", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_NoBackground)) {
        ImVec2 availableSize = ImGui::GetContentRegionAvail();
        float widthScale = availableSize.x / (float)image.m_image->GetWidth();
        float heightScale = availableSize.y / (float)image.m_image->GetHeight();
        float imageScale = std::min(widthScale, heightScale);
        ImVec2 imageSize = ImVec2((float)image.m_image->GetWidth() * imageScale, (float)image.m_image->GetHeight() * imageScale);

        ImVec2 cursorPos = ImGui::GetCursorPos();
        if (availableSize.x > imageSize.x) {
            cursorPos.x += (availableSize.x - imageSize.x) * 0.5f;
        }
        if (availableSize.y > imageSize.y) {
            cursorPos.y += (availableSize.y - imageSize.y) * 0.5f;
        }
        ImGui::SetCursorPos(cursorPos);
        ImGui::Image((ImTextureID)image.m_imageDS, imageSize);
    }
    ImGui::EndChild();
}

void RND_Renderer::ImGuiOverlay::DrawCreditsPage() {
    DrawPageHeader(ICON_MS_FAVORITE, "Credits");

    ImGui::TextWrapped("Hey there!");
    ImGui::Text("");
    ImGui::TextWrapped("This mod is free and open-source, but it took a lot of late nights to create.");
    ImGui::TextWrapped("If you're enjoying the mod, you can donate here to suggest additions and help test features early. Thanks!");
    DrawLink(ICON_KI_GITHUB " https://github.com/sponsors/Crementif/", "https://github.com/sponsors/Crementif/");
    DrawLink(ICON_KI_PATREON " https://patreon.com/crementif", "https://patreon.com/crementif");
    ImGui::Text("");
    ImGui::Text("- Crementif");
    DrawNavScrollStop("##AfterDonate");

    DrawSupporterWall();
    DrawNavScrollStop("##AfterSupporters");

    ImGui::SeparatorText("Credits");
    DrawCreditsList();
    ImGui::Text("");

    ImGui::Dummy(ImVec2(0.0f, 10.0f));
    DrawNavScrollStop("##CreditsBottom");
}

static const char* GetControllerModMenuPrompt() {
    auto& caps = VRManager::instance().XR->m_capabilities;
    switch (caps.activeControllerType) {
        case ControllerType::OculusTouch:
        case ControllerType::Pico4:
        case ControllerType::HPReverbG2:
        case ControllerType::ViveCosmos:
            return "Hold the " ICON_KI_BUTTON_X " button on your left controller for 1 second to open the mod menu.\nAlternatively, for regular gamepad controllers, use the " ICON_KI_BUTTON_START " button.";
        case ControllerType::ValveIndex:
            return "Hold the " ICON_KI_BUTTON_A " button on your left controller for 1 second to open the mod menu.\nAlternatively, for regular gamepad controllers, use the " ICON_KI_BUTTON_START " button.";
        default:
            return "Hold the " ICON_KI_BUTTON_START " button on your gamepad from Cemu's input settings for 1 second to open the mod menu.";
    }
}

static const char* GetControllerMenuNavPrompt() {
    auto& caps = VRManager::instance().XR->m_capabilities;
    switch (caps.activeControllerType) {
        case ControllerType::OculusTouch:
        case ControllerType::Pico4:
        case ControllerType::HPReverbG2:
        case ControllerType::ViveCosmos:
        case ControllerType::ValveIndex:
            return ICON_KI_STICK_LEFT_TOP " Navigate      " ICON_KI_BUTTON_A " Select      " ICON_KI_BUTTON_B " Exit      " ICON_KI_BUTTON_LT " " ICON_KI_BUTTON_RT " Switch Page";
        default:
            return ICON_KI_STICK_LEFT_TOP " Navigate      " ICON_KI_BUTTON_A " Select      " ICON_KI_BUTTON_B " Exit      " ICON_KI_BUTTON_L " " ICON_KI_BUTTON_R " Switch Page";
    }
}

static void DrawTutorialPromptAndControllerVerification(bool isMenuOpen) {
    auto& settings = GetSettings();

    float alphaForNotify = settings.tutorialPromptShown ? 0.9f : 0.98f;
    float timeLimit = settings.tutorialPromptShown ? 14.0f : 60.0f;

    if (!settings.tutorialPromptShown) {
        if (isMenuOpen || ImGui::GetTime() > timeLimit) {
            settings.tutorialPromptShown = true;
            ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
        }
    }

    if (isMenuOpen || ImGui::GetTime() >= timeLimit) {
        return;
    }

    ImVec2 fullWindowWidth = ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y);
    ImGui::SetNextWindowBgAlpha(alphaForNotify);
    ImGui::SetNextWindowPos(fullWindowWidth * 0.5f, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("HelpNotify", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs)) {
        if (!settings.tutorialPromptShown) {
            ImGui::Text("First-Time Setup:");
            ImGui::Separator();
            ImGui::Text("To get started, open the BetterVR menu to configure various settings and to see the controller guide.");
            ImGui::Text("This is where you can adjust the camera mode, player height, and other options to suit your preferences.");
            ImGui::Spacing();
            ImGui::Text("%s", GetControllerModMenuPrompt());
            ImGui::Text("");
            ImGui::Text("TO CONTINUE: Try holding the button and open the menu");
        }
        else {
            ImGui::Text("%s", GetControllerModMenuPrompt());
        }
    }
    ImGui::End();
}

static void DrawUpdateBanner(bool isMenuOpen) {
    constexpr double kShownFor = 20.0;
    constexpr double kFadeOutFor = 2.0;
    static double s_shownSince = -1.0;
    static bool s_dismissed = false;

    const std::optional<std::string> newerRelease = UpdateCheck::GetNewerRelease();
    s_dismissed |= isMenuOpen;
    if (!newerRelease || s_dismissed) {
        return;
    }
    if (s_shownSince < 0.0) {
        s_shownSince = ImGui::GetTime();
    }
    const double shownFor = ImGui::GetTime() - s_shownSince;
    if (shownFor >= kShownFor) {
        s_dismissed = true;
        return;
    }

    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, (float)std::clamp((kShownFor - shownFor) / kFadeOutFor, 0.0, 1.0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 16.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.10f, 0.24f, 0.42f, 0.96f));
    ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, displaySize.y * 0.12f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    if (ImGui::Begin("UpdateBanner", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs)) {
        if (ImGuiMenus::g_titleFont != nullptr) {
            ImGui::PushFont(ImGuiMenus::g_titleFont);
        }
        ImGui::Text(ICON_KI_INFO_CIRCLE "  BetterVR %s is out!", newerRelease->c_str());
        if (ImGuiMenus::g_titleFont != nullptr) {
            ImGui::PopFont();
        }
        ImGui::Spacing();
        ImGui::Text("You're on %s. Get the new version from %s", BETTERVR_VERSION_STRING, UpdateCheck::kReleasesUrl);
        ImGui::Text("Cemu's window has a button that opens the page.");
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

// only cutscenes marked PROMPT_ON in patch_Settings_Cutscenes.asm offer it, once per cutscene and never over dialogue
static void DrawCutsceneCameraPrompt(bool isMenuOpen) {
    constexpr double kShowAfter = 3.0;
    constexpr double kShownFor = 5.0;
    constexpr double kMinimalShownFor = 2.5;
    constexpr double kFadeOutFor = 1.0;
    constexpr double kChainedCutsceneGap = 3.0;
    constexpr uint32_t kSwitchesBeforeMinimal = 3;
    static double s_cutsceneStart = -1.0;
    static double s_cutsceneEnd = -1.0;
    static double s_shownSince = -1.0;
    static uint32_t s_seenToggles = 0;

    auto& settings = GetSettings();
    const Game::Presentation game = Game::GetPresentation();
    const double now = ImGui::GetTime();
    if (!game.cutsceneActive) {
        if (s_cutsceneStart >= 0.0) {
            s_cutsceneEnd = now;
            s_cutsceneStart = -1.0;
        }
        s_seenToggles = game.cutsceneCameraToggles;
        return;
    }
    if (s_cutsceneStart < 0.0) {
        s_cutsceneStart = now;
        if (now - s_cutsceneEnd > kChainedCutsceneGap) {
            s_shownSince = -1.0;
        }
    }

    // switching shows the minimal prompt as confirmation, even in cutscenes that don't offer it
    if (game.cutsceneCameraToggles != s_seenToggles) {
        s_seenToggles = game.cutsceneCameraToggles;
        s_shownSince = now;
        if (settings.cutsceneCameraSwitches < kSwitchesBeforeMinimal) {
            settings.cutsceneCameraSwitches = settings.cutsceneCameraSwitches + 1;
            ImGui::MarkIniSettingsDirty();
        }
    }

    const ControllerType controllerType = VRManager::instance().XR->m_capabilities.activeControllerType;
    const bool canShow = game.cutsceneCameraSwitchable && !game.dialogVisible && !isMenuOpen && controllerType != ControllerType::Unknown && controllerType != ControllerType::Simple;
    if (s_shownSince < 0.0) {
        if (!game.cutsceneShowsCameraPrompt || !canShow || now - s_cutsceneStart < kShowAfter) {
            return;
        }
        s_shownSince = now;
    }

    const bool minimal = settings.cutsceneCameraSwitches >= kSwitchesBeforeMinimal;
    const double shownFor = now - s_shownSince;
    const double showFor = minimal ? kMinimalShownFor : kShownFor;
    if (!canShow || shownFor >= showFor) {
        return;
    }

    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, (float)std::clamp((showFor - shownFor) / kFadeOutFor, 0.0, 1.0) * (minimal ? 0.7f : 1.0f));
    ImGui::SetNextWindowBgAlpha(minimal ? 0.6f : 0.85f);
    ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, displaySize.y * (minimal ? 0.9f : 0.85f)), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    if (ImGui::Begin("CutsceneCameraPrompt", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoInputs)) {
        if (minimal) {
            ImGui::SetWindowFontScale(0.8f);
            ImGui::Text(ICON_KI_STICK_LEFT_TOP " %s person", game.cutsceneFirstPerson ? "Third" : "First");
        }
        else {
            ImGui::Text(ICON_KI_STICK_LEFT_TOP "  Click the left stick to watch this cutscene in %s person", game.cutsceneFirstPerson ? "third" : "first");
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
}

void RND_Renderer::ImGuiOverlay::DrawHelpMenu() {
    auto& isMenuOpen = ImGuiMenus::g_isOpen;
    auto& settings = GetSettings();

    DrawTutorialPromptAndControllerVerification(isMenuOpen);
    DrawUpdateBanner(isMenuOpen);
    DrawCutsceneCameraPrompt(isMenuOpen);

    static bool wasMenuPrevOpened = false;
    static bool shouldLogSavedSettingsOnClose = false;
    static ImVec2 lastMousePos = ImVec2(-FLT_MAX, -FLT_MAX);
    static double lastMouseActivityTime = -FLT_MAX;
    static bool showTitleBar = false;

    if (!isMenuOpen && wasMenuPrevOpened) {
        wasMenuPrevOpened = false;
        if (shouldLogSavedSettingsOnClose) {
            Settings_LogSavedSettings();
            shouldLogSavedSettingsOnClose = false;
        }
    }

    if (!isMenuOpen) {
        return;
    }

    if (isMenuOpen && !wasMenuPrevOpened) {
        ImGui::SetNextWindowFocus();
        wasMenuPrevOpened = true;
        shouldLogSavedSettingsOnClose = false;
        lastMousePos = ImGui::GetIO().MousePos;
        lastMouseActivityTime = -FLT_MAX;
        showTitleBar = false;
    }

    ImVec2 displaySize = ImVec2(ImGui::GetIO().DisplaySize.x, ImGui::GetIO().DisplaySize.y);
    ImVec2 windowSize = displaySize * ImVec2(0.925f, 1.0f);

    auto& selectedPage = ImGuiMenus::g_currentPage;
    if (!ImGuiMenus::IsPageAvailable(selectedPage)) {
        selectedPage = ImGuiMenus::PLAYSTYLE_PAGE;
    }

    bool shouldStayOpen = true;

    // the title bar only exists to give mouse users a close button, so it stays hidden until the mouse is actually being moved
    const ImGuiIO& io = ImGui::GetIO();
    ImVec2 windowTopLeft = (displaySize - windowSize) * 0.5f;
    bool isMouseOverTitleBar = showTitleBar && io.MousePos.x >= windowTopLeft.x && io.MousePos.x <= windowTopLeft.x + windowSize.x && io.MousePos.y >= windowTopLeft.y && io.MousePos.y <= windowTopLeft.y + ImGui::GetFrameHeight();
    if (io.MousePos.x != lastMousePos.x || io.MousePos.y != lastMousePos.y || isMouseOverTitleBar) {
        lastMousePos = io.MousePos;
        lastMouseActivityTime = ImGui::GetTime();
    }
    showTitleBar = (ImGui::GetTime() - lastMouseActivityTime) < 2.5;

    ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    if (!showTitleBar) {
        windowFlags |= ImGuiWindowFlags_NoTitleBar;
    }

    ImGui::SetNextWindowPos(displaySize * 0.5f, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(windowSize, ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    if (ImGui::Begin("BetterVR Settings & Help##Settings", &shouldStayOpen, windowFlags)) {
        bool changed = false;
        float footerHeight = ImGui::GetFrameHeight() * 1.5f;
        ImGuiWindow* sidebarWindow = nullptr;
        ImGuiWindow* contentWindow = nullptr;
        SidebarNavTarget sidebarNavTarget;

        float sidebarWidth = std::min(215.0f, windowSize.x * 0.3f);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05490196f, 0.07843138f, 0.09411765f, 1.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 12.0f));
        if (ImGui::BeginChild("Sidebar", ImVec2(sidebarWidth, -footerHeight), false, ImGuiWindowFlags_NavFlattened | ImGuiWindowFlags_AlwaysUseWindowPadding)) {
            sidebarWindow = ImGui::GetCurrentWindow();
            sidebarNavTarget = DrawSidebar(selectedPage);
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        FollowPageSwitch(sidebarWindow, sidebarNavTarget, selectedPage);

        ImGui::SameLine(0.0f, 0.0f);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 8.0f));
        ImGui::PushID(selectedPage);
        if (ImGui::BeginChild("Content", ImVec2(0.0f, -footerHeight), false, ImGuiWindowFlags_NavFlattened | ImGuiWindowFlags_AlwaysUseWindowPadding)) {
            contentWindow = ImGui::GetCurrentWindow();
            if (WantsToEnterContentPane(sidebarWindow, sidebarNavTarget)) {
                EnterContentPane(contentWindow, sidebarNavTarget.enterContentRequested);
            }
            switch (selectedPage) {
                case ImGuiMenus::PLAYSTYLE_PAGE: DrawPlaystylePage(&changed); break;
                case ImGuiMenus::COMFORT_PAGE: DrawComfortPage(&changed); break;
                case ImGuiMenus::CONTROLS_PAGE: DrawControlsPage(&changed); break;
                case ImGuiMenus::DISPLAY_PAGE: DrawDisplayPage(&changed); break;
                case ImGuiMenus::SYSTEM_PAGE: DrawSystemPage(&changed); break;
                case ImGuiMenus::GUIDE_CONTROLS_PAGE: DrawGuidePage(0); break;
                case ImGuiMenus::GUIDE_EQUIPMENT_PAGE: DrawGuidePage(1); break;
                case ImGuiMenus::GUIDE_SWING_PAGE: DrawGuidePage(2); break;
                case ImGuiMenus::GUIDE_WHISTLE_PAGE: DrawGuidePage(3); break;
                case ImGuiMenus::CREDITS_PAGE: DrawCreditsPage(); break;
                case ImGuiMenus::DEBUG_PAGE: DrawDebugPage(&changed); break;
                default: break;
            }
        }
        ImGui::EndChild();
        ImGui::PopID();
        ImGui::PopStyleVar();

        if (settings.bootDirectlyIntoGame && settings.bootDirectlyTitleId != CemuHost::GetCurrentTitleId()) {
            settings.bootDirectlyTitleId = CemuHost::GetCurrentTitleId();
            changed = true;
        }
        if (changed) {
            shouldLogSavedSettingsOnClose = true;
            ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
        }

        ImGui::Separator();
        ImGui::Spacing();
        const char* navText = GetControllerMenuNavPrompt();
        float textWidth = ImGui::CalcTextSize(navText).x;
        float availW = ImGui::GetContentRegionAvail().x;
        if (availW > textWidth) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (availW - textWidth) * 0.5f);
        }
        ImGui::TextUnformatted(navText);

        ApplyPaneNavFallback(sidebarWindow, contentWindow, sidebarNavTarget);
        DrawRestartNotice();
    }

    ImGui::End();
    ImGui::PopStyleVar(2);

    if (!shouldStayOpen) {
        isMenuOpen = false;
    }
}
