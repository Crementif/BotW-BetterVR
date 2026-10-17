#include "pch.h"

#include "imgui_menus.h"

#include "cemu/cemu_host.h"
#include "game/botw/debug/weapon_attack_debugger.h"
#include "imgui_internal.h"
#include "instance.h"
#include "rendering/renderer.h"
#include "rendering/openxr.h"
#include "utils/profiler.h"
#include "utils/mod_settings.h"
#include "utils/update_check.h"
#include "supporters_generated.h"

static constexpr ImVec4 kDescriptionColor = ImVec4(0.63f, 0.71f, 0.78f, 1.0f);
static constexpr ImVec4 kSectionCaptionColor = ImVec4(0.40f, 0.66f, 1.0f, 1.0f);
static constexpr ImVec4 kHintColor = ImVec4(0.35f, 0.95f, 0.45f, 1.0f);
static constexpr ImVec4 kHighTierSupporterColor = ImVec4(0.78f, 0.53f, 1.0f, 1.0f);
static constexpr ImVec4 kSupporterColor = ImVec4(0.29f, 0.80f, 0.90f, 1.0f);
static constexpr ImVec4 kCreditNameColor = ImVec4(1.0f, 0.80f, 0.42f, 1.0f);
static constexpr ImVec4 kLinkColor = ImVec4(0.26f, 0.59f, 0.98f, 1.0f);
static constexpr ImVec4 kLinkHoveredColor = ImVec4(0.45f, 0.72f, 1.0f, 1.0f);

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
    ImGui::Dummy(ImVec2(0.0f, 4.0f));
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

static bool BeginSettingsSection(const char* id, const char* caption = nullptr) {
    if (caption != nullptr) {
        DrawSectionCaption(caption);
    }
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(10.0f, 9.0f));
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoSavedSettings)) {
        ImGui::PopStyleVar();
        return false;
    }
    ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthStretch, 0.55f);
    ImGui::TableSetupColumn("##widget", ImGuiTableColumnFlags_WidthStretch, 0.45f);
    return true;
}

static void EndSettingsSection() {
    ImGui::EndTable();
    ImGui::PopStyleVar();
}

template <typename DrawWidget>
static void DrawSetting(const char* label, const char* description, DrawWidget&& drawWidget) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    if (description != nullptr && description[0] != '\0') {
        ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
        ImGui::TextWrapped("%s", description);
        ImGui::PopStyleColor();
    }
    ImGui::TableSetColumnIndex(1);
    ImGui::PushID(label);
    ImGui::PushItemWidth(-FLT_MIN);
    drawWidget();
    ImGui::PopItemWidth();
    ImGui::PopID();
}

template <typename DrawWidget>
static void DrawSetting(const char* label, DrawWidget&& drawWidget) {
    DrawSetting(label, nullptr, std::forward<DrawWidget>(drawWidget));
}

struct MenuPageEntry {
    uint8_t page;
    const char* label;
};

static constexpr MenuPageEntry kSettingsPages[] = {
    { ImGuiMenus::PLAYSTYLE_PAGE, ICON_KI_PLAYER "  Playstyle" },
    { ImGuiMenus::COMFORT_PAGE, ICON_KI_FEATHER "  Comfort" },
    { ImGuiMenus::COMBAT_PAGE, ICON_KI_SWORD "  Combat" },
    { ImGuiMenus::CONTROLS_PAGE, ICON_KI_GAMEPAD "  Controls" },
    { ImGuiMenus::INTERFACE_PAGE, ICON_KI_UI "  Interface" },
    { ImGuiMenus::SYSTEM_PAGE, ICON_KI_COG "  System" },
};

static constexpr MenuPageEntry kHowToPlayPages[] = {
    { ImGuiMenus::GUIDE_CONTROLS_PAGE, ICON_KI_QUESTION_CIRCLE "  The Controls" },
    { ImGuiMenus::GUIDE_EQUIPMENT_PAGE, ICON_KI_QUESTION_CIRCLE "  Weapons & Runes" },
    { ImGuiMenus::GUIDE_SWING_PAGE, ICON_KI_QUESTION_CIRCLE "  Swinging & Blocking" },
    { ImGuiMenus::GUIDE_WHISTLE_PAGE, ICON_KI_QUESTION_CIRCLE "  Whistle & Magnesis" },
};

static constexpr MenuPageEntry kMorePages[] = {
    { ImGuiMenus::CREDITS_PAGE, ICON_KI_HEART "  Credits" },
    { ImGuiMenus::DEBUG_PAGE, ICON_KI_WRENCH "  Debug" },
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
        if (isSelectedPage) {
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
    { ICON_KI_SWORD, "Acudofy", "Sword and stab analysis system" },
    { ICON_KI_SEARCH, "leoetlino", "Made the BotW Decomp project, which was useful while reverse engineering" },
    { ICON_KI_GAUGE, "Exzap", "Technical support and optimization help" },
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
    const float predictedHz = GetSettings().performanceOverlayFrequency;

    // the interval between the predicted display times of the frames that were submitted, averaged over half a second
    const float appMs = (float)renderer->GetAverageFrameTimeMs();
    const float appFps = appMs > 0.0000001f ? (1000.0f / appMs) : 0.0f;

    if (renderText) {
        ImGui::Text("The visualization refresh rate of your headset is set to %.0f Hz", predictedHz);
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

void RND_Renderer::ImGuiOverlay::DrawPlaystylePage(bool* changed) {
    auto& settings = GetSettings();
    const CameraMode cameraMode = settings.cameraMode;

    DrawPageHeader(ICON_KI_PLAYER, "Playstyle");

    if (BeginSettingsSection("##CameraMode")) {
        DrawSetting("Camera Mode", "First person uses motion controls. Third person uses a controller.", [&]() {
            settings.cameraMode.AddRadioToGUI(changed, true);
        });
        EndSettingsSection();
    }

    if (cameraMode == CameraMode::THIRD_PERSON) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, kHintColor);
        ImGui::TextWrapped(ICON_KI_INFO_CIRCLE "  Use a regular controller and set up its input in Cemu!");
        ImGui::PopStyleColor();

        if (BeginSettingsSection("##ThirdPerson", "Third-Person Camera")) {
            DrawSetting("Camera Distance", [&]() {
                settings.thirdPlayerDistance.AddSliderToGUI(changed, 0.5f, 0.65f);
            });
            DrawSetting("Shoot Arrows From Camera", "Aims the bow from the camera instead of from Link's hands.", [&]() {
                settings.thirdPersonBowCameraAim.AddToGUI(changed);
            });
            EndSettingsSection();
        }
    }
    else {
        if (BeginSettingsSection("##FirstPerson", "Player Height")) {
            DrawSetting("Play Position", "Standing will use your real height. Seated will auto-calibrate Link's height when you load into the game.", [&]() {
                settings.playMode.AddRadioToGUI(changed, true);
            });
            if (settings.GetPlayMode() == PlayMode::SEATED) {
                DrawSetting("Seated Height", "Sit how you want to play, then reset to measure again.", [&]() {
                    if (ImGui::Button("Reset Seated Height", ImVec2(-FLT_MIN, 0.0f))) {
                        VRManager::instance().XR->RequestSeatedHeightCalibration();
                    }
                    ImGui::PushStyleColor(ImGuiCol_Text, kDescriptionColor);
                    if (auto eyeHeight = VRManager::instance().XR->GetCalibratedSeatedEyeHeight()) {
                        ImGui::Text("Measured eye height: %.2fm above the floor", eyeHeight.value());
                    }
                    else {
                        ImGui::TextUnformatted("Not measured yet, using your real height");
                    }
                    ImGui::PopStyleColor();
                });
            }
            DrawSetting("Height Adjustment", [&]() {
                auto formatHeight = [&](float height) {
                    if (height < -.01f) {
                        float heightInches = height * -39.3700787f;
                        int32_t heightFeet = std::floor(heightInches / 12.0f);
                        heightInches -= heightFeet * 12.0f;
                        return fmt::format("-{0:.02f}m / {1}\'{2:.02f}\"", -height, heightFeet, heightInches);
                    }
                    if (height > .01f) {
                        float heightInches = height * 39.3700787f;
                        int32_t heightFeet = std::floor(heightInches / 12.0f);
                        heightInches -= heightFeet * 12.0f;
                        return fmt::format("+{0:.02f}m / {1}\'{2:.02f}\"", height, heightFeet, heightInches);
                    }
                    return std::string("0.0");
                };
                settings.playerHeightOffset.AddToGUI(changed, -0.5f, 1.0f, formatHeight);
            });
            EndSettingsSection();
        }
    }
}

void RND_Renderer::ImGuiOverlay::DrawComfortPage(bool* changed) {
    auto& settings = GetSettings();
    const CameraMode cameraMode = settings.cameraMode;

    DrawPageHeader(ICON_KI_FEATHER, "Comfort");

    if (cameraMode == CameraMode::FIRST_PERSON) {
        if (BeginSettingsSection("##General")) {
            DrawSetting("Turn Mode", "Snap turning can help if you get motion sick while rotating the camera.", [&]() {
                settings.turnMode.AddComboToGUI(changed);
            });
            DrawSetting("Prevent Falling Over", "Stops Link from ragdolling, which sends the camera tumbling.", [&]() {
                settings.preventFirstPersonRagdoll.AddToGUI(changed);
            });
            EndSettingsSection();
        }
    }

    if (BeginSettingsSection("##Cutscenes", "Cutscenes")) {
        if (cameraMode == CameraMode::FIRST_PERSON) {
            DrawSetting("Camera In Cutscenes", [&]() {
                settings.cutsceneCameraMode.AddComboToGUI(changed);
            });
        }
        DrawSetting("Show All Cutscenes In 2D Theatre Mode", "First person cutscenes still play in 3D.", [&]() {
            settings.useBlackBarsForCutscenes.AddToGUI(changed);
        });
        if (cameraMode == CameraMode::FIRST_PERSON) {
            DrawSetting("Keep The Camera Still In Cutscenes", [&]() {
                settings.alwaysPreventFirstPersonCutsceneCameraMovement.AddToGUI(changed);
            });
        }
        EndSettingsSection();
    }

    if (cameraMode != CameraMode::FIRST_PERSON) {
        DrawPageNote("More comfort options become available in First Person mode.");
    }
}

void RND_Renderer::ImGuiOverlay::DrawCombatPage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_KI_SWORD, "Combat");

    if (settings.cameraMode == CameraMode::FIRST_PERSON) {
        if (BeginSettingsSection("##Attacks", "Melee Attacks")) {
            DrawSetting("Attack Sensitivity", "Relaxed picks up lighter swings than normal does.", [&]() {
                settings.swingSensitivity.AddRadioToGUI(changed);
            });
            DrawSetting("One Hit Per Swing", "A swing damages each enemy once. Turn it off to hit repeatedly instead, for less damage per hit.", [&]() {
                settings.oneHitPerSwing.AddToGUI(changed);
            });
            EndSettingsSection();
        }
    }
    else {
        DrawPageNote("Motion controls for melee attacks only work in First Person mode.");
    }

    if (BeginSettingsSection("##Aiming", "Bow Aiming")) {
        DrawSetting("Bow Aiming Arc Visibility", "The arc shows where your arrow will land while you aim.", [&]() {
            settings.bowArcOpacity.AddPercentToGUI(changed, 0.0f, 100.0f);
        });
        EndSettingsSection();
    }
}

void RND_Renderer::ImGuiOverlay::DrawControlsPage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_KI_GAMEPAD, "Controls");

    if (settings.cameraMode == CameraMode::THIRD_PERSON) {
        DrawPageNote("Third person is played with a regular controller, which you set up in Cemu's toolbar under Options > Input Settings.");
    }

    if (BeginSettingsSection("##Sticks", "Thumbsticks")) {
        if (settings.cameraMode == CameraMode::FIRST_PERSON) {
            DrawSetting("Walking Direction", "Walk where you're looking, or where your controller points.", [&]() {
                settings.walkingDirection.AddComboToGUI(changed);
            });
        }
        DrawSetting("Thumbstick Deadzone", "Fixes Link walking on his own when the stick rests off-center.", [&]() {
            settings.stickDeadzone.AddPercentToGUI(changed, 0.0f, 50.0f);
        });
        DrawSetting("Stick Direction Threshold", "How far you push the stick before it counts as a direction.", [&]() {
            settings.axisThreshold.AddPercentToGUI(changed, 10.0f, 90.0f);
        });
        EndSettingsSection();
    }

    if (VRManager::instance().XR->m_capabilities.isOculusLinkRuntime) {
        if (BeginSettingsSection("##Fixes", "Controller Fixes")) {
            DrawSetting("Fix Controller Rotation", "Leave this on Auto unless your sword swings don't come out.", [&]() {
                settings.buggyAngularVelocity.AddComboToGUI(changed);
            });
            EndSettingsSection();
        }
    }
    else {
        settings.buggyAngularVelocity = AngularVelocityFixerMode::AUTO;
    }
}

void RND_Renderer::ImGuiOverlay::DrawInterfacePage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_KI_UI, "Interface");

    if (BeginSettingsSection("##Hud", "HUD & Menus")) {
        DrawSetting("UI Follows Where You Look", "When off, the HUD stays in one place instead of following your head.", [&]() {
            settings.uiFollowsGaze.AddToGUI(changed);
        });
        DrawSetting("Menu/HUD Distance & Size", [&]() {
            float distance = settings.hudDistance;
            if (ImGui::SliderFloat("##hudDistance", &distance, settings.hudDistance.min, settings.hudDistance.max, FormatDistance(distance).c_str())) {
                settings.hudDistance = std::clamp(distance, settings.hudDistance.min, settings.hudDistance.max);
                *changed = true;
            }
            ImGui::PushItemWidth(-(ImGui::CalcTextSize("Reset").x + ImGui::GetStyle().FramePadding.x * 2.0f + ImGui::GetStyle().ItemSpacing.x));
            float size = settings.hudSize;
            if (ImGui::SliderFloat("##hudSize", &size, settings.hudSize.min, settings.hudSize.max, "%.2fx size")) {
                settings.hudSize = std::clamp(size, settings.hudSize.min, settings.hudSize.max);
                *changed = true;
            }
            ImGui::PopItemWidth();
            ImGui::SameLine();
            if (ImGui::Button("Reset")) {
                settings.hudDistance.Reset();
                settings.hudSize.Reset();
                *changed = true;
            }
        });
        EndSettingsSection();
    }
}

void RND_Renderer::ImGuiOverlay::DrawSystemPage(bool* changed) {
    auto& settings = GetSettings();

    DrawPageHeader(ICON_KI_COG, "System");

    if (BeginSettingsSection("##General")) {
        DrawSetting("Boot Directly Into BotW", "Skips Cemu's game list on startup.", [&]() {
            settings.bootDirectlyIntoGame.AddToGUI(changed);
        });
        DrawSetting("Show FPS Overlay", [&]() {
            settings.performanceOverlay.AddComboToGUI(changed);
        });
        if (settings.performanceOverlay != PerformanceOverlayMode::DISABLE) {
            static const uint32_t freqOptions[] = { 30, 60, 72, 80, 90, 120, 144 };
            uint32_t currentFreq = settings.performanceOverlayFrequency;
            int freqIdx = 5;
            for (int i = 0; i < std::size(freqOptions); i++) {
                if (freqOptions[i] == currentFreq) {
                    freqIdx = i;
                    break;
                }
            }

            DrawSetting("Headset Refresh Rate", "The graph's target line is drawn at this refresh rate.", [&]() {
                if (ImGui::SliderInt("##RefreshRate", &freqIdx, 0, (int)std::size(freqOptions) - 1, fmt::format("{} Hz", freqOptions[freqIdx]).c_str())) {
                    settings.performanceOverlayFrequency = freqOptions[freqIdx];
                    *changed = true;
                }
            });
        }
        EndSettingsSection();
    }

    if (BeginSettingsSection("##Troubleshooting", "Troubleshooting")) {
        DrawSetting("Foveated Rendering", "Draws the edges of your view in less detail, so the game runs faster.", [&]() {
            settings.foveatedRendering.AddComboToGUI(changed);
        });
#ifdef _DEBUG
        DrawSetting("Enable Debugger Tools", "Lowers performance and can cause crashes.", [&]() {
            settings.enableDebuggerTools.AddToGUI(changed);
        });
#endif
        DrawSetting("Reset All Settings", [&]() {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.18f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.22f, 0.22f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.45f, 0.12f, 0.12f, 1.0f));
            if (ImGui::Button("Reset Everything")) {
                for (ModSettingBase* option : settings.GetOptions()) {
                    option->Reset();
                }
                *changed = true;
            }
            ImGui::PopStyleColor(3);
        });
        EndSettingsSection();
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
    DrawPageHeader(ICON_KI_WRENCH, "Debug");

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

void RND_Renderer::ImGuiOverlay::DrawGuidePage(const char* icon, const char* title, size_t helpImageIndex) {
    DrawPageHeader(icon, title);

    if (helpImageIndex >= m_helpImages.size()) {
        return;
    }
    const auto& image = m_helpImages[helpImageIndex];

    if (ImGui::BeginChild("HelpImage", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_NoBackground)) {
        ImVec2 availableSize = ImGui::GetContentRegionAvail();
        float widthScale = availableSize.x / (float)image.m_image->GetWidth();
        float heightScale = availableSize.y / (float)image.m_image->GetHeight();
        float imageScale = std::min(std::min(widthScale, heightScale), 1.0f);
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
    DrawPageHeader(ICON_KI_HEART, "Credits");

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
        if (const std::optional<std::string> newerRelease = UpdateCheck::GetNewerRelease()) {
            ImGui::Spacing();
            ImGui::Text("BetterVR %s is out! Get it from github.com/Crementif/BotW-BetterVR/releases", newerRelease->c_str());
        }
    }
    ImGui::End();
}

void RND_Renderer::ImGuiOverlay::DrawHelpMenu() {
    auto& isMenuOpen = ImGuiMenus::g_isOpen;
    auto& settings = GetSettings();

    DrawTutorialPromptAndControllerVerification(isMenuOpen);

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
                case ImGuiMenus::COMBAT_PAGE: DrawCombatPage(&changed); break;
                case ImGuiMenus::CONTROLS_PAGE: DrawControlsPage(&changed); break;
                case ImGuiMenus::INTERFACE_PAGE: DrawInterfacePage(&changed); break;
                case ImGuiMenus::SYSTEM_PAGE: DrawSystemPage(&changed); break;
                case ImGuiMenus::GUIDE_CONTROLS_PAGE: DrawGuidePage(ICON_KI_QUESTION_CIRCLE, "The Controls", 0); break;
                case ImGuiMenus::GUIDE_EQUIPMENT_PAGE: DrawGuidePage(ICON_KI_QUESTION_CIRCLE, "Weapons & Runes", 1); break;
                case ImGuiMenus::GUIDE_SWING_PAGE: DrawGuidePage(ICON_KI_QUESTION_CIRCLE, "Swinging & Blocking", 2); break;
                case ImGuiMenus::GUIDE_WHISTLE_PAGE: DrawGuidePage(ICON_KI_QUESTION_CIRCLE, "Whistle & Magnesis", 3); break;
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
    }

    ImGui::End();
    ImGui::PopStyleVar(2);

    if (!shouldStayOpen) {
        isMenuOpen = false;
    }
}
