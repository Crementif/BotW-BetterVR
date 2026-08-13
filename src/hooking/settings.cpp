#include "pch.h"

#include "cemu_hooks.h"
#include "imgui_internal.h"
#include "instance.h"
#include "hooking/entity_debugger.h"
#include "utils/mod_settings.h"

ModSettings g_settings = {};

bool ModSettings::IsDebuggingToolsEnabled() const {
    return VRManager::instance().XR->GetRenderer() != nullptr && VRManager::instance().Hooks->m_entityDebugger && GetSettings().enableDebuggerTools;
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
            //Log::print<INFO>("Deserialized \"{}\" to \"{}\" from line \"{}\"", option->name, option->Serialize(), line);
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

HWND CemuHooks::m_cemuTopWindow = NULL;
HWND CemuHooks::m_cemuRenderWindow = NULL;
uint64_t CemuHooks::s_memoryBaseAddress = 0;
std::atomic_uint32_t CemuHooks::s_framesSinceLastCameraUpdate = 0;
std::atomic_uint32_t CemuHooks::s_recordingOutputMode = 0;


std::unordered_set<ScreenId> prevEnabledScreens = {};

void CemuHooks::InitWindowHandles() {
    // find HWND that starts with Cemu in its title
    struct EnumWindowsData {
        DWORD cemuPid;
        HWND outHwnd;
    } enumData = { .cemuPid = GetCurrentProcessId(), .outHwnd = NULL };

    EnumWindows([](HWND iteratedHwnd, LPARAM data) -> BOOL {
        EnumWindowsData* enumData = (EnumWindowsData*)data;
        DWORD currPid;
        GetWindowThreadProcessId(iteratedHwnd, &currPid);
        if (currPid == enumData->cemuPid) {
            constexpr size_t bufSize = 256;
            wchar_t buf[bufSize];
            GetWindowTextW(iteratedHwnd, buf, bufSize);
            if (wcsstr(buf, L"Cemu") != nullptr) {
                enumData->outHwnd = iteratedHwnd;
                return FALSE;
            }
        }
        return TRUE;
    },
    (LPARAM)&enumData);
    m_cemuTopWindow = enumData.outHwnd;

    // find the most nested child window since that's the rendering window
    HWND iteratedHwnd = m_cemuTopWindow;
    while (true) {
        HWND nextIteratedHwnd = FindWindowExW(iteratedHwnd, NULL, NULL, NULL);
        if (nextIteratedHwnd == NULL) {
            break;
        }
        iteratedHwnd = nextIteratedHwnd;
    }
    m_cemuRenderWindow = iteratedHwnd;
}

void CemuHooks::hook_UpdateSettings(PPCInterpreter_t* hCPU) {
    // Log::print("Updated settings!");
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (s_damageStateNameAddress == 0) {
        s_damageStateNameAddress = hCPU->gpr[8];
        Log::print<PPC>("Registered Damage state string address: {:#X}", s_damageStateNameAddress);
    }

    uint32_t ppc_tableOfCutsceneEventSettings = hCPU->gpr[6];
    uint32_t recordingOutputMode = 0;
    readMemoryBE(0x10416BF4, &recordingOutputMode);
    s_recordingOutputMode.store(recordingOutputMode, std::memory_order_relaxed);

    // push the host-controlled performance flags into the PPC-visible flag block
    // (same repurposed bytes as DISABLE_PPC_LOGGING_GET/recordingOutputMode above)
    setMemory<uint32_t>(0x10416BF8, GetEffectiveRenderSkipMask());
    setMemory<uint32_t>(0x10416BFC, ShouldUseSynthesizedRightEye() ? 1u : 0u);
    
    if (GetSettings().IsDebuggingToolsEnabled()) {
        VRManager::instance().Hooks->m_entityDebugger->UpdateEntityMemory();
    }

    UpdateFloatParamOverrides();

    ++s_framesSinceLastCameraUpdate;

#ifdef _DEBUG
    constexpr uint32_t maxScreenIdx = std::to_underlying(ScreenId::ScreenId_END);
    std::unordered_set<ScreenId> currentEnabledScreens;
    for (uint32_t i = 0; i < maxScreenIdx; i++) {
        ScreenId id = (ScreenId)i;
        bool hasScreen = IsScreenOpen(id);

        if (hasScreen) {            

            if (!prevEnabledScreens.contains(id)) {
                if (currentEnabledScreens.empty()) {
                    Log::print<INFO>("---------");
                }
                Log::print<INFO>("Screen {} is ON", ScreenIdToString((ScreenId)i));
            }
            currentEnabledScreens.emplace(id);
        }
        else if (prevEnabledScreens.contains(id)) {
            Log::print<INFO>("Screen {} is OFF", ScreenIdToString((ScreenId)i));
        }
    }
    prevEnabledScreens = currentEnabledScreens;
#endif

    initCutsceneDefaultSettings(ppc_tableOfCutsceneEventSettings);
}

void CemuHooks::hook_OSReportToConsole(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t strPtr = hCPU->gpr[3];
    if (strPtr == 0) {
        return;
    }
    char* str = (char*)(s_memoryBaseAddress + strPtr);
    if (str == nullptr) {
        return;
    }
    if (str[0] != '\0') {
        Log::print<PPC>(str);
    }
}

constexpr uint32_t playerVtable = 0x101E5FFC;
void CemuHooks::hook_RouteActorJob(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const uint32_t actorPtr = hCPU->gpr[3];
    const uint32_t jobNamePtr = hCPU->gpr[4];
    const uint32_t side = hCPU->gpr[5]; // 0 = left, 1 = right
    const uint32_t monoFrame = hCPU->gpr[6]; // nonzero while the synthesized-right-eye single pass is active

    // a mono frame has exactly one pass, so every job runs on it (vanilla game behavior)
    if (monoFrame != 0) {
        hCPU->gpr[3] = 0;
        return;
    }

    // This runs for every actor x every job x both eyes each frame, so the routing
    // decision has to stay allocation-free. The job name pointer is one of the seven
    // string constants baked into the graphic pack's codecave, so the only string
    // compare that matters ("job0_1" or not) is cached per pointer, and the player
    // check is the same vtable compare the assembly-side stubs use.
    static std::array<std::pair<uint32_t, bool>, 16> s_jobNameIsJob0_1 = {};
    static size_t s_jobNameCount = 0;

    bool isJob0_1 = false;
    bool cached = false;
    for (size_t i = 0; i < s_jobNameCount; i++) {
        if (s_jobNameIsJob0_1[i].first == jobNamePtr) {
            isJob0_1 = s_jobNameIsJob0_1[i].second;
            cached = true;
            break;
        }
    }
    if (!cached) {
        isJob0_1 = std::strcmp((const char*)(s_memoryBaseAddress + jobNamePtr), "job0_1") == 0;
        if (s_jobNameCount < s_jobNameIsJob0_1.size()) {
            s_jobNameIsJob0_1[s_jobNameCount++] = { jobNamePtr, isJob0_1 };
        }
    }

    const bool isPlayer = getMemory<uint32_t>(actorPtr + offsetof(BaseProc, vtable)) == playerVtable;

    // exit r3:
    // 0 = perform job
    // 1 = skip job
    // 2 = altered job (player climbing portion of job0_1)
    uint32_t result = 0;
    if (isJob0_1) {
        if (isPlayer) {
            // only run the climbing portion of this actor job on the left eye's side
            // so that later jobs on the left side can use the state set by this portion of code
            if (side == 0) {
                result = 2;
            }
        }
        else if (side == 0) {
            result = 1;
        }
    }
    else if (side == 1) {
        // every other job runs on the left eye only
        result = 1;
    }

    hCPU->gpr[3] = result;
}

// todo: this only runs when it's shown for the first time!
void CemuHooks::hook_CreateNewScreen(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const char* screenName = (const char*)(s_memoryBaseAddress + hCPU->gpr[7]);
    ScreenId screenId = (ScreenId)hCPU->gpr[5];
    Log::print<CONTROLS>("Creating new screen \"{}\" with ID {:08X}...", screenName, std::to_underlying(screenId));

    // todo: When a pickup screen is shown, we should track if the user does a short grip press, and if it was the left and right hand.
    if (screenId == ScreenId::PickUp_00) {
        Log::print<CONTROLS>("PickUp screen detected, waiting for grip button press to bind item to hand...");
    }
}


enum class GX2_BLENDFACTOR {
    ZERO = 0x00,
    ONE = 0x01,
    SRC_COLOR = 0x02,
    ONE_MINUS_SRC_COLOR = 0x03,
    SRC_ALPHA = 0x04,
    ONE_MINUS_SRC_ALPHA = 0x05,
    DST_ALPHA = 0x06,
    ONE_MINUS_DST_ALPHA = 0x07,
    DST_COLOR = 0x08,
    ONE_MINUS_DST_COLOR = 0x09,
    SRC_ALPHA_SATURATE = 0x0A,
    BOTH_SRC_ALPHA = 0x0B,
    BOTH_INV_SRC_ALPHA = 0x0C,
    CONST_COLOR = 0x0D,
    ONE_MINUS_CONST_COLOR = 0x0E,
    SRC1_COLOR = 0x0F,
    INV_SRC1_COLOR = 0x10,
    SRC1_ALPHA = 0x11,
    INV_SRC1_ALPHA = 0x12,
    CONST_ALPHA = 0x13,
    ONE_MINUS_CONST_ALPHA = 0x14
};

enum class GX2_COMBINEFUNC {
    DST_PLUS_SRC = 0,
    SRC_MINUS_DST = 1,
    MIN_DST_SRC = 2,
    MAX_DST_SRC = 3,
    DST_MINUS_SRC = 4
};

void CemuHooks::hook_FixUIBlending(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t renderTargetIndex = hCPU->gpr[3];
    GX2_BLENDFACTOR colorSrcFactor = (GX2_BLENDFACTOR)hCPU->gpr[4];
    GX2_BLENDFACTOR colorDstFactor = (GX2_BLENDFACTOR)hCPU->gpr[5];
    GX2_COMBINEFUNC colorCombineFunc = (GX2_COMBINEFUNC)hCPU->gpr[6];
    uint32_t separateAlphaBlend = hCPU->gpr[7];
    GX2_BLENDFACTOR alphaSrcFactor = (GX2_BLENDFACTOR)hCPU->gpr[8];
    GX2_BLENDFACTOR alphaDstFactor = (GX2_BLENDFACTOR)hCPU->gpr[9];
    GX2_COMBINEFUNC alphaCombineFunc = (GX2_COMBINEFUNC)hCPU->gpr[10];

    {
        bool matchesColorSettings = colorSrcFactor == GX2_BLENDFACTOR::DST_COLOR && colorDstFactor == GX2_BLENDFACTOR::SRC_ALPHA && colorCombineFunc == GX2_COMBINEFUNC::DST_PLUS_SRC;
        bool matchesAlpha = alphaSrcFactor == GX2_BLENDFACTOR::SRC_ALPHA && alphaDstFactor == GX2_BLENDFACTOR::ONE_MINUS_SRC_ALPHA && alphaCombineFunc == GX2_COMBINEFUNC::DST_PLUS_SRC;

        if (matchesColorSettings && matchesAlpha) {
            hCPU->gpr[7] = 1;
            hCPU->gpr[8] = std::to_underlying(GX2_BLENDFACTOR::ZERO);
            hCPU->gpr[9] = std::to_underlying(GX2_BLENDFACTOR::DST_ALPHA);

            //Log::print<VERBOSE>("FixUIBlending called with renderTargetIndex: {}, colorSrcFactor: {}, colorDstFactor: {}, colorCombineFunc: {}, separateAlphaBlend: {}, alphaSrcFactor: {}, alphaDstFactor: {}, alphaCombineFunc: {}", renderTargetIndex, std::to_underlying(colorSrcFactor), std::to_underlying(colorDstFactor), std::to_underlying(colorCombineFunc), separateAlphaBlend, std::to_underlying(alphaSrcFactor), std::to_underlying(alphaDstFactor), std::to_underlying(alphaCombineFunc));
        }
    }
}
