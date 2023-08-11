#include "cemu_hooks.h"

std::atomic<data_VRSettingsIn> g_settings = {};
uint64_t CemuHooks::s_memoryBaseAddress = 0;
std::atomic_uint32_t CemuHooks::s_framesSinceLastCameraUpdate = 0;

void CemuHooks::hook_UpdateSettings(PPCInterpreter_t* hCPU) {
    // Log::print("Updated settings!");
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t ppc_settingsOffset = hCPU->gpr[5];
    data_VRSettingsIn settings = {};

    readMemory(ppc_settingsOffset, &settings);
    swapEndianness(settings.modeSetting);
    swapEndianness(settings.alternatingEyeRenderingSetting);
    swapEndianness(settings.eyeSeparationSetting);
    swapEndianness(settings.headPositionSensitivitySetting);
    swapEndianness(settings.heightPositionOffsetSetting);
    swapEndianness(settings.hudScaleSetting);
    swapEndianness(settings.menuScaleSetting);

    g_settings.store(settings);
    s_framesSinceLastCameraUpdate++;
}

data_VRSettingsIn CemuHooks::GetSettings() {
    return g_settings.load();
}

// currently unused
void CemuHooks::hook_CreateNewScreen(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const char* screenName = (const char*)(s_memoryBaseAddress + hCPU->gpr[7]);
    ScreenId screenId = (ScreenId)hCPU->gpr[5];
    // Log::print("Switching to new screen \"{}\" with ID {:08X}...", screenName, std::to_underlying(screenId));
}
