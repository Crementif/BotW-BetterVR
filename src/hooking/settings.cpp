#include "cemu_hooks.h"
#include "rendering/openxr.h"

std::atomic<data_VRSettingsIn> g_settings = {};
uint64_t CemuHooks::s_memoryBaseAddress = 0;


void CemuHooks::hook_UpdateSettings(PPCInterpreter_t* hCPU) {
    //Log::print("Updated settings!");
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
}

data_VRSettingsIn CemuHooks::GetSettings() {
    return g_settings.load();
}