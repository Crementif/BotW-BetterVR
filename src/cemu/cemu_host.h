#pragma once
#include "cemu/guest_memory.h"

using HookFunction = void (*)(PPCInterpreter_t* hCPU);

class CemuHost {
public:
    CemuHost();

    static uint64_t GetCurrentTitleId();
    // false issues the right view through the replay again
    static void SetMultiviewActive(bool active);
    static void RegisterHook(const char* name, HookFunction function);
    // 0,0 is the top left, 1,1 the bottom right
    static void SetFoveationCentres(const float left[2], const float right[2]);
    static void SetFoveatedShadingActive(bool active);
    // makes the foveation easy to see
    static void SetFoveationExaggerated(bool exaggerated);
    // 0 shows the whole image
    static void CropWindowToAspectRatio(float aspectRatio);
    static std::filesystem::path GetUserDataPath();
    // only call this while handling a host event
    static void AttachToNextSubmit(VkSemaphore wait, uint64_t waitValue, VkSemaphore signal, uint64_t signalValue);

private:
    static uint8_t* GetGuestMemoryBase();
};
