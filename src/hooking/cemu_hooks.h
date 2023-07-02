#pragma once
#include "rendering/openxr.h"

class CemuHooks {
public:
    CemuHooks() {
        HMODULE cemuModuleHandle = GetModuleHandleA(NULL);
        checkAssert(cemuModuleHandle != NULL, "Failed to get handle of Cemu process which is required for interfacing with Cemu!");

        gameMeta_getTitleId = (gameMeta_getTitleIdPtr_t)GetProcAddress(cemuModuleHandle, "gameMeta_getTitleId");
        memory_getBase = (memory_getBasePtr_t)GetProcAddress(cemuModuleHandle, "memory_getBase");
        osLib_registerHLEFunction = (osLib_registerHLEFunctionPtr_t)GetProcAddress(cemuModuleHandle, "osLib_registerHLEFunction");
        checkAssert(gameMeta_getTitleId != NULL && memory_getBase != NULL && osLib_registerHLEFunction != NULL, "Failed to get function pointers of Cemu functions! Is this hook being used on Cemu?");

        bool isSupportedTitleId = gameMeta_getTitleId() == 0x00050000101C9300 || gameMeta_getTitleId() == 0x00050000101C9400 || gameMeta_getTitleId() == 0x00050000101C9500;
        checkAssert(isSupportedTitleId, std::format("Expected title IDs for Breath of the Wild (00050000-101C9300, 00050000-101C9400 or 00050000-101C9500) but received {:16x}!", gameMeta_getTitleId()).c_str());

        s_memoryBaseAddress = (uint64_t)memory_getBase();
        checkAssert(s_memoryBaseAddress != 0, "Failed to get memory base address of Cemu process!");

        osLib_registerHLEFunction("coreinit", "hook_UpdateSettings", &hook_UpdateSettings);
        osLib_registerHLEFunction("coreinit", "hook_UpdateCamera", &hook_UpdateCamera);
        osLib_registerHLEFunction("coreinit", "hook_UpdateProjectionMatrix", &hook_UpdateProjectionMatrix);
        //osLib_registerHLEFunction("coreinit", "hook_UpdateInterface", &hook_UpdateInterface);
    };
    ~CemuHooks() {
    };

    static data_VRSettingsIn GetSettings();
private:
    osLib_registerHLEFunctionPtr_t osLib_registerHLEFunction;
    memory_getBasePtr_t memory_getBase;
    gameMeta_getTitleIdPtr_t gameMeta_getTitleId;

    static uint64_t s_memoryBaseAddress;

    static void hook_UpdateSettings(PPCInterpreter_t* hCPU);
    static void hook_UpdateProjectionMatrix(PPCInterpreter_t* hCPU);
    static void hook_UpdateCamera(PPCInterpreter_t* hCPU);

#pragma region MEMORY_POINTERS
    template <typename T>
    static void swapEndianness(T& val) {
        union U {
            T val;
            std::array<std::uint8_t, sizeof(T)> raw;
        } src, dst;

        src.val = val;
        std::reverse_copy(src.raw.begin(), src.raw.end(), dst.raw.begin());
        val = dst.val;
    }

    template <typename T>
    static void writeMemoryBE(uint64_t offset, T* valuePtr) {
        swapEndianness(*valuePtr);
        memcpy((void*)(s_memoryBaseAddress + offset), (void*)valuePtr, sizeof(T));
    }

    template <typename T>
    static void writeMemory(uint64_t offset, T* valuePtr) {
        memcpy((void*)(s_memoryBaseAddress + offset), (void*)valuePtr, sizeof(T));
    }

    template <typename T>
    static void readMemoryBE(uint64_t offset, T* resultPtr) {
        uint64_t memoryAddress = s_memoryBaseAddress + offset;
        memcpy(resultPtr, (void*)memoryAddress, sizeof(T));
        swapEndianness(*resultPtr);
    }

    template <typename T>
    static void readMemory(uint64_t offset, T* resultPtr) {
        uint64_t memoryAddress = s_memoryBaseAddress + offset;
        memcpy(resultPtr, (void*)memoryAddress, sizeof(T));
    }
#pragma endregion
};
