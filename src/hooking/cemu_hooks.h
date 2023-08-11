#pragma once
#include "rendering/openxr.h"

class CemuHooks {
public:
    CemuHooks() {
        m_cemuHandle = GetModuleHandleA(NULL);
        checkAssert(m_cemuHandle != NULL, "Failed to get handle of Cemu process which is required for interfacing with Cemu!");

        gameMeta_getTitleId = (gameMeta_getTitleIdPtr_t)GetProcAddress(m_cemuHandle, "gameMeta_getTitleId");
        memory_getBase = (memory_getBasePtr_t)GetProcAddress(m_cemuHandle, "memory_getBase");
        osLib_registerHLEFunction = (osLib_registerHLEFunctionPtr_t)GetProcAddress(m_cemuHandle, "osLib_registerHLEFunction");
        checkAssert(gameMeta_getTitleId != NULL && memory_getBase != NULL && osLib_registerHLEFunction != NULL, "Failed to get function pointers of Cemu functions! Is this hook being used on Cemu?");

        bool isSupportedTitleId = gameMeta_getTitleId() == 0x00050000101C9300 || gameMeta_getTitleId() == 0x00050000101C9400 || gameMeta_getTitleId() == 0x00050000101C9500;
        checkAssert(isSupportedTitleId, std::format("Expected title IDs for Breath of the Wild (00050000-101C9300, 00050000-101C9400 or 00050000-101C9500) but received {:16x}!", gameMeta_getTitleId()).c_str());

        s_memoryBaseAddress = (uint64_t)memory_getBase();
        checkAssert(s_memoryBaseAddress != 0, "Failed to get memory base address of Cemu process!");

        osLib_registerHLEFunction("coreinit", "hook_UpdateSettings", &hook_UpdateSettings);
        osLib_registerHLEFunction("coreinit", "hook_UpdateCameraPositionAndTarget", &hook_UpdateCameraPositionAndTarget);
        osLib_registerHLEFunction("coreinit", "hook_UpdateCameraRotation", &hook_UpdateCameraRotation);
        osLib_registerHLEFunction("coreinit", "hook_UpdateCameraOffset", &hook_UpdateCameraOffset);
        osLib_registerHLEFunction("coreinit", "hook_CalculateCameraAspectRatio", &hook_CalculateCameraAspectRatio);
        osLib_registerHLEFunction("coreinit", "hook_CreateNewScreen", &hook_CreateNewScreen);
    };
    ~CemuHooks() {
        FreeLibrary(m_cemuHandle);
    };

    static data_VRSettingsIn GetSettings();
    static uint32_t GetFramesSinceLastCameraUpdate() { return s_framesSinceLastCameraUpdate.load(); }

private:
    HMODULE m_cemuHandle;

    osLib_registerHLEFunctionPtr_t osLib_registerHLEFunction;
    memory_getBasePtr_t memory_getBase;
    gameMeta_getTitleIdPtr_t gameMeta_getTitleId;

    static uint64_t s_memoryBaseAddress;
    static std::atomic_uint32_t s_framesSinceLastCameraUpdate;

    static void hook_UpdateSettings(PPCInterpreter_t* hCPU);
    static void hook_UpdateCameraPositionAndTarget(PPCInterpreter_t* hCPU);
    static void hook_UpdateCameraRotation(PPCInterpreter_t* hCPU);
    static void hook_UpdateCameraOffset(PPCInterpreter_t* hCPU);
    static void hook_CalculateCameraAspectRatio(PPCInterpreter_t* hCPU);
    static void hook_CreateNewScreen(PPCInterpreter_t* hCPU);

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
