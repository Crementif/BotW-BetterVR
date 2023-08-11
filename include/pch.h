#pragma once

#include <atomic>
#include <string>
#include <type_traits>

#include <Windows.h>
#include <winrt/base.h>

// These macros mess with some of Vulkan's functions
#undef CreateEvent
#undef CreateSemaphore

#define VK_USE_PLATFORM_WIN32_KHR
#define VK_NO_PROTOTYPES
#include <vulkan/vk_layer.h>
#include <vulkan/vulkan_core.h>

#define VKROOTS_NEGOTIATION_INTERFACE VRLayer_NegotiateLoaderLayerInterfaceVersion

#include "vkroots.h"

#include <d3d12.h>
#include <D3Dcompiler.h>
#include <dxgi1_6.h>

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "D3DCompiler.lib")

#include <wrl/client.h>

using Microsoft::WRL::ComPtr;

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D12
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>


struct data_VRSettingsIn {
    int32_t modeSetting;
    int32_t alternatingEyeRenderingSetting;
    float eyeSeparationSetting;
    float headPositionSensitivitySetting;
    float heightPositionOffsetSetting;
    float hudScaleSetting;
    float menuScaleSetting;
};

struct data_VRCameraIn {
    float posX;
    float posY;
    float posZ;
    float targetX;
    float targetY;
    float targetZ;
};

struct data_VRCameraOut {
    uint32_t enabled;
    float posX;
    float posY;
    float posZ;
    float targetX;
    float targetY;
    float targetZ;
};

struct data_VRCameraRotationOut {
    uint32_t enabled;
    float rotX;
    float rotY;
    float rotZ;
};


struct data_VRProjectionMatrixOut {
    float aspectRatio;
    float fovY;
    float offsetX;
    float offsetY;
};

struct data_VRCameraOffsetOut {
    float aspectRatio;
    float fovY;
    float offsetX;
    float offsetY;
};

struct data_VRCameraAspectRatioOut {
    float aspectRatio;
    float fovY;
};

enum class ScreenId {
    GamePadBG_00 = 0x0,
    Title_00 = 0x1,
    MainScreen3D_00 = 0x2,
    Message3D_00 = 0x3,
    AppCamera_00 = 0x4,
    WolfLinkHeartGauge_00 = 0x5,
    EnergyMeterDLC_00 = 0x6,
    MainHorse_00 = 0x7,
    MiniGame_00 = 0x8,
    ReadyGo_00 = 0x9,
    KeyNum_00 = 0xA,
    MessageGet_00 = 0xB,
    DoCommand_00 = 0xC,
    SousaGuide_00 = 0xD,
    GameTitle_00 = 0xE,
    DemoName_00 = 0xF,
    DemoNameEnemy_00 = 0x10,
    MessageSp_00_NoTop = 0x11,
    ShopBG_00 = 0x12,
    ShopBtnList5_00 = 0x13,
    ShopBtnList20_00 = 0x14,
    ShopBtnList15_00 = 0x15,
    ShopInfo_00 = 0x16,
    ShopHorse_00 = 0x17,
    Rupee_00 = 0x18,
    KologNum_00 = 0x19,
    AkashNum_00 = 0x1A,
    MamoNum_00 = 0x1B,
    Time_00 = 0x1C,
    PauseMenuBG_00 = 0x1D,
    SeekPadMenuBG_00 = 0x1E,
    AppTool_00 = 0x1F,
    AppAlbum_00 = 0x20,
    AppPictureBook_00 = 0x21,
    AppMapDungeon_00 = 0x22,
    MainScreenMS_00 = 0x23,
    MainScreenHeartIchigekiDLC_00 = 0x24,
    MainScreen_00 = 0x25,
    MainDungeon_00 = 0x26,
    ChallengeWin_00 = 0x27,
    PickUp_00 = 0x28,
    MessageTipsRunTime_00 = 0x29,
    AppMap_00 = 0x2A,
    AppSystemWindowNoBtn_00 = 0x2B,
    AppHome_00 = 0x2C,
    MainShortCut_00 = 0x2D,
    PauseMenu_00 = 0x2E,
    PauseMenuInfo_00 = 0x2F,
    GameOver_00 = 0x30,
    HardMode_00 = 0x31,
    SaveTransferWindow_00 = 0x32,
    MessageTipsPauseMenu_00 = 0x33,
    MessageTips_00 = 0x34,
    OptionWindow_00 = 0x35,
    AmiiboWindow_00 = 0x36,
    SystemWindowNoBtn_00 = 0x37,
    ControllerWindow_00 = 0x38,
    SystemWindow_01 = 0x39,
    SystemWindow_00 = 0x3A,
    PauseMenuRecipe_00 = 0x3B,
    PauseMenuMantan_00 = 0x3C,
    PauseMenuEiketsu_00 = 0x3D,
    AppSystemWindow_00 = 0x3E,
    DLCWindow_00 = 0x3F,
    HardModeTextDLC_00 = 0x40,
    TestButton = 0x41,
    TestPocketUIDRC = 0x42,
    TestPocketUITV = 0x43,
    BoxCursorTV = 0x44,
    FadeDemo_00 = 0x45,
    StaffRoll_00 = 0x46,
    StaffRollDLC_00 = 0x47,
    End_00 = 0x48,
    DLCSinJuAkashiNum_00 = 0x49,
    MessageDialog = 0x4A,
    DemoMessage = 0x4B,
    MessageSp_00 = 0x4C,
    Thanks_00 = 0x4D,
    Fade = 0x4E,
    KeyBoradTextArea_00 = 0x4F,
    LastComplete_00 = 0x50,
    OPtext_00 = 0x51,
    LoadingWeapon_00 = 0x52,
    MainHardMode_00 = 0x53,
    LoadSaveIcon_00 = 0x54,
    FadeStatus_00 = 0x55,
    Skip_00 = 0x56,
    ChangeController_00 = 0x57,
    ChangeControllerDRC_00 = 0x58,
    DemoStart_00 = 0x59,
    BootUp_00 = 0x5A,
    BootUp_00_2 = 0x5B,
    ChangeControllerNN_00 = 0x5C,
    AppMenuBtn_00 = 0x5D,
    HomeMenuCapture_00 = 0x5E,
    HomeMenuCaptureDRC_00 = 0x5F,
    HomeNixSign_00 = 0x60,
    ErrorViewer_00 = 0x61,
    ErrorViewerDRC_00 = 0x62,
};

#include "cemu.h"
#include "utils/logger.h"