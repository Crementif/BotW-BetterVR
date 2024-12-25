#pragma once

#include <atomic>
#include <string>
#include <variant>
#include <functional>
#include <type_traits>
#include <ranges>
#include <set>
#include <unordered_set>

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

#include <imgui.h>
#include <imgui_impl_vulkan.h>


inline std::string& toLower(std::string str) {
    std::ranges::transform(str, str.begin(), [](unsigned char c) { return std::tolower(c); });
    return str;
}

template<class T, template<class...> class U>
inline constexpr bool is_instance_of_v = std::false_type{};

template<template<class...> class U, class... Vs>
inline constexpr bool is_instance_of_v<U<Vs...>,U> = std::true_type{};

template <typename T1, typename T2>
constexpr bool HAS_FLAG(T1 flags, T2 test_flag) { return (flags & (T1)test_flag) == (T1)test_flag; }

template <typename T>
inline T swapEndianness(T val) {
    if constexpr (std::is_floating_point<T>::value) {
        union {
            T f;
            uint32_t i;
        } bits;

        bits.f = val;
        bits.i = (bits.i & 0x000000FF) << 24 | (bits.i & 0x0000FF00) << 8  | (bits.i & 0x00FF0000) >> 8  | (bits.i & 0xFF000000) >> 24;

        return bits.f;
    }
    else if constexpr (std::is_integral<T>::value) {
        if constexpr (sizeof(T) == 1) {
            return val;
        }
        else if constexpr (sizeof(T) == 2) {
            return static_cast<T>((val << 8) | (val >> 8));
        }
        else if constexpr (sizeof(T) == 4) {
            return ((val & 0x000000FF) << 24) | ((val & 0x0000FF00) <<  8) | ((val & 0x00FF0000) >>  8) | ((val & 0xFF000000) >> 24);
        }
        else {
            union U {
                T val;
                std::array<std::uint8_t, sizeof(T)> raw;
            } src, dst;

            src.val = val;
            std::reverse_copy(src.raw.begin(), src.raw.end(), dst.raw.begin());
            return dst.val;
        }
    }
    else {
        union U {
            T val;
            std::array<std::uint8_t, sizeof(T)> raw;
        } src, dst;

        src.val = val;
        std::reverse_copy(src.raw.begin(), src.raw.end(), dst.raw.begin());
        return dst.val;
    }
}

struct BETypeCompatible {
};

template<typename T>
struct BEType : BETypeCompatible {
    T val;

    BEType() = default;

    BEType(T x) : val(swapEndianness(x)) {}

    explicit operator T() {
        return swapEndianness(val);
    }

    BEType<T>& operator =(T x) {
        val = swapEndianness(x);
        return *this;
    }

    BEType<T>& operator =(const BEType<T>& other) {
        val = other.val;
        return *this;
    }

    T getLE() const {
        return swapEndianness(val);
    }

    T getBE() const {
        return val;
    }


    bool operator ==(const BEType<T>& other) const { return val == other.val; }
    bool operator ==(const T& other) const { return swapEndianness(val) == other; }
    friend bool operator ==(const T& lhs, const BEType<T>& rhs) { return lhs == swapEndianness(rhs.val);}

    bool operator !=(const BEType<T>& other) const { return val != other.val; }
    bool operator !=(const T& other) const { return swapEndianness(val) != other.val; }
    friend bool operator !=(const T& lhs, const BEType<T>& rhs) { return lhs != swapEndianness(rhs.val); }

    bool operator <(const BEType<T>& other) const { return swapEndianness(val) < swapEndianness(other.val); }
    bool operator <(const T& other) const { return swapEndianness(val) < other; }
    friend bool operator <(const T& lhs, const BEType<T>& rhs) { return lhs < swapEndianness(rhs.val); }

    bool operator >(const BEType<T>& other) const { return swapEndianness(val) > swapEndianness(other.val); }
    bool operator >(const T& other) const { return swapEndianness(val) > other; }
    friend bool operator >(const T& lhs, const BEType<T>& rhs) { return lhs > swapEndianness(rhs.val); }

    bool operator <=(const BEType<T>& other) const { return swapEndianness(val) <= swapEndianness(other.val); }
    bool operator <=(const T& other) const { return swapEndianness(val) <= other; }
    friend bool operator <=(const T& lhs, const BEType<T>& rhs) { return lhs <= swapEndianness(rhs.val); }

    bool operator >=(const BEType<T>& other) const { return swapEndianness(val) >= swapEndianness(other.val); }
    bool operator >=(const T& other) const { return swapEndianness(val) >= other; }
    friend bool operator >=(const T& lhs, const BEType<T>& rhs) { return lhs >= swapEndianness(rhs.val); }
};


template<typename T>
inline constexpr bool is_BEType_v = std::is_base_of_v<BETypeCompatible, T>;

struct BEMatrix34 : BETypeCompatible {
    BEType<float> x_x;
    BEType<float> y_x;
    BEType<float> z_x;
    BEType<float> pos_x;
    BEType<float> x_y;
    BEType<float> y_y;
    BEType<float> z_y;
    BEType<float> pos_y;
    BEType<float> x_z;
    BEType<float> y_z;
    BEType<float> z_z;
    BEType<float> pos_z;

    float DistanceSq(BEMatrix34 other) const {
        return (pos_x.getLE() - other.pos_x.getLE()) * (pos_x.getLE() - other.pos_x.getLE()) + (pos_y.getLE() - other.pos_y.getLE()) * (pos_y.getLE() - other.pos_y.getLE()) + (pos_z.getLE() - other.pos_z.getLE()) * (pos_z.getLE() - other.pos_z.getLE());
    }

    std::array<std::array<float, 4>, 3> getLE() const {
        std::array<float, 4> row0 = { x_x.getLE(), y_x.getLE(), z_x.getLE(), pos_x.getLE() };
        std::array<float, 4> row1 = { x_y.getLE(), y_y.getLE(), z_y.getLE(), pos_y.getLE() };
        std::array<float, 4> row2 = { x_z.getLE(), y_z.getLE(), z_z.getLE(), pos_z.getLE() };
        return { row0, row1, row2 };
    }

    void setLE(std::array<std::array<float, 4>, 3> mtx) {
        x_x = mtx[0][0];
        y_x = mtx[0][1];
        z_x = mtx[0][2];
        pos_x = mtx[0][3];
        x_y = mtx[1][0];
        y_y = mtx[1][1];
        z_y = mtx[1][2];
        pos_y = mtx[1][3];
        x_z = mtx[2][0];
        y_z = mtx[2][1];
        z_z = mtx[2][2];
        pos_z = mtx[2][3];
    }
};

struct BEVec2 : BETypeCompatible {
    BEType<float> x;
    BEType<float> y;

    BEVec2() = default;
    BEVec2(float x, float y): x(x), y(y) {}
    BEVec2(BEType<float> x, BEType<float> y): x(x), y(y) {}
};

struct BEVec3 : BETypeCompatible {
    BEType<float> x;
    BEType<float> y;
    BEType<float> z;

    BEVec3() = default;
    BEVec3(float x, float y, float z): x(x), y(y), z(z) {}

    float DistanceSq(BEVec3 other) const {
        return (x.getLE() - other.x.getLE()) * (x.getLE() - other.x.getLE()) + (y.getLE() - other.y.getLE()) * (y.getLE() - other.y.getLE()) + (z.getLE() - other.z.getLE()) * (z.getLE() - other.z.getLE());
    }
};

struct data_VRSettingsIn {
    BEType<int32_t> cameraModeSetting;
    BEType<int32_t> guiFollowSetting;
    BEType<int32_t> alternatingEyeRenderingSetting;
    BEType<int32_t> cropFlatTo16x9Setting;
};

struct data_VRCameraIn {
    BEType<float> posX;
    BEType<float> posY;
    BEType<float> posZ;
    BEType<float> targetX;
    BEType<float> targetY;
    BEType<float> targetZ;
};

struct data_VRCameraOut {
    BEType<uint32_t> enabled;
    BEType<float> posX;
    BEType<float> posY;
    BEType<float> posZ;
    BEType<float> targetX;
    BEType<float> targetY;
    BEType<float> targetZ;
};

struct data_VRCameraRotationOut {
    BEType<uint32_t> enabled;
    BEType<float> rotX;
    BEType<float> rotY;
    BEType<float> rotZ;
};


struct data_VRProjectionMatrixOut {
    BEType<float> aspectRatio;
    BEType<float> fovY;
    BEType<float> offsetX;
    BEType<float> offsetY;
};

struct data_VRCameraOffsetOut {
    BEType<float> aspectRatio;
    BEType<float> fovY;
    BEType<float> offsetX;
    BEType<float> offsetY;
};

struct data_VRCameraAspectRatioOut {
    BEType<float> aspectRatio;
    BEType<float> fovY;
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