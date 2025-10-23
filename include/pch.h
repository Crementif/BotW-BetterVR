#pragma once

#include <atomic>
#include <string>
#include <variant>
#include <functional>
#include <type_traits>
#include <ranges>
#include <set>
#include <unordered_set>
#include <queue>

#include <Windows.h>
#include <winrt/base.h>

// These macros mess with some of Vulkan's functions
#undef CreateEvent
#undef CreateSemaphore

#define VK_USE_PLATFORM_WIN32_KHR
#define VK_NO_PROTOTYPES
#include <vulkan/vk_layer.h>
#include <vulkan/vulkan_core.h>

// vkroots vulkan layer framework includes
#define VKROOTS_NEGOTIATION_INTERFACE VRLayer_NegotiateLoaderLayerInterfaceVersion
#include "vkroots.h"

// D3D12 includes
#include <d3d12.h>
#include <D3Dcompiler.h>
#include <dxgi1_6.h>

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "D3DCompiler.lib")
#pragma comment(lib, "dxguid.lib")

#include <wrl/client.h>

using Microsoft::WRL::ComPtr;

// OpenXR includes
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D12
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

// ImGui includes
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_impl_vulkan.h>
#include <implot3d.h>
#include <implot.h>

// glm includes
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

inline glm::fvec2 ToGLM(const XrVector2f& vec) {
    return glm::make_vec2(&vec.x);
}

inline glm::fvec3 ToGLM(const XrVector3f& vec) {
    return glm::make_vec3(&vec.x);
}

inline glm::fquat ToGLM(const XrQuaternionf& quat) {
    return glm::fquat(quat.w, quat.x, quat.y, quat.z);
}


inline XrVector2f ToXR(const glm::fvec2& vec) {
    return { vec.x, vec.y };
}

inline XrVector3f ToXR(const glm::fvec3& vec) {
    return { vec.x, vec.y, vec.z };
}

inline XrQuaternionf ToXR(const glm::fquat& quat) {
    return { quat.x, quat.y, quat.z, quat.w };
}

inline std::string toLower(std::string str) {
    std::ranges::transform(str, str.begin(), [](unsigned char c) { return std::tolower(c); });
    return str;
}

inline uint32_t stringToHash(const char* str) {
    uint32_t hash = 0;
    while (*str) {
        hash = (hash << 7) + *str++;
    }
    return hash;
}

inline std::string wcharToUtf8(const wchar_t* wstr) {
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    std::string str(size_needed, 0);
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, &str[0], size_needed, nullptr, nullptr);
    return str;
}

#define PADDED_BYTES(from, up) uint8_t byte_##from##[ ## (up-from+0x04) ## ]

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
    BEVec3(BEType<float> x, BEType<float> y, BEType<float> z): x(x), y(y), z(z) {}
    BEVec3(float x, float y, float z): x(x), y(y), z(z) {}

    float DistanceSq(BEVec3 other) const {
        return (x.getLE() - other.x.getLE()) * (x.getLE() - other.x.getLE()) + (y.getLE() - other.y.getLE()) * (y.getLE() - other.y.getLE()) + (z.getLE() - other.z.getLE()) * (z.getLE() - other.z.getLE());
    }

    glm::fvec3 getLE() const {
        return { x.getLE(), y.getLE(), z.getLE() };
    }

    bool operator==(const BEVec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    void operator=(const glm::fvec3& other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
};

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

    BEMatrix34() = default;

    float DistanceSq(const BEMatrix34& other) const {
        return (pos_x.getLE() - other.pos_x.getLE()) * (pos_x.getLE() - other.pos_x.getLE()) + (pos_y.getLE() - other.pos_y.getLE()) * (pos_y.getLE() - other.pos_y.getLE()) + (pos_z.getLE() - other.pos_z.getLE()) * (pos_z.getLE() - other.pos_z.getLE());
    }

    std::array<std::array<float, 4>, 3> getLE() const {
        std::array row0 = { x_x.getLE(), y_x.getLE(), z_x.getLE(), pos_x.getLE() };
        std::array row1 = { x_y.getLE(), y_y.getLE(), z_y.getLE(), pos_y.getLE() };
        std::array row2 = { x_z.getLE(), y_z.getLE(), z_z.getLE(), pos_z.getLE() };
        return { row0, row1, row2 };
    }

    glm::mat4x3 getLEMatrix() const {
        return glm::mat4x3(
            glm::vec3(x_x.getLE(), x_y.getLE(), x_z.getLE()),      // X basis column
            glm::vec3(y_x.getLE(), y_y.getLE(), y_z.getLE()),      // Y basis column
            glm::vec3(z_x.getLE(), z_y.getLE(), z_z.getLE()),      // Z basis column
            glm::vec3(pos_x.getLE(), pos_y.getLE(), pos_z.getLE()) // translation column
        );
    }

    void setLEMatrix(const glm::mat4x3& m) {
        // m[col][row]
        x_x = m[0][0];
        x_y = m[0][1];
        x_z = m[0][2];
        y_x = m[1][0];
        y_y = m[1][1];
        y_z = m[1][2];
        z_x = m[2][0];
        z_y = m[2][1];
        z_z = m[2][2];

        pos_x = m[3][0];
        pos_y = m[3][1];
        pos_z = m[3][2];
    }

    BEVec3 getPos() const {
        return { pos_x, pos_y, pos_z };
    }

    void setPos(glm::fvec3 pos) {
        pos_x = pos.x;
        pos_y = pos.y;
        pos_z = pos.z;
    }

    glm::fquat getRotLE() const {
        return glm::quat_cast(glm::fmat3(
            x_x.getLE(), x_y.getLE(), x_z.getLE(),
            y_x.getLE(), y_y.getLE(), y_z.getLE(),
            z_x.getLE(), z_y.getLE(), z_z.getLE()
        ));
    }

	void setRotLE(const glm::fquat& rotation) {
        glm::fmat3 rotMat = glm::mat3_cast(rotation);

        x_x = rotMat[0][0];
        y_x = rotMat[0][1];
        z_x = rotMat[0][2];
        x_y = rotMat[1][0];
        y_y = rotMat[1][1];
        z_y = rotMat[1][2];
        x_z = rotMat[2][0];
        y_z = rotMat[2][1];
        z_z = rotMat[2][2];
    }

    glm::mat4 getNewMatrix() {
        glm::mat4 translationMatrix = glm::translate(glm::identity<glm::mat4>(), glm::fvec3(pos_x.getLE(), pos_y.getLE(), pos_z.getLE()));
        glm::mat4 rotationMatrix = glm::mat4(
            x_x.getLE(), y_x.getLE(), z_x.getLE(), 0.0f,
            x_y.getLE(), y_y.getLE(), z_y.getLE(), 0.0f,
            x_z.getLE(), y_z.getLE(), z_z.getLE(), 0.0f,
            0.0f,       0.0f,       0.0f,       1.0f
        );
        return translationMatrix * rotationMatrix;
    }

    void setNewMatrix(const glm::fmat4& mtx) {
        glm::fmat3 rotationPart = glm::fmat3(mtx);
        glm::fvec3 translationPart = glm::fvec3(mtx[3]);
        x_x = rotationPart[0][0];
        y_x = rotationPart[0][1];
        z_x = rotationPart[0][2];
        x_y = rotationPart[1][0];
        y_y = rotationPart[1][1];
        z_y = rotationPart[1][2];
        x_z = rotationPart[2][0];
        y_z = rotationPart[2][1];
        z_z = rotationPart[2][2];
        pos_x = translationPart.x;
        pos_y = translationPart.y;
        pos_z = translationPart.z;
    }
};

struct BEMatrix44 : BETypeCompatible {
    BEType<float> a00;
    BEType<float> a01;
    BEType<float> a02;
    BEType<float> a03;
    BEType<float> a10;
    BEType<float> a11;
    BEType<float> a12;
    BEType<float> a13;
    BEType<float> a20;
    BEType<float> a21;
    BEType<float> a22;
    BEType<float> a23;
    BEType<float> a30;
    BEType<float> a31;
    BEType<float> a32;
    BEType<float> a33;

    BEMatrix44() = default;

    glm::fmat4 getLE() const {
        return glm::fmat4(
            a00.getLE(), a01.getLE(), a02.getLE(), a03.getLE(),
            a10.getLE(), a11.getLE(), a12.getLE(), a13.getLE(),
            a20.getLE(), a21.getLE(), a22.getLE(), a23.getLE(),
            a30.getLE(), a31.getLE(), a32.getLE(), a33.getLE()
        );
    }

    void operator=(glm::fmat4 mtx) {
        a00 = mtx[0][0];
        a01 = mtx[0][1];
        a02 = mtx[0][2];
        a03 = mtx[0][3];
        a10 = mtx[1][0];
        a11 = mtx[1][1];
        a12 = mtx[1][2];
        a13 = mtx[1][3];
        a20 = mtx[2][0];
        a21 = mtx[2][1];
        a22 = mtx[2][2];
        a23 = mtx[2][3];
        a30 = mtx[3][0];
        a31 = mtx[3][1];
        a32 = mtx[3][2];
        a33 = mtx[3][3];
    }
};

struct data_VRSettingsIn {
    BEType<int32_t> cameraModeSetting;
    BEType<int32_t> leftHandedSetting;
    BEType<int32_t> guiFollowSetting;
    BEType<float> playerHeightSetting;
    BEType<int32_t> enable2DVRView;
    BEType<int32_t> cropFlatTo16x9Setting;
    BEType<int32_t> enableDebugOverlay;
    BEType<int32_t> buggyAngularVelocity;

    bool IsLeftHanded() const {
        return leftHandedSetting == 1;
    }

    bool IsFirstPersonMode() const {
        return cameraModeSetting == 1;
    }

    bool IsThirdPersonMode() const {
        return cameraModeSetting == 0;
    }

    bool UIFollowsLookingDirection() const {
        return guiFollowSetting == 1;
    }

    bool Is2DVRViewEnabled() const {
        return enable2DVRView == 1;
    }

    bool ShouldFlatPreviewBeCroppedTo16x9() const {
        return cropFlatTo16x9Setting == 1;
    }

    bool ShowDebugOverlay() const {
        return enableDebugOverlay.getLE() != 0;
    }

    float GetZNear() const {
        return 0.1f;
    }

    float GetZFar() const {
        return 25000.0f;
    }

    enum class AngularVelocityFixerMode {
        AUTO = 0, // Angular velocity fixer is automatically enabled for Oculus Link
        FORCED_ON = 1,
        FORCED_OFF = 2,

    };

    AngularVelocityFixerMode AngularVelocityFixer_GetMode() {
        return (AngularVelocityFixerMode)buggyAngularVelocity.getLE();
    }
};



#pragma pack(push, 1)
struct BESeadProjection {
    BEType<bool> dirty;
    BEType<bool> deviceDirty;
    BEType<uint8_t> pad0;
    BEType<uint8_t> pad1;
    BEMatrix44 matrix;
    BEMatrix44 deviceMatrix;
    BEType<uint32_t> devicePosture;
    BEType<float> deviceZScale;
    BEType<float> deviceZOffset;
    BEType<uint32_t> __vftable;
};

struct BESeadPerspectiveProjection : BESeadProjection {
    BEType<float> zNear;
    BEType<float> zFar;
    BEType<float> fovYRadiansOrAngle;
    BEType<float> fovySin;
    BEType<float> fovyCos;
    BEType<float> fovyTan;
    BEType<float> aspect;
    BEVec2 offset;
};
#pragma pack(pop)
static_assert(sizeof(BESeadProjection) == 0x94, "BESeadProjection size mismatch");
static_assert(sizeof(BESeadPerspectiveProjection) == 0xB8, "BESeadPerspectiveProjection size mismatch");

#pragma pack(push, 1)
struct BESeadCamera {
    BEMatrix34 mtx;
    BEType<uint32_t> __vftable;
};
struct BESeadLookAtCamera : BESeadCamera {
    BEVec3 pos;
    BEVec3 at;
    BEVec3 up;

    bool operator==(const BESeadLookAtCamera& other) const {
        return pos == other.pos && at == other.at && up == other.up;
    }
};
#pragma pack(pop)
static_assert(sizeof(BESeadCamera) == 0x34, "BESeadCamera size mismatch");
static_assert(sizeof(BESeadLookAtCamera) == 0x58, "BESeadLookAtCamera size mismatch");
struct data_VRProjectionMatrixOut {
    BEType<float> aspectRatio;
    BEType<float> fovY;
    BEType<float> offsetX;
    BEType<float> offsetY;
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

#include "game_structs.h"
#include "cemu.h"
#include "utils/logger.h"

constexpr uint32_t SEMAPHORE_TO_VULKAN = 0;
constexpr uint32_t SEMAPHORE_TO_D3D12 = 1;
