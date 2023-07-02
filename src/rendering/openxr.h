#pragma once

#include "swapchain.h"

class OpenXR {
    friend class RND_Renderer;

public:
    OpenXR();
    ~OpenXR();

    enum EyeSide : uint8_t {
        LEFT = 0,
        RIGHT = 1
    };

    struct Capabilities {
        LUID adapter;
        D3D_FEATURE_LEVEL minFeatureLevel;
        bool supportsOrientational;
        bool supportsPositional;
        bool supportsMutatableFOV;
    } m_capabilities = {};

    void CreateSession(const XrGraphicsBindingD3D12KHR& d3d12Binding);
    std::array<XrViewConfigurationView, 2> GetViewConfigurations();
    void UpdateTime(EyeSide side, XrTime predictedDisplayTime);
    std::optional<XrSpaceLocation> UpdateSpaces(XrTime predictedDisplayTime);
    void ProcessEvents();

    XrSession GetSession() { return m_session; }
    XrView GetPredictedView(EyeSide side) { return m_updatedViews[side]; };
    RND_Renderer* GetRenderer() { return m_renderer.get(); }
private:
    XrInstance m_instance = XR_NULL_HANDLE;
    XrSystemId m_systemId = XR_NULL_SYSTEM_ID;
    XrSession m_session = XR_NULL_HANDLE;
    XrSpace m_stageSpace = XR_NULL_HANDLE;
    XrSpace m_headSpace = XR_NULL_HANDLE;

    std::unique_ptr<RND_Renderer> m_renderer;

    std::array<XrTime, 2> m_frameTimes = { 0, 0 };
    std::array<XrView, 2> m_updatedViews = {};

    XrDebugUtilsMessengerEXT m_debugMessengerHandle = XR_NULL_HANDLE;

    PFN_xrGetD3D12GraphicsRequirementsKHR func_xrGetD3D12GraphicsRequirementsKHR = nullptr;
    PFN_xrConvertTimeToWin32PerformanceCounterKHR func_xrConvertTimeToWin32PerformanceCounterKHR = nullptr;
    PFN_xrConvertWin32PerformanceCounterToTimeKHR func_xrConvertWin32PerformanceCounterToTimeKHR = nullptr;
    PFN_xrCreateDebugUtilsMessengerEXT func_xrCreateDebugUtilsMessengerEXT = nullptr;
    PFN_xrDestroyDebugUtilsMessengerEXT func_xrDestroyDebugUtilsMessengerEXT = nullptr;
};