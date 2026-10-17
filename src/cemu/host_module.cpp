#include "pch.h"
#include "capture.h"
#include "cemu_host.h"
#include "cemu_settings.h"
#include "instance.h"
#include "vulkan_device.h"
#include "vulkan_loader.h"
#include "game/game.h"
#include "game/botw/pack_shared_memory.h"
#include "bettervr_version.h"
#include "utils/eye_resolution.h"
#include "utils/mod_settings.h"
#include "utils/platform.h"
#include "utils/update_check.h"

#include <fstream>

#include "Cafe/HostModule/HostModule.h"

PFN_vkGetInstanceProcAddr VulkanLoader::GetInstanceProcAddr() {
    return HostModule::GetVkInstanceProcAddr();
}

PFN_vkGetDeviceProcAddr VulkanLoader::GetDeviceProcAddr() {
    return HostModule::GetVkDeviceProcAddr();
}

// relative to the image height, full rate inside the inner radius and quarter rate past the outer one
static constexpr float kFoveationInnerRadius = 0.3f;
static constexpr float kFoveationOuterRadius = 0.45f;
static constexpr float kExaggeratedFoveationInnerRadius = 0.1f;
static constexpr float kExaggeratedFoveationOuterRadius = 0.15f;

void CemuHost::SetMultiviewActive(bool active) {
    HostModule::SetMultiviewStereoActive(active);
}

void CemuHost::SetFoveationCentres(const float left[2], const float right[2]) {
    HostModule::SetFoveationCentres(left, right);
}

void CemuHost::SetFoveatedShadingActive(bool active) {
    HostModule::SetFoveatedShadingActive(active);
}

void CemuHost::SetFoveationExaggerated(bool exaggerated) {
    if (exaggerated) {
        HostModule::SetFoveationRadii(kExaggeratedFoveationInnerRadius, kExaggeratedFoveationOuterRadius);
    }
    else {
        HostModule::SetFoveationRadii(kFoveationInnerRadius, kFoveationOuterRadius);
    }
}

uint64_t CemuHost::GetCurrentTitleId() {
    return HostModule::GetTitleId();
}

void CemuHost::RegisterHook(const char* name, HookFunction function) {
    HostModule::RegisterHLEFunction("coreinit", name, function);
}

std::filesystem::path CemuHost::GetUserDataPath() {
    return HostModule::GetUserDataPath();
}

void CemuHost::AttachToNextSubmit(VkSemaphore wait, uint64_t waitValue, VkSemaphore signal, uint64_t signalValue) {
    HostModule::AttachToNextSubmit({ wait, waitValue }, { signal, signalValue });
}

void Platform::ShowFatalError(const char* title, const char* message) {
    HostModule::ShowErrorDialog(title, message);
}

uint8_t* CemuHost::GetGuestMemoryBase() {
    return HostModule::GetGuestMemoryBase();
}

// next to Cemu, in share/Cemu for the AppImage or in the source tree for development builds
static std::filesystem::path FindPacksDirectory() {
    const std::filesystem::path executableDirectory = Platform::GetExecutableDirectory();
    for (const std::filesystem::path& bundled : { executableDirectory / "BetterVR" / "packs", executableDirectory.parent_path() / "share" / "Cemu" / "packs" }) {
        std::error_code error;
        if (std::filesystem::is_directory(bundled, error)) {
            return bundled;
        }
    }
#if defined(BETTERVR_PACKS_DIR)
    return BETTERVR_PACKS_DIR;
#else
    return {};
#endif
}

static std::filesystem::path WriteGraphicsPack(const std::filesystem::path& source, uint32_t eyeWidth, uint32_t eyeHeight) {
    const std::filesystem::path target = HostModule::GetUserDataPath() / "BetterVR" / source.filename();
    std::filesystem::remove_all(target);
    std::filesystem::create_directories(target);
    std::filesystem::copy(source, target, std::filesystem::copy_options::recursive);

    std::ifstream sourceRules(source / "rules.txt", std::ios::binary);
    std::string rules((std::istreambuf_iterator<char>(sourceRules)), std::istreambuf_iterator<char>());
    EyeResolution::FillGraphicsRules(rules, eyeWidth, eyeHeight, EyeResolution::kMinimumWidth, [](const EyeResolution::Preset& preset, const std::string& label) {
        Log::print<INFO>("Graphics pack preset {} is {}", preset.multiplierLabel, label);
    });
    std::ofstream(target / "rules.txt", std::ios::binary | std::ios::trunc) << rules;
    return target;
}

#if defined(_WIN32)
static std::wstring ReadEnvironmentVariable(const wchar_t* name) {
    const DWORD size = GetEnvironmentVariableW(name, nullptr, 0);
    if (size == 0) {
        return {};
    }
    std::wstring value(size, L'\0');
    value.resize(GetEnvironmentVariableW(name, value.data(), size));
    return value;
}

static void RemoveBetterVRLayers(const wchar_t* name, wchar_t separator) {
    std::wstring kept;
    for (const auto entry : std::views::split(ReadEnvironmentVariable(name), separator)) {
        const std::wstring layer(entry.begin(), entry.end());
        std::wstring lowered = layer;
        std::ranges::transform(lowered, lowered.begin(), [](wchar_t c) { return (wchar_t)towlower(c); });
        if (layer.empty() || lowered.contains(L"bettervr")) {
            continue;
        }
        if (!kept.empty()) {
            kept += separator;
        }
        kept += layer;
    }
    SetEnvironmentVariableW(name, kept.empty() ? nullptr : kept.c_str());
}

// older releases hooked Cemu as a Vulkan layer or as a vulkan-1.dll next to Cemu.exe
static void KeepOlderBetterVRReleasesOut() {
    SetEnvironmentVariableW(L"ENABLE_BETTERVR_MOD", nullptr);
    SetEnvironmentVariableW(L"ENABLE_BETTERVR_MOD_ADMIN", nullptr);
    SetEnvironmentVariableW(L"DISABLE_BETTERVR_MOD", L"1");
    RemoveBetterVRLayers(L"VK_INSTANCE_LAYERS", L';');
    RemoveBetterVRLayers(L"VK_LOADER_LAYERS_ENABLE", L',');
    const std::wstring disabledLayers = ReadEnvironmentVariable(L"VK_LOADER_LAYERS_DISABLE");
    SetEnvironmentVariableW(L"VK_LOADER_LAYERS_DISABLE", (disabledLayers.empty() ? std::wstring(L"*bettervr*") : disabledLayers + L",*bettervr*").c_str());

    std::error_code error;
    const std::filesystem::path besideExecutable = Platform::GetExecutableDirectory() / "vulkan-1.dll";
    if (!std::filesystem::exists(besideExecutable, error)) {
        return;
    }
    // Cemu loads vulkan-1.dll by name, which resolves to the one already loaded
    if (LoadLibraryExW(L"vulkan-1.dll", nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32) != nullptr) {
        Log::print<WARNING>("Ignoring {}, Cemu uses the Vulkan loader in System32 instead", besideExecutable.string());
    }
    else {
        Log::print<ERROR>("Cemu has to use {} as its Vulkan loader, since System32 has none", besideExecutable.string());
    }
}
#endif

class BetterVRModule : public HostModule::Module {
public:
    const char* GetName() const override {
        return "BetterVR";
    }

    void OnBeforeConfigLoad(const std::filesystem::path& settingsFile) override {
        Log::SetFilePath(HostModule::GetUserDataPath() / "BetterVR_log.txt");
#if defined(_WIN32)
        KeepOlderBetterVRReleasesOut();
        // overlay layers that hook Cemu's presents get in the way of the OpenXR runtime
        SetEnvironmentVariableW(L"DISABLE_VULKAN_OBS_CAPTURE", L"1");
        SetEnvironmentVariableW(L"DISABLE_LAYER_NV_PRESENT_1", L"1");
#endif
        CemuSettings::Sync(settingsFile.parent_path() / "settings.xml", settingsFile);
        static std::filesystem::path s_settingsFile = settingsFile;
        std::atexit([] { CemuSettings::ShareBack(s_settingsFile, s_settingsFile.parent_path() / "settings.xml"); });
        CemuSettings::Enforce(settingsFile);
    }

    // the OpenXR instance has to exist before Cemu loads the graphic packs, since the presets need the headset's resolution
    void OnInitialize() override {
        const std::filesystem::path packs = FindPacksDirectory();
        std::error_code error;
        if (!std::ranges::all_of(Game::GetTitles(), [&](const Game::Title& title) { return std::filesystem::exists(packs / title.vrPack / "rules.txt", error); })) {
            HostModule::ShowErrorDialog("BetterVR is missing its files", "The BetterVR folder that came with BetterVR_Launcher.exe is missing, so it runs without BetterVR and its graphic packs. Keep the BetterVR folder next to BetterVR_Launcher.exe, as it is in the download.");
            return;
        }
        Settings_ReadFile(HostModule::GetUserDataPath() / "BetterVR_settings.ini");
        if (GetSettings().bootDirectlyIntoGame && Game::FindTitle(GetSettings().bootDirectlyTitleId) != nullptr) {
            HostModule::BootTitleOnStart(GetSettings().bootDirectlyTitleId);
        }
        UpdateCheck::Start([](const std::string& tag) {
            const std::string message = fmt::format("BetterVR {} is out. You're on {}.", tag, BETTERVR_VERSION_STRING);
            HostModule::ShowLinkDialog("BetterVR update", message.c_str(), "Open release page", UpdateCheck::kReleasesUrl);
        });
        HostModule::RegisterBundledGraphicPacks(packs / "graphicPacks");
        try {
            const std::array<XrViewConfigurationView, 2> views = VRManager::instance().XR->GetViewConfigurations();
            for (const Game::Title& title : Game::GetTitles()) {
                const std::filesystem::path graphicsPack = WriteGraphicsPack(packs / title.graphicsPack, views[0].recommendedImageRectWidth, views[0].recommendedImageRectHeight);
                HostModule::RegisterGraphicPack(graphicsPack, HostModule::PackOrder::AfterUser);
            }
        }
        catch (const std::exception& e) {
            Log::print<ERROR>("BetterVR is off for this run: {}", e.what());
            return;
        }

        m_available = true;
        HostModule::UseMultiviewStereo();
        Game::RegisterHooks();
        for (const Game::Title& title : Game::GetTitles()) {
            HostModule::RegisterGraphicPack(packs / title.vrPack, HostModule::PackOrder::BeforeUser);
            if (!title.requiredPack.empty()) {
                HostModule::RegisterGraphicPack(packs / "graphicPacks" / title.requiredPack, HostModule::PackOrder::AfterUser);
            }
        }
        // well above the headset's refresh rate, so FPS++'s FPS limit doesn't cap the headset
        HostModule::SetVsyncFrequency(180);
    }

    bool CreateVkInstance(const VkInstanceCreateInfo& createInfo, VkInstance& instance, VkResult& result) override {
        const Game::Title* title = m_available ? Game::FindTitle(HostModule::GetTitleId()) : nullptr;
        m_active = title != nullptr;
        if (!m_active) {
            return false;
        }
        HostModule::UseFoveatedShading(title->renderWidth, title->renderHeight, kFoveationInnerRadius, kFoveationOuterRadius);
        const auto createInstance = (PFN_vkCreateInstance)VulkanLoader::GetInstanceProcAddr()(VK_NULL_HANDLE, "vkCreateInstance");
        result = VulkanDevice::CreateInstance(createInstance, &createInfo, nullptr, &instance);
        return true;
    }

    VkPhysicalDevice SelectPhysicalDevice(VkInstance instance) override {
        return m_active ? VRManager::instance().XR->GetVulkanGraphicsDevice(instance) : VK_NULL_HANDLE;
    }

    bool CreateVkDevice(VkPhysicalDevice physicalDevice, const VkDeviceCreateInfo& createInfo, VkDevice& device, VkResult& result) override {
        if (!m_active) {
            return false;
        }

        // vkroots normally gets this link from the Vulkan loader
        VkLayerDeviceLink layerLink = {};
        layerLink.pfnNextGetInstanceProcAddr = VulkanLoader::GetInstanceProcAddr();
        layerLink.pfnNextGetDeviceProcAddr = VulkanLoader::GetDeviceProcAddr();

        VkLayerDeviceCreateInfo layerCreateInfo = {};
        layerCreateInfo.sType = VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO;
        layerCreateInfo.pNext = createInfo.pNext;
        layerCreateInfo.function = VK_LAYER_LINK_INFO;
        layerCreateInfo.u.pLayerInfo = &layerLink;

        VkDeviceCreateInfo linkedCreateInfo = createInfo;
        linkedCreateInfo.pNext = &layerCreateInfo;
        result = VulkanDevice::CreateDevice(*vkroots::LookupDispatch(physicalDevice), physicalDevice, &linkedCreateInfo, nullptr, &device);
        return true;
    }

    void OnVkDeviceCreated(const HostModule::VulkanDevice& device) override {
        if (!m_active) {
            return;
        }
        VRManager::instance().Init(device.instance, device.physicalDevice, device.device);
        VRManager::instance().InitSession();
    }

    void OnHostEvent(const HostModule::HostEvent& event, VkCommandBuffer commandBuffer) override {
        if (!m_active || event.surface == nullptr) {
            return;
        }

        // the mod records into Cemu's command buffers through vkroots, which only knows the ones it allocated
        if (vkroots::LookupDispatch(commandBuffer) == nullptr) {
            vkroots::tables::AssignDispatchTable(commandBuffer, VRManager::instance().VK->GetDeviceDispatch());
        }

        const HostModule::EventSurface& eventSurface = *event.surface;
        const Capture::Surface surface = {
            .image = eventSurface.image,
            .extent = eventSurface.extent,
            .format = eventSurface.format,
            .aspectMask = eventSurface.aspectMask,
            .sliceIndex = eventSurface.sliceIndex,
        };
        if (VRManager::instance().XR->GetRenderer() == nullptr) {
            return;
        }

        std::optional<Capture::Surface> rightSurface;
        if (event.rightSurface != nullptr) {
            rightSurface = surface;
            rightSurface->image = event.rightSurface->image;
            rightSurface->sliceIndex = event.rightSurface->sliceIndex;
        }

        const Capture::FrameState state = Capture::ReadFrameState().value_or(Capture::FrameState{});
        switch ((PackHostEvent)event.id) {
            case PackHostEvent::Capture3DColor:
                return Capture::On3DColor(state, commandBuffer, surface, rightSurface ? &*rightSurface : nullptr);
            case PackHostEvent::Capture3DDepth:
                return Capture::On3DDepth(state, commandBuffer, surface, rightSurface ? &*rightSurface : nullptr);
            case PackHostEvent::Capture2D:
                return Capture::OnHud(state, commandBuffer, surface);
        }
        Log::print<WARNING>("Ignoring host event {} that the pack doesn't define", event.id);
    }

    void OnFrameSubmitted(HostModule::FrameKind kind) override {
        if (m_active) {
            Capture::OnFrameSubmitted();
        }
    }

private:
    bool m_available = false;
    bool m_active = false;
};

void BetterVR_RegisterHostModule() {
    static BetterVRModule s_module;
    HostModule::Register(&s_module);
}
