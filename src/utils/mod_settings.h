#pragma once

class ModSettingBase;

inline std::vector<ModSettingBase*>& SettingRegistry() {
    static std::vector<ModSettingBase*> registry;
    return registry;
}

class ModSettingBase {
public:
    const char* name;

    ModSettingBase(const char* name): name(name) {
        SettingRegistry().push_back(this);
    }

    ModSettingBase(const ModSettingBase&) = delete;
    ModSettingBase& operator=(const ModSettingBase&) = delete;

    virtual ~ModSettingBase() = default;

    [[nodiscard]] virtual std::string Serialize() const = 0;

    virtual void Deserialize(std::string_view valueString) = 0;

    virtual void Reset() = 0;

    void ResetWithError(std::string_view valueString) {
        Reset();
        Log::print<ERROR>("{} had invalid value \"{}\". Resetting to \"{}\"", name, valueString, Serialize());
    }

    void AddResetToGUI(bool* changed) {
        ImGui::PushID(name);
        if (ImGui::Button("Reset")) {
            Reset();
            *changed = true;
        }
        ImGui::PopID();
    }
};

template <typename T>
class ModSetting : public ModSettingBase {
private:
    std::atomic<T> m_value;

public:
    const T defaultValue;

    ModSetting(const char* name, T defaultValue): ModSettingBase(name), m_value(defaultValue), defaultValue(defaultValue) {}

    T load(std::memory_order order = std::memory_order_seq_cst) const {
        return m_value.load(order);
    }

    operator T() const {
        return load();
    }

    T operator=(const T value) {
        return Set(value);
    }

    virtual T NormalizeValue(T value) const {
        return value;
    }

    virtual T Set(const T value, std::memory_order order = std::memory_order_seq_cst) {
        const T newValue = NormalizeValue(value);
        m_value.store(newValue, order);
        return newValue;
    }

    void Reset() override {
        Set(defaultValue);
    }

    template <typename DrawWidget>
    void AddWidgetToGUI(bool* changed, DrawWidget&& drawWidget) {
        T value = load();
        ImGui::PushID(name);
        const bool edited = drawWidget(&value);
        ImGui::PopID();
        if (edited) {
            *this = value;
            *changed = true;
        }
    }
};

template <typename T>
concept SettingNumber = std::integral<T> || std::floating_point<T>;

template <SettingNumber T>
class NumberSetting : public ModSetting<T> {
public:
    using ModSetting<T>::operator=;

    const T min;
    const T max;

    NumberSetting(const char* name, T defaultValue, T min = std::numeric_limits<T>::lowest(), T max = std::numeric_limits<T>::max()): ModSetting<T>(name, defaultValue), min(min), max(max) {
        this->Set(defaultValue);
    }

    T NormalizeValue(T value) const override {
        if (value < min) {
            Log::print<ERROR>("Tried to set {} to too low value {}, setting to minimum value {} instead", this->name, value, min);
            return min;
        }
        if (value > max) {
            Log::print<ERROR>("Tried to set {} to too high value {}, setting to maximum value {} instead", this->name, value, max);
            return max;
        }
        return value;
    }

    [[nodiscard]] std::string Serialize() const override {
        std::array<char, 32> buffer;
        char* valueEnd;
        if constexpr (std::floating_point<T>) {
            valueEnd = std::to_chars(buffer.data(), buffer.data() + buffer.size(), this->load(), std::chars_format::general, 6).ptr;
        }
        else {
            valueEnd = std::to_chars(buffer.data(), buffer.data() + buffer.size(), this->load()).ptr;
        }
        return std::string(buffer.data(), valueEnd);
    }

    void Deserialize(std::string_view valueString) override {
        if (valueString.starts_with('+')) {
            valueString.remove_prefix(1);
        }
        T parsed{};
        const auto [parseEnd, ec] = std::from_chars(valueString.data(), valueString.data() + valueString.size(), parsed);
        if (ec == std::errc::result_out_of_range) {
            parsed = valueString.starts_with('-') ? std::numeric_limits<T>::lowest() : std::numeric_limits<T>::max();
        }
        else if (ec != std::errc()) {
            this->ResetWithError(valueString);
            return;
        }
        *this = parsed;
    }

    template <typename Fmt = const char*>
    void AddSliderToGUI(bool* changed, T minValue, T maxValue, Fmt&& format = "%.2f") {
        this->AddWidgetToGUI(changed, [&](T* value) {
            if constexpr (std::is_invocable_v<Fmt&, T>) {
                return DrawSlider(value, minValue, maxValue, std::invoke(format, *value).c_str());
            }
            else {
                return DrawSlider(value, minValue, maxValue, format);
            }
        });
    }

    template <typename Fmt = const char*>
    void AddToGUI(bool* changed, float windowWidth, T minValue, T maxValue, Fmt&& format = "%.2f") {
        ImGui::PushItemWidth(windowWidth * 0.35f);
        AddSliderToGUI(changed, minValue, maxValue, std::forward<Fmt>(format));
        ImGui::PopItemWidth();
        ImGui::SameLine();
        this->AddResetToGUI(changed);
    }

    void AddPercentToGUI(bool* changed, float windowWidth, float minPercent, float maxPercent) {
        ImGui::PushItemWidth(windowWidth * 0.35f);
        this->AddWidgetToGUI(changed, [&](T* value) {
            float percent = (float)*value * 100.0f;
            if (!ImGui::SliderFloat("##value", &percent, minPercent, maxPercent, "%.0f%%")) {
                return false;
            }
            *value = (T)(percent / 100.0f);
            return true;
        });
        ImGui::PopItemWidth();
        ImGui::SameLine();
        this->AddResetToGUI(changed);
    }

private:
    static bool DrawSlider(T* value, T minValue, T maxValue, const char* format) {
        if constexpr (std::floating_point<T>) {
            return ImGui::SliderFloat("##value", value, minValue, maxValue, format);
        }
        else {
            int intValue = (int)*value;
            if (!ImGui::SliderInt("##value", &intValue, (int)minValue, (int)maxValue, format)) {
                return false;
            }
            *value = (T)intValue;
            return true;
        }
    }
};

using FloatSetting = NumberSetting<float>;
using UIntSetting = NumberSetting<uint32_t>;

class BoolSetting : public ModSetting<bool> {
private:
    void (*m_onChange)(bool);

public:
    using ModSetting<bool>::operator=;

    BoolSetting(const char* name, bool defaultValue, void (*onChange)(bool) = nullptr): ModSetting<bool>(name, defaultValue), m_onChange(onChange) {
        if (m_onChange != nullptr) {
            m_onChange(defaultValue);
        }
    }

    bool Set(const bool value, std::memory_order order = std::memory_order_seq_cst) override {
        const bool newValue = ModSetting<bool>::Set(value, order);
        if (m_onChange != nullptr) {
            m_onChange(newValue);
        }
        return newValue;
    }

    [[nodiscard]] std::string Serialize() const override {
        return load() ? "true" : "false";
    }

    void Deserialize(std::string_view valueString) override {
        if (IEquals(valueString, "true")) {
            *this = true;
        }
        else if (IEquals(valueString, "false")) {
            *this = false;
        }
        else {
            //numeric load backup for older syntax
            int parsed = 0;
            const auto [parseEnd, ec] = std::from_chars(valueString.data(), valueString.data() + valueString.size(), parsed);
            if (ec != std::errc()) {
                ResetWithError(valueString);
            }
            else {
                *this = parsed != 0;
            }
        }
    }

    void AddToGUI(bool* changed) {
        AddWidgetToGUI(changed, [](bool* value) { return ImGui::Checkbox("##value", value); });
    }
};

class StringSetting : public ModSettingBase {
private:
    std::string m_value;

public:
    const std::string defaultValue;

    StringSetting(const char* name, std::string defaultValue): ModSettingBase(name), m_value(defaultValue), defaultValue(std::move(defaultValue)) {}

    const std::string& Get() const {
        return m_value;
    }

    operator const std::string&() const {
        return Get();
    }

    void Set(std::string value) {
        m_value = NormalizeValue(std::move(value));
    }

    [[nodiscard]] std::string Serialize() const override {
        return m_value;
    }

    void Deserialize(std::string_view valueString) override {
        Set(std::string(valueString));
    }

    void Reset() override {
        m_value = defaultValue;
    }

    virtual std::string NormalizeValue(std::string value) const {
        return value;
    }
};

template <typename T>
struct EnumEntry {
    T value;
    const char* name;
    const char* displayName = nullptr;
};

template <typename T>
struct EnumTable;

template <typename T>
requires(std::is_enum_v<T>)
constexpr const EnumEntry<T>* FindEnumEntry(T value) {
    for (const EnumEntry<T>& entry : EnumTable<T>::entries) {
        if (entry.value == value) {
            return &entry;
        }
    }
    return nullptr;
}

template <typename T>
requires(std::is_enum_v<T>)
constexpr const char* EnumName(T value) {
    const EnumEntry<T>* entry = FindEnumEntry(value);
    return entry != nullptr ? entry->name : "";
}

template <typename T>
requires(std::is_enum_v<T>)
constexpr const char* EnumDisplayName(T value) {
    const EnumEntry<T>* entry = FindEnumEntry(value);
    return entry != nullptr && entry->displayName != nullptr ? entry->displayName : "";
}

template <typename T>
requires(std::is_enum_v<T>)
auto SelectableEnumEntries() {
    return EnumTable<T>::entries | std::views::filter([](const EnumEntry<T>& entry) { return entry.displayName != nullptr; });
}

template <typename T>
requires(std::is_enum_v<T>)
class EnumSetting : public ModSetting<T> {
public:
    using ModSetting<T>::operator=;

    EnumSetting(const char* name, T defaultValue): ModSetting<T>(name, defaultValue) {
        if (!IsSelectable(defaultValue)) {
            Log::print<ERROR>("{} was given default value \"{}\", which users can't select", name, EnumName(defaultValue));
        }
    }

    static constexpr bool IsSelectable(T value) {
        const EnumEntry<T>* entry = FindEnumEntry(value);
        return entry != nullptr && entry->displayName != nullptr;
    }

    T NormalizeValue(T value) const override {
        if (IsSelectable(value)) {
            return value;
        }
        Log::print<ERROR>("Tried to set {} to an invalid value, resetting to default value {} instead", this->name, EnumName(this->defaultValue));
        return this->defaultValue;
    }

    [[nodiscard]] std::string Serialize() const override {
        return std::string(EnumName(this->load()));
    }

    void Deserialize(std::string_view valueString) override {
        //numeric load backup for older syntax
        std::underlying_type_t<T> parsed{};
        const bool isNumeric = std::from_chars(valueString.data(), valueString.data() + valueString.size(), parsed).ec == std::errc();
        for (const EnumEntry<T>& entry : SelectableEnumEntries<T>()) {
            if (IEquals(entry.name, valueString) || (isNumeric && std::to_underlying(entry.value) == parsed)) {
                *this = entry.value;
                return;
            }
        }
        this->ResetWithError(valueString);
    }

    void AddRadioToGUI(bool* changed) {
        const T current = this->load();
        bool first = true;
        ImGui::PushID(this->name);
        for (const EnumEntry<T>& entry : SelectableEnumEntries<T>()) {
            if (first) {
                first = false;
            }
            else {
                ImGui::SameLine();
            }
            if (ImGui::RadioButton(entry.displayName, entry.value == current)) {
                *this = entry.value;
                *changed = true;
            }
        }
        ImGui::PopID();
    }

    void AddComboToGUI(bool* changed) {
        const T current = this->load();
        ImGui::PushID(this->name);
        if (ImGui::BeginCombo("##value", EnumDisplayName(current))) {
            for (const EnumEntry<T>& entry : SelectableEnumEntries<T>()) {
                const bool isSelected = entry.value == current;
                if (ImGui::Selectable(entry.displayName, isSelected)) {
                    *this = entry.value;
                    *changed = true;
                }
                if (isSelected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::PopID();
    }
};

enum class EventMode : int32_t {
    NO_EVENT = 0,
    ALWAYS_FIRST_PERSON = 1,
    FOLLOW_DEFAULT_EVENT_SETTINGS = 2,
    ALWAYS_THIRD_PERSON = 3,
};

template <>
struct EnumTable<EventMode> {
    static constexpr auto entries = std::to_array<EnumEntry<EventMode>>({
        { EventMode::NO_EVENT, "NO_EVENT" },
        { EventMode::ALWAYS_FIRST_PERSON, "ALWAYS_FIRST_PERSON", "First Person (Always)" },
        { EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS, "FOLLOW_DEFAULT_EVENT_SETTINGS", "Optimal Settings (Mix Of Third/First)" },
        { EventMode::ALWAYS_THIRD_PERSON, "ALWAYS_THIRD_PERSON", "Third Person (Always)" }
    });
};

enum class CameraMode : int32_t {
    THIRD_PERSON = 0,
    FIRST_PERSON = 1,
};

template <>
struct EnumTable<CameraMode> {
    static constexpr auto entries = std::to_array<EnumEntry<CameraMode>>({
        { CameraMode::FIRST_PERSON, "FIRST_PERSON", "First Person (Recommended)" },
        { CameraMode::THIRD_PERSON, "THIRD_PERSON", "Third Person" }
    });
};

enum class PlayMode : int32_t {
    SEATED = 0,
    STANDING = 1,
};

template <>
struct EnumTable<PlayMode> {
    static constexpr auto entries = std::to_array<EnumEntry<PlayMode>>({
        { PlayMode::STANDING, "STANDING", "Standing" },
        { PlayMode::SEATED, "SEATED", "Seated" }
    });
};

enum class AngularVelocityFixerMode : int32_t {
    AUTO = 0, // Angular velocity fixer is automatically enabled for Oculus Link
    FORCED_ON = 1,
    FORCED_OFF = 2,
};

template <>
struct EnumTable<AngularVelocityFixerMode> {
    static constexpr auto entries = std::to_array<EnumEntry<AngularVelocityFixerMode>>({
        { AngularVelocityFixerMode::AUTO, "AUTO", "Auto (Oculus Link)" },
        { AngularVelocityFixerMode::FORCED_ON, "FORCED_ON", "Forced On" },
        { AngularVelocityFixerMode::FORCED_OFF, "FORCED_OFF", "Forced Off" }
    });
};

enum class PerformanceOverlayMode : int32_t {
    DISABLE = 0,
    WINDOW_ONLY = 1,
    WINDOW_AND_VR = 2,
    WINDOW_AND_VR_WITH_PROFILER = 3,
};

template <>
struct EnumTable<PerformanceOverlayMode> {
    static constexpr auto entries = std::to_array<EnumEntry<PerformanceOverlayMode>>({
        { PerformanceOverlayMode::DISABLE, "DISABLE", "Disable" },
        { PerformanceOverlayMode::WINDOW_ONLY, "WINDOW_ONLY", "Only show in Cemu window" },
        { PerformanceOverlayMode::WINDOW_AND_VR, "WINDOW_AND_VR", "Show in both Cemu and VR" },
        { PerformanceOverlayMode::WINDOW_AND_VR_WITH_PROFILER, "WINDOW_AND_VR_WITH_PROFILER", "Show in both Cemu and VR with profiler" }
    });
};

enum class WalkingDirection : int32_t {
    CAMERA = 0,
    CONTROLLER = 1,
};

template <>
struct EnumTable<WalkingDirection> {
    static constexpr auto entries = std::to_array<EnumEntry<WalkingDirection>>({
        { WalkingDirection::CAMERA, "CAMERA", "Camera / Headset" },
        { WalkingDirection::CONTROLLER, "CONTROLLER", "Controller" }
    });
};

enum class SwingSensitivity : int32_t {
    SWING_EASY = 0,
    SWING_NORMAL = 1,
    SWING_CUSTOM = 2,
};

template <>
struct EnumTable<SwingSensitivity> {
    static constexpr auto entries = std::to_array<EnumEntry<SwingSensitivity>>({
        { SwingSensitivity::SWING_EASY, "SWING_EASY", "Relaxed" },
        { SwingSensitivity::SWING_NORMAL, "SWING_NORMAL", "Normal" },
        { SwingSensitivity::SWING_CUSTOM, "SWING_CUSTOM", "Custom" }
    });
};

enum class TurnMode : int32_t {
    SMOOTH_SLOW = 0,
    SMOOTH_NORMAL = 1,
    SMOOTH_FAST = 2,
    SNAP_30 = 3,
    SNAP_45 = 4,
    SNAP_60 = 5,
};

template <>
struct EnumTable<TurnMode> {
    static constexpr auto entries = std::to_array<EnumEntry<TurnMode>>({
        { TurnMode::SMOOTH_SLOW, "SMOOTH_SLOW", "Smooth Turn (Slow)" },
        { TurnMode::SMOOTH_NORMAL, "SMOOTH_NORMAL", "Smooth Turn (Normal)" },
        { TurnMode::SMOOTH_FAST, "SMOOTH_FAST", "Smooth Turn (Fast)" },
        { TurnMode::SNAP_30, "SNAP_30", "30 deg Snap (Recommended)" },
        { TurnMode::SNAP_45, "SNAP_45", "45 deg Snap" },
        { TurnMode::SNAP_60, "SNAP_60", "60 deg Snap" }
    });
};

struct ModSettings {
    static constexpr float kDefaultAxisThreshold = 0.5f;
    static constexpr float kDefaultStickDeadzone = 0.15f;

    // playing mode settings
    EnumSetting<CameraMode> cameraMode{ "CameraMode", CameraMode::FIRST_PERSON };
    EnumSetting<PlayMode> playMode{ "PlayMode", PlayMode::STANDING };
    FloatSetting thirdPlayerDistance{ "ThirdPlayerDistance", 0.5f, 0.0f };
    BoolSetting thirdPersonBowCameraAim{ "ThirdPersonBowCameraAim", true };
    EnumSetting<EventMode> cutsceneCameraMode{ "CutsceneCameraMode", EventMode::FOLLOW_DEFAULT_EVENT_SETTINGS };
    BoolSetting useBlackBarsForCutscenes{ "UseBlackBarsForCutscenes", false };

    // first-person settings
    FloatSetting playerHeightOffset{ "PlayerHeightOffset", 0.0f };
    BoolSetting leftHanded{ "LeftHanded", false };
    BoolSetting uiFollowsGaze{ "UiFollowsGaze", true };
    FloatSetting hudDistance{ "HudDistance", 1.85f, 0.5f, 2.5f };
    FloatSetting hudSize{ "HudSize", 0.85f, 0.4f, 1.75f };
    FloatSetting bowArcOpacity{ "BowArcTransparency", 0.7f, 0.0f, 1.0f };

    // performance settings
    BoolSetting skipDrcRendering{ "SkipDrcRendering", true };
    // Experimental mono-throughput path. It currently mirrors the one rendered eye;
    // depth reprojection is deliberately disabled until its final-eye output passes
    // stereo validation, so this must not be presented as stereoscopic rendering.
    BoolSetting synthesizedRightEye{ "SynthesizedRightEye", false };
    // bits 1-12 of the PPC-side VR_RENDER_SKIP_MASK: skip per-frame render preparation on
    // the right eye pass (see the bit table in patch_RND_StereoRendering_Optimizations.asm);
    // bit 0 is owned by skipDrcRendering. DEFAULT OFF: in the pipelined frame loop the
    // right-eye calc pass rebuilds state the next drawn frame depends on, so plain skips
    // blank the world on alternating frames (visible as heavy flicker). The bits stay
    // exposed for experiments, but need "preserve the skipped pass's outputs" patches
    // (preventModelQueueClear-style) per subsystem before any can ship enabled.
    UIntSetting rightEyeCalcSkipMask{ "RightEyeCalcSkipMask", 0, 0, 0x3FFE };

    // advanced settings
    BoolSetting enableDebuggerTools{ "EnableDebugOverlay", false };
    BoolSetting debugShowEntityBoxesIn3DView{ "DebugShowEntityBoxesIn3DView", false };
    BoolSetting debugShowRoomscalePhysics{ "DebugShowRoomscalePhysics", false };
    BoolSetting debugShowRaycastLines{ "DebugShowRaycastLines", false };
    BoolSetting debugShowWeaponAxes{ "DebugShowWeaponAxes", false };
    BoolSetting alwaysPreventFirstPersonCutsceneCameraMovement{ "AlwaysPreventFirstPersonCutsceneCameraMovement", false };
    BoolSetting preventFirstPersonRagdoll{ "PreventFirstPersonRagdoll", true };
    EnumSetting<AngularVelocityFixerMode> buggyAngularVelocity{ "BuggyAngularVelocity", AngularVelocityFixerMode::AUTO };
    EnumSetting<PerformanceOverlayMode> performanceOverlay{ "PerformanceOverlay", PerformanceOverlayMode::DISABLE };
    UIntSetting performanceOverlayFrequency{ "PerformanceOverlayFrequency", 90 };
    BoolSetting logRendering{ "LogRendering", false, Log::SetCategory<RENDERING> };
    BoolSetting logInterop{ "LogInterop", false, Log::SetCategory<INTEROP> };
    BoolSetting logControls{ "LogControls", false, Log::SetCategory<CONTROLS> };
    BoolSetting logPpc{ "LogPpc", false, Log::SetCategory<PPC> };
    BoolSetting logXrDebugUtils{ "LogXrDebugUtils", false, Log::SetCategory<XR_DEBUGUTILS> };
    BoolSetting logArrowShotCapture{ "LogArrowShotCapture", false, Log::SetCategory<ARROW_SHOT_CAPTURE> };
    BoolSetting logRoomscale{ "LogRoomscale", false, Log::SetCategory<ROOMSCALE> };
    BoolSetting logVerbose{ "LogVerbose", false, Log::SetCategory<VERBOSE> };
    BoolSetting logTimestamps{ "LogTimestamps", true, &Log::SetShowTimestamps };
    BoolSetting logThreadIds{ "LogThreadIds", false, &Log::SetShowThreadIds };
    BoolSetting tutorialPromptShown{ "TutorialPromptShown", false };
    BoolSetting bootDirectlyIntoGame{ "BootDirectlyIntoGame", false };
    StringSetting bootDirectlyTitleId{ "BootDirectlyTitleId", "" };

    // Input settings
    FloatSetting axisThreshold{ "AxisThreshold", kDefaultAxisThreshold, 0.0f, 1.0f };
    FloatSetting stickDeadzone{ "StickDeadzone", kDefaultStickDeadzone, 0.0f, 1.0f };
    EnumSetting<WalkingDirection> walkingDirection{ "WalkingDirection", WalkingDirection::CAMERA };
    EnumSetting<TurnMode> turnMode{ "TurnMode", TurnMode::SMOOTH_NORMAL };
    EnumSetting<SwingSensitivity> swingSensitivity{ "SwingSensitivity", SwingSensitivity::SWING_NORMAL };
    FloatSetting customStabSpeedThreshold{ "CustomStabSpeedThreshold", 0.05f, 0.01f, 0.50f };
    FloatSetting customStabAccThreshold{ "CustomStabAccThreshold", 7.0f, 1.0f, 15.0f };
    FloatSetting customStabSteadinessCone{ "CustomStabSteadinessCone", 30.0f, 15.0f, 85.0f };
    FloatSetting customStabAngularSteadiness{ "CustomStabAngularSteadiness", 4.5f, 1.0f, 15.0f };
    FloatSetting customStabTravelDistance{ "CustomStabTravelDistance", 0.20f, 0.05f, 0.50f };
    FloatSetting customMinGoodStabDuration{ "CustomMinGoodStabDuration", 0.040f, 0.005f, 0.100f };
    FloatSetting customSlashSpeedThreshold{ "CustomSlashSpeedThreshold", 1.5f, 0.1f, 5.0f };
    FloatSetting customSlashAccThreshold{ "CustomSlashAccThreshold", 20.0f, 3.0f, 40.0f };
    FloatSetting customSlashVelocityThreshold{ "CustomSlashVelocityThreshold", 7.0f, 1.0f, 15.0f };
    FloatSetting customSlashAccDriftThreshold{ "CustomSlashAccDriftThreshold", 10.0f, 2.0f, 30.0f };
    FloatSetting customSlashTravelAngle{ "CustomSlashTravelAngle", 36.0f, 10.0f, 90.0f };
    FloatSetting customMinGoodSwingDuration{ "CustomMinGoodSwingDuration", 0.040f, 0.005f, 0.100f };
    FloatSetting customMaxBadDuration{ "CustomMaxBadDuration", 0.022f, 0.005f, 0.100f };
    FloatSetting customGoodSampleGracePeriod{ "CustomGoodSampleGracePeriod", 40.0f, 10.0f, 200.0f };
    FloatSetting customSmoothingTimeConstant{ "CustomSmoothingTimeConstant", 0.020f, 0.005f, 0.100f };
    FloatSetting customAngularDriftMinVelocity{ "CustomAngularDriftMinVelocity", 0.5f, 0.1f, 3.0f };
    FloatSetting customDamageOutputScale{ "CustomDamageOutputScale", 1.0f, 0.10f, 2.00f };

    static std::span<ModSettingBase* const> GetOptions() {
        return SettingRegistry();
    }

    void ResetCustomWeaponSensitivity() {
        customStabSpeedThreshold.Reset();
        customStabAccThreshold.Reset();
        customStabSteadinessCone.Reset();
        customStabAngularSteadiness.Reset();
        customStabTravelDistance.Reset();
        customMinGoodStabDuration.Reset();
        customSlashSpeedThreshold.Reset();
        customSlashAccThreshold.Reset();
        customSlashVelocityThreshold.Reset();
        customSlashAccDriftThreshold.Reset();
        customSlashTravelAngle.Reset();
        customMinGoodSwingDuration.Reset();
        customMaxBadDuration.Reset();
        customGoodSampleGracePeriod.Reset();
        customSmoothingTimeConstant.Reset();
        customAngularDriftMinVelocity.Reset();
        customDamageOutputScale.Reset();
    }

    CameraMode GetCameraMode() const { return cameraMode; }

    PlayMode GetPlayMode() const { return playMode; }
    bool DoesUIFollowGaze() const { return uiFollowsGaze; }
    bool IsLeftHanded() const { return leftHanded; }
    float GetPlayerHeightOffset() const {
        // disable height offset in third-person mode
        return GetCameraMode() == CameraMode::THIRD_PERSON ? 0.0f : playerHeightOffset;
    }
    EventMode GetCutsceneCameraMode() const {
        // if in third-person mode, always use third-person cutscene camera
        return GetCameraMode() == CameraMode::THIRD_PERSON ? EventMode::ALWAYS_THIRD_PERSON : cutsceneCameraMode;
    }
    bool UseBlackBarsForCutscenes() const { return useBlackBarsForCutscenes; }

    float GetBowArcOpacity() const { return bowArcOpacity; }
    bool ShouldAimThirdPersonBowFromCamera() const { return thirdPersonBowCameraAim; }
    bool AlwaysPreventFirstPersonCutsceneCameraMovement() const { return alwaysPreventFirstPersonCutsceneCameraMovement; }
    bool ShouldPreventFirstPersonRagdoll() const { return preventFirstPersonRagdoll; }
    bool ShouldBootDirectlyIntoGame() const { return bootDirectlyIntoGame; }
    AngularVelocityFixerMode AngularVelocityFixer_GetMode() const { return buggyAngularVelocity; }
    SwingSensitivity GetSwingSensitivity() const { return swingSensitivity; }
    WalkingDirection GetWalkingDirection() const { return walkingDirection; }
    int32_t GetSnapTurnAngle() const {
        switch (turnMode.load()) {
            case TurnMode::SNAP_30: return 30;
            case TurnMode::SNAP_45: return 45;
            case TurnMode::SNAP_60: return 60;
            default: return 0;
        }
    }
    float GetSmoothTurnSpeed() const {
        switch (turnMode.load()) {
            case TurnMode::SMOOTH_SLOW: return 60.0f;
            case TurnMode::SMOOTH_NORMAL: return 120.0f;
            case TurnMode::SMOOTH_FAST: return 240.0f;
            default: return 0.0f;
        }
    }
    float GetWeaponDamageOutputScale() const { return GetSwingSensitivity() == SwingSensitivity::SWING_CUSTOM ? customDamageOutputScale : 1.0f; }

    bool ShouldSkipDrcRendering() const { return skipDrcRendering; }
    bool UseSynthesizedRightEye() const { return synthesizedRightEye; }
    uint32_t GetRenderSkipMask() const { return (rightEyeCalcSkipMask & 0x3FFEu) | (skipDrcRendering ? 1u : 0u); }

    bool IsDebuggingToolsEnabled() const;
    bool ShouldShowRoomPhysics() const { return IsDebuggingToolsEnabled() && debugShowRoomscalePhysics; }
    bool ShouldShowEntityBoxesIn3DView() const { return IsDebuggingToolsEnabled() && debugShowEntityBoxesIn3DView; }
    bool ShouldShowRaycastLines() const { return IsDebuggingToolsEnabled() && debugShowRaycastLines; }
    bool ShouldShowWeaponAxes() const { return IsDebuggingToolsEnabled() && debugShowWeaponAxes; }

    // By default BotW's camera uses 0.1f for near plane and 25000.0f for far plane, except maybe some indoor areas? But for simplicity, we'll use the default values everywhere.
    float GetZNear() const { return 0.1f; }
    float GetZFar() const { return 25000.0f; }

    std::string ToString() const {
        std::string buffer = "";
        std::format_to(std::back_inserter(buffer), " - Camera Mode: {}\n", EnumDisplayName(GetCameraMode()));
        std::format_to(std::back_inserter(buffer), " - Left Handed: {}\n", IsLeftHanded() ? "Yes" : "No");
        std::format_to(std::back_inserter(buffer), " - Player Height: {} meters\n", GetPlayerHeightOffset());
        return buffer;
    }
};

extern ModSettings& GetSettings();
extern void InitSettings();
extern void Settings_LogSavedSettings();
