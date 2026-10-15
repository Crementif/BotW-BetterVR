#include "pch.h"
#include "weapon_attack_debugger.h"
#include "cemu/cemu_host.h"
#include "game/botw/player/weapon.h"

namespace WeaponAttackDebugger {
    struct WeaponLiveDebugState {
        bool hasWeapon = false;
        uint32_t weaponPtr = 0;
        std::string actorName;
        float baseDamage = 0.0f;
        float multiplier = 1.0f;
        float estimatedDamage = 0.0f;
        uint32_t setupDamage = 0;
        uint32_t finalizedDamage = 0;
        uint32_t damageMgrDamage = 0;
    };

    struct AttackPeriodSummary {
        AttackType attackType = AttackType::None;
        float damageIntegral = 0.0f;
        float duration = 0.0f;
        float peakDamage = 0.0f;
    };

    struct AttackPeriodTracker {
        bool wasActive = false;
        XrTime previousSampleTime = 0;
        AttackType currentAttackType = AttackType::None;
        float currentDamageIntegral = 0.0f;
        float currentDuration = 0.0f;
        float currentPeakDamage = 0.0f;
        std::array<AttackPeriodSummary, 3> history = {};
        size_t historyCount = 0;
    };

    constexpr ImVec4 kPassColor = ImVec4(0.35f, 0.95f, 0.45f, 1.0f);
    constexpr ImVec4 kFailColor = ImVec4(0.95f, 0.35f, 0.35f, 1.0f);
    constexpr ImVec4 kNeutralColor = ImVec4(0.90f, 0.85f, 0.45f, 1.0f);
    constexpr uint32_t kMotionPlotSampleCount = 30;
    static std::array<AttackPeriodTracker, 2> s_attackPeriodTrackers = {};
    static std::array<bool, 2> s_showWeaponSensitivityOverlay = { false, false };

    static const char* ToString(AttackType attackType) {
        switch (attackType) {
            case AttackType::Slash:
                return "Slash";
            case AttackType::Stab:
                return "Stab";
            default:
                return "None";
        }
    }

    static const char* ToString(WeaponType weaponType) {
        switch (weaponType) {
            case WeaponType::SmallSword:
                return "Small Sword";
            case WeaponType::LargeSword:
                return "Large Sword";
            case WeaponType::Spear:
                return "Spear";
            case WeaponType::Bow:
                return "Bow";
            case WeaponType::Shield:
                return "Shield";
            default:
                return "Unknown";
        }
    }

    static const char* ToString(OpenXR::EyeSide side) {
        return side == EyeSide::LEFT ? "Left Hand" : "Right Hand";
    }

    static void DrawMotionPlots(OpenXR::EyeSide side, const WeaponMotionAnalyser& analyser);

    static void DrawStatusValue(const char* label, float value, float threshold, const char* unit, bool passed, bool higherIsBetter = true) {
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%s", label);
        ImGui::SameLine(220.0f);
        const char* comparator = higherIsBetter ? ">=" : "<=";
        ImGui::TextColored(passed ? kPassColor : kFailColor, "%.3f %s  %s  %.3f %s", value, unit, comparator, threshold, unit);
    }

    static void DrawStatusText(const char* label, const char* value, const ImVec4& color) {
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%s", label);
        ImGui::SameLine(220.0f);
        ImGui::TextColored(color, "%s", value);
    }

    static float ClampProgress(float value, float maxValue) {
        if (maxValue <= 0.0f) {
            return 0.0f;
        }
        return std::clamp(value / maxValue, 0.0f, 1.0f);
    }

    static WeaponLiveDebugState ReadWeaponLiveDebugState(uint32_t weaponPtr) {
        WeaponLiveDebugState state = {};
        if (weaponPtr == 0) {
            return state;
        }

        Weapon weapon = {};
        Guest::Read(weaponPtr, &weapon);

        state.hasWeapon = true;
        state.weaponPtr = weaponPtr;
        state.actorName = weapon.name.getLE();
        state.setupDamage = weapon.setupAttackSensor.powerForPlayers.getLE();
        state.finalizedDamage = weapon.finalizedAttackSensor.powerForPlayers.getLE();
        state.multiplier = weapon.finalizedAttackSensor.multiplier.getLE();
        if (state.multiplier <= 0.0f) {
            state.multiplier = weapon.setupAttackSensor.multiplier.getLE();
        }
        if (state.multiplier <= 0.0f) {
            state.multiplier = 1.0f;
        }

        if (const uint32_t damageMgrPtr = weapon.damageMgrPtr.getLE(); damageMgrPtr != 0) {
            DamageMgr damageMgr = {};
            Guest::Read(damageMgrPtr, &damageMgr);
            state.damageMgrDamage = damageMgr.damage.getLE();
        }

        state.baseDamage = (float)state.finalizedDamage;
        if (state.baseDamage <= 0.0f) {
            state.baseDamage = (float)state.setupDamage;
        }
        if (state.baseDamage <= 0.0f) {
            state.baseDamage = (float)state.damageMgrDamage;
        }

        state.estimatedDamage = state.baseDamage * state.multiplier;
        return state;
    }

    static void PushHistoryEntry(AttackPeriodTracker& tracker) {
        if (tracker.currentDuration <= 0.0f && tracker.currentDamageIntegral <= 0.0f && tracker.currentPeakDamage <= 0.0f) {
            return;
        }

        for (size_t i = tracker.history.size() - 1; i > 0; --i) {
            tracker.history[i] = tracker.history[i - 1];
        }

        tracker.history[0] = AttackPeriodSummary {
            .attackType = tracker.currentAttackType,
            .damageIntegral = tracker.currentDamageIntegral,
            .duration = tracker.currentDuration,
            .peakDamage = tracker.currentPeakDamage,
        };
        tracker.historyCount = std::min(tracker.historyCount + 1, tracker.history.size());
    }

    static void UpdateAttackPeriodTracker(OpenXR::EyeSide side, const WeaponDebugSnapshot& snapshot, const WeaponLiveDebugState& liveState) {
        auto& tracker = s_attackPeriodTrackers[side];
        const bool isActive = snapshot.attackActive || snapshot.hitboxEnabled;
        const float liveDamage = liveState.estimatedDamage;

        if (isActive && tracker.wasActive && snapshot.sampleTime > tracker.previousSampleTime) {
            const float dt = std::clamp((float)(snapshot.sampleTime - tracker.previousSampleTime) / 1e9f, 0.0f, 0.25f);
            tracker.currentDamageIntegral += liveDamage * dt;
            tracker.currentDuration += dt;
        }

        if (isActive) {
            tracker.currentAttackType = snapshot.lockedAttackType;
            tracker.currentPeakDamage = std::max(tracker.currentPeakDamage, liveDamage);
            tracker.previousSampleTime = snapshot.sampleTime;
        }
        else if (tracker.wasActive) {
            PushHistoryEntry(tracker);
            tracker.currentAttackType = AttackType::None;
            tracker.currentDamageIntegral = 0.0f;
            tracker.currentDuration = 0.0f;
            tracker.currentPeakDamage = 0.0f;
            tracker.previousSampleTime = snapshot.sampleTime;
        }

        if (!tracker.wasActive && isActive) {
            tracker.currentAttackType = snapshot.lockedAttackType;
            tracker.currentDamageIntegral = 0.0f;
            tracker.currentDuration = 0.0f;
            tracker.currentPeakDamage = liveDamage;
            tracker.previousSampleTime = snapshot.sampleTime;
        }

        if (!isActive && !tracker.wasActive) {
            tracker.previousSampleTime = snapshot.sampleTime;
        }

        tracker.wasActive = isActive;
    }

    static void DrawAttackPeriodSummary(OpenXR::EyeSide side) {
        const auto& tracker = s_attackPeriodTrackers[side];
        ImGui::Text("Current period: %s | %.3f dmg*s | %.3f s | peak %.1f",
            ToString(tracker.currentAttackType),
            tracker.currentDamageIntegral,
            tracker.currentDuration,
            tracker.currentPeakDamage
        );

        if (tracker.historyCount == 0) {
            ImGui::TextDisabled("Last 3 periods: none yet");
            return;
        }

        ImGui::Text("Last 3 periods:");
        for (size_t i = 0; i < tracker.historyCount; ++i) {
            const auto& entry = tracker.history[i];
            ImGui::BulletText("%s | %.3f dmg*s | %.3f s | peak %.1f",
                ToString(entry.attackType),
                entry.damageIntegral,
                entry.duration,
                entry.peakDamage
            );
        }
    }

    static void DrawSensitivityPanel(OpenXR::EyeSide side, const WeaponMotionAnalyser& analyser) {
        const WeaponDebugSnapshot snapshot = analyser.GetDebugSnapshot();
        const WeaponLiveDebugState liveState = ReadWeaponLiveDebugState(Weapons::GetHeldActor(side));
        UpdateAttackPeriodTracker(side, snapshot, liveState);

        ImGui::PushID((int)side);
        DrawStatusText("Held weapon", liveState.hasWeapon ? liveState.actorName.c_str() : "None", liveState.hasWeapon ? kPassColor : kNeutralColor);
        ImGui::Text("Weapon type: %s | Attack: %s | Active: %s | Hitbox: %s",
            ToString(snapshot.weaponType),
            ToString(snapshot.lockedAttackType),
            snapshot.attackActive ? "Yes" : "No",
            snapshot.hitboxEnabled ? "Yes" : "No"
        );
        ImGui::Text("Current damage: %.1f | Multiplier: %.2f | Estimated output: %.1f",
            liveState.baseDamage,
            liveState.multiplier,
            liveState.estimatedDamage
        );
        ImGui::Text("Bad samples: %.3f / %.3f s | Swing power: %.2f | Arm calibration: %.3f m",
            snapshot.badSampleAccumTime,
            snapshot.maxBadDuration,
            snapshot.swingPower,
            snapshot.calibratedArmLength
        );

        if (ImGui::CollapsingHeader("Stab", ImGuiTreeNodeFlags_DefaultOpen)) {
            DrawStatusValue("Stab speed", snapshot.forwardStabSpeed, snapshot.stabSpeedThreshold, "m/s", snapshot.stabSpeedOk);
            DrawStatusValue("Stab acceleration", snapshot.forwardStabAcceleration, snapshot.stabAccThreshold, "m/s^2", snapshot.stabAccelOk);
            DrawStatusValue("Stab direction", snapshot.stabDirectionAbsZ, snapshot.stabLinearSteadinessThreshold, "ratio", snapshot.stabDirectionOk);
            DrawStatusValue("Stab angular steadiness", snapshot.stabAngularXY, snapshot.stabAngularSteadinessThreshold, "rad/s", snapshot.stabAngularOk, false);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Stab duration");
            ImGui::SameLine(220.0f);
            ImGui::TextColored(snapshot.stabDurationOk ? kPassColor : kFailColor, "%.3f s / %.3f s", snapshot.goodStabAccumTime, snapshot.minGoodStabDuration);
            ImGui::SameLine();
            ImGui::ProgressBar(ClampProgress(snapshot.goodStabAccumTime, snapshot.minGoodStabDuration), ImVec2(120.0f, 0.0f));
        }

        if (ImGui::CollapsingHeader("Swing", ImGuiTreeNodeFlags_DefaultOpen)) {
            DrawStatusValue("Swing speed", snapshot.slashAngularSpeed, snapshot.slashSpeedThreshold, "rad/s", snapshot.slashSpeedOk);
            DrawStatusValue("Swing acceleration", snapshot.slashAngularAcceleration, snapshot.slashAccThreshold, "rad/s^2", snapshot.slashAccelOk);
            DrawStatusValue("Swing velocity gate", snapshot.slashAngularSpeed, snapshot.slashVelocityThreshold, "rad/s", snapshot.slashVelocityOk);
            DrawStatusValue("Swing drift", snapshot.slashAngularDrift, snapshot.slashAccDriftThreshold, "rad/s^2", snapshot.slashDriftOk, false);
            ImGui::AlignTextToFramePadding();
            ImGui::Text("Swing duration");
            ImGui::SameLine(220.0f);
            ImGui::TextColored(snapshot.slashDurationOk ? kPassColor : kFailColor, "%.3f s / %.3f s", snapshot.goodSwingAccumTime, snapshot.minGoodSwingDuration);
            ImGui::SameLine();
            ImGui::ProgressBar(ClampProgress(snapshot.goodSwingAccumTime, snapshot.minGoodSwingDuration), ImVec2(120.0f, 0.0f));
        }

        ImGui::SeparatorText("Attack Periods");
        DrawAttackPeriodSummary(side);

        if (ImGui::CollapsingHeader("Motion Graphs")) {
            DrawMotionPlots(side, analyser);
        }

        ImGui::PopID();
    }

    static void DrawMotionPlots(OpenXR::EyeSide side, const WeaponMotionAnalyser& analyser) {
        const WeaponDebugSnapshot snapshot = analyser.GetDebugSnapshot();
        const auto& samples = analyser.GetSamples();
        const uint32_t lastSampleIdx = analyser.GetLastSampleIndex();
        const auto oldestIdx = [&](uint32_t j) { return (lastSampleIdx + 1 + WeaponMotionAnalyser::MAX_SAMPLES - kMotionPlotSampleCount + j) % WeaponMotionAnalyser::MAX_SAMPLES; };

        ImGui::PushID((int)side);
        ImGui::Text("%s", ToString(side));
        ImGui::Text("Weapon: %s | Showing last %u samples", ToString(snapshot.weaponType), kMotionPlotSampleCount);
        ImGui::Text("Attack: %s | Active: %s | Bad: %.3f s | Power: %.2f",
            ToString(snapshot.lockedAttackType),
            snapshot.attackActive ? "Yes" : "No",
            snapshot.badSampleAccumTime,
            snapshot.swingPower
        );

        {
            std::array<float, kMotionPlotSampleCount> t{}, avX{}, avY{}, avZ{}, maskSlash{}, maskStab{};
            for (uint32_t j = 0; j < kMotionPlotSampleCount; ++j) {
                const auto& s = samples[oldestIdx(j)];
                t[j] = static_cast<float>(j);
                avX[j] = s.localAngularVelocity.x;
                avY[j] = s.localAngularVelocity.y;
                avZ[j] = s.localAngularVelocity.z;
                maskSlash[j] = (s.attackType == AttackType::Slash) ? 100.0f : -100.0f;
                maskStab[j] = (s.attackType == AttackType::Stab) ? 100.0f : -100.0f;
            }

            if (ImPlot::BeginPlot("Weapon Steadiness", ImVec2(-1.0f, 120.0f), ImPlotFlags_NoTitle)) {
                ImPlot::SetupAxes("Sample", "Angular Velocity", ImPlotAxisFlags_Lock, ImPlotAxisFlags_Lock);
                ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMotionPlotSampleCount - 1, ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -15, 15, ImPlotCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 0.3f), 3);
                ImPlot::PlotShaded("Slash", t.data(), maskSlash.data(), kMotionPlotSampleCount, -100.0f);
                ImPlot::SetNextLineStyle(ImVec4(1, 0.5f, 0, 0.3f), 3);
                ImPlot::PlotShaded("Stab", t.data(), maskStab.data(), kMotionPlotSampleCount, -100.0f);

                ImPlot::SetNextLineStyle(ImVec4(0, 1, 0.5f, 0.5f), 2);
                ImPlot::PlotLine("X", t.data(), avX.data(), kMotionPlotSampleCount);
                ImPlot::PlotLine("Y", t.data(), avY.data(), kMotionPlotSampleCount);
                ImPlot::PlotLine("Z", t.data(), avZ.data(), kMotionPlotSampleCount);
                ImPlot::EndPlot();
            }
        }

        {
            std::array<float, kMotionPlotSampleCount> t{}, lvX{}, lvY{}, lvZ{}, accZ{}, maskSlash{}, maskStab{};
            for (uint32_t j = 0; j < kMotionPlotSampleCount; ++j) {
                const auto& s = samples[oldestIdx(j)];
                t[j] = static_cast<float>(j);
                lvX[j] = s.localLinearVelocity.x;
                lvY[j] = s.localLinearVelocity.y;
                lvZ[j] = s.localLinearVelocity.z;
                accZ[j] = s.smoothedLinearAcceleration.z;
                maskSlash[j] = (s.attackType == AttackType::Slash) ? 100.0f : -100.0f;
                maskStab[j] = (s.attackType == AttackType::Stab) ? 100.0f : -100.0f;
            }

            if (ImPlot::BeginPlot("Controller Linear Velocity", ImVec2(-1.0f, 120.0f), ImPlotFlags_NoTitle)) {
                ImPlot::SetupAxes("Sample", "Linear Velocity", ImPlotAxisFlags_Lock, ImPlotAxisFlags_Lock);
                ImPlot::SetupAxisLimits(ImAxis_X1, 0, kMotionPlotSampleCount - 1, ImPlotCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -6, 6, ImPlotCond_Always);

                ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 0.01f), 3);
                ImPlot::PlotShaded("Slash", t.data(), maskSlash.data(), kMotionPlotSampleCount, -100.0f);
                ImPlot::SetNextLineStyle(ImVec4(1, 0.5f, 0, 0.01f), 3);
                ImPlot::PlotShaded("Stab", t.data(), maskStab.data(), kMotionPlotSampleCount, -100.0f);

                ImPlot::SetNextLineStyle(ImVec4(0, 1, 0.5f, 0.5f), 2);
                ImPlot::PlotLine("X", t.data(), lvX.data(), kMotionPlotSampleCount);
                ImPlot::PlotLine("Y", t.data(), lvY.data(), kMotionPlotSampleCount);
                ImPlot::PlotLine("Z", t.data(), lvZ.data(), kMotionPlotSampleCount);
                ImPlot::PlotLine("Acc Z (smoothed)", t.data(), accZ.data(), kMotionPlotSampleCount);
                ImPlot::EndPlot();
            }
        }

        if (side == EyeSide::LEFT) {
            ImGui::Separator();
        }
        ImGui::PopID();
    }
    void DrawOverlays() {
        for (uint8_t sideIdx = 0; sideIdx < s_showWeaponSensitivityOverlay.size(); ++sideIdx) {
            if (!s_showWeaponSensitivityOverlay[sideIdx]) {
                continue;
            }

            OpenXR::EyeSide side = (OpenXR::EyeSide)sideIdx;
            std::string windowName = fmt::format("Attack Debugger - {}", ToString(side));
            if (ImGui::Begin(windowName.c_str(), &s_showWeaponSensitivityOverlay[sideIdx])) {
                DrawSensitivityPanel(side, Weapons::GetMotionAnalyser(side));
            }
            ImGui::End();
        }
    }

    bool IsOverlayVisible(OpenXR::EyeSide side) {
        return s_showWeaponSensitivityOverlay[side];
    }

    void SetOverlayVisible(OpenXR::EyeSide side, bool visible) {
        s_showWeaponSensitivityOverlay[side] = visible;
    }
}
