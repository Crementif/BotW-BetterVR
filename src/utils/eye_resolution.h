#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace EyeResolution {
    inline constexpr uint32_t kSearchAlignment = 8;
    inline constexpr uint32_t kSearchRadius = 32;
    inline constexpr double kPreferredMatchDistance = 32.0;
    inline constexpr double kMinimumWidth = 1280.0;
    inline constexpr double kMinimumHeight = 720.0;

    struct TextureRedefineRule {
        uint32_t width;
        uint32_t height;
        double weight;
        double heightBias;
    };

    // the pack's texture rules, which scale with the eye resolution and should land on whole pixels
    inline constexpr std::array<TextureRedefineRule, 12> kTextureRedefineRules = {{
        { 640, 368, 8.0, 0.49 },
        { 640, 360, 8.0, 0.0 },
        { 384, 192, 7.0, 0.0 },
        { 320, 192, 7.0, 0.0 },
        { 320, 180, 7.0, 0.0 },
        { 192, 96, 6.0, 0.0 },
        { 160, 96, 6.0, 0.0 },
        { 160, 90, 6.0, 0.0 },
        { 128, 48, 5.0, 0.0 },
        { 96, 48, 5.0, 0.0 },
        { 80, 45, 4.0, 0.0 },
        { 64, 64, 4.0, 0.0 },
    }};

    struct Preset {
        const char* nameToken;
        const char* widthToken;
        const char* heightToken;
        double multiplier;
        const char* multiplierLabel;
    };

    inline constexpr Preset kPresets[] = {
        { "__BETTERVR_05_NAME__", "__BETTERVR_05_WIDTH__", "__BETTERVR_05_HEIGHT__", 0.5, "0.5x" },
        { "__BETTERVR_075_NAME__", "__BETTERVR_075_WIDTH__", "__BETTERVR_075_HEIGHT__", 0.75, "0.75x" },
        { "__BETTERVR_10_NAME__", "__BETTERVR_10_WIDTH__", "__BETTERVR_10_HEIGHT__", 1.0, "1x" },
        { "__BETTERVR_125_NAME__", "__BETTERVR_125_WIDTH__", "__BETTERVR_125_HEIGHT__", 1.25, "1.25x" },
        { "__BETTERVR_15_NAME__", "__BETTERVR_15_WIDTH__", "__BETTERVR_15_HEIGHT__", 1.5, "1.5x" },
        { "__BETTERVR_175_NAME__", "__BETTERVR_175_WIDTH__", "__BETTERVR_175_HEIGHT__", 1.75, "1.75x" },
        { "__BETTERVR_20_NAME__", "__BETTERVR_20_WIDTH__", "__BETTERVR_20_HEIGHT__", 2.0, "2x" },
        { "__BETTERVR_225_NAME__", "__BETTERVR_225_WIDTH__", "__BETTERVR_225_HEIGHT__", 2.25, "2.25x" },
        { "__BETTERVR_30_NAME__", "__BETTERVR_30_WIDTH__", "__BETTERVR_30_HEIGHT__", 3.0, "3x" },
    };

    inline double GetTextureRedefinePenalty(uint32_t width, uint32_t height) {
        const auto distanceToInteger = [](double value) { return std::abs(value - std::round(value)); };
        double penalty = 0.0;
        for (const TextureRedefineRule& rule : kTextureRedefineRules) {
            const double overwriteWidth = (double(width) / 1280.0) * rule.width;
            const double overwriteHeight = ((double(height) / 720.0) * rule.height) + rule.heightBias;
            penalty += (distanceToInteger(overwriteWidth) + distanceToInteger(overwriteHeight)) * rule.weight;
        }
        return penalty;
    }

    inline std::pair<uint32_t, uint32_t> GetNearestRulesCompatible(double width, double height, double minimumWidth) {
        if (width <= 0.0 || height <= 0.0) {
            return { 0, 0 };
        }

        const double minimumScale = std::max({ 1.0, minimumWidth / width, kMinimumHeight / height });
        width *= minimumScale;
        height *= minimumScale;

        struct Candidate {
            uint32_t width;
            uint32_t height;
            double distance;
            double penalty;
        };

        const auto evaluateCandidate = [width, height](uint32_t candidateWidth, uint32_t candidateHeight) {
            const double distance = std::hypot(double(candidateWidth) - width, double(candidateHeight) - height);
            return Candidate{ candidateWidth, candidateHeight, distance, GetTextureRedefinePenalty(candidateWidth, candidateHeight) };
        };
        const auto alignDown = [](uint32_t value) { return value - (value % kSearchAlignment); };

        const uint32_t targetWidth = std::max(1u, static_cast<uint32_t>(std::llround(width)));
        const uint32_t targetHeight = std::max(1u, static_cast<uint32_t>(std::llround(height)));
        const Candidate targetCandidate = evaluateCandidate(targetWidth, targetHeight);
        Candidate bestNearbyMatch = targetCandidate;

        const uint32_t searchStartWidth = std::max((uint32_t)minimumWidth, alignDown(targetWidth > kSearchRadius ? targetWidth - kSearchRadius : kSearchAlignment));
        const uint32_t searchStartHeight = std::max(kSearchAlignment, alignDown(targetHeight > kSearchRadius ? targetHeight - kSearchRadius : kSearchAlignment));
        const uint32_t searchEndWidth = targetWidth + kSearchRadius;
        const uint32_t searchEndHeight = targetHeight + kSearchRadius;

        for (uint32_t candidateWidth = searchStartWidth; candidateWidth <= searchEndWidth; candidateWidth += kSearchAlignment) {
            for (uint32_t candidateHeight = searchStartHeight; candidateHeight <= searchEndHeight; candidateHeight += kSearchAlignment) {
                const Candidate candidate = evaluateCandidate(candidateWidth, candidateHeight);
                if (candidate.distance <= kPreferredMatchDistance && (candidate.penalty < bestNearbyMatch.penalty || (candidate.penalty == bestNearbyMatch.penalty && candidate.distance < bestNearbyMatch.distance))) {
                    bestNearbyMatch = candidate;
                }
            }
        }

        if (bestNearbyMatch.penalty < targetCandidate.penalty) {
            return { bestNearbyMatch.width, bestNearbyMatch.height };
        }
        return { targetCandidate.width, targetCandidate.height };
    }

    inline bool ReplaceToken(std::string& contents, std::string_view token, const std::string& value) {
        bool replacedAny = false;
        size_t searchPosition = 0;
        while ((searchPosition = contents.find(token, searchPosition)) != std::string::npos) {
            contents.replace(searchPosition, token.size(), value);
            searchPosition += value.size();
            replacedAny = true;
        }
        return replacedAny;
    }

    // fills in the Graphics pack's presets for the headset's eye resolution, wide presets are widened to 16:9 for Recording Mode
    // onPreset(preset, "<width>x<height>") for each preset. False when the rules had no placeholders.
    template <typename OnPreset>
    bool FillGraphicsRules(std::string& rules, uint32_t eyeWidth, uint32_t eyeHeight, double minimumWidth, OnPreset&& onPreset) {
        bool replacedAny = false;
        for (const Preset& preset : kPresets) {
            const auto [width, height] = GetNearestRulesCompatible(double(eyeWidth) * preset.multiplier, double(eyeHeight) * preset.multiplier, minimumWidth);
            const std::string widthText = std::to_string(width);
            const std::string heightText = std::to_string(height);
            const std::string label = widthText + "x" + heightText;
            onPreset(preset, label);
            replacedAny |= ReplaceToken(rules, preset.nameToken, label);
            replacedAny |= ReplaceToken(rules, preset.widthToken, widthText);
            replacedAny |= ReplaceToken(rules, preset.heightToken, heightText);
        }
        return replacedAny;
    }
}
