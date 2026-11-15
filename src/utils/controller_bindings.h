#pragma once

enum class ControllerType : uint8_t {
    Unknown,
    Simple,
    OculusTouch,
    ValveIndex,
    Pico4,
    HPReverbG2,
    ViveCosmos,
    SteamFrame,
};

struct ControllerActionBindings {
    XrAction inGameGripPoseAction = XR_NULL_HANDLE;
    XrAction inGameAimPoseAction = XR_NULL_HANDLE;
    XrAction inMenuGripPoseAction = XR_NULL_HANDLE;
    XrAction inMenuAimPoseAction = XR_NULL_HANDLE;
    XrAction moveAction = XR_NULL_HANDLE;
    XrAction cameraAction = XR_NULL_HANDLE;
    XrAction grab_interactAction = XR_NULL_HANDLE;
    XrAction jumpAction = XR_NULL_HANDLE;
    XrAction run_interactAction = XR_NULL_HANDLE;
    XrAction useRune_dpadMenu_Action = XR_NULL_HANDLE;
    XrAction inGame_modMenuAction = XR_NULL_HANDLE;
    XrAction useLeftItemAction = XR_NULL_HANDLE;
    XrAction useRightItemAction = XR_NULL_HANDLE;
    XrAction crouch_scopeAction = XR_NULL_HANDLE;
    XrAction inGame_inventory_mapAction = XR_NULL_HANDLE;
    XrAction rumbleAction = XR_NULL_HANDLE;
    XrAction scrollAction = XR_NULL_HANDLE;
    XrAction navigateAction = XR_NULL_HANDLE;
    XrAction selectAction = XR_NULL_HANDLE;
    XrAction backAction = XR_NULL_HANDLE;
    XrAction sortAction = XR_NULL_HANDLE;
    XrAction holdAction = XR_NULL_HANDLE;
    XrAction leftGripAction = XR_NULL_HANDLE;
    XrAction rightGripAction = XR_NULL_HANDLE;
    XrAction leftTriggerAction = XR_NULL_HANDLE;
    XrAction rightTriggerAction = XR_NULL_HANDLE;
    XrAction inMenu_modMenuAction = XR_NULL_HANDLE;
    XrAction inMenu_inventory_mapAction = XR_NULL_HANDLE;
    XrAction cancelAction = XR_NULL_HANDLE;
    XrAction sprintAction = XR_NULL_HANDLE;
    XrAction scopeAction = XR_NULL_HANDLE;
    std::array<XrAction, 4> inGameDpadActions = {};
    std::array<XrAction, 4> inMenuDpadActions = {};
};

static XrPath GetPath(XrInstance instance, const char* str) {
    XrPath path;
    checkXRResult(xrStringToPath(instance, str, &path), fmt::format("Failed to get path for {}", str).c_str());
    return path;
}

static void SuggestProfileBindings(XrInstance instance, const char* interactionProfile, const XrActionSuggestedBinding* bindings, uint32_t bindingCount, bool fatalOnUnsupported) {
    XrInteractionProfileSuggestedBinding suggestedBindingsInfo = { XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
    suggestedBindingsInfo.interactionProfile = GetPath(instance, interactionProfile);
    suggestedBindingsInfo.countSuggestedBindings = bindingCount;
    suggestedBindingsInfo.suggestedBindings = bindings;

    XrResult result = xrSuggestInteractionProfileBindings(instance, &suggestedBindingsInfo);
    if (!fatalOnUnsupported && result == XR_ERROR_PATH_UNSUPPORTED) {
        Log::print<INFO>("Runtime doesn't support the {} interaction profile - skipping its suggested bindings.", interactionProfile);
        return;
    }
    checkXRResult(result, fmt::format("Failed to suggest bindings for the {} interaction profile!", interactionProfile).c_str());
}

inline void SuggestControllerBindings(XrInstance instance, const ControllerActionBindings& a, bool enablePicoBindings = true, bool enablePicoUltraBindings = true, bool enableCosmosBindings = true, bool enableHPMixedRealityBindings = true, bool enableFrameBindings = true) {
    {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/menu/click") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/select/click") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/select/click") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/menu/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/select/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/menu/click") }
        };
        SuggestProfileBindings(instance, "/interaction_profiles/khr/simple_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
    }

    {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
        };
        SuggestProfileBindings(instance, "/interaction_profiles/oculus/touch_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
    }

    {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/b/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/a/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/b/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/a/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/force") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/a/click") },
        };
        SuggestProfileBindings(instance, "/interaction_profiles/valve/index_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
    }

    if (enablePicoBindings || enablePicoUltraBindings) {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
        };
        if (enablePicoBindings) {
            SuggestProfileBindings(instance, "/interaction_profiles/bytedance/pico4_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
            SuggestProfileBindings(instance, "/interaction_profiles/bytedance/pico_neo3_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), false);
        }
        if (enablePicoUltraBindings) {
            SuggestProfileBindings(instance, "/interaction_profiles/bytedance/pico_ultra_controller_bd", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), false);
        }
    }

    if (enableHPMixedRealityBindings) {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
        };
        SuggestProfileBindings(instance, "/interaction_profiles/hp/mixed_reality_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
    }

    if (enableCosmosBindings) {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/click") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/click") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/left/input/y/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/click") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/click") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/x/click") },
        };
        SuggestProfileBindings(instance, "/interaction_profiles/htc/vive_cosmos_controller", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), true);
    }

    if (enableFrameBindings) {
        std::array suggestedBindings = {
            // === gameplay suggestions ===
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inGameAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.moveAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.cameraAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },

            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.grab_interactAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.run_interactAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.cancelAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.jumpAction, .binding = GetPath(instance, "/user/hand/right/input/x/click") },
            XrActionSuggestedBinding{ .action = a.useRune_dpadMenu_Action, .binding = GetPath(instance, "/user/hand/left/input/bumper/click") },
            XrActionSuggestedBinding{ .action = a.sprintAction, .binding = GetPath(instance, "/user/hand/right/input/bumper/click") },
            XrActionSuggestedBinding{ .action = a.inGame_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/view/click") },

            XrActionSuggestedBinding{ .action = a.useLeftItemAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.useRightItemAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },

            XrActionSuggestedBinding{ .action = a.crouch_scopeAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.scopeAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick/click") },
            XrActionSuggestedBinding{ .action = a.inGame_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/menu/click") },

            XrActionSuggestedBinding{ .action = a.inGameDpadActions[0], .binding = GetPath(instance, "/user/hand/left/input/dpad_up/click") },
            XrActionSuggestedBinding{ .action = a.inGameDpadActions[1], .binding = GetPath(instance, "/user/hand/left/input/dpad_right/click") },
            XrActionSuggestedBinding{ .action = a.inGameDpadActions[2], .binding = GetPath(instance, "/user/hand/left/input/dpad_down/click") },
            XrActionSuggestedBinding{ .action = a.inGameDpadActions[3], .binding = GetPath(instance, "/user/hand/left/input/dpad_left/click") },

            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/left/output/haptic") },
            XrActionSuggestedBinding{ .action = a.rumbleAction, .binding = GetPath(instance, "/user/hand/right/output/haptic") },

            // === menu suggestions ===
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/left/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuGripPoseAction, .binding = GetPath(instance, "/user/hand/right/input/grip/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/left/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.inMenuAimPoseAction, .binding = GetPath(instance, "/user/hand/right/input/aim/pose") },
            XrActionSuggestedBinding{ .action = a.scrollAction, .binding = GetPath(instance, "/user/hand/right/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.navigateAction, .binding = GetPath(instance, "/user/hand/left/input/thumbstick") },
            XrActionSuggestedBinding{ .action = a.selectAction, .binding = GetPath(instance, "/user/hand/right/input/a/click") },
            XrActionSuggestedBinding{ .action = a.backAction, .binding = GetPath(instance, "/user/hand/right/input/b/click") },
            XrActionSuggestedBinding{ .action = a.holdAction, .binding = GetPath(instance, "/user/hand/right/input/x/click") },
            XrActionSuggestedBinding{ .action = a.sortAction, .binding = GetPath(instance, "/user/hand/right/input/y/click") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.leftGripAction, .binding = GetPath(instance, "/user/hand/left/input/bumper/click") },
            XrActionSuggestedBinding{ .action = a.rightGripAction, .binding = GetPath(instance, "/user/hand/right/input/squeeze/value") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.leftTriggerAction, .binding = GetPath(instance, "/user/hand/left/input/bumper/click") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/trigger/value") },
            XrActionSuggestedBinding{ .action = a.rightTriggerAction, .binding = GetPath(instance, "/user/hand/right/input/bumper/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_inventory_mapAction, .binding = GetPath(instance, "/user/hand/right/input/menu/click") },
            XrActionSuggestedBinding{ .action = a.inMenu_modMenuAction, .binding = GetPath(instance, "/user/hand/left/input/view/click") },

            XrActionSuggestedBinding{ .action = a.inMenuDpadActions[0], .binding = GetPath(instance, "/user/hand/left/input/dpad_up/click") },
            XrActionSuggestedBinding{ .action = a.inMenuDpadActions[1], .binding = GetPath(instance, "/user/hand/left/input/dpad_right/click") },
            XrActionSuggestedBinding{ .action = a.inMenuDpadActions[2], .binding = GetPath(instance, "/user/hand/left/input/dpad_down/click") },
            XrActionSuggestedBinding{ .action = a.inMenuDpadActions[3], .binding = GetPath(instance, "/user/hand/left/input/dpad_left/click") },
        };
        SuggestProfileBindings(instance, "/interaction_profiles/valve/frame_controller_valve", suggestedBindings.data(), (uint32_t)suggestedBindings.size(), false);
    }
}

inline ControllerType DetectActiveControllerType(XrInstance instance, XrSession session) {
    XrPath topLevelPath;
    xrStringToPath(instance, "/user/hand/left", &topLevelPath);

    XrInteractionProfileState profileState = { XR_TYPE_INTERACTION_PROFILE_STATE };
    XrResult profileResult = xrGetCurrentInteractionProfile(session, topLevelPath, &profileState);
    if (profileResult != XR_SUCCESS) {
        Log::print<INFO>(" - Active Controller Type: Unknown (xrGetCurrentInteractionProfile failed: {})", (int)profileResult);
        return ControllerType::Unknown;
    }
    if (profileState.interactionProfile == XR_NULL_PATH) {
        Log::print<INFO>(" - Active Controller Type: Unknown (no controllers bound yet)");
        return ControllerType::Unknown;
    }

    uint32_t requiredCapacity = 0;
    xrPathToString(instance, profileState.interactionProfile, 0, &requiredCapacity, nullptr);
    std::string pathStr(requiredCapacity, '\0');
    xrPathToString(instance, profileState.interactionProfile, requiredCapacity, &requiredCapacity, pathStr.data());

    ControllerType result = ControllerType::Unknown;

    if (pathStr.find("oculus") != std::string::npos)
        result = ControllerType::OculusTouch;
    else if (pathStr.find("frame_controller") != std::string::npos)
        result = ControllerType::SteamFrame;
    else if (pathStr.find("valve") != std::string::npos)
        result = ControllerType::ValveIndex;
    else if (pathStr.find("bytedance") != std::string::npos)
        result = ControllerType::Pico4;
    else if (pathStr.find("hp/mixed") != std::string::npos)
        result = ControllerType::HPReverbG2;
    else if (pathStr.find("cosmos") != std::string::npos)
        result = ControllerType::ViveCosmos;
    else if (pathStr.find("simple") != std::string::npos)
        result = ControllerType::Simple;

    Log::print<INFO>(" - Active Controller Type: {} (profile: {})", (int)result, pathStr);
    return result;
}
