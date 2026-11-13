#include "pch.h"
#include "controls.h"
#include "button_state.h"
#include "openxr_motion_bridge.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/camera.h"
#include "game/botw/player/player.h"
#include "game/botw/player/skeleton.h"
#include "game/botw/player/weapon.h"
#include "rendering/haptics.h"
#include "rendering/imgui_menus.h"
#include "rendering/tracking.h"
#include "font_material_symbols.h"

static ButtonState s_inventoryMapButton;
static ButtonState s_modMenuButton;
static ButtonState s_crouchScopeButton;
static ButtonState s_runButton;
static ButtonState s_runeButton;
static ButtonState s_menuHoldButton;
static std::array<ButtonState, 2> s_grabButtons;

enum class PullSlot { None, LeftShoulder, RightShoulder, LeftWaist };
static std::array<PullSlot, 2> s_pullSlots = {};

struct HandVelocitySample {
    XrTime time = 0;
    glm::fvec3 velocity = {};
};

// only touched by hook_InjectXRInput
struct ControlState {
    uint32_t previousButtonHold = 0;
    uint32_t previousCombinedHold = 0;
    VPADButtons previousStickHold = VPAD_BUTTON_NONE;
    XrTime previousInputTime = 0;
    bool inGame = false;
    bool wasInGame = false;
    bool mapOpen = false; // map = true, inventory = false

    bool preventInputs = false;
    std::chrono::steady_clock::time_point preventInputsTime;
    bool preventGrabInputs = false;
    std::chrono::steady_clock::time_point preventGrabTime;

    bool dpadMenuOpenRequested = false;
    bool wasDpadMenuOpen = false;
    EquipType lastDpadMenuOpen = EquipType::None;
    bool dpadMenuSelectionAlreadyEquipped = false;
    bool runeNeedsReequip = false;
    float runeReequipTimer = 0.0f;

    EquipType rightHandEquipType = EquipType::None;
    EquipType leftHandEquipType = EquipType::None;
    EquipType rightHandPreviousEquipType = EquipType::None;
    EquipType leftHandPreviousEquipType = EquipType::None;
    int rightHandEquipChangeFrames = 0;
    int leftHandEquipChangeFrames = 0;
    EquipType lastEquipTypeHeld = EquipType::None;
    bool hasSomethingInLeftHand = false;
    bool hasSomethingInRightHand = false;
    bool isThrowableObjectHeld = false;

    bool isLockingOnTarget = false;
    bool isShieldGuarding = false;
    bool triggerPressedOverBodySlot = false;
    float previousLeftHandVelocity = 0.0f;

    bool previousRightTrigger = false;
    bool previousRightThrowGrip = false;
    bool handThrowCarrying = false;
    bool handThrowArmed = false;
    bool handThrowArmedByGrip = false;
    uint32_t samplesSinceThrowableSeen = UINT32_MAX;
    std::array<HandVelocitySample, 4> rightHandVelocities = {};
    uint32_t nextRightHandVelocity = 0;
};
static ControlState s_state;

static void UpdateControlButtons(const OpenXR::InputState& inputs, bool overlayOpen) {
    const auto time = inputs.shared.inputTime;
    static bool previousInGame = inputs.shared.in_game;
    if (previousInGame != inputs.shared.in_game) {
        s_inventoryMapButton.Cancel();
        s_modMenuButton.Cancel();
        previousInGame = inputs.shared.in_game;
    }
    const bool gameplay = inputs.shared.in_game && !overlayOpen;
    s_inventoryMapButton.Update(!overlayOpen && inputs.shared.inventory_map.isActive, inputs.shared.inventory_map.currentState, time);
    s_modMenuButton.Update(inputs.shared.modMenu.isActive, inputs.shared.modMenu.currentState, time);
    s_crouchScopeButton.Update(gameplay && inputs.inGame.crouch_scope.isActive, inputs.inGame.crouch_scope.currentState, time);
    s_runButton.Update(gameplay && inputs.inGame.run_interact.isActive, inputs.inGame.run_interact.currentState, time);
    s_runeButton.Update(gameplay && inputs.inGame.useRune_dpadMenu.isActive, inputs.inGame.useRune_dpadMenu.currentState, time);
    s_menuHoldButton.Update(!inputs.shared.in_game && !overlayOpen && inputs.inMenu.hold.isActive, inputs.inMenu.hold.currentState, time);
    for (size_t side = 0; side < s_grabButtons.size(); ++side)
        s_grabButtons[side].Update(gameplay && inputs.inGame.grab[side].isActive, inputs.inGame.grab[side].currentState > 0.51f, time);
}

static uint32_t ButtonIfHeld(const XrActionStateBoolean& action, VPADButtons button) {
    return action.currentState ? button : 0;
}

static void SpreadWeaponDetectionOverFrames(ControlState& state) {
    // Spread the weapon detection from link's attachement bones over several frames.
    // Each bone isn't necessarily checked each frames which can give wrong results sometimes
    // Also, some animations seem to make the weapon not detected correctly (shooting arrow)
    constexpr uint8_t REQUIRED_FRAMES = 5;
    if (state.rightHandEquipType != state.rightHandPreviousEquipType) {
        state.rightHandEquipChangeFrames++;
        if (state.rightHandEquipChangeFrames > REQUIRED_FRAMES) {
            state.rightHandEquipChangeFrames = 0;
        }
        else
            state.rightHandEquipType = state.rightHandPreviousEquipType;
    }
    else {
        state.rightHandEquipChangeFrames = 0;
    }

    if (state.leftHandEquipType != state.leftHandPreviousEquipType) {
        state.leftHandEquipChangeFrames++;
        if (state.leftHandEquipChangeFrames > REQUIRED_FRAMES) {
            state.leftHandEquipChangeFrames = 0;
        }
        else
            state.leftHandEquipType = state.leftHandPreviousEquipType;
    }
    else {
        state.leftHandEquipChangeFrames = 0;
    }
}

struct HandGestureState {
    bool isBehindHead;
    bool isBehindHeadWithWaistOffset;
    bool isCloseToHead;
    bool isCloseToMouth;
    bool isCloseToWaist;
    bool isNearChestHeight;
    bool isOnLeftSide;  // true = left side of body, false = right side
};

static bool ShouldOverrideFirstPersonTurning(bool inGame) {
    return inGame && GameState::IsFirstPerson() && !Events::IsActive() && !Screens::IsVisible(ScreenId::AppCamera_00);
}

static void UpdateFirstPersonTurning(const XrActionStateVector2f& rightStickSource, bool inGame) {
    static bool s_snapTurnStickLatched = false;
    const float stickX = rightStickSource.currentState.x;

    if (!ShouldOverrideFirstPersonTurning(inGame) || ImGuiMenus::g_isOpen) {
        Camera::CancelSnapTurn();
        Camera::SetSmoothTurnInput(0.0f);
        s_snapTurnStickLatched = false;
    }
    else if (GetSettings().GetSnapTurnAngle() > 0) {
        Camera::SetSmoothTurnInput(0.0f);

        const float axisThreshold = GetSettings().axisThreshold;
        const float releaseThreshold = std::max<float>((float)GetSettings().stickDeadzone, axisThreshold * 0.5f);
        if (s_snapTurnStickLatched) {
            s_snapTurnStickLatched = std::fabs(stickX) > releaseThreshold;
        }
        else if (std::fabs(stickX) >= axisThreshold) {
            Camera::RequestSnapTurn(stickX > 0.0f ? 1 : -1);
            s_snapTurnStickLatched = true;
        }
    }
    else {
        s_snapTurnStickLatched = false;
        Camera::SetSmoothTurnInput(std::fabs(stickX) > (float)GetSettings().stickDeadzone ? stickX : 0.0f);
    }
}

// Gesture detection functions
static HandGestureState CalculateHandGesture(
    const glm::fvec3& handPos,
    const glm::fmat4& headsetMatrix,
    const glm::fvec3& headsetPos
) {
    HandGestureState gesture = {};

    constexpr float SHOULDER_RADIUS = 0.35f;
    constexpr float MOUTH_RADIUS = 0.2f;
    constexpr float WAIST_OFFSET_Y = 0.45f;
    constexpr float MIN_SHIELD_GUARD_OFFSET_Y = 0.40f;
    constexpr float MAX_SHIELD_GUARD_OFFSET_Y = 0.60f;
    constexpr float SHIELD_GUARD_REFERENCE_HEIGHT_MIN = 1.60f;
    constexpr float SHIELD_GUARD_REFERENCE_HEIGHT_MAX = 2.00f;
    
    glm::vec3 headsetForward = -glm::normalize(glm::vec3(headsetMatrix[2]));
    headsetForward.y = 0.0f;
    if (glm::length2(headsetForward) < 0.0001f) {
        headsetForward = glm::vec3(0.0f, 0.0f, -1.0f);
    }
    else {
        headsetForward = glm::normalize(headsetForward);
    }
    
    glm::fvec3 headsetRight = glm::normalize(glm::fvec3(headsetMatrix[0]));
    glm::fvec3 headToHand = handPos - headsetPos;
    
    float forwardDot = glm::dot(headsetForward, headToHand);
    gesture.isBehindHead = (forwardDot < 0.0f);
    
    glm::vec3 flatForward = glm::normalize(glm::vec3(headsetForward.x, 0.0f, headsetForward.z));
    glm::vec3 flatHandOffset = glm::vec3(headToHand.x, 0.0f, headToHand.z);
    
    constexpr float WAIST_BEHIND_OFFSET = -0.05f; //0.15f
    float flatForwardDot = glm::dot(flatForward, flatHandOffset) + WAIST_BEHIND_OFFSET;
    gesture.isBehindHeadWithWaistOffset = (flatForwardDot < 0.0f);
    
    float rightDot = glm::dot(headsetRight, headToHand);
    gesture.isOnLeftSide = (rightDot < 0.0f);
    
    constexpr float SHOULDER_RADIUS_SQ = SHOULDER_RADIUS * SHOULDER_RADIUS;
    gesture.isCloseToHead = (glm::length2(headToHand) < SHOULDER_RADIUS_SQ);

    // the mouth sits below and in front of the eyes, where the controllers end up when you cup your hands to whistle
    constexpr float MOUTH_FORWARD = 0.08f;
    constexpr float MOUTH_BELOW_EYES = 0.10f;
    constexpr float MOUTH_RADIUS_SQ = MOUTH_RADIUS * MOUTH_RADIUS;
    const glm::fvec3 mouthPos = headsetPos + headsetForward * MOUTH_FORWARD - glm::fvec3(0.0f, MOUTH_BELOW_EYES, 0.0f);
    gesture.isCloseToMouth = (glm::length2(handPos - mouthPos) < MOUTH_RADIUS_SQ);
    
    glm::fvec3 waistPos = headsetPos - glm::fvec3(0.0f, WAIST_OFFSET_Y, 0.0f);
    gesture.isCloseToWaist = (handPos.y < waistPos.y);

    float shieldGuardHeightT = (headsetPos.y - SHIELD_GUARD_REFERENCE_HEIGHT_MIN) / (SHIELD_GUARD_REFERENCE_HEIGHT_MAX - SHIELD_GUARD_REFERENCE_HEIGHT_MIN);
    float shieldGuardOffsetY = std::lerp(MIN_SHIELD_GUARD_OFFSET_Y, MAX_SHIELD_GUARD_OFFSET_Y, glm::clamp(shieldGuardHeightT, 0.0f, 1.0f));
    glm::fvec3 chestPos = headsetPos - glm::fvec3(0.0f, shieldGuardOffsetY, 0.0f);
    gesture.isNearChestHeight = (handPos.y > chestPos.y);

    return gesture;
}

static bool IsHandOverLeftShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && gesture.isOnLeftSide;
}

static bool IsHandOverRightShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && !gesture.isOnLeftSide;
}

static bool IsHandOverLeftWaistSlot(const HandGestureState& gesture) {
    return gesture.isBehindHeadWithWaistOffset && gesture.isCloseToWaist && gesture.isOnLeftSide;
}

// a hand swinging past the hip shouldn't take the slate out
static bool IsHandSettledAtWaist(const OpenXR::InputState& inputs, size_t side) {
    constexpr float MAX_WAIST_SLOT_SPEED = 1.2f;
    const XrSpaceVelocity& velocity = inputs.shared.poseVelocity[side];
    if ((velocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) == 0)
        return true;
    return glm::length(ToGLM(velocity.linearVelocity)) < MAX_WAIST_SLOT_SPEED;
}

static bool IsHandOverRightWaistSlot(const HandGestureState& gesture) {
    return gesture.isBehindHeadWithWaistOffset && gesture.isCloseToWaist && !gesture.isOnLeftSide;
}

static bool IsHandOverMouthSlot(const HandGestureState& gesture) {
    return gesture.isCloseToMouth && !gesture.isBehindHead;
}

static bool IsHandNotOverAnySlot(const HandGestureState& gesture) {
    return !gesture.isBehindHead && !gesture.isBehindHeadWithWaistOffset && !gesture.isCloseToMouth;
}

static PullSlot GetBodySlotUnderHand(const HandGestureState& gesture) {
    if (IsHandOverLeftShoulderSlot(gesture))
        return PullSlot::LeftShoulder;
    if (IsHandOverRightShoulderSlot(gesture))
        return PullSlot::RightShoulder;
    if (IsHandOverLeftWaistSlot(gesture))
        return PullSlot::LeftWaist;
    return PullSlot::None;
}

static void UpdatePullSlots(const OpenXR::InputState& inputs, const HandGestureState& left, const HandGestureState& right) {
    const std::array<const HandGestureState*, 2> hands = { &left, &right };
    for (size_t side = 0; side < hands.size(); ++side) {
        if (!s_grabButtons[side].IsDown() && s_grabButtons[side].lastEvent != ButtonState::Event::ShortPress)
            s_pullSlots[side] = PullSlot::None;
        else if (s_grabButtons[side].lastEvent == ButtonState::Event::Pressed) {
            const PullSlot slot = GetBodySlotUnderHand(*hands[side]);
            s_pullSlots[side] = slot == PullSlot::LeftWaist && !IsHandSettledAtWaist(inputs, side) ? PullSlot::None : slot;
        }
    }
}

// the left shoulder holds the bow, the right shoulder the melee weapon and the left waist the Sheikah Slate
static bool IsBodySlotItemInHand(PullSlot slot, const ControlState& state) {
    switch (slot) {
        case PullSlot::LeftShoulder: return state.leftHandEquipType == EquipType::Bow;
        case PullSlot::RightShoulder: return state.rightHandEquipType == EquipType::Melee;
        case PullSlot::LeftWaist: return state.leftHandEquipType == EquipType::SheikahSlate;
        default: return false;
    }
}

// returns false when the slot's item is already in hand
static bool EquipFromBodySlot(PullSlot slot, uint32_t& buttonHold, ControlState& state) {
    if (slot == PullSlot::None || IsBodySlotItemInHand(slot, state))
        return false;

    switch (slot) {
        case PullSlot::LeftShoulder:
            buttonHold |= VPAD_BUTTON_ZR;
            state.lastEquipTypeHeld = EquipType::Bow;
            break;
        case PullSlot::RightShoulder:
            buttonHold |= VPAD_BUTTON_Y;
            state.lastEquipTypeHeld = EquipType::Melee;
            break;
        default:
            buttonHold |= VPAD_BUTTON_L;
            state.lastEquipTypeHeld = EquipType::SheikahSlate;
            break;
    }
    return true;
}

static void ToggleBodySlotEquip(PullSlot slot, uint32_t& buttonHold, ControlState& state) {
    if (!EquipFromBodySlot(slot, buttonHold, state))
        buttonHold |= VPAD_BUTTON_B;
}

static bool OpenDpadMenuRuneButton(ButtonState::Event lastEvent, uint32_t& buttonHold, ControlState& state) {
    if (lastEvent == ButtonState::Event::LongPress && state.rightHandEquipType != EquipType::MagnetGlove) {
        if (state.leftHandEquipType == EquipType::SheikahSlate) {
            state.dpadMenuSelectionAlreadyEquipped = true;
            state.runeNeedsReequip = true;
        }
        buttonHold |= VPAD_BUTTON_UP;
        state.lastDpadMenuOpen = EquipType::SheikahSlate;
        state.dpadMenuOpenRequested = true;
        return true;
    }
    return false;
}

static bool OpenDpadMenuBodySlots(ButtonState::Event lastEvent, HandGestureState handGesture, uint32_t& buttonHold, ControlState& state) {
    if (lastEvent == ButtonState::Event::LongPress && !state.dpadMenuOpenRequested && state.rightHandEquipType != EquipType::MagnetGlove) {
        if (IsHandOverRightShoulderSlot(handGesture)) {
            if (state.leftHandEquipType == EquipType::Bow) {
                buttonHold |= VPAD_BUTTON_LEFT;
                state.lastDpadMenuOpen = EquipType::Arrow;
            }
            else {
                if (state.rightHandEquipType == EquipType::Melee)
                    state.dpadMenuSelectionAlreadyEquipped = true;
                buttonHold |= VPAD_BUTTON_RIGHT;
                state.lastDpadMenuOpen = EquipType::Melee;
            }
        }
        else if (IsHandOverLeftShoulderSlot(handGesture)) {
            if (state.rightHandEquipType == EquipType::Melee) {
                buttonHold |= VPAD_BUTTON_LEFT;
                state.lastDpadMenuOpen = EquipType::Shield;
            }
            else {
                // Force a bow equip so the correct menu opens.
                if (state.leftHandEquipType != EquipType::Bow)
                    buttonHold |= VPAD_BUTTON_ZR;
                else
                    state.dpadMenuSelectionAlreadyEquipped = true;
                buttonHold |= VPAD_BUTTON_RIGHT;
                state.lastDpadMenuOpen = EquipType::Bow;
            }
        }
        else {
            if (state.leftHandEquipType == EquipType::SheikahSlate) {
                state.dpadMenuSelectionAlreadyEquipped = true;
                state.runeNeedsReequip = true;
            }
            buttonHold |= VPAD_BUTTON_UP;
            state.lastDpadMenuOpen = EquipType::SheikahSlate;
        }
        state.dpadMenuOpenRequested = true;
        return true;
    }
    return false;
}

static void KeepDpadMenuOpen(uint32_t& buttonHold, ControlState& state) {
    if (!state.dpadMenuOpenRequested)
        return;

    switch (state.lastDpadMenuOpen) {
        case EquipType::Shield:
        case EquipType::Arrow:
            buttonHold |= VPAD_BUTTON_LEFT;
            break;
        case EquipType::Melee:
        case EquipType::Bow:
            buttonHold |= VPAD_BUTTON_RIGHT;
            break;
        case EquipType::SheikahSlate:
            buttonHold |= VPAD_BUTTON_UP;
            break;
        default:
            break;
    }
}

static bool CloseDpadMenu(OpenXR::InputState& inputs, ControlState& state) {
    if (!state.dpadMenuOpenRequested)
        return false;

    if (!inputs.inMenu.leftGrip.currentState && !inputs.inMenu.rightGrip.currentState && !inputs.inMenu.sort.currentState) {
        state.dpadMenuOpenRequested = false;
        state.wasDpadMenuOpen = true;
    }
    return true;
}

static void EquipWeaponOnDpadMenuExit(uint32_t& buttonHold, ControlState& state, float dt) {
    if (state.runeNeedsReequip) {
        constexpr float RUNE_REEQUIP_TIME = 0.5f;
        if (state.runeReequipTimer >= RUNE_REEQUIP_TIME) {
            if (state.leftHandEquipType != EquipType::SheikahSlate)
                buttonHold |= VPAD_BUTTON_L;
            state.runeNeedsReequip = false;
            state.runeReequipTimer = 0.0f;
        }
        else
            state.runeReequipTimer += dt;
    }

    if (!state.wasDpadMenuOpen) 
        return;

    if (!state.dpadMenuSelectionAlreadyEquipped) {
        switch (state.lastDpadMenuOpen) {
            case EquipType::Melee:
                buttonHold |= VPAD_BUTTON_Y;
                state.lastEquipTypeHeld = EquipType::Melee;
                break;
            case EquipType::Bow:
                buttonHold |= VPAD_BUTTON_ZR;
                state.lastEquipTypeHeld = EquipType::Bow;
                break;
            case EquipType::SheikahSlate:
                buttonHold |= VPAD_BUTTON_L;
                state.lastEquipTypeHeld = EquipType::SheikahSlate;
                break;
        }
    }
    else
        state.dpadMenuSelectionAlreadyEquipped = false;

    state.wasDpadMenuOpen = false;
    state.lastDpadMenuOpen = EquipType::None;
}

// Input handling functions
static void ProcessLeftHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    ControlState& state,
    const HandGestureState& leftGesture,
    XrActionStateVector2f& rightStickSource,
    const std::chrono::steady_clock::time_point& now,
    float dt,
    bool suppressGrabActions
) {
    constexpr RumbleParameters leftRumbleRaise = { true, 0, RumbleType::Raising, 0.5f, false, 0.2, 1.0f, 1.0f };
    constexpr RumbleParameters leftRumbleFall = { true, 0, RumbleType::Falling, 0.5f, false, 0.3, 0.1f, 0.75f };
    constexpr RumbleParameters RuneRumble = { true, 0, RumbleType::OscillationSmooth, 1.0f, false, 1.0, 0.25f, 0.25f };
    
    const ButtonState::Event grabEvent = suppressGrabActions ? ButtonState::Event::None : s_grabButtons[0].lastEvent;
    bool isGrabPressed = grabEvent == ButtonState::Event::ShortPress;
    bool isGrabPressedLong = grabEvent == ButtonState::Event::LongPress;
    bool isCurrentGrabPressed = !suppressGrabActions && (s_grabButtons[0].IsDown() || (grabEvent == ButtonState::Event::ShortPress && s_pullSlots[0] != PullSlot::None));
    
    if (state.leftHandEquipType == EquipType::SheikahSlate)
        Haptics::Play(RuneRumble);

    // Gesture enabled only when both melee weapon and shield are in hands to prevent 2 handed weapons and quick drawing shield 
    if (!inputs.inGame.useLeftItem.currentState && state.leftHandEquipType == EquipType::Shield && state.rightHandEquipType == EquipType::Melee && (leftGesture.isNearChestHeight)) {
        buttonHold |= VPAD_BUTTON_ZL;
        rightStickSource.currentState.y = 0.2f; // Force disable the lock on view when holding shield
        state.isShieldGuarding = true;
        if ((state.previousButtonHold & VPAD_BUTTON_ZL) == 0)
            Haptics::Play(leftRumbleRaise);
    }
    else if (!inputs.inGame.useLeftItem.currentState)
        state.isShieldGuarding = false;

    if (!state.isShieldGuarding && state.previousButtonHold & VPAD_BUTTON_ZL)
        Haptics::Play(leftRumbleFall);

    float ACCELERATION_THRESHOLD = 35.0f;
    auto handVelocity = glm::length(ToGLM(inputs.shared.poseVelocity[0].linearVelocity));
    float acceleration = dt > 0.0f ? (handVelocity - state.previousLeftHandVelocity) / dt : 0.0f;
    state.previousLeftHandVelocity = handVelocity;
    if (acceleration > ACCELERATION_THRESHOLD && state.leftHandEquipType == EquipType::Shield && state.isShieldGuarding) {
        buttonHold |= VPAD_BUTTON_A;
        Haptics::Play(leftRumbleFall);
    }

    if (IsHandOverLeftShoulderSlot(leftGesture) || IsHandOverRightShoulderSlot(leftGesture)) {
        if (OpenDpadMenuBodySlots(grabEvent, leftGesture, buttonHold, state)) {
            s_pullSlots[0] = PullSlot::None;
            s_grabButtons[0].Cancel();
            return;
        }

        if (!state.preventGrabInputs && isGrabPressed) {
            Haptics::Play(leftRumbleFall);
            ToggleBodySlotEquip(GetBodySlotUnderHand(leftGesture), buttonHold, state);
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }
        return;  // Don't process normal input when over slots
    }
    
    if (IsHandOverLeftWaistSlot(leftGesture) && IsHandSettledAtWaist(inputs, 0)) {    
        if (OpenDpadMenuBodySlots(grabEvent, leftGesture, buttonHold, state)) {
            s_pullSlots[0] = PullSlot::None;
            s_grabButtons[0].Cancel();
            return;
        }

        if (!state.preventGrabInputs && isGrabPressed) {
            Haptics::Play(leftRumbleFall);
            ToggleBodySlotEquip(PullSlot::LeftWaist, buttonHold, state);
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }
        return;
    }

    if (isCurrentGrabPressed) {
        EquipFromBodySlot(s_pullSlots[0], buttonHold, state);
        if (s_pullSlots[0] != PullSlot::None) {
            s_pullSlots[0] = PullSlot::None;
            s_grabButtons[0].Cancel();
            return;
        }
    }

    // reach across to the right waist slot to drop what's in the left hand
    if (IsHandOverRightWaistSlot(leftGesture))
    {
        if (isGrabPressedLong) {
            Haptics::Play(leftRumbleFall);
            Weapons::RequestDrop(0);
            s_grabButtons[0].Cancel();
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }
        return;
    }

    if (isGrabPressed) {
        if (!state.preventGrabInputs && !Player::IsRiding()) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }

    // Master Sword QTE fix. is_riding_mount check to prevent conflict with master cycle brake function
    if (IsHandNotOverAnySlot(leftGesture) && !suppressGrabActions && s_grabButtons[0].IsLongHeld() && !Player::IsRiding()) {
        if (!state.preventGrabInputs) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
}

static void ProcessRightHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    ControlState& state,
    const HandGestureState& rightGesture,
    XrActionStateVector2f& rightStickSource,
    const std::chrono::steady_clock::time_point& now,
    bool suppressGrabActions
) {
    constexpr RumbleParameters rightRumbleFall = { true, 1, RumbleType::Falling, 0.5f, false, 0.3, 0.1f, 0.75f };
    constexpr RumbleParameters rightRumbleInfiniteRaise = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters OverSlotsRumble = { false, 1, RumbleType::OscillationRaisingSawtoothWave, 1.0f, false, 1.0, 0.25f, 0.25f };

    const ButtonState::Event grabEvent = suppressGrabActions ? ButtonState::Event::None : s_grabButtons[1].lastEvent;
    bool isGrabPressedShort = grabEvent == ButtonState::Event::ShortPress;
    bool isGrabPressedLong = grabEvent == ButtonState::Event::LongPress;
    bool isCurrentGrabPressed = !suppressGrabActions && (s_grabButtons[1].IsDown() || (grabEvent == ButtonState::Event::ShortPress && s_pullSlots[1] != PullSlot::None));
    bool isTriggerPressed = inputs.inGame.useRightItem.currentState;
    
    if (IsHandOverLeftShoulderSlot(rightGesture) || IsHandOverRightShoulderSlot(rightGesture)) {
        if (OpenDpadMenuBodySlots(grabEvent, rightGesture, buttonHold, state)) {
            s_pullSlots[1] = PullSlot::None;
            s_grabButtons[1].Cancel();
            return;
        }

        if (IsHandOverRightShoulderSlot(rightGesture) && state.rightHandEquipType == EquipType::Melee && !state.handThrowCarrying && !inputs.inGame.useRightItem.currentState)
            Haptics::Play(OverSlotsRumble);

        if (!state.preventGrabInputs && isGrabPressedShort) { 
            Haptics::Play(rightRumbleFall);
            ToggleBodySlotEquip(GetBodySlotUnderHand(rightGesture), buttonHold, state);
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }

        
        if (isTriggerPressed)
        {
            // Keep going or start shoot arrow action even if hand gets over right shoulder slot
            if (state.leftHandEquipType == EquipType::Bow)
            {
                Haptics::Play(rightRumbleInfiniteRaise);
                buttonHold |= VPAD_BUTTON_ZR;
                state.triggerPressedOverBodySlot = true;
            }
            else if (state.rightHandEquipType == EquipType::Melee && !state.handThrowCarrying) {
                Haptics::Play(rightRumbleInfiniteRaise);
                buttonHold |= VPAD_BUTTON_R;
                state.triggerPressedOverBodySlot = true;
            }
        }
        else if (state.triggerPressedOverBodySlot) {
            Haptics::Stop(1, RumbleType::Raising);
            state.triggerPressedOverBodySlot = false;
        }
        return;  // Don't process normal input when over slots
    }
    // if hand not on shoulder slot but trigger still pressed, stop throw rumbles
    else if (state.triggerPressedOverBodySlot && state.rightHandEquipType == EquipType::Melee) {
        Haptics::Stop(1, RumbleType::Raising);
    }

    if (IsHandOverLeftWaistSlot(rightGesture) && IsHandSettledAtWaist(inputs, 1)) {   
        if (OpenDpadMenuBodySlots(grabEvent, rightGesture, buttonHold, state)) {
            s_pullSlots[1] = PullSlot::None;
            s_grabButtons[1].Cancel();
            return;
        }

        if (!state.preventGrabInputs && isGrabPressedShort) {
            Haptics::Play(rightRumbleFall);
            ToggleBodySlotEquip(PullSlot::LeftWaist, buttonHold, state);
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }
        return;
    }

    if (IsHandOverRightWaistSlot(rightGesture))
    {
        if (isGrabPressedLong) {
            Haptics::Play(rightRumbleFall);
            Weapons::RequestDrop(1);
            s_grabButtons[1].Cancel();
            state.preventGrabInputs = true;
            state.preventGrabTime = now;
        }
        return;
    }
    
    if (isCurrentGrabPressed) {
        if (EquipFromBodySlot(s_pullSlots[1], buttonHold, state))
            Haptics::Play(rightRumbleFall);
        if (s_pullSlots[1] != PullSlot::None) {
            s_pullSlots[1] = PullSlot::None;
            s_grabButtons[1].Cancel();
            return;
        }
    }

    if (isGrabPressedShort) {
        if (!state.preventGrabInputs) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }

    // Master Sword QTE fix
    if (IsHandNotOverAnySlot(rightGesture) && !suppressGrabActions && s_grabButtons[1].IsLongHeld()) {
        if (!state.preventGrabInputs) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
}

static void ProcessLeftTriggerBindings(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    ControlState& state
) {
    if (!inputs.inGame.useLeftItem.currentState) {
        state.isLockingOnTarget = false;
        return;
    }

    constexpr RumbleParameters raiseRumble = { true, 0, RumbleType::Raising, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters fallRumble = { true, 0, RumbleType::Falling, 0.5f, false, 0.25, 0.3f, 0.3f };

    // Reset the guard state to trigger again the lock on camera
    if (!state.isLockingOnTarget && state.previousButtonHold & VPAD_BUTTON_ZL) {
        if (state.leftHandEquipType == EquipType::SheikahSlate) {
            Haptics::Play(raiseRumble);
            buttonHold |= VPAD_BUTTON_B; // Cancel rune grab
        }
        buttonHold &= ~VPAD_BUTTON_ZL;
    }

    else {
        if (!state.isShieldGuarding)
            Haptics::Play(raiseRumble);
        buttonHold |= VPAD_BUTTON_ZL;
        state.isLockingOnTarget = true;
        state.isShieldGuarding = true;
    }    
}

// the trigger or grip throws, but one that was already held when picking something up (like a bomb) has to be pressed again
static void ProcessHandThrow(uint32_t& buttonHold, const OpenXR::InputState& inputs, ControlState& state) {
    constexpr XrDuration releaseVelocityWindowNs = 100'000'000;
    constexpr uint32_t throwableMissedSamplesTolerated = 3;
    constexpr float boostStartSpeed = 4.0f;
    constexpr float boostFullSpeed = 8.0f;
    constexpr float maxThrowBoost = 1.8f;
    constexpr RumbleParameters readyRumble = { true, 1, RumbleType::Fixed, 0.5f, false, 0.25, 0.15f, 0.15f };
    constexpr RumbleParameters throwRumble = { true, 1, RumbleType::Falling, 0.5f, false, 0.3, 0.1f, 0.75f };

    const XrSpaceVelocity& handVelocity = inputs.shared.poseVelocity[OpenXR::EyeSide::RIGHT];
    if ((handVelocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT) != 0) {
        state.rightHandVelocities[state.nextRightHandVelocity] = { inputs.shared.inputTime, ToGLM(handVelocity.linearVelocity) };
        state.nextRightHandVelocity = (state.nextRightHandVelocity + 1) % (uint32_t)state.rightHandVelocities.size();
    }

    const bool triggerHeld = inputs.inGame.useRightItem.currentState;
    const bool triggerPressed = triggerHeld && !state.previousRightTrigger;
    const bool triggerReleased = !triggerHeld && state.previousRightTrigger;
    state.previousRightTrigger = triggerHeld;
    const bool gripHeld = s_grabButtons[1].IsDown();
    const bool gripPressed = gripHeld && !state.previousRightThrowGrip;
    const bool gripReleased = !gripHeld && state.previousRightThrowGrip;
    state.previousRightThrowGrip = gripHeld;

    if (state.isThrowableObjectHeld)
        state.samplesSinceThrowableSeen = 0;
    else if (state.samplesSinceThrowableSeen != UINT32_MAX)
        state.samplesSinceThrowableSeen++;
    state.handThrowCarrying = state.samplesSinceThrowableSeen <= throwableMissedSamplesTolerated && GameState::IsFirstPerson() && !Events::IsActive();
    if (!state.handThrowCarrying) {
        state.handThrowArmed = false;
        return;
    }

    if ((triggerPressed || gripPressed) && !state.handThrowArmed) {
        state.handThrowArmed = true;
        state.handThrowArmedByGrip = gripPressed;
        Haptics::Play(readyRumble);
    }
    if (!state.handThrowArmed || !(state.handThrowArmedByGrip ? gripReleased : triggerReleased))
        return;
    state.handThrowArmed = false;

    // the hand is usually already slowing down when you let go
    std::optional<glm::fvec3> releaseVelocity;
    for (const HandVelocitySample& sample : state.rightHandVelocities) {
        if (sample.time == 0 || inputs.shared.inputTime - sample.time > releaseVelocityWindowNs)
            continue;
        if (!releaseVelocity.has_value() || glm::length2(sample.velocity) > glm::length2(releaseVelocity.value()))
            releaseVelocity = sample.velocity;
    }
    // hard throws fly further than the hand alone, so putting effort in pays off
    if (releaseVelocity.has_value()) {
        const float handSpeed = glm::length(releaseVelocity.value());
        const float boost = glm::mix(1.0f, maxThrowBoost, glm::smoothstep(boostStartSpeed, boostFullSpeed, handSpeed));
        Player::RequestHandThrow(glm::mat3(Camera::GetReferenceMtx()) * releaseVelocity.value() * boost);
        Log::print<CONTROLS>("[Throw] released with hand speed {:.2f}, boosted {:.2f}x", handSpeed, boost);
    }
    else {
        Log::print<CONTROLS>("[Throw] released without a hand speed");
    }
    buttonHold |= VPAD_BUTTON_R;
    Haptics::Play(throwRumble);
}

static void ProcessRightTriggerBindings(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    ControlState& state,
    HandGestureState rightGesture
) {
    if (IsHandOverRightShoulderSlot(rightGesture))
        return;

    if (!inputs.inGame.useRightItem.currentState) {
        Haptics::Stop(1, RumbleType::Raising);
        state.triggerPressedOverBodySlot = false;
        return;
    }
    
    constexpr RumbleParameters rightRumbleFixed = { true, 1, RumbleType::Fixed, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters leftRumbleFixed = { true, 0, RumbleType::Fixed, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters rightRumbleInfiniteRaiseBow = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters rightRumbleInfiniteRaiseWeaponThrow = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters rightRumbleFiniteRaise = { true, 1, RumbleType::Raising, 0.5f, false, 0.25, 1.0f, 1.0f };
    
    
    if (state.handThrowCarrying)
        return;

    if (state.hasSomethingInLeftHand || state.hasSomethingInRightHand) {
        if (state.isThrowableObjectHeld) {
            Haptics::Play(rightRumbleFixed);
            buttonHold |= VPAD_BUTTON_R;  // Throw object
        }
        else if (state.leftHandEquipType == EquipType::Bow || state.lastEquipTypeHeld == EquipType::Bow) {
            Haptics::Play(rightRumbleInfiniteRaiseBow);
            buttonHold |= VPAD_BUTTON_ZR;  // Shoot bow
        }
        else if (state.leftHandEquipType == EquipType::SheikahSlate) {
            buttonHold |= VPAD_BUTTON_A; // Use rune
            Haptics::Play(leftRumbleFixed);
        }
        // the triggerPressedOverBodySlot check prevents the melee attack rumbles to wrongly start when the throw weapon inputs 
        // hasn't been released yet. Necessary because the melee attack action won't start if the throw input is still held.
        else if (!state.triggerPressedOverBodySlot){
            Haptics::Play(rightRumbleInfiniteRaiseWeaponThrow);
            buttonHold |= VPAD_BUTTON_Y;  // Melee attack
        }
    }
    else {
        if (state.lastEquipTypeHeld == EquipType::Melee) {
            buttonHold |= VPAD_BUTTON_Y;
            Haptics::Play(rightRumbleFixed);
        }
        else if (state.lastEquipTypeHeld == EquipType::Bow) {
            buttonHold |= VPAD_BUTTON_ZR;
            Haptics::Play(rightRumbleFixed);
        }
        else if (state.lastEquipTypeHeld == EquipType::SheikahSlate) {
            buttonHold |= VPAD_BUTTON_L;
            if ((state.previousButtonHold & VPAD_BUTTON_L) == 0) {
                Haptics::Play(rightRumbleFixed);
                Haptics::Play(leftRumbleFixed);
            }
        }
        else if (Player::IsRiding(true)) {
            buttonHold |= VPAD_BUTTON_Y;  // Soothe the horse
            Haptics::Play(rightRumbleFixed);
        }
    }
}

static void ProcessMenuInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    ControlState& state
) {
    if (!CloseDpadMenu(inputs, state))
        buttonHold |= ButtonIfHeld(inputs.inMenu.sort, VPAD_BUTTON_Y);

    if (!state.preventInputs) {
        buttonHold |= ButtonIfHeld(inputs.inMenu.back, VPAD_BUTTON_B);
        // only a press that started in the menu closes it
        if (s_inventoryMapButton.IsDown())
            buttonHold |= state.mapOpen ? VPAD_BUTTON_MINUS : VPAD_BUTTON_PLUS;
    }

    buttonHold |= ButtonIfHeld(inputs.inMenu.select, VPAD_BUTTON_A);
    buttonHold |= ButtonIfHeld(inputs.inMenu.leftTrigger, VPAD_BUTTON_L);
    buttonHold |= ButtonIfHeld(inputs.inMenu.rightTrigger, VPAD_BUTTON_R);

    if (s_menuHoldButton.lastEvent == ButtonState::Event::ShortPress)
        buttonHold |= VPAD_BUTTON_X;
}

static void ProcessInputPrevention(ControlState& state, std::chrono::steady_clock::time_point now, std::chrono::milliseconds delay)
{
    if (state.inGame != state.wasInGame) {
        state.preventInputs = true;
        state.preventInputsTime = now;
    }

    if (state.preventInputs && now >= state.preventInputsTime + delay)
        state.preventInputs = false;

    if (state.preventGrabInputs && now >= state.preventGrabTime + delay)
        state.preventGrabInputs = false;
}

static void ProcessModMenuInput(std::atomic_bool& isMenuOpen, OpenXR::InputState& inputs, VPADStatus& vpadInputs, XrActionStateVector2f& leftStickSource, XrActionStateVector2f& rightStickSource)
{
    if (s_modMenuButton.lastEvent == ButtonState::Event::LongPress) {
        isMenuOpen = !isMenuOpen;
    }

    ImGuiMenus::ProcessInputs(inputs, vpadInputs);

    if (isMenuOpen || ImGuiMenus::BlocksGameInput()/*this is used for the entity inspector*/) {
        vpadInputs = {};
        leftStickSource.currentState = { 0.0f, 0.0f };
        rightStickSource.currentState = { 0.0f, 0.0f };
    }
}

enum class GuideContext { None, OnFoot };

struct GuideState {
    std::mutex mutex;
    Game::Guide guide = {};
    GuideContext context = GuideContext::None;
    XrTime shownAt = 0;
};
static GuideState s_guide;

static GuideContext CurrentGuideContext() {
    if (Player::IsRiding() || Player::IsGliding())
        return GuideContext::None;
    return GuideContext::OnFoot;
}

static void SetGuide(const Game::Guide& guide, GuideContext context, XrTime shownAt) {
    std::scoped_lock lock(s_guide.mutex);
    if (guide.count == 0 && s_guide.guide.count == 0)
        return;
    const uint64_t generation = s_guide.guide.generation + 1;
    s_guide.guide = guide;
    s_guide.guide.generation = generation;
    s_guide.context = context;
    s_guide.shownAt = shownAt;
}

static Game::Guide BuildGuide(GuideContext context) {
    const float leftSide = -1.0f;

    auto place = [&](float sideways, float up, float ahead) {
        return glm::fvec3(sideways, up, ahead);
    };

    Game::Guide guide = {};
    auto& cards = guide.cards;
    if (context == GuideContext::OnFoot) {
        cards[0] = { ICON_MS_BOW_ARC, "Bow", nullptr, place(0.40f * leftSide, 0.02f, 0.36f), place(0.17f * leftSide, -0.12f, -0.10f), 0.09f };
        cards[1] = { ICON_MS_SWORDS, "Weapon", nullptr, place(-0.40f * leftSide, 0.02f, 0.36f), place(-0.17f * leftSide, -0.12f, -0.10f), 0.09f };
        cards[2] = { ICON_MS_TABLET_ANDROID, "Runes", nullptr, place(0.36f * leftSide, -0.36f, 0.42f), place(0.24f * leftSide, -0.62f, -0.02f), 0.10f };
        cards[3] = { ICON_MS_DOWNLOAD, "Drop", "Hold grip", place(-0.36f * leftSide, -0.36f, 0.42f), place(-0.24f * leftSide, -0.62f, -0.02f), 0.10f };
        cards[4] = { ICON_MS_SPORTS, "Whistle", "Both grips", place(0.0f, -0.24f, 0.44f), place(0.0f, -0.10f, 0.09f), 0.06f };
        cards[5] = { nullptr, nullptr, "Grip to equip, hold for menu", place(0.0f, 0.14f, 0.50f), {}, 0.0f };
        guide.count = 6;
    }
    return guide;
}

static void UpdateGuide(const OpenXR::InputState& inputs, bool menuOpen) {
    constexpr XrTime GuideDuration = 12'000'000'000;
    const XrTime now = inputs.shared.inputTime;
    const bool allowed = inputs.shared.in_game && !menuOpen && !ImGuiMenus::BlocksGameInput() && !Events::IsActive();
    const GuideContext context = allowed ? CurrentGuideContext() : GuideContext::None;

    bool visible;
    GuideContext shownContext;
    XrTime shownAt;
    {
        std::scoped_lock lock(s_guide.mutex);
        visible = s_guide.guide.count > 0;
        shownContext = s_guide.context;
        shownAt = s_guide.shownAt;
    }

    if (visible && (context != shownContext || now - shownAt >= GuideDuration)) {
        SetGuide({}, GuideContext::None, 0);
        visible = false;
    }
    if (context == GuideContext::None || s_modMenuButton.lastEvent != ButtonState::Event::ShortPress)
        return;
    if (visible)
        SetGuide({}, GuideContext::None, 0);
    else
        SetGuide(BuildGuide(context), context, now);
}

Game::Guide Controls::GetGuide() {
    std::scoped_lock lock(s_guide.mutex);
    return s_guide.guide;
}

static void ProcessHandGesture(const std::optional<glm::fmat4>& headsetPose, OpenXR::InputState& inputs, HandGestureState& leftGesture, HandGestureState& rightGesture, ControlState& state)
{
    if (headsetPose.has_value()) {
        const auto headsetMtx = headsetPose.value();
        const glm::fvec3 headsetPos(headsetMtx[3]);
        
        const auto leftHandPos = ToGLM(inputs.shared.poseLocation[0].pose.position);
        const auto rightHandPos = ToGLM(inputs.shared.poseLocation[1].pose.position);
        
        leftGesture = CalculateHandGesture(leftHandPos, headsetMtx, headsetPos);
        rightGesture = CalculateHandGesture(rightHandPos, headsetMtx, headsetPos);
    }
}

static void ProcessJoystickInput(VPADButtons& oldXRStickHold, VPADButtons& newXRStickHold, VPADStatus& vpadStatus, XrActionStateVector2f& leftStickSource, XrActionStateVector2f& rightStickSource, bool disableRightStickHorizontalRotation)
{
    vpadStatus.leftStick = { leftStickSource.currentState.x + vpadStatus.leftStick.x.getLE(), leftStickSource.currentState.y + vpadStatus.leftStick.y.getLE() };

    const float axisThreshold = GetSettings().axisThreshold;
    const float holdThreshold = axisThreshold * 0.5f;
    if (leftStickSource.currentState.x <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_LEFT) && leftStickSource.currentState.x <= -holdThreshold))
        newXRStickHold |= VPAD_STICK_L_EMULATION_LEFT;
    else if (leftStickSource.currentState.x >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_RIGHT) && leftStickSource.currentState.x >= holdThreshold))
        newXRStickHold |= VPAD_STICK_L_EMULATION_RIGHT;

    if (leftStickSource.currentState.y <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_DOWN) && leftStickSource.currentState.y <= -holdThreshold))
        newXRStickHold |= VPAD_STICK_L_EMULATION_DOWN;
    else if (leftStickSource.currentState.y >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_UP) && leftStickSource.currentState.y >= holdThreshold))
        newXRStickHold |= VPAD_STICK_L_EMULATION_UP;

    const float rightStickX = disableRightStickHorizontalRotation ? 0.0f : rightStickSource.currentState.x;
    vpadStatus.rightStick = { rightStickX + vpadStatus.rightStick.x.getLE(), rightStickSource.currentState.y + vpadStatus.rightStick.y.getLE() };

    if (!disableRightStickHorizontalRotation) {
        if (rightStickSource.currentState.x <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_LEFT) && rightStickSource.currentState.x <= -holdThreshold))
            newXRStickHold |= VPAD_STICK_R_EMULATION_LEFT;
        else if (rightStickSource.currentState.x >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_RIGHT) && rightStickSource.currentState.x >= holdThreshold))
            newXRStickHold |= VPAD_STICK_R_EMULATION_RIGHT;
    }

    if (rightStickSource.currentState.y <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_DOWN) && rightStickSource.currentState.y <= -holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_DOWN;
    else if (rightStickSource.currentState.y >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_UP) && rightStickSource.currentState.y >= holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_UP;

    oldXRStickHold = newXRStickHold;
}

static void RemapRightStickToCameraZoom(uint32_t& buttonHold, XrActionStateVector2f& rightStickSource) {
    if (!Screens::IsVisible(ScreenId::AppCamera_00)) {
        return;
    }

    const float axisThreshold = GetSettings().axisThreshold;
    if (rightStickSource.currentState.y >= axisThreshold) {
        buttonHold |= VPAD_BUTTON_UP;
    }
    else if (rightStickSource.currentState.y <= -axisThreshold) {
        buttonHold |= VPAD_BUTTON_DOWN;
    }

    rightStickSource.currentState = { 0.0f, 0.0f };
}

static void ReadHandObservations(ControlState& state) {
    const Weapons::HandObservations hands = Weapons::ReadHandObservations();
    state.leftHandEquipType = hands.equipTypes[OpenXR::EyeSide::LEFT];
    state.rightHandEquipType = hands.equipTypes[OpenXR::EyeSide::RIGHT];
    state.hasSomethingInLeftHand = hands.occupied[OpenXR::EyeSide::LEFT];
    state.hasSomethingInRightHand = hands.occupied[OpenXR::EyeSide::RIGHT];
    state.isThrowableObjectHeld = hands.throwableHeld;
}

static void UpdatePreviousValues(ControlState& state, uint32_t buttonHold, XrTime inputTime) {
    state.previousButtonHold = buttonHold;
    state.previousInputTime = inputTime;
    state.wasInGame = state.inGame;
    // a hand the weapon hook didn't report is empty
    if (state.inGame) {
        state.leftHandPreviousEquipType = state.leftHandEquipType;
        state.rightHandPreviousEquipType = state.rightHandEquipType;
        Weapons::ResetHandObservations();
    }
}

static const char* GetEquipTypeName(EquipType type) {
    switch (type) {
        case EquipType::None: return "None";
        case EquipType::Melee: return "Melee";
        case EquipType::Shield: return "Shield";
        case EquipType::Bow: return "Bow";
        case EquipType::Arrow: return "Arrow";
        case EquipType::SheikahSlate: return "SheikahSlate";
        case EquipType::MagnetGlove: return "MagnetGlove";
        case EquipType::ThrowableObject: return "ThrowableObject";
    }
    return "?";
}

static void TraceControlChanges(const ControlState& state, uint32_t buttonHold) {
    if (!Log::IsCategoryEnabled(CONTROLS))
        return;

    static constexpr std::array<std::pair<uint32_t, const char*>, 16> ButtonNames = { {
        { VPAD_BUTTON_A, "A" }, { VPAD_BUTTON_B, "B" }, { VPAD_BUTTON_X, "X" }, { VPAD_BUTTON_Y, "Y" },
        { VPAD_BUTTON_LEFT, "Left" }, { VPAD_BUTTON_RIGHT, "Right" }, { VPAD_BUTTON_UP, "Up" }, { VPAD_BUTTON_DOWN, "Down" },
        { VPAD_BUTTON_ZL, "ZL" }, { VPAD_BUTTON_ZR, "ZR" }, { VPAD_BUTTON_L, "L" }, { VPAD_BUTTON_R, "R" },
        { VPAD_BUTTON_PLUS, "Plus" }, { VPAD_BUTTON_MINUS, "Minus" }, { VPAD_BUTTON_STICK_R, "StickR" }, { VPAD_BUTTON_STICK_L, "StickL" },
    } };

    static uint32_t s_tracedHold = 0;
    if (buttonHold != s_tracedHold) {
        std::string changes;
        for (const auto& [mask, name] : ButtonNames) {
            if (((buttonHold ^ s_tracedHold) & mask) != 0)
                fmt::format_to(std::back_inserter(changes), " {}{}", (buttonHold & mask) != 0 ? '+' : '-', name);
        }
        Log::print<CONTROLS>("[Controls] buttons{}", changes);
        s_tracedHold = buttonHold;
    }

    static EquipType s_tracedLeft = EquipType::None;
    static EquipType s_tracedRight = EquipType::None;
    static bool s_tracedThrowable = false;
    if (state.inGame && (state.leftHandEquipType != s_tracedLeft || state.rightHandEquipType != s_tracedRight || state.isThrowableObjectHeld != s_tracedThrowable)) {
        Log::print<CONTROLS>("[Controls] hands left={} right={} throwable={}", GetEquipTypeName(state.leftHandEquipType), GetEquipTypeName(state.rightHandEquipType), state.isThrowableObjectHeld);
        s_tracedLeft = state.leftHandEquipType;
        s_tracedRight = state.rightHandEquipType;
        s_tracedThrowable = state.isThrowableObjectHeld;
    }

    static bool s_tracedInGame = false;
    static bool s_tracedModMenu = false;
    static bool s_tracedDpadMenu = false;
    const bool modMenuOpen = ImGuiMenus::g_isOpen;
    if (state.inGame != s_tracedInGame || modMenuOpen != s_tracedModMenu || state.dpadMenuOpenRequested != s_tracedDpadMenu) {
        Log::print<CONTROLS>("[Controls] inGame={} modMenu={} dpadMenu={}", state.inGame, modMenuOpen, state.dpadMenuOpenRequested);
        s_tracedInGame = state.inGame;
        s_tracedModMenu = modMenuOpen;
        s_tracedDpadMenu = state.dpadMenuOpenRequested;
    }
}

// for players without their VR controllers in reach
static void ToggleModMenuOnLongStartPress(uint32_t physicalHold) {
    static auto startBtnLastTime = std::chrono::steady_clock::now();
    static bool startBtnWasDown = false;
    static bool startBtnActionConsumed = false;

    if ((physicalHold & VPAD_BUTTON_PLUS) != 0) {
        if (!startBtnWasDown) {
            startBtnLastTime = std::chrono::steady_clock::now();
            startBtnActionConsumed = false;
        }
        else if (!startBtnActionConsumed && std::chrono::steady_clock::now() - startBtnLastTime > std::chrono::milliseconds(500)) {
            ImGuiMenus::g_isOpen = !ImGuiMenus::g_isOpen;
            startBtnActionConsumed = true;
        }
        startBtnWasDown = true;
    }
    else {
        startBtnWasDown = false;
    }
}

static float ControllerWalkYaw(const glm::fquat& controllerQuat) {
    static float s_lastYaw = 0.0f;
    static bool s_tippedOver = false;

    const glm::fvec3 forward = controllerQuat * glm::fvec3(0.0f, 0.0f, -1.0f);
    const glm::fvec3 up = controllerQuat * glm::fvec3(0.0f, 1.0f, 0.0f);
    const glm::fvec3 flatForward = glm::fvec3(forward.x, 0.0f, forward.z);
    if (glm::length(flatForward) < 0.2f) {
        return s_lastYaw;
    }

    const glm::fvec3 uprightUp = glm::normalize(glm::fvec3(0.0f, 1.0f, 0.0f) - forward * forward.y);
    const float upAlignment = glm::dot(up, uprightUp);
    if (std::abs(forward.y) < 0.5f || upAlignment > 0.3f) {
        s_tippedOver = false;
    }
    else if (upAlignment < -0.3f) {
        s_tippedOver = true;
    }

    const glm::fvec3 heading = s_tippedOver ? -flatForward : flatForward;
    s_lastYaw = std::atan2(heading.x, heading.z);
    return s_lastYaw;
}

static void ApplyControllerWalkingDirection(const ControlState& state, const OpenXR::InputState& inputs, const std::optional<glm::fmat4>& headsetPose, XrActionStateVector2f& leftStickSource) {
    if (!state.inGame || !GameState::IsFirstPerson() || GetSettings().GetWalkingDirection() != WalkingDirection::CONTROLLER || Player::IsClimbingLadder())
        return;

    const auto& leftPoseLoc = inputs.shared.poseLocation[OpenXR::EyeSide::LEFT];
    if (headsetPose.has_value() && (leftPoseLoc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
        const glm::fquat headsetQuat = glm::quat_cast(glm::fmat3(*headsetPose));
        const glm::fquat controllerQuat = ToGLM(leftPoseLoc.pose.orientation);
        const glm::fvec3 headsetFwd = headsetQuat * glm::fvec3(0.0f, 0.0f, -1.0f);
        const float headsetAngle = std::atan2(headsetFwd.x, headsetFwd.z);
        const float controllerAngle = ControllerWalkYaw(controllerQuat);
        const float rotationAngle = controllerAngle - headsetAngle;
        const float cosA = std::cos(rotationAngle);
        const float sinA = std::sin(rotationAngle);
        const float newX = leftStickSource.currentState.x * cosA - leftStickSource.currentState.y * sinA;
        const float newY = leftStickSource.currentState.x * sinA + leftStickSource.currentState.y * cosA;
        leftStickSource.currentState.x = newX;
        leftStickSource.currentState.y = newY;
    }
}

static void ProcessGameplayInput(uint32_t& buttonHold, OpenXR::InputState& inputs, ControlState& state, const HandGestureState& leftGesture, const HandGestureState& rightGesture, XrActionStateVector2f& leftStickSource, XrActionStateVector2f& rightStickSource, const std::chrono::steady_clock::time_point& now, float dt) {
    SpreadWeaponDetectionOverFrames(state);

    EquipWeaponOnDpadMenuExit(buttonHold, state, dt);

    if (!state.preventInputs) {
        // prevent jump when exiting menus with B button
        buttonHold |= ButtonIfHeld(inputs.inGame.jump_cancel, VPAD_BUTTON_X);

        if (s_crouchScopeButton.IsLongHeld()) {
            buttonHold |= VPAD_BUTTON_STICK_R;
        }

        if (s_inventoryMapButton.lastEvent == ButtonState::Event::ShortPress) {
            buttonHold |= VPAD_BUTTON_PLUS;
            state.mapOpen = false;
        }
        if (s_inventoryMapButton.IsLongHeld()) {
            buttonHold |= VPAD_BUTTON_MINUS;
            state.mapOpen = true;
        }
    }

    if (s_crouchScopeButton.lastEvent == ButtonState::Event::ShortPress && Events::IsActive()) {
        Events::ToggleCameraMode();
    }
    else if (s_crouchScopeButton.lastEvent == ButtonState::Event::ShortPress) {
        buttonHold |= VPAD_BUTTON_STICK_L;
    }
    
    // Optional rune inputs (for seated players)
    OpenDpadMenuRuneButton(s_runeButton.lastEvent, buttonHold, state);
    if (s_runeButton.lastEvent == ButtonState::Event::ShortPress) {
        buttonHold |= VPAD_BUTTON_L;  // Equip rune
        state.lastEquipTypeHeld = EquipType::SheikahSlate;
    }
    
    // If climbing or paragliding, make the B button cancel instantly the action instead of long press to run
    if (Player::IsClimbing() || Player::IsGliding()) {
        buttonHold |= ButtonIfHeld(inputs.inGame.run_interact, VPAD_BUTTON_B);
    }
    else if (Player::IsRiding())
    {
        buttonHold |= ButtonIfHeld(inputs.inGame.run_interact, VPAD_BUTTON_A);
        // Skip when the respective hand is inside a body slot so grips can still equip/unequip
        if (s_grabButtons[1].IsDown() && IsHandNotOverAnySlot(rightGesture))
            buttonHold |= VPAD_BUTTON_A;
        if (s_grabButtons[0].IsDown() && IsHandNotOverAnySlot(leftGesture))
            buttonHold |= VPAD_BUTTON_B;
    }
    else {
        if (s_runButton.IsLongHeld()) {
            buttonHold |= VPAD_BUTTON_B; // Run
        }
        else
            buttonHold |= ButtonIfHeld(inputs.inGame.run_interact, VPAD_BUTTON_A);
    }

    if (IsHandOverMouthSlot(leftGesture) && IsHandOverMouthSlot(rightGesture)) {
        if (s_grabButtons[0].IsDown() && s_grabButtons[1].IsDown()) {
            Haptics::Play({ true, 0, RumbleType::OscillationRaisingSawtoothWave, 1.0f, false, 0.25, 0.2f, 0.2f });
            buttonHold |= VPAD_BUTTON_DOWN;
        }
    }
    
    const bool suppressGrabActionsForTwoHandGrip = Skeleton::IsTwoHandGripActive() || Skeleton::TwoHandGripConsumesGrabInput() || state.handThrowCarrying;
    ProcessLeftHandInGameInput(buttonHold, inputs, state, leftGesture, rightStickSource, now, dt, suppressGrabActionsForTwoHandGrip);
    ProcessRightHandInGameInput(buttonHold, inputs, state, rightGesture, rightStickSource, now, suppressGrabActionsForTwoHandGrip);
    const bool bothGripInputsReleased = inputs.inGame.grab[OpenXR::EyeSide::LEFT].currentState <= Weapons::TwoHandGripButtonThreshold && inputs.inGame.grab[OpenXR::EyeSide::RIGHT].currentState <= Weapons::TwoHandGripButtonThreshold;
    if (Skeleton::TwoHandGripConsumesGrabInput() && bothGripInputsReleased)
        Skeleton::ReleaseTwoHandGripGrabInput();
    
    ProcessHandThrow(buttonHold, inputs, state);
    ProcessLeftTriggerBindings(buttonHold, inputs, state);
    ProcessRightTriggerBindings(buttonHold, inputs, state, rightGesture);

    RemapRightStickToCameraZoom(buttonHold, rightStickSource);
}

static void hook_InjectXRInput(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;
    const int32_t physicalSampleCount = (int32_t)hCPU->gpr[3];
    hCPU->gpr[3] = 0;
    if (hCPU->gpr[4] == 0 || hCPU->gpr[5] == 0)
        return;

    uint32_t vpadStatusOffset = hCPU->gpr[4];
    VPADStatus vpadStatus = {};

    if (!Tracking::IsSessionRunning()) {
        return;
    }

    if (physicalSampleCount > 0)
        Guest::Read(vpadStatusOffset, &vpadStatus);

    ToggleModMenuOnLongStartPress(vpadStatus.hold.getLE());

    // take the hands and head once so the gestures below all use the same sample
    OpenXR::InputState inputs = Tracking::SampleHands(GameState::IsShowingMenu());
    if (inputs.shared.inputTime <= 0)
        return;
    const std::optional<glm::fmat4> headsetPose = Tracking::HeadPose();
    UpdateControlButtons(inputs, ImGuiMenus::g_isOpen.load());

    ControlState& state = s_state;
    const float dt = (float)(inputs.shared.inputTime - state.previousInputTime) / 1000000000.0f;
    state.inGame = inputs.shared.in_game;
    ReadHandObservations(state);
    if (!state.inGame)
        Weapons::CancelDrops();

    uint32_t newXRBtnHold = 0;

    XrActionStateVector2f& leftStickSource = state.inGame ? inputs.inGame.move : inputs.inMenu.navigate;
    XrActionStateVector2f& rightStickSource = state.inGame ? inputs.inGame.camera : inputs.inMenu.scroll;

    float stickDeadzone = GetSettings().stickDeadzone;
    auto applyDeadzone = [stickDeadzone](XrVector2f& v) {
        if (std::abs(v.x) < stickDeadzone) v.x = 0.0f;
        if (std::abs(v.y) < stickDeadzone) v.y = 0.0f;
    };
    applyDeadzone(leftStickSource.currentState);
    applyDeadzone(rightStickSource.currentState);

    ApplyControllerWalkingDirection(state, inputs, headsetPose, leftStickSource);

    //Delay to wait before allowing specific inputs again
    constexpr std::chrono::milliseconds delay{ 400 };
    const auto now = std::chrono::steady_clock::now();

    ProcessInputPrevention(state, now, delay);

    auto& isMenuOpen = ImGuiMenus::g_isOpen;
    ProcessModMenuInput(isMenuOpen, inputs, vpadStatus, leftStickSource, rightStickSource);
    UpdateGuide(inputs, isMenuOpen);
    HandGestureState leftGesture = {};
    HandGestureState rightGesture = {};
    ProcessHandGesture(headsetPose, inputs, leftGesture, rightGesture, state);
    UpdatePullSlots(inputs, leftGesture, rightGesture);

    KeepDpadMenuOpen(newXRBtnHold, state);

    if (isMenuOpen) {
        Weapons::CancelDrops();
    }
    else if (state.inGame) {
        ProcessGameplayInput(newXRBtnHold, inputs, state, leftGesture, rightGesture, leftStickSource, rightStickSource, now, dt);
    }
    else {
        ProcessMenuInput(newXRBtnHold, inputs, state);
    }

    UpdateFirstPersonTurning(rightStickSource, state.inGame);

    Haptics::Update();

    const bool disableRightStickHorizontalRotation = ShouldOverrideFirstPersonTurning(state.inGame) && !ImGuiMenus::g_isOpen && Camera::AppliesFirstPersonTurning();
    VPADButtons newXRStickHold = VPAD_BUTTON_NONE;
    ProcessJoystickInput(state.previousStickHold, newXRStickHold, vpadStatus, leftStickSource, rightStickSource, disableRightStickHorizontalRotation);

    uint32_t combinedHold = (vpadStatus.hold.getLE() | (newXRBtnHold | newXRStickHold));
    vpadStatus.hold = combinedHold;
    vpadStatus.trig = (combinedHold & ~state.previousCombinedHold) | (vpadStatus.trig.getLE() & combinedHold);
    vpadStatus.release = (~combinedHold & state.previousCombinedHold);
    state.previousCombinedHold = combinedHold;

    vpadStatus.vpadErr = 0;
    vpadStatus.batteryLevel = 0xC0;

    vpadStatus.tpData.touch = 0;
    vpadStatus.tpData.validity = 3;

    OpenXRMotionBridge::UpdateVPADStatus(inputs, vpadStatus);

    Guest::Write(vpadStatusOffset, &vpadStatus);

    hCPU->gpr[3] = 1;

    TraceControlChanges(state, newXRBtnHold);
    UpdatePreviousValues(state, newXRBtnHold, inputs.shared.inputTime);
}


static void hook_XRRumble_VPADControlMotor(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint8_t* pattern = Guest::HostPointer<uint8_t>(hCPU->gpr[4]);
    uint8_t length = (uint8_t)hCPU->gpr[5];
    Haptics::PlayGamePattern(pattern, length);
}

static void hook_XRRumble_VPADStopMotor(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    Haptics::StopGamePattern();
}

void Controls::RegisterHooks() {
    CemuHost::RegisterHook("hook_InjectXRInput", &hook_InjectXRInput);
    CemuHost::RegisterHook("hook_XRRumble_VPADControlMotor", &hook_XRRumble_VPADControlMotor);
    CemuHost::RegisterHook("hook_XRRumble_VPADStopMotor", &hook_XRRumble_VPADStopMotor);
}
