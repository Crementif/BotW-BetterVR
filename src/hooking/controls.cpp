#include "cemu_hooks.h"
#include "../instance.h"
#include "openxr_motion_bridge.h"


void spreadWeaponDetectionOverFrames(OpenXR::GameState& gameState) {
    // Spread the weapon detection from link's attachement bones over several frames.
    // Each bone isn't necessarily checked each frames which can give wrong results sometimes
    // Also, some animations seem to make the weapon not detected correctly (shooting arrow)
    constexpr uint8_t REQUIRED_FRAMES = 5;
    if (gameState.right_hand_current_equip_type != gameState.right_hand_previous_frame_equip_type) {
        gameState.right_hand_equip_type_change_requested_over_frames++;
        if (gameState.right_hand_equip_type_change_requested_over_frames > REQUIRED_FRAMES) {
            gameState.right_hand_equip_type_change_requested_over_frames = 0;
        }
        else
            gameState.right_hand_current_equip_type = gameState.right_hand_previous_frame_equip_type;
    }
    else {
        gameState.right_hand_equip_type_change_requested_over_frames = 0;
    }

    if (gameState.left_hand_current_equip_type != gameState.left_hand_previous_frame_equip_type) {
        gameState.left_hand_equip_type_change_requested_over_frames++;
        if (gameState.left_hand_equip_type_change_requested_over_frames > REQUIRED_FRAMES) {
            gameState.left_hand_equip_type_change_requested_over_frames = 0;
        }
        else
            gameState.left_hand_current_equip_type = gameState.left_hand_previous_frame_equip_type;
    }
    else {
        gameState.left_hand_equip_type_change_requested_over_frames = 0;
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
    bool isFarEnoughFromStoredPosition;
    float magnesisForwardAmount;
    float magnesisVerticalAmount;
};

int getMagnesisForwardFrameInterval(float v)
{
    if (v <= 0.0f)    return 0;
    if (v <= 0.25f) return 10;
    if (v < 0.5f)   return 5;
    if (v < 0.75f)  return 2;
    return 1;
}

// Gesture detection functions
HandGestureState calculateHandGesture(
    OpenXR::GameState& gameState,
    const glm::fvec3& handPos,
    const glm::fmat4& headsetMatrix,
    const glm::fvec3& headsetPos,
    const bool ProcessStoredPositionDistanceCheck,
    const glm::fvec3& storedHandPos
) {
    HandGestureState gesture = {};
    
    // Calculate directional vectors
    glm::fvec3 headsetForward = -glm::normalize(glm::fvec3(headsetMatrix[2]));
    headsetForward.y = 0.0f;
    headsetForward = glm::normalize(headsetForward);
    
    glm::fvec3 headsetRight = glm::normalize(glm::fvec3(headsetMatrix[0]));
    glm::fvec3 headToHand = handPos - headsetPos;
    
    // Check if hand is behind head (for shoulder slots)
    float forwardDot = glm::dot(headsetForward, headToHand);
    gesture.isBehindHead = (forwardDot < 0.0f);
    
    // Check if hand is behind head (for waist slot) - uses flattened vectors
    glm::vec3 flatForward = glm::normalize(glm::vec3(headsetForward.x, 0.0f, headsetForward.z));
    glm::vec3 flatHandOffset = glm::vec3(headToHand.x, 0.0f, headToHand.z);
    
    constexpr float WAIST_BEHIND_OFFSET = -0.05f; //0.15f
    float flatForwardDot = glm::dot(flatForward, flatHandOffset) + WAIST_BEHIND_OFFSET;
    gesture.isBehindHeadWithWaistOffset = (flatForwardDot < 0.0f);
    
    // Check which side of body the hand is on
    float rightDot = glm::dot(headsetRight, headToHand);
    gesture.isOnLeftSide = (rightDot < 0.0f);
    
    // Check distance from head (for shoulder slots)
    constexpr float SHOULDER_RADIUS = 0.35f;
    constexpr float SHOULDER_RADIUS_SQ = SHOULDER_RADIUS * SHOULDER_RADIUS;
    gesture.isCloseToHead = (glm::length2(headToHand) < SHOULDER_RADIUS_SQ);

    // Check distance from head (for mouth slot)
    constexpr float MOUTH_RADIUS = 0.2f;
    constexpr float MOUTH_RADIUS_SQ = MOUTH_RADIUS * MOUTH_RADIUS;
    gesture.isCloseToMouth = (glm::length2(headToHand) < MOUTH_RADIUS_SQ);
    
    // Check distance from waist (rough estimate)
    glm::fvec3 waistPos = headsetPos - glm::fvec3(0.0f, 0.45f, 0.0f);
    gesture.isCloseToWaist = (handPos.y < waistPos.y);

    // Check hand height for shield (rough estimate)
    glm::fvec3 chestPos = headsetPos - glm::fvec3(0.0f, 0.3f, 0.0f);
    gesture.isNearChestHeight = (handPos.y > chestPos.y);

    // Check distance from stored position
    if (ProcessStoredPositionDistanceCheck) {
        constexpr float DISTANCE_THRESHOLD = 0.015f;
        constexpr float MAX_HAND_DISTANCE = 0.075f;
        auto delta = handPos - storedHandPos;
        auto distance = glm::length(delta);
        gesture.isFarEnoughFromStoredPosition = distance > DISTANCE_THRESHOLD;

        // Process magnesis gesture
        if (!gesture.isFarEnoughFromStoredPosition) {
            gesture.magnesisForwardAmount = gesture.magnesisVerticalAmount = 0.0f;
        }
        else {
            const glm::vec3 headsetUp(0.0f, 1.0f, 0.0f);
            auto forwardAmount = glm::dot(delta, headsetForward);
            auto verticalAmount = glm::dot(delta, headsetUp);

            auto remapSigned = [&](float value) {
                float sign = glm::sign(value);
                float absValue = glm::abs(value);

                float t = (absValue - DISTANCE_THRESHOLD) /
                          (MAX_HAND_DISTANCE - DISTANCE_THRESHOLD);

                t = glm::clamp(t, 0.0f, 1.0f);
                return t * sign;
            };

            gesture.magnesisForwardAmount = remapSigned(forwardAmount);
            gesture.magnesisVerticalAmount = remapSigned(verticalAmount);
            //Log::print<INFO>("magnesisForwardAmount frames skip : {}", gameState.magnesis_forward_frames_interval);
            if (gameState.magnesis_forward_frames_interval > 0) {
                gesture.magnesisForwardAmount = 0.0f;
            }
            else if (gameState.magnesis_forward_frames_interval <= -1)
            {
                gameState.magnesis_forward_frames_interval = getMagnesisForwardFrameInterval(glm::abs(gesture.magnesisForwardAmount));
            }
            
            gameState.magnesis_forward_frames_interval--;
            //Log::print<INFO>("gesture.magnesisForwardAmount  : {}", gesture.magnesisForwardAmount);
        }
    }

    return gesture;
}

bool isHandOverLeftShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && gesture.isOnLeftSide;
}

bool isHandOverRightShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && !gesture.isOnLeftSide;
}

bool isHandOverLeftWaistSlot(const HandGestureState& gesture) {
    return gesture.isBehindHeadWithWaistOffset && gesture.isCloseToWaist && gesture.isOnLeftSide;
}

bool isHandOverRightWaistSlot(const HandGestureState& gesture) {
    return gesture.isBehindHeadWithWaistOffset && gesture.isCloseToWaist && !gesture.isOnLeftSide;
}

bool isHandOverMouthSlot(const HandGestureState& gesture) {
    return gesture.isCloseToMouth && !gesture.isBehindHead;
}

bool isHandFarEnoughFromStoredPosition(const HandGestureState& gesture) {
    return gesture.isFarEnoughFromStoredPosition;
}

bool isHandNotOverAnySlot(const HandGestureState& gesture) {
    return !gesture.isBehindHead && !gesture.isBehindHeadWithWaistOffset && !gesture.isCloseToMouth;
}

bool openDpadMenuRuneButton(ButtonState::Event lastEvent, uint32_t& buttonHold, OpenXR::GameState& gameState) {
    if (lastEvent == ButtonState::Event::LongPress && gameState.right_hand_current_equip_type != EquipType::MagnetGlove) {
        if (gameState.left_hand_current_equip_type == EquipType::SheikahSlate) {
            gameState.dpad_menu_selection_already_equipped = true;
            gameState.rune_need_reequip = true;
        }
        buttonHold |= VPAD_BUTTON_UP;
        gameState.last_dpad_menu_open = EquipType::SheikahSlate;
        gameState.dpad_menu_open_requested = true;
        gameState.current_dpad_menu_button = OpenXR::DpadMenuButton::Rune;
        return true;
    }
    return false;
}

bool openDpadMenuBodySlots(ButtonState::Event lastEvent, HandGestureState handGesture, uint32_t& buttonHold, OpenXR::GameState& gameState, bool (*gripButton)(OpenXR::InputState)) {
    if (lastEvent == ButtonState::Event::LongPress && !gameState.dpad_menu_open_requested && gameState.right_hand_current_equip_type != EquipType::MagnetGlove) {
        if (isHandOverRightShoulderSlot(handGesture)) {
            //open arrow menu if bow is equipped in left hand
            if (gameState.left_hand_current_equip_type == EquipType::Bow) {
                buttonHold |= VPAD_BUTTON_LEFT;
                gameState.last_dpad_menu_open = EquipType::Arrow;
            }
            //else open melee weapon menu
            else {
                if (gameState.right_hand_current_equip_type == EquipType::Melee)
                    gameState.dpad_menu_selection_already_equipped = true;
                buttonHold |= VPAD_BUTTON_RIGHT;
                gameState.last_dpad_menu_open = EquipType::Melee;
            }
        }
        else if (isHandOverLeftShoulderSlot(handGesture)) {
            //open shield menu if melee is equipped in right hand
            if (gameState.right_hand_current_equip_type == EquipType::Melee) {
                buttonHold |= VPAD_BUTTON_LEFT;
                gameState.last_dpad_menu_open = EquipType::Shield;
            }
            //else open bow menu
            else {
                // Force a bow equip so the correct menu opens.
                if (gameState.left_hand_current_equip_type != EquipType::Bow)
                    buttonHold |= VPAD_BUTTON_ZR;
                else
                    gameState.dpad_menu_selection_already_equipped = true;
                buttonHold |= VPAD_BUTTON_RIGHT;
                gameState.last_dpad_menu_open = EquipType::Bow;
            }
        }
        // if not over shoulders slots, then it's over waist
        else {
            if (gameState.left_hand_current_equip_type == EquipType::SheikahSlate) {
                gameState.dpad_menu_selection_already_equipped = true;
                gameState.rune_need_reequip = true;
            }
            buttonHold |= VPAD_BUTTON_UP;
            gameState.last_dpad_menu_open = EquipType::SheikahSlate;
        }
        gameState.dpad_menu_open_requested = true;
        gameState.current_dpad_menu_button = gripButton;
        return true;
    }
    return false;
}

void keepDpadMenuOpen(uint32_t& buttonHold, OpenXR::GameState& gameState) {
    if (!gameState.dpad_menu_open_requested)
        return;

    switch (gameState.last_dpad_menu_open) {
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

void closeDpadMenu(OpenXR::InputState& inputs, OpenXR::GameState& gameState) {
    if (!gameState.dpad_menu_open_requested)
        return;

    if (!gameState.current_dpad_menu_button(inputs)) {
        gameState.dpad_menu_open_requested = false;
        gameState.was_dpad_menu_open = true;
        gameState.current_dpad_menu_button = nullptr;
    }
}

void equipWeaponOnDpadMenuExit(uint32_t& buttonHold, OpenXR::GameState& gameState, float dt) {
    // if dpad menu was just closed, equip the weapon/item from the last dpad menu opened if not already equipped
    if (gameState.rune_need_reequip) {
        constexpr float RUNE_REEQUIP_TIME = 0.5f;
        if (gameState.rune_reequip_timer >= RUNE_REEQUIP_TIME) {
            // If rune selected in dpad menu is not the same as the one already in hand -> reequip
            if (gameState.left_hand_current_equip_type != EquipType::SheikahSlate)
                buttonHold |= VPAD_BUTTON_L;
            gameState.rune_need_reequip = false;
            gameState.rune_reequip_timer = 0.0f;
        }
        else
            gameState.rune_reequip_timer += dt;
    }

    if (!gameState.was_dpad_menu_open) 
        return;

    if (!gameState.dpad_menu_selection_already_equipped) {
        switch (gameState.last_dpad_menu_open) {
            case EquipType::Melee:
                buttonHold |= VPAD_BUTTON_Y;
                gameState.last_equip_type_held = EquipType::Melee;
                break;
            case EquipType::Bow:
                buttonHold |= VPAD_BUTTON_ZR;
                gameState.last_equip_type_held = EquipType::Bow;
                break;
            case EquipType::SheikahSlate:
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_equip_type_held = EquipType::SheikahSlate;
                break;
        }
    }
    else
        gameState.dpad_menu_selection_already_equipped = false;

    gameState.was_dpad_menu_open = false;
    gameState.last_dpad_menu_open = EquipType::None;
}

// Input handling functions
void processLeftHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    const HandGestureState& leftGesture,
    XrActionStateVector2f& rightStickSource,
    const std::chrono::steady_clock::time_point& now,
    float dt
) {
    constexpr std::chrono::milliseconds INPUT_DELAY(400);
    
    constexpr RumbleParameters leftRumbleRaise = { true, 0, RumbleType::Raising, 0.5f, false, 0.2, 1.0f, 1.0f };
    constexpr RumbleParameters leftRumbleFall = { true, 0, RumbleType::Falling, 0.5f, false, 0.3, 0.1f, 0.75f };
    constexpr RumbleParameters RuneRumble = { true, 0, RumbleType::OscillationSmooth, 1.0f, false, 1.0, 0.25f, 0.25f };
    
    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();
    bool isGrabPressed = inputs.shared.grabState[0].lastEvent == ButtonState::Event::ShortPress;
    bool isGrabPressedLong = inputs.shared.grabState[0].lastEvent == ButtonState::Event::LongPress;
    bool isCurrentGrabPressed = inputs.shared.grabState[0].wasDownLastFrame;
    bool isCurrentInteractPressed = inputs.shared.grabState[0].wasDownLastFrame;
    
    // Rune rumbles
    if (gameState.left_hand_current_equip_type == EquipType::SheikahSlate)
        rumbleMgr->enqueueInputsRumbleCommand(RuneRumble);

    // Handle shield
    // if shield with lock on isn't already being used with left trigger, use gesture to guard without lock on instead.
    // Gesture enabled only when both melee weapon and shield are in hands to prevent 2 handed weapons and quick drawing shield 
    // alone with Left Trigger to trigger it. So people can still move hands freely without the shield appearing when not wanted.
    if (!inputs.inGame.useLeftItem.currentState && gameState.left_hand_current_equip_type == EquipType::Shield && gameState.right_hand_current_equip_type == EquipType::Melee && (leftGesture.isNearChestHeight)) {
        buttonHold |= VPAD_BUTTON_ZL;
        rightStickSource.currentState.y = 0.2f; // Force disable the lock on view when holding shield
        gameState.is_shield_guarding = true;
        if ((gameState.previous_button_hold & VPAD_BUTTON_ZL) == 0)
            rumbleMgr->enqueueInputsRumbleCommand(leftRumbleRaise);
    }
    else if (!inputs.inGame.useLeftItem.currentState)
        gameState.is_shield_guarding = false;

    if (!gameState.is_shield_guarding && gameState.previous_button_hold & VPAD_BUTTON_ZL)
        rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFall);

    // Handle Parry gesture
    float ACCELERATION_THRESHOLD = 35.0f;
    auto handVelocity = glm::length(ToGLM(inputs.shared.poseVelocity[0].linearVelocity));
    float acceleration = (handVelocity - gameState.previous_left_hand_velocity) / dt;
    gameState.previous_left_hand_velocity = handVelocity;
    if (acceleration > ACCELERATION_THRESHOLD && gameState.left_hand_current_equip_type == EquipType::Shield && gameState.is_shield_guarding) {
        buttonHold |= VPAD_BUTTON_A;
        rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFall);
    }

    // Handle shoulder slot interactions
    if (isHandOverLeftShoulderSlot(leftGesture) || isHandOverRightShoulderSlot(leftGesture)) {
        if (openDpadMenuBodySlots(inputs.shared.grabState[0].lastEvent, leftGesture, buttonHold, gameState, OpenXR::DpadMenuButton::LGrab))
            // Don't process normal input when opening dpad menu
            return;

        // Handle equip/unequip
        if (!gameState.prevent_grab_inputs && isGrabPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFall);
            if (isHandOverLeftShoulderSlot(leftGesture)) {
                // Left shoulder = Bow
                if (gameState.left_hand_current_equip_type != EquipType::Bow) {
                    buttonHold |= VPAD_BUTTON_ZR;
                    gameState.last_equip_type_held = EquipType::Bow;
                } else {
                    buttonHold |= VPAD_BUTTON_B;  // Unequip
                }
            }
            else {
                // Right shoulder = Melee weapon
                if (gameState.right_hand_current_equip_type != EquipType::Melee) {
                    buttonHold |= VPAD_BUTTON_Y;
                    gameState.last_equip_type_held = EquipType::Melee;
                } else {
                    buttonHold |= VPAD_BUTTON_B;  // Unequip
                }
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        return;  // Don't process normal input when over slots
    }
    
    // Handle waist slot interaction (Rune)
    if (isHandOverLeftWaistSlot(leftGesture)) {    
        // Handle dpad menu
        if (openDpadMenuBodySlots(inputs.shared.grabState[0].lastEvent, leftGesture, buttonHold, gameState, OpenXR::DpadMenuButton::LGrab))
            // Don't process normal input when opening dpad menu
            return;

        if (!gameState.prevent_grab_inputs && isGrabPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFall);
            if (gameState.left_hand_current_equip_type != EquipType::SheikahSlate) {
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_equip_type_held = EquipType::SheikahSlate;
            } else {
                buttonHold |= VPAD_BUTTON_B;  // Unequip
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        return;
    }

    if (isCurrentGrabPressed) {
        // Magnesis motion controls
        if (gameState.right_hand_current_equip_type == EquipType::MagnetGlove) {
            // null right joystick Y to let the magnesis motion controls handle it.
            rightStickSource.currentState.y = 0.0f;

            if (!gameState.left_hand_position_stored) {
                gameState.stored_left_hand_position = ToGLM(inputs.shared.poseLocation[0].pose.position);
                gameState.left_hand_position_stored = true;
                rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFall);
            }

            if (gameState.left_hand_position_stored) {
                rightStickSource.currentState.y = leftGesture.magnesisVerticalAmount;
                if (leftGesture.magnesisForwardAmount > 0.0f)
                    buttonHold |= VPAD_BUTTON_UP;
                else if (leftGesture.magnesisForwardAmount < 0.0f)
                    buttonHold |= VPAD_BUTTON_DOWN;
            }
        }

        // Pull gesture
        if (gameState.left_hand_was_over_left_shoulder_slot) {
            if (gameState.left_hand_current_equip_type != EquipType::Bow) {
                buttonHold |= VPAD_BUTTON_ZR;
                gameState.last_equip_type_held = EquipType::Bow;
            }
        }
        else if (gameState.left_hand_was_over_right_shoulder_slot) {
            if (gameState.right_hand_current_equip_type != EquipType::Melee) {
                buttonHold |= VPAD_BUTTON_Y;
                gameState.last_equip_type_held = EquipType::Melee;
            }
        }
        else if (gameState.left_hand_was_over_left_waist_slot) {
            if (gameState.left_hand_current_equip_type != EquipType::SheikahSlate) {
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_equip_type_held = EquipType::SheikahSlate;
            }
        }
    }
    else
        gameState.left_hand_position_stored = false;

    // Left hand item drop broken rn, makes the game thinks link is empty handed in right hand too.
    // Waiting for a fix before uncommenting
    //if (isHandOverRightWaistSlot(leftGesture))
    //{
    //    if (isCurrentGrabPressed)
    //        rumbleMgr->enqueueInputsRumbleCommand(grabSlotRumble);
    //    //Handle drop action
    //    if (isGrabPressedLong) {
    //        inputs.inGame.drop_weapon[0] = true;
    //        gameState.prevent_grab_inputs = true;
    //        gameState.prevent_grab_time = now;
    //    }
    //    return;
    //}

    // Handle interact action. is_riding_mount check added to prevent conflict with master cycle brake function
    if (!gameState.prevent_grab_inputs && !gameState.is_riding_mount) {
        if (isHandNotOverAnySlot(leftGesture) && isCurrentInteractPressed) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
}

void processRightHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    const HandGestureState& rightGesture,
    XrActionStateVector2f& rightStickSource,
    const std::chrono::steady_clock::time_point& now
) {
    constexpr std::chrono::milliseconds INPUT_DELAY(400);
    
    constexpr RumbleParameters rightRumbleFall = { true, 1, RumbleType::Falling, 0.5f, false, 0.3, 0.1f, 0.75f };
    constexpr RumbleParameters rightRumbleInfiniteRaise = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters OverSlotsRumble = { false, 1, RumbleType::OscillationRaisingSawtoothWave, 1.0f, false, 1.0, 0.25f, 0.25f };

    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();
    bool isGrabPressedShort = inputs.shared.grabState[1].lastEvent == ButtonState::Event::ShortPress;
    bool isGrabPressedLong = inputs.shared.grabState[1].lastEvent == ButtonState::Event::LongPress;
    bool isCurrentGrabPressed = inputs.shared.grabState[1].wasDownLastFrame;
    bool isCurrentInteractPressed = inputs.inGame.interactState[1].wasDownLastFrame;
    bool isTriggerPressed = inputs.inGame.useRightItem.currentState;
    
    // Handle shoulder slot interactions
    if (isHandOverLeftShoulderSlot(rightGesture) || isHandOverRightShoulderSlot(rightGesture)) {
        // Handle dpad menu
        if (openDpadMenuBodySlots(inputs.shared.grabState[1].lastEvent, rightGesture, buttonHold, gameState, OpenXR::DpadMenuButton::RGrab))
            // Don't process normal input when opening dpad menu
            return;

        // Haptics to help finding the body slot for weapons throw
        if (isHandOverRightShoulderSlot(rightGesture) && gameState.right_hand_current_equip_type == EquipType::Melee && !inputs.inGame.useRightItem.currentState)
            rumbleMgr->enqueueInputsRumbleCommand(OverSlotsRumble);

        // Handle equip/unequip
        if (!gameState.prevent_grab_inputs && isGrabPressedShort) { 
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
            if (isHandOverRightShoulderSlot(rightGesture)) {
                // Right shoulder = Melee weapon
                if (gameState.right_hand_current_equip_type != EquipType::Melee) {
                    buttonHold |= VPAD_BUTTON_Y;
                    gameState.last_equip_type_held = EquipType::Melee;
                } else {
                    buttonHold |= VPAD_BUTTON_B;
                }
            }
            else {
                // Left shoulder = Bow
                if (gameState.left_hand_current_equip_type != EquipType::Bow) {
                    buttonHold |= VPAD_BUTTON_ZR;
                    gameState.last_equip_type_held = EquipType::Bow;
                } else {
                    buttonHold |= VPAD_BUTTON_B;
                }
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }

        
        if (isTriggerPressed)
        {
            // Keep going or start shoot arrow action even if hand gets over right shoulder slot
            if (gameState.left_hand_current_equip_type == EquipType::Bow)
            {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleInfiniteRaise);
                buttonHold |= VPAD_BUTTON_ZR;
                gameState.trigger_pressed_over_body_slot = true;
            }
            // Handle weapon throw
            else if (gameState.right_hand_current_equip_type == EquipType::Melee) {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleInfiniteRaise);
                buttonHold |= VPAD_BUTTON_R;
                gameState.trigger_pressed_over_body_slot = true;
            }
        }
        else if (gameState.trigger_pressed_over_body_slot) {
            rumbleMgr->stopInputsRumble(1, RumbleType::Raising);
            gameState.trigger_pressed_over_body_slot = false;
        }
        return;  // Don't process normal input when over slots
    }
    // if hand not on shoulder slot but trigger still pressed, stop throw rumbles
    else if (gameState.trigger_pressed_over_body_slot && gameState.right_hand_current_equip_type == EquipType::Melee) {
        rumbleMgr->stopInputsRumble(1, RumbleType::Raising);
    }

    // Handle waist slot interaction (Rune)
    if (isHandOverLeftWaistSlot(rightGesture)) {   
        // Handle dpad menu
        if (openDpadMenuBodySlots(inputs.shared.grabState[1].lastEvent, rightGesture, buttonHold, gameState, OpenXR::DpadMenuButton::RGrab))
            // Don't process normal input when opening dpad menu
            return;

        if (!gameState.prevent_grab_inputs && isGrabPressedShort) {
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
            if (gameState.left_hand_current_equip_type != EquipType::SheikahSlate) {
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_equip_type_held = EquipType::SheikahSlate;
            } else {
                buttonHold |= VPAD_BUTTON_B;  // Unequip
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        return;
    }

    if (isHandOverRightWaistSlot(rightGesture))
    {
        //Handle drop action
        if (isGrabPressedLong) {
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
            inputs.inGame.drop_weapon[1] = true;
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        return;
    }
    
    if (isCurrentGrabPressed) {
        // Magnesis motion controls
        if (gameState.right_hand_current_equip_type == EquipType::MagnetGlove) {
            // null right joystick Y to let the magnesis motion controls handle it.
            rightStickSource.currentState.y = 0.0f;

            if (!gameState.right_hand_position_stored) {
                gameState.stored_right_hand_position = ToGLM(inputs.shared.poseLocation[1].pose.position);
                gameState.right_hand_position_stored = true;
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
            }

            if (gameState.right_hand_position_stored) {
                rightStickSource.currentState.y = rightGesture.magnesisVerticalAmount;
                if (rightGesture.magnesisForwardAmount > 0.0f)
                    buttonHold |= VPAD_BUTTON_UP;
                else if (rightGesture.magnesisForwardAmount < 0.0f)
                    buttonHold |= VPAD_BUTTON_DOWN;
            }
        }

        // Pull gesture
        if (gameState.right_hand_was_over_left_shoulder_slot) {
            if (gameState.left_hand_current_equip_type != EquipType::Bow) {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
                buttonHold |= VPAD_BUTTON_ZR;
                gameState.last_equip_type_held = EquipType::Bow;
            }
        }
        else if (gameState.right_hand_was_over_right_shoulder_slot) {
            if (gameState.right_hand_current_equip_type != EquipType::Melee) {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
                buttonHold |= VPAD_BUTTON_Y;
                gameState.last_equip_type_held = EquipType::Melee;
            }
        }
        else if (gameState.right_hand_was_over_left_waist_slot) {
            if (gameState.left_hand_current_equip_type != EquipType::SheikahSlate) {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFall);
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_equip_type_held = EquipType::SheikahSlate;
            }
        }
    }
    else
        gameState.right_hand_position_stored = false;

    if (!gameState.prevent_grab_inputs) {
        if (isHandNotOverAnySlot(rightGesture) && isCurrentInteractPressed) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
}

void processLeftTriggerBindings(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState
) {
    if (!inputs.inGame.useLeftItem.currentState) {
        // reset lock on state when trigger is released
        gameState.is_locking_on_target = false;
        return;
    }

    constexpr RumbleParameters raiseRumble = { true, 0, RumbleType::Raising, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters fallRumble = { true, 0, RumbleType::Falling, 0.5f, false, 0.25, 0.3f, 0.3f };

    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();

    // Guard + lock on
    // Reset the guard state to trigger again the lock on camera
    if (!gameState.is_locking_on_target && gameState.previous_button_hold & VPAD_BUTTON_ZL) {
        // cancel rune use to let the shield guard happen
        if (gameState.left_hand_current_equip_type == EquipType::SheikahSlate) {
            rumbleMgr->enqueueInputsRumbleCommand(raiseRumble);
            buttonHold |= VPAD_BUTTON_B; // Cancel rune grab
        }
        buttonHold &= ~VPAD_BUTTON_ZL;
    }

    else {
        if (!gameState.is_shield_guarding)
            rumbleMgr->enqueueInputsRumbleCommand(raiseRumble);
        buttonHold |= VPAD_BUTTON_ZL;
        gameState.is_locking_on_target = true;
        gameState.is_shield_guarding = true;
    }    
}

void processRightTriggerBindings(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    HandGestureState rightGesture
) {
    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();

    if (isHandOverRightShoulderSlot(rightGesture))
        return;

    if (!inputs.inGame.useRightItem.currentState) {
        rumbleMgr->stopInputsRumble(1, RumbleType::Raising);
        gameState.trigger_pressed_over_body_slot = false;
        return;
    }
    
    constexpr RumbleParameters rightRumbleFixed = { true, 1, RumbleType::Fixed, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters leftRumbleFixed = { true, 0, RumbleType::Fixed, 0.5f, false, 0.25, 0.3f, 0.3f };
    constexpr RumbleParameters rightRumbleInfiniteRaiseBow = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters rightRumbleInfiniteRaiseWeaponThrow = { true, 1, RumbleType::Raising, 0.5f, true, 1.0, 0.25f, 0.25f };
    constexpr RumbleParameters rightRumbleFiniteRaise = { true, 1, RumbleType::Raising, 0.5f, false, 0.25, 1.0f, 1.0f };
    
    
    if (gameState.has_something_in_left_hand || gameState.has_something_in_right_hand) {
        if (gameState.is_throwable_object_held) {
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFixed);
            buttonHold |= VPAD_BUTTON_R;  // Throw object
        }
        else if (gameState.left_hand_current_equip_type == EquipType::Bow || gameState.last_equip_type_held == EquipType::Bow) {
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleInfiniteRaiseBow);
            buttonHold |= VPAD_BUTTON_ZR;  // Shoot bow
        }
        else if (gameState.left_hand_current_equip_type == EquipType::SheikahSlate) {
            buttonHold |= VPAD_BUTTON_A; // Use rune
            rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFixed);
        }
        // gameState.trigger_was_pressed_over_body_slot check prevents the melee attack rumbles to wrongly start when the throw weapon inputs 
        // hasn't been released yet. Necessary because the melee attack action won't start if the throw input is still held.
        else if (!gameState.trigger_pressed_over_body_slot){
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleInfiniteRaiseWeaponThrow);
            buttonHold |= VPAD_BUTTON_Y;  // Melee attack
        }
    }
    else {
        // Re-equip last held weapon/item when empty-handed
        if (gameState.last_equip_type_held == EquipType::Melee) {
            buttonHold |= VPAD_BUTTON_Y;
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFixed);
        }
        else if (gameState.last_equip_type_held == EquipType::Bow) {
            buttonHold |= VPAD_BUTTON_ZR;
            rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFixed);
        }
        else if (gameState.last_equip_type_held == EquipType::SheikahSlate) {
            buttonHold |= VPAD_BUTTON_L;
            if ((gameState.previous_button_hold & VPAD_BUTTON_L) == 0) {
                rumbleMgr->enqueueInputsRumbleCommand(rightRumbleFixed);
                rumbleMgr->enqueueInputsRumbleCommand(leftRumbleFixed);
            }
        }
    }
}

void processMenuInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState
) {
    auto mapButton = [](XrActionStateBoolean& state, VPADButtons btn) -> uint32_t { 
        return state.currentState ? btn : 0;
    };

    closeDpadMenu(inputs, gameState);

    if (!gameState.prevent_inputs) {
        buttonHold |= mapButton(inputs.inMenu.back, VPAD_BUTTON_B);
        if (gameState.map_open)
            buttonHold |= mapButton(inputs.shared.inventory_map, VPAD_BUTTON_MINUS);
        else
            buttonHold |= mapButton(inputs.shared.inventory_map, VPAD_BUTTON_PLUS);
    }

    buttonHold |= mapButton(inputs.inMenu.select, VPAD_BUTTON_A);
    buttonHold |= mapButton(inputs.inMenu.sort, VPAD_BUTTON_Y);
    buttonHold |= mapButton(inputs.inMenu.leftTrigger, VPAD_BUTTON_L);
    buttonHold |= mapButton(inputs.inMenu.rightTrigger, VPAD_BUTTON_R);
    buttonHold |= mapButton(inputs.inMenu.rotate, VPAD_BUTTON_STICK_R);

    if (inputs.inMenu.holdState.lastEvent == ButtonState::Event::ShortPress)
        buttonHold |= VPAD_BUTTON_X;
}

void processInputPrevention(OpenXR::GameState& gameState, std::chrono::steady_clock::time_point now, std::chrono::milliseconds delay)
{
    // check if we need to prevent inputs from happening
    if (gameState.in_game != gameState.was_in_game) {
        gameState.prevent_inputs = true;
        gameState.prevent_inputs_time = now;
    }

    if (gameState.prevent_inputs && now >= gameState.prevent_inputs_time + delay)
        gameState.prevent_inputs = false;

    if (gameState.prevent_grab_inputs && now >= gameState.prevent_grab_time + delay)
        gameState.prevent_grab_inputs = false;
}

void processModMenuInput(std::atomic_bool& isMenuOpen, OpenXR::InputState& inputs, VPADStatus& vpadInputs, RND_Renderer::ImGuiOverlay* imguiOverlay, XrActionStateVector2f& leftStickSource, XrActionStateVector2f& rightStickSource)
{
    if (inputs.shared.modMenuState.lastEvent == ButtonState::Event::LongPress && inputs.shared.modMenuState.longFired_actedUpon) {
        isMenuOpen = !isMenuOpen;
        inputs.shared.modMenuState.longFired_actedUpon = false;
    }

    // allow the gamepad inputs to control the imgui overlay
    imguiOverlay->ProcessInputs(inputs, vpadInputs);

    // ignore stick input when the help menu is open
    if (isMenuOpen || imguiOverlay->ShouldBlockGameInput()/*this is used for the entity inspector*/) {
        vpadInputs = {};
        leftStickSource.currentState = { 0.0f, 0.0f };
        rightStickSource.currentState = { 0.0f, 0.0f };
    }
}

void processHandGesture(RND_Renderer* renderer, OpenXR::InputState& inputs, HandGestureState& leftGesture, HandGestureState& rightGesture, OpenXR::GameState& gameState)
{
    auto headsetPose = renderer->GetMiddlePose();
    if (headsetPose.has_value()) {
        const auto headsetMtx = headsetPose.value();
        const glm::fvec3 headsetPos(headsetMtx[3]);
        
        const auto leftHandPos = ToGLM(inputs.shared.poseLocation[0].pose.position);
        const auto rightHandPos = ToGLM(inputs.shared.poseLocation[1].pose.position);
        
        leftGesture = calculateHandGesture(gameState, leftHandPos, headsetMtx, headsetPos, gameState.left_hand_position_stored, gameState.stored_left_hand_position);
        rightGesture = calculateHandGesture(gameState, rightHandPos, headsetMtx, headsetPos, gameState.right_hand_position_stored, gameState.stored_right_hand_position);
    }
}

void processJoystickInput(VPADButtons& oldXRStickHold, VPADButtons& newXRStickHold, VPADStatus& vpadStatus, XrActionStateVector2f& leftStickSource, XrActionStateVector2f& rightStickSource)
{
    // movement/navigation stick
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

    vpadStatus.rightStick = { rightStickSource.currentState.x + vpadStatus.rightStick.x.getLE(), rightStickSource.currentState.y + vpadStatus.rightStick.y.getLE() };

    if (rightStickSource.currentState.x <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_LEFT) && rightStickSource.currentState.x <= -holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_LEFT;
    else if (rightStickSource.currentState.x >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_RIGHT) && rightStickSource.currentState.x >= holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_RIGHT;

    if (rightStickSource.currentState.y <= -axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_DOWN) && rightStickSource.currentState.y <= -holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_DOWN;
    else if (rightStickSource.currentState.y >= axisThreshold || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_UP) && rightStickSource.currentState.y >= holdThreshold))
        newXRStickHold |= VPAD_STICK_R_EMULATION_UP;


    oldXRStickHold = newXRStickHold;
}

XrTime prev_sample = 0;

void updatePreviousValues(OpenXR::GameState& gameState, uint32_t& buttonHold, HandGestureState& leftGesture, HandGestureState& rightGesture, XrTime inputTime)
{
    // (re)set values for next frame
    gameState.previous_button_hold = buttonHold;
    gameState.was_in_game = gameState.in_game;
    if (gameState.in_game) {
        gameState.has_something_in_right_hand = false;                    // updated in hook_ChangeWeaponMtx
        gameState.has_something_in_left_hand = false;                     // updated in hook_ChangeWeaponMtx
        gameState.is_throwable_object_held = false;                       // updated in hook_ChangeWeaponMtx
        gameState.left_hand_previous_frame_equip_type = gameState.left_hand_current_equip_type;   // use the previous values if no new values are written from hook_ChangeWeaponMtx this frame
        gameState.right_hand_previous_frame_equip_type = gameState.right_hand_current_equip_type; // use the previous values if no new values are written from hook_ChangeWeaponMtx this frame
        gameState.left_hand_current_equip_type = EquipType::None;                      // updated in hook_ChangeWeaponMtx
        gameState.right_hand_current_equip_type = EquipType::None;                     // updated in hook_ChangeWeaponMtx
    }
    
    // Pull gesture
    gameState.right_hand_was_over_left_shoulder_slot = isHandOverLeftShoulderSlot(rightGesture);
    gameState.right_hand_was_over_right_shoulder_slot = isHandOverRightShoulderSlot(rightGesture);
    gameState.right_hand_was_over_left_waist_slot = isHandOverLeftWaistSlot(rightGesture);
    gameState.left_hand_was_over_left_shoulder_slot = isHandOverLeftShoulderSlot(leftGesture);
    gameState.left_hand_was_over_right_shoulder_slot = isHandOverRightShoulderSlot(leftGesture);
    gameState.left_hand_was_over_left_waist_slot = isHandOverLeftWaistSlot(leftGesture);

    prev_sample = inputTime;
}

void CemuHooks::hook_InjectXRInput(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    auto mapXRButtonToVpad = [](XrActionStateBoolean& state, VPADButtons mapping) -> uint32_t {
        return state.currentState ? mapping : 0;
    };

    // read existing vpad as to not overwrite it
    uint32_t vpadStatusOffset = hCPU->gpr[4];
    VPADStatus vpadStatus = {};

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (!renderer) {
        return;
    }
    auto* imguiOverlay = renderer->m_imguiOverlay.get();
    if (!imguiOverlay) {
        return;
    }

    // todo: revert this to unblock gamepad input
    readMemory(vpadStatusOffset, &vpadStatus);

    static auto startBtnLastTime = std::chrono::steady_clock::now();
    static bool startBtnWasDown = false;
    static bool startBtnActionConsumed = false;

    if ((vpadStatus.hold.getLE() & VPAD_BUTTON_PLUS) != 0) {
        if (!startBtnWasDown) {
            startBtnLastTime = std::chrono::steady_clock::now();
            startBtnActionConsumed = false;
        }
        else if (!startBtnActionConsumed && std::chrono::steady_clock::now() - startBtnLastTime > std::chrono::milliseconds(500)) {
            VRManager::instance().XR->m_isMenuOpen = !VRManager::instance().XR->m_isMenuOpen;
            startBtnActionConsumed = true;
        }
        startBtnWasDown = true;
    }
    else {
        startBtnWasDown = false;
    }

    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();

    // fetch input state
    OpenXR::InputState inputs = VRManager::instance().XR->m_input.load();
    inputs.inGame.drop_weapon[0] = inputs.inGame.drop_weapon[1] = false;

    float dt = (float)(inputs.shared.inputTime - prev_sample) / 1000000000.0f;

    // fetch game state
    auto gameState = VRManager::instance().XR->m_gameState.load(); 
    gameState.in_game = inputs.shared.in_game;

    // buttons
    static uint32_t oldCombinedHold = 0; 
    uint32_t newXRBtnHold = 0;

    // fetching stick inputs
    XrActionStateVector2f& leftStickSource = gameState.in_game ? inputs.inGame.move : inputs.inMenu.navigate;
    XrActionStateVector2f& rightStickSource = gameState.in_game ? inputs.inGame.camera : inputs.inMenu.scroll;

    // Apply deadzone
    float stickDeadzone = GetSettings().stickDeadzone;
    auto applyDeadzone = [stickDeadzone](XrVector2f& v) {
        if (std::abs(v.x) < stickDeadzone) v.x = 0.0f;
        if (std::abs(v.y) < stickDeadzone) v.y = 0.0f;
    };
    applyDeadzone(leftStickSource.currentState);
    applyDeadzone(rightStickSource.currentState);

    //Delay to wait before allowing specific inputs again
    constexpr std::chrono::milliseconds delay{ 400 };
    const auto now = std::chrono::steady_clock::now();

    processInputPrevention(gameState, now, delay);

    auto& isMenuOpen = VRManager::instance().XR->m_isMenuOpen;
    processModMenuInput(isMenuOpen, inputs, vpadStatus, imguiOverlay, leftStickSource, rightStickSource);

    // Calculate hand gestures
    HandGestureState leftGesture = {};
    HandGestureState rightGesture = {};
    processHandGesture(renderer, inputs, leftGesture, rightGesture, gameState);

    keepDpadMenuOpen(newXRBtnHold, gameState);

    // Process inputs
    if (isMenuOpen) {
        // ignore inputs when the mod menu is open
    }
    else if (gameState.in_game) {
        spreadWeaponDetectionOverFrames(gameState);

        equipWeaponOnDpadMenuExit(newXRBtnHold, gameState, dt);

        if (!gameState.prevent_inputs) {
            // prevent jump when exiting menus with B button
            newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.jump_cancel, VPAD_BUTTON_X);

            //// Scope
            if (inputs.inGame.crouch_scopeState.lastEvent == ButtonState::Event::LongPress) {
                newXRBtnHold |= VPAD_BUTTON_STICK_R;
            }

            // Handle map and inventory menu toggle
            if (inputs.shared.inventory_mapState.lastEvent == ButtonState::Event::ShortPress) {
                newXRBtnHold |= VPAD_BUTTON_PLUS;
                gameState.map_open = false;
            }
            if (inputs.shared.inventory_mapState.lastEvent == ButtonState::Event::LongPress) {
                newXRBtnHold |= VPAD_BUTTON_MINUS;
                gameState.map_open = true;
            }
        }

        if (inputs.inGame.crouch_scopeState.lastEvent == ButtonState::Event::ShortPress) {
            newXRBtnHold |= VPAD_BUTTON_STICK_L;
        }
        
        // Optional rune inputs (for seated players)
        openDpadMenuRuneButton(inputs.shared.useRune_runeMenuState.lastEvent, newXRBtnHold, gameState);
        if (inputs.shared.useRune_runeMenuState.lastEvent == ButtonState::Event::ShortPress) {
            newXRBtnHold |= VPAD_BUTTON_L;  // Equip rune
            gameState.last_equip_type_held = EquipType::SheikahSlate;
        }
        
        // If climbing or paragliding, make the B button cancel instantly the action instead of long press to run
        if (gameState.is_climbing || gameState.is_paragliding) {
            newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.run_interact, VPAD_BUTTON_B);
        }
        else if (gameState.is_riding_mount)
        {
            // A button to interact when riding a mount
            newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.run_interact, VPAD_BUTTON_A);
            // grabs to accelerate and brake when riding master cycle
            // Skip when the respective hand is inside a body slot so grips can still equip/unequip
            if (inputs.inGame.interactState[1].wasDownLastFrame && isHandNotOverAnySlot(rightGesture))
                newXRBtnHold |= VPAD_BUTTON_A;
            if (inputs.inGame.interactState[0].wasDownLastFrame && isHandNotOverAnySlot(leftGesture))
                newXRBtnHold |= VPAD_BUTTON_B;
        }
        else {
            if (inputs.inGame.runState.lastEvent == ButtonState::Event::LongPress) {
                newXRBtnHold |= VPAD_BUTTON_B; // Run
            }
            else
                newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.run_interact, VPAD_BUTTON_A);
        }

        // Whistle gesture
        if (isHandOverMouthSlot(leftGesture) && isHandOverMouthSlot(rightGesture)) {
            if (inputs.shared.grabState[0].wasDownLastFrame && inputs.shared.grabState[1].wasDownLastFrame) {
                rumbleMgr->enqueueInputsRumbleCommand({ true, 0, RumbleType::OscillationRaisingSawtoothWave, 1.0f, false, 0.25, 0.2f, 0.2f });
                newXRBtnHold |= VPAD_BUTTON_DOWN;
            }
        }
        
        // Hand-specific input
        processLeftHandInGameInput(newXRBtnHold, inputs, gameState, leftGesture, rightStickSource, now, dt);
        processRightHandInGameInput(newXRBtnHold, inputs, gameState, rightGesture, rightStickSource, now);
        
        // Trigger handling
        processLeftTriggerBindings(newXRBtnHold, inputs, gameState);
        processRightTriggerBindings(newXRBtnHold, inputs, gameState, rightGesture);
    }
    else {
        processMenuInput(newXRBtnHold, inputs, gameState);
    }

    // Update rumble/haptics
    rumbleMgr->updateHaptics();

    // sticks
    static VPADButtons oldXRStickHold = VPAD_BUTTON_NONE;
    VPADButtons newXRStickHold = VPAD_BUTTON_NONE;
    processJoystickInput(oldXRStickHold, newXRStickHold, vpadStatus, leftStickSource, rightStickSource);

    // calculate new hold, trigger and release
    uint32_t combinedHold = (vpadStatus.hold.getLE() | (newXRBtnHold | newXRStickHold));
    vpadStatus.hold = combinedHold;
    vpadStatus.trig = (combinedHold & ~oldCombinedHold);
    vpadStatus.release = (~combinedHold & oldCombinedHold);
    oldCombinedHold = combinedHold;

    // misc
    vpadStatus.vpadErr = 0;
    vpadStatus.batteryLevel = 0xC0;

    // touch
    vpadStatus.tpData.touch = 0;
    vpadStatus.tpData.validity = 3;

    // motion
    OpenXRMotionBridge::UpdateVPADStatus(inputs, vpadStatus);

    // write the input back to VPADStatus
    writeMemory(vpadStatusOffset, &vpadStatus);

    // set r3 to 1 for hooked VPADRead function to return success
    hCPU->gpr[3] = 1;

    updatePreviousValues(gameState, newXRBtnHold, leftGesture, rightGesture, inputs.shared.inputTime);

    VRManager::instance().XR->m_gameState.store(gameState);
    VRManager::instance().XR->m_input.store(inputs);
}


// some ideas:
// - quickly pressing the grip button without a weapon while there's a nearby weapon and there's enough slots = pick up weapon
// - holding the grip button without a weapon while there's a nearby weapon = temporarily hold weapon
// - holding the grip button a weapon equipped = opens weapon dpad menu
// - quickly press the grip button while holding a weapon = drops current weapon

void CemuHooks::hook_CreateNewActor(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    // if (VRManager::instance().XR->GetRenderer() == nullptr || VRManager::instance().XR->GetRenderer()->m_layer3D.GetStatus() == RND_Renderer::Layer3D::Status3D::UNINITIALIZED) {
    //     hCPU->gpr[3] = 0;
    //     return;
    // }
    hCPU->gpr[3] = 0;

    // OpenXR::InputState inputs = VRManager::instance().XR->m_input.load();
    // if (!inputs.shared.in_game) {
    //     hCPU->gpr[3] = 0;
    //     return;
    // }
    //
    // // test if controller is connected
    // if (inputs.shared.grab[OpenXR::EyeSide::LEFT].currentState == XR_TRUE && inputs.shared.grab[OpenXR::EyeSide::LEFT].changedSinceLastSync == XR_TRUE) {
    //     Log::print("Trying to spawn new thing!");
    //     hCPU->gpr[3] = 1;
    // }
    // else if (inputs.shared.grab[OpenXR::EyeSide::RIGHT].currentState == XR_TRUE && inputs.shared.grab[OpenXR::EyeSide::RIGHT].changedSinceLastSync == XR_TRUE) {
    //     Log::print("Trying to spawn new thing!");
    //     hCPU->gpr[3] = 1;
    // }
    // else {
    //     hCPU->gpr[3] = 0;
    // }
}