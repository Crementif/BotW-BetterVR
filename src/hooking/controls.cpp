#include "cemu_hooks.h"
#include "../instance.h"

enum VPADButtons : uint32_t {
    VPAD_BUTTON_A                 = 0x8000,
    VPAD_BUTTON_B                 = 0x4000,
    VPAD_BUTTON_X                 = 0x2000,
    VPAD_BUTTON_Y                 = 0x1000,
    VPAD_BUTTON_LEFT              = 0x0800,
    VPAD_BUTTON_RIGHT             = 0x0400,
    VPAD_BUTTON_UP                = 0x0200,
    VPAD_BUTTON_DOWN              = 0x0100,
    VPAD_BUTTON_ZL                = 0x0080,
    VPAD_BUTTON_ZR                = 0x0040,
    VPAD_BUTTON_L                 = 0x0020,
    VPAD_BUTTON_R                 = 0x0010,
    VPAD_BUTTON_PLUS              = 0x0008,
    VPAD_BUTTON_MINUS             = 0x0004,
    VPAD_BUTTON_HOME              = 0x0002,
    VPAD_BUTTON_SYNC              = 0x0001,
    VPAD_BUTTON_STICK_R           = 0x00020000,
    VPAD_BUTTON_STICK_L           = 0x00040000,
    VPAD_BUTTON_TV                = 0x00010000,
    VPAD_STICK_R_EMULATION_LEFT   = 0x04000000,
    VPAD_STICK_R_EMULATION_RIGHT  = 0x02000000,
    VPAD_STICK_R_EMULATION_UP     = 0x01000000,
    VPAD_STICK_R_EMULATION_DOWN   = 0x00800000,
    VPAD_STICK_L_EMULATION_LEFT   = 0x40000000,
    VPAD_STICK_L_EMULATION_RIGHT  = 0x20000000,
    VPAD_STICK_L_EMULATION_UP     = 0x10000000,
    VPAD_STICK_L_EMULATION_DOWN   = 0x08000000,
};

struct BEDir {
    BEVec3 x;
    BEVec3 y;
    BEVec3 z;

    BEDir() = default;
};

struct BETouchData {
    BEType<uint16_t> x;
    BEType<uint16_t> y;
    BEType<uint16_t> touch;
    BEType<uint16_t> validity;
};

struct VPADStatus {
    BEType<uint32_t> hold;
    BEType<uint32_t> trig;
    BEType<uint32_t> release;
    BEVec2 leftStick;
    BEVec2 rightStick;
    BEVec3 acc;
    BEType<float> accMagnitude;
    BEType<float> accAcceleration;
    BEVec2 accXY;
    BEVec3 gyroChange;
    BEVec3 gyroOrientation;
    int8_t vpadErr;
    uint8_t padding1[1];
    BETouchData tpData;
    BETouchData tpProcessed1;
    BETouchData tpProcessed2;
    uint8_t padding2[2];
    BEDir dir;
    uint8_t headphoneStatus;
    uint8_t padding3[3];
    BEVec3 magnet;
    uint8_t slideVolume;
    uint8_t batteryLevel;
    uint8_t micStatus;
    uint8_t slideVolume2;
    uint8_t padding4[8];
};
static_assert(sizeof(VPADStatus) == 0xAC);

enum JoyDir {
    Up,
    Right,
    Down,
    Left,
    None
};

constexpr float AXIS_THRESHOLD = 0.5f;
constexpr float HOLD_THRESHOLD = 0.1f;

JoyDir getJoystickDirection(const XrVector2f& stick)
{
    if (stick.y >= AXIS_THRESHOLD)       return JoyDir::Up;
    if (stick.y <= -AXIS_THRESHOLD)      return JoyDir::Down;
    if (stick.x <= -AXIS_THRESHOLD)      return JoyDir::Left;
    if (stick.x >= AXIS_THRESHOLD)       return JoyDir::Right;

    return JoyDir::None;
}


struct HandGestureState {
    bool isBehindHead;
    bool isBehindHeadWithWaistOffset;
    bool isCloseToHead;
    bool isCloseToWaist;
    bool isOnLeftSide;  // true = left side of body, false = right side
};

// Gesture detection functions
HandGestureState calculateHandGesture(
    const glm::fvec3& handPos,
    const glm::fmat4& headsetMatrix,
    const glm::fvec3& headsetPos
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
    
    constexpr float WAIST_BEHIND_OFFSET = 0.15f;
    float flatForwardDot = glm::dot(flatForward, flatHandOffset) + WAIST_BEHIND_OFFSET;
    gesture.isBehindHeadWithWaistOffset = (flatForwardDot < 0.0f);
    
    // Check which side of body the hand is on
    float rightDot = glm::dot(headsetRight, headToHand);
    gesture.isOnLeftSide = (rightDot < 0.0f);
    
    // Check distance from head (for shoulder slots)
    constexpr float SHOULDER_RADIUS = 0.35f;
    constexpr float SHOULDER_RADIUS_SQ = SHOULDER_RADIUS * SHOULDER_RADIUS;
    gesture.isCloseToHead = (glm::length2(headToHand) < SHOULDER_RADIUS_SQ);
    
    // Check distance from waist (rough estimate)
    glm::fvec3 waistPos = headsetPos - glm::fvec3(0.0f, 0.5f, 0.0f);
    gesture.isCloseToWaist = (handPos.y < waistPos.y);
    
    return gesture;
}

bool isHandOverLeftShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && gesture.isOnLeftSide;
}

bool isHandOverRightShoulderSlot(const HandGestureState& gesture) {
    return gesture.isBehindHead && gesture.isCloseToHead && !gesture.isOnLeftSide;
}

bool isHandOverWaistSlot(const HandGestureState& gesture) {
    return gesture.isBehindHeadWithWaistOffset && gesture.isCloseToWaist && gesture.isOnLeftSide;
}


// Input handling functions
void handleLeftHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    const HandGestureState& leftGesture,
    JoyDir leftStickDir,
    XrActionStateVector2f& leftStickSource,
    const std::chrono::steady_clock::time_point& now
) {
    constexpr std::chrono::milliseconds INPUT_DELAY(400);
    
    RumbleParameters overSlotRumble = { true, 0, 0.5f, true, 1.0, 0.25f, 0.25f };
    RumbleParameters grabSlotRumble = { true, 0, 0.5f, false, 0.25, 0.3f, 0.3f };
    
    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();
    bool isGrabPressed = inputs.inGame.grabState[0].wasDownLastFrame;
    
    // Handle shoulder slot interactions
    if (isHandOverLeftShoulderSlot(leftGesture) || isHandOverRightShoulderSlot(leftGesture)) {
        // Provide haptic feedback when hand is over equipped slot
        if (isHandOverRightShoulderSlot(leftGesture) && gameState.right_equip_type == EquipType::Melee) {
            rumbleMgr->enqueueInputsRumbleCommand(overSlotRumble);
        }
        else if (isHandOverLeftShoulderSlot(leftGesture) && gameState.left_equip_type == EquipType::Bow) {
            rumbleMgr->enqueueInputsRumbleCommand(overSlotRumble);
        }
        
        // Handle equip/unequip
        if (!gameState.prevent_grab_inputs && isGrabPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(grabSlotRumble);
            
            if (isHandOverLeftShoulderSlot(leftGesture)) {
                // Left shoulder = Bow
                if (gameState.left_equip_type != EquipType::Bow) {
                    buttonHold |= VPAD_BUTTON_ZR;
                    gameState.last_item_held = EquipType::Bow;
                } else {
                    buttonHold |= VPAD_BUTTON_B;  // Unequip
                }
            }
            else {
                // Right shoulder = Melee weapon
                if (gameState.right_equip_type != EquipType::Melee) {
                    buttonHold |= VPAD_BUTTON_Y;
                    gameState.last_item_held = EquipType::Melee;
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
    if (isHandOverWaistSlot(leftGesture)) {
        if (gameState.left_equip_type == EquipType::Rune) {
            rumbleMgr->enqueueInputsRumbleCommand(overSlotRumble);
        }
        
        if (!gameState.prevent_grab_inputs && isGrabPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(grabSlotRumble);
            
            if (gameState.left_equip_type != EquipType::Rune) {
                buttonHold |= VPAD_BUTTON_L;
                gameState.last_item_held = EquipType::Rune;
            } else {
                buttonHold |= VPAD_BUTTON_B;  // Unequip
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        return;
    }
    
    // Normal hand input (not over slots)
    rumbleMgr->stopInputsRumble(0);
    
    if (isGrabPressed) {
        // Handle grab action
        if (!gameState.prevent_grab_inputs) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
    
    // Handle D-pad menu with left stick while grabbing
    if (!gameState.prevent_menu_inputs && isGrabPressed) {
        switch (leftStickDir) {
            case JoyDir::Up:    buttonHold |= VPAD_BUTTON_UP; break;
            case JoyDir::Right: buttonHold |= VPAD_BUTTON_RIGHT; break;
            case JoyDir::Down:  buttonHold |= VPAD_BUTTON_DOWN; break;
            case JoyDir::Left:  buttonHold |= VPAD_BUTTON_LEFT; break;
            case JoyDir::None:  break;
        }
        
        if (leftStickDir != JoyDir::None) {
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
            leftStickSource.currentState = { 0.0f, 0.0f };  // Prevent movement
        }
    }
}

void handleRightHandInGameInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    const HandGestureState& rightGesture,
    JoyDir rightStickDir,
    const std::chrono::steady_clock::time_point& now
) {
    constexpr std::chrono::milliseconds INPUT_DELAY(400);
    
    RumbleParameters overSlotRumble = { true, 1, 0.5f, true, 1.0, 0.25f, 0.25f };
    RumbleParameters grabSlotRumble = { true, 1, 0.5f, false, 0.25, 0.3f, 0.3f };
    
    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();
    bool isGrabPressed = inputs.inGame.grabState[1].wasDownLastFrame;
    bool isTriggerPressed = inputs.inGame.rightTrigger.currentState;
    
    // Handle shoulder slot interactions
    if (isHandOverLeftShoulderSlot(rightGesture) || isHandOverRightShoulderSlot(rightGesture)) {
        // Haptic feedback when over equipped slot
        if (isHandOverRightShoulderSlot(rightGesture) && gameState.right_equip_type == EquipType::Melee) {
            rumbleMgr->enqueueInputsRumbleCommand(overSlotRumble);
        }
        else if (isHandOverLeftShoulderSlot(rightGesture) && gameState.left_equip_type == EquipType::Bow) {
            rumbleMgr->enqueueInputsRumbleCommand(overSlotRumble);
        }
        
        // Handle equip/unequip
        if (!gameState.prevent_grab_inputs && isGrabPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(grabSlotRumble);
            
            if (isHandOverRightShoulderSlot(rightGesture)) {
                // Right shoulder = Melee weapon
                if (gameState.right_equip_type != EquipType::Melee) {
                    buttonHold |= VPAD_BUTTON_Y;
                    gameState.last_item_held = EquipType::Melee;
                } else {
                    buttonHold |= VPAD_BUTTON_B;
                }
            }
            else {
                // Left shoulder = Bow
                if (gameState.left_equip_type != EquipType::Bow) {
                    buttonHold |= VPAD_BUTTON_ZR;
                    gameState.last_item_held = EquipType::Bow;
                } else {
                    buttonHold |= VPAD_BUTTON_B;
                }
            }
            
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        
        // Handle weapon throw when over shoulder slot
        if (isTriggerPressed) {
            rumbleMgr->enqueueInputsRumbleCommand(grabSlotRumble);
            buttonHold |= VPAD_BUTTON_R;
        }
        
        return;  // Don't process normal input when over slots
    }
    
    // Normal hand input (not over slots)
    rumbleMgr->stopInputsRumble(1);
    
    if (isGrabPressed) {
        // Handle drop action
        if (rightStickDir == JoyDir::Down) {
            inputs.inGame.drop_weapon[1] = true;
            gameState.prevent_grab_inputs = true;
            gameState.prevent_grab_time = now;
        }
        // Handle grab action
        else if (!gameState.prevent_grab_inputs) {
            buttonHold |= VPAD_BUTTON_A;
        }
    }
}

void handleRightTriggerAttack(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState
) {
    if (!inputs.inGame.rightTrigger.currentState) {
        return;
    }
    
    RumbleParameters attackRumble = { true, 1, 0.5f, false, 0.25, 0.3f, 0.3f };
    auto* rumbleMgr = VRManager::instance().XR->GetRumbleManager();
    
    rumbleMgr->enqueueInputsRumbleCommand(attackRumble);
    
    if (gameState.has_something_in_hand) {
        if (gameState.is_throwable_object_held) {
            buttonHold |= VPAD_BUTTON_R;  // Throw object
        }
        else if (gameState.left_equip_type == EquipType::Bow) {
            buttonHold |= VPAD_BUTTON_ZR;  // Shoot bow
        }
        else if (gameState.left_equip_type == EquipType::Rune) {
            buttonHold |= VPAD_BUTTON_A;  // Use rune
        }
        else {
            buttonHold |= VPAD_BUTTON_Y;  // Melee attack
        }
    }
    else {
        // Re-equip last held weapon when empty-handed
        if (gameState.last_item_held == EquipType::Melee) {
            buttonHold |= VPAD_BUTTON_Y;
        }
        else if (gameState.last_item_held == EquipType::Bow) {
            buttonHold |= VPAD_BUTTON_ZR;
        }
        else if (gameState.last_item_held == EquipType::Rune) {
            buttonHold |= VPAD_BUTTON_L;
        }
    }
}

void handleMenuInput(
    uint32_t& buttonHold,
    OpenXR::InputState& inputs,
    OpenXR::GameState& gameState,
    JoyDir leftStickDir
) {
    auto mapButton = [](XrActionStateBoolean& state, VPADButtons btn) -> uint32_t {
        return state.currentState ? btn : 0;
    };
    
    if (!gameState.prevent_menu_inputs) {
        if (gameState.map_open) {
            buttonHold |= mapButton(inputs.inMenu.mapAndInventory, VPAD_BUTTON_MINUS);
        } else {
            buttonHold |= mapButton(inputs.inMenu.mapAndInventory, VPAD_BUTTON_PLUS);
        }
    }
    
    buttonHold |= mapButton(inputs.inMenu.select, VPAD_BUTTON_A);
    buttonHold |= mapButton(inputs.inMenu.back, VPAD_BUTTON_B);
    buttonHold |= mapButton(inputs.inMenu.sort, VPAD_BUTTON_Y);
    buttonHold |= mapButton(inputs.inMenu.hold, VPAD_BUTTON_X);
    buttonHold |= mapButton(inputs.inMenu.leftTrigger, VPAD_BUTTON_L);
    buttonHold |= mapButton(inputs.inMenu.rightTrigger, VPAD_BUTTON_R);
    
    // D-pad navigation with left grip
    if (inputs.inMenu.leftGrip.currentState) {
        switch (leftStickDir) {
            case JoyDir::Up:    buttonHold |= VPAD_BUTTON_UP; break;
            case JoyDir::Right: buttonHold |= VPAD_BUTTON_RIGHT; break;
            case JoyDir::Down:  buttonHold |= VPAD_BUTTON_DOWN; break;
            case JoyDir::Left:  buttonHold |= VPAD_BUTTON_LEFT; break;
            case JoyDir::None:  break;
        }
    }
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
    // todo: revert this to unblock gamepad input
    if (!(renderer && renderer->m_imguiOverlay && renderer->m_imguiOverlay->ShouldBlockGameInput())) {
        readMemory(vpadStatusOffset, &vpadStatus);
    }

    // fetch input state
    OpenXR::InputState inputs = VRManager::instance().XR->m_input.load();
    inputs.inGame.drop_weapon[0] = inputs.inGame.drop_weapon[1] = false;

    // fetch game state
    auto gameState = VRManager::instance().XR->m_gameState.load();
    gameState.in_game = inputs.inGame.in_game;

    // buttons
    static uint32_t oldCombinedHold = 0;
    uint32_t newXRBtnHold = 0;

    // fetching stick inputs
    XrActionStateVector2f& leftStickSource = gameState.in_game ? inputs.inGame.move : inputs.inMenu.navigate;
    XrActionStateVector2f& rightStickSource = gameState.in_game ? inputs.inGame.camera : inputs.inMenu.scroll;

    JoyDir leftJoystickDir = getJoystickDirection(leftStickSource.currentState);
    JoyDir rightJoystickDir = getJoystickDirection(rightStickSource.currentState);

    //Delay to wait before allowing specific inputs again
    constexpr std::chrono::milliseconds delay{ 400 };
    const auto now = std::chrono::steady_clock::now();

    // check if we need to prevent inputs from happening (fix menu reopening when exiting it and grab object when quitting dpad menu)
    if (gameState.in_game != gameState.was_in_game) {
        gameState.prevent_menu_inputs = true;
        gameState.prevent_menu_time = now;
    }

    if (gameState.prevent_menu_inputs && now >= gameState.prevent_menu_time + delay)
        gameState.prevent_menu_inputs = false;

    if (gameState.prevent_grab_inputs && now >= gameState.prevent_grab_time + delay)
        gameState.prevent_grab_inputs = false;

    //Log::print<INFO>("last_weapon_held_hand : {}", gameState.last_weapon_held_hand);
    //Log::print<INFO>("Is weapon held : {}", gameState.is_weapon_or_object_held);
    //Log::print<INFO>("Weapon Type : {}", (int)gameState.left_weapon_type);
    //Log::print<INFO>("Weapon Type : {}", (int)gameState.right_weapon_type);

    // Calculate hand gestures
    HandGestureState leftGesture = {};
    HandGestureState rightGesture = {};

    auto headsetPose = VRManager::instance().XR->GetRenderer()->GetMiddlePose();
    if (headsetPose.has_value()) {
        const auto headsetMtx = headsetPose.value();
        const glm::fvec3 headsetPos(headsetMtx[3]);
        
        const auto leftHandPos = ToGLM(inputs.inGame.poseLocation[0].pose.position);
        const auto rightHandPos = ToGLM(inputs.inGame.poseLocation[1].pose.position);
        
        leftGesture = calculateHandGesture(leftHandPos, headsetMtx, headsetPos);
        rightGesture = calculateHandGesture(rightHandPos, headsetMtx, headsetPos);
    }
    
    // Process input
    if (gameState.in_game) {
        // Handle menu toggle
        if (!gameState.prevent_menu_inputs) {
            if (inputs.inGame.mapAndInventoryState.lastEvent == ButtonState::Event::LongPress) {
                newXRBtnHold |= VPAD_BUTTON_MINUS;
                gameState.map_open = true;
            }
            if (inputs.inGame.mapAndInventoryState.lastEvent == ButtonState::Event::ShortPress) {
                newXRBtnHold |= VPAD_BUTTON_PLUS;
                gameState.map_open = false;
            }
        }
        
        // Handle crouch and scope
        if (inputs.inGame.crouchAndScopeState.lastEvent == ButtonState::Event::LongPress) {
            newXRBtnHold |= VPAD_BUTTON_STICK_R;  // Scope
        }
        if (inputs.inGame.crouchAndScopeState.lastEvent == ButtonState::Event::ShortPress) {
            newXRBtnHold |= VPAD_BUTTON_STICK_L;  // Crouch
        }
        
        // Basic actions
        newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.jump, VPAD_BUTTON_X);
        newXRBtnHold |= mapXRButtonToVpad(inputs.inGame.interact, VPAD_BUTTON_A);
        
        if (inputs.inGame.runState.lastEvent == ButtonState::Event::LongPress) {
            newXRBtnHold |= VPAD_BUTTON_B;  // Run
        }
        
        // Hand-specific input
        handleLeftHandInGameInput(newXRBtnHold, inputs, gameState, leftGesture, 
                                   leftJoystickDir, leftStickSource, now);
        handleRightHandInGameInput(newXRBtnHold, inputs, gameState, rightGesture, 
                                    rightJoystickDir, now);
        
        // Combat actions
        handleRightTriggerAttack(newXRBtnHold, inputs, gameState);
        
        // Rune cancel with left trigger
        if (inputs.inGame.leftTrigger.currentState && 
            gameState.left_equip_type == EquipType::Rune) {
            RumbleParameters rumble = { true, 1, 0.5f, false, 0.25, 0.3f, 0.3f };
            VRManager::instance().XR->GetRumbleManager()->enqueueInputsRumbleCommand(rumble);
            newXRBtnHold |= VPAD_BUTTON_B;
        }
    }
    else {
        handleMenuInput(newXRBtnHold, inputs, gameState, leftJoystickDir);
    }
    
    // Update rumble/haptics
    VRManager::instance().XR->GetRumbleManager()->updateHaptics();



    // sticks
    static uint32_t oldXRStickHold = 0;
    uint32_t newXRStickHold = 0;

    // movement/navigation stick
    if (inputs.inGame.in_game) {
        auto isolateYaw = [](const glm::fquat& q) -> glm::fquat {
            glm::vec3 euler = glm::eulerAngles(q);
            euler.x = 0.0f;
            euler.z = 0.0f;
            return glm::angleAxis(euler.y, glm::vec3(0, 1, 0));
        };

        glm::fquat controllerRotation = ToGLM(inputs.inGame.poseLocation[OpenXR::EyeSide::LEFT].pose.orientation);
        glm::fquat controllerYawRotation = isolateYaw(controllerRotation);

        glm::fquat moveRotation = inputs.inGame.pose[OpenXR::EyeSide::LEFT].isActive ? glm::inverse(VRManager::instance().XR->m_inputCameraRotation.load() * controllerYawRotation) : glm::identity<glm::fquat>();

        glm::vec3 localMoveVec(leftStickSource.currentState.x, 0.0f, leftStickSource.currentState.y);

        glm::vec3 worldMoveVec = moveRotation * localMoveVec;

        float inputLen = glm::length(glm::vec2(leftStickSource.currentState.x, leftStickSource.currentState.y));
        if (inputLen > 1e-3f) {
            worldMoveVec = glm::normalize(worldMoveVec) * inputLen;
        }
        else {
            worldMoveVec = glm::vec3(0.0f);
        }

        worldMoveVec = {0, 0, 0};

        vpadStatus.leftStick = { worldMoveVec.x + leftStickSource.currentState.x + vpadStatus.leftStick.x.getLE(), worldMoveVec.z + leftStickSource.currentState.y + vpadStatus.leftStick.y.getLE() };
    }
    else {
        vpadStatus.leftStick = { leftStickSource.currentState.x + vpadStatus.leftStick.x.getLE(), leftStickSource.currentState.y + vpadStatus.leftStick.y.getLE() };
    }

    if (leftStickSource.currentState.x <= -AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_LEFT) && leftStickSource.currentState.x <= -HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_L_EMULATION_LEFT;
    else if (leftStickSource.currentState.x >= AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_RIGHT) && leftStickSource.currentState.x >= HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_L_EMULATION_RIGHT;

    if (leftStickSource.currentState.y <= -AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_DOWN) && leftStickSource.currentState.y <= -HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_L_EMULATION_DOWN;
    else if (leftStickSource.currentState.y >= AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_L_EMULATION_UP) && leftStickSource.currentState.y >= HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_L_EMULATION_UP;

    vpadStatus.rightStick = {rightStickSource.currentState.x + vpadStatus.rightStick.x.getLE(), rightStickSource.currentState.y + vpadStatus.rightStick.y.getLE()};

    if (rightStickSource.currentState.x <= -AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_LEFT) && rightStickSource.currentState.x <= -HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_R_EMULATION_LEFT;
    else if (rightStickSource.currentState.x >= AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_RIGHT) && rightStickSource.currentState.x >= HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_R_EMULATION_RIGHT;

    if (rightStickSource.currentState.y <= -AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_DOWN) && rightStickSource.currentState.y <= -HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_R_EMULATION_DOWN;
    else if (rightStickSource.currentState.y >= AXIS_THRESHOLD || (HAS_FLAG(oldXRStickHold, VPAD_STICK_R_EMULATION_UP) && rightStickSource.currentState.y >= HOLD_THRESHOLD))
        newXRStickHold |= VPAD_STICK_R_EMULATION_UP;

    oldXRStickHold = newXRStickHold;

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
    vpadStatus.dir.x = glm::fvec3{1, 0, 0};
    vpadStatus.dir.y = glm::fvec3{0, 1, 0};
    vpadStatus.dir.z = glm::fvec3{0, 0, 1};
    vpadStatus.accXY = {1.0f, 0.0f};

    // write the input back to VPADStatus
    writeMemory(vpadStatusOffset, &vpadStatus);

    // set r3 to 1 for hooked VPADRead function to return success
    hCPU->gpr[3] = 1;

    // (re)set values for next frame
    gameState.was_in_game = gameState.in_game;
    gameState.has_something_in_hand = false; // updated in hook_ChangeWeaponMtx
    gameState.is_throwable_object_held = false; // updated in hook_ChangeWeaponMtx
    gameState.right_equip_type = EquipType::None; // updated in hook_ChangeWeaponMtx
    gameState.left_equip_type = EquipType::None; // updated in hook_ChangeWeaponMtx

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
    // if (!inputs.inGame.in_game) {
    //     hCPU->gpr[3] = 0;
    //     return;
    // }
    //
    // // test if controller is connected
    // if (inputs.inGame.grab[OpenXR::EyeSide::LEFT].currentState == XR_TRUE && inputs.inGame.grab[OpenXR::EyeSide::LEFT].changedSinceLastSync == XR_TRUE) {
    //     Log::print("Trying to spawn new thing!");
    //     hCPU->gpr[3] = 1;
    // }
    // else if (inputs.inGame.grab[OpenXR::EyeSide::RIGHT].currentState == XR_TRUE && inputs.inGame.grab[OpenXR::EyeSide::RIGHT].changedSinceLastSync == XR_TRUE) {
    //     Log::print("Trying to spawn new thing!");
    //     hCPU->gpr[3] = 1;
    // }
    // else {
    //     hCPU->gpr[3] = 0;
    // }
}