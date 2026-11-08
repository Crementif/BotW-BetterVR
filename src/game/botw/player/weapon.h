#pragma once

#include "weapon_motion.h"

namespace Weapons {
    constexpr bool TwoHandGripEnabled = true;
    constexpr bool CarryHandPinEnabled = true;
    constexpr float TwoHandGripButtonThreshold = 0.5f;

    void RegisterHooks();
    void OnGameFrame();

    uint32_t GetHeldActor(uint32_t side);
    WeaponType GetHeldType(uint32_t side);
    bool IsArrowNockedInRightHand();
    // a bow, the Sheikah Slate or the magnesis glove, which need the HUD to stay in front of the player
    bool IsAimingItemHeld();
    WeaponMotionAnalyser& GetMotionAnalyser(uint32_t side);
    bool CanEngageTwoHandGrip();

    struct HandObservations {
        std::array<EquipType, 2> equipTypes;
        std::array<bool, 2> occupied;
        bool throwableHeld;
    };
    HandObservations ReadHandObservations();
    void ResetHandObservations();
    bool IsThrowableHeld();
    bool IsThrowableCarriedInBothHands();

    void RequestDrop(uint32_t side);
    void CancelDrops();
}
