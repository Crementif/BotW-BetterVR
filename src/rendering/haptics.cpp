#include "pch.h"
#include "haptics.h"
#include "instance.h"

static RumbleManager* GetRumbleManager() {
    auto& xr = VRManager::instance().XR;
    return xr ? xr->GetRumbleManager() : nullptr;
}

void Haptics::Play(const RumbleParameters& rumble) {
    if (RumbleManager* manager = GetRumbleManager())
        manager->enqueueInputsRumbleCommand(rumble);
}

void Haptics::Stop(int hand, RumbleType rumbleType) {
    if (RumbleManager* manager = GetRumbleManager())
        manager->stopInputsRumble(hand, rumbleType);
}

void Haptics::Update() {
    if (RumbleManager* manager = GetRumbleManager())
        manager->updateHaptics();
}

void Haptics::PlayGamePattern(uint8_t* pattern, uint8_t length) {
    if (RumbleManager* manager = GetRumbleManager())
        manager->controlMotor(pattern, length);
}

void Haptics::StopGamePattern() {
    if (RumbleManager* manager = GetRumbleManager())
        manager->stopMotor();
}
