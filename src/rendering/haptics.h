#pragma once

// requests from any hook get queued and played in Update
namespace Haptics {
    void Play(const RumbleParameters& rumble);
    void Stop(int hand, RumbleType rumbleType);
    void Update();

    void PlayGamePattern(uint8_t* pattern, uint8_t length);
    void StopGamePattern();
}
