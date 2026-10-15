#pragma once

namespace Player {
    void RegisterHooks();
    void OnGameFrame();

    uint32_t GetAddress();
    uint32_t GetMtxAddress();
    bool TryRead(PlayerBase& player);

    PlayerMoveBitFlags GetMoveFlags();
    bool IsRiding(bool ignoreSandSeal = false);
    bool IsGliding();
    bool IsClimbing();
    bool IsClimbingLadder();

    bool TrackDamageFloatParam(uint32_t actionPtr, uint32_t destFloatPtr, std::string_view paramName);
}
