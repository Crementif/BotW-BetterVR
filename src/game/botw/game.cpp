#include "pch.h"
#include "game/game.h"
#include "game_state.h"
#include "game/botw/camera/camera.h"
#include "game/botw/camera/stereo_view.h"
#include "game/botw/debug/debug_hooks.h"
#include "game/botw/debug/weapon_attack_debugger.h"
#include "game/botw/input/controls.h"
#include "game/botw/interface/interface_hooks.h"
#include "game/botw/player/bow.h"
#include "game/botw/player/magnesis.h"
#include "game/botw/player/player.h"
#include "game/botw/player/roomscale.h"
#include "game/botw/player/skeleton.h"
#include "game/botw/player/weapon.h"

static constexpr std::array kTitles = {
    Game::Title{
        .name = "Breath of the Wild",
        .titleIds = { 0x00050000101C9300, 0x00050000101C9400, 0x00050000101C9500 },
        .vrPack = "BreathOfTheWild_BetterVR",
        .graphicsPack = "BreathOfTheWild_Graphics",
        // the game crashes in VR without it
        .requiredPack = "BreathOfTheWild/Mods/FPS++",
        .renderWidth = 1280,
        .renderHeight = 720,
    },
};

std::span<const Game::Title> Game::GetTitles() {
    return kTitles;
}

const Game::Title* Game::FindTitle(uint64_t titleId) {
    for (const Title& title : kTitles) {
        if (std::ranges::contains(title.titleIds, titleId)) {
            return &title;
        }
    }
    return nullptr;
}

void Game::RegisterHooks() {
    GameState::RegisterHooks();
    Camera::RegisterHooks();
    StereoView::RegisterHooks();
    Player::RegisterHooks();
    Skeleton::RegisterHooks();
    Weapons::RegisterHooks();
    Bow::RegisterHooks();
    Magnesis::RegisterHooks();
    Roomscale::RegisterHooks();
    Controls::RegisterHooks();
    Interface::RegisterHooks();
    DebugHooks::RegisterHooks();
}

glm::mat4 Game::GetTrackingOrigin() {
    return Camera::GetReferenceMtx();
}

float Game::GetComfortFadeAmount() {
    return Camera::GetSnapTurnFadeAmount();
}

void Game::QueueWorldOverlays(long presentedFrameIdx) {
    Bow::QueueAimingArcPreview(presentedFrameIdx);
}

void Game::DrawDebugWindows() {
    WeaponAttackDebugger::DrawOverlays();
}

bool Game::IsCapturingMono() {
    return GameState::UseMonoFrameBufferTemporarilyDuringMenusOrPictures();
}

bool Game::UsesBlackBars() {
    return GameState::UseBlackBarsDuringEvents();
}

uint32_t Game::GetSharedMemoryAddress() {
    return StereoView::GetSharedMemoryAddress();
}
