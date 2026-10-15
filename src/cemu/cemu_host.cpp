#include "pch.h"
#include "cemu_host.h"
#include "host_window.h"
#include "game/game.h"

CemuHost::CemuHost() {
    const uint64_t titleId = GetCurrentTitleId();
    checkAssert(Game::FindTitle(titleId) != nullptr, fmt::format("BetterVR doesn't support title {:016x}", titleId).c_str());

    Guest::s_baseAddress = (uint64_t)GetGuestMemoryBase();
    checkAssert(Guest::s_baseAddress != 0, "Failed to get memory base address of Cemu process!");

    HostWindow::Discover();
}
