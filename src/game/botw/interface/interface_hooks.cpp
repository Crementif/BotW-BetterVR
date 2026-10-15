#include "pch.h"
#include "interface_hooks.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"

static void hook_FixStaminaGaugeScreenPosition(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (GameState::IsThirdPerson()) {
        return;
    }

    BEVec2 staminaGauge2DPos = BEVec2{ -482.0f, +170.0f }; // nested under hearts
    BEVec3 playerPosToCameraPos = BEVec3{ 0.0f, 0.0, 0.0f };
    Guest::Write(hCPU->gpr[4], &staminaGauge2DPos);
    Guest::Write(hCPU->gpr[5], &playerPosToCameraPos);
}

static void hook_FixExtraStaminaGaugeIconPositions(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    hCPU->fpr[29].fp0 = 1.0f;

    if (GameState::IsThirdPerson()) {
        return;
    }

    hCPU->instructionPointer = 0x02FB29C4;
}

// todo: this only runs when it's shown for the first time!
static void hook_CreateNewScreen(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    const char* screenName = Guest::CString(hCPU->gpr[7]);
    ScreenId screenId = (ScreenId)hCPU->gpr[5];
    Log::print<CONTROLS>("Creating new screen \"{}\" with ID {:08X}...", screenName, std::to_underlying(screenId));
}

enum class GX2_BLENDFACTOR {
    ZERO = 0x00,
    ONE = 0x01,
    SRC_COLOR = 0x02,
    ONE_MINUS_SRC_COLOR = 0x03,
    SRC_ALPHA = 0x04,
    ONE_MINUS_SRC_ALPHA = 0x05,
    DST_ALPHA = 0x06,
    ONE_MINUS_DST_ALPHA = 0x07,
    DST_COLOR = 0x08,
    ONE_MINUS_DST_COLOR = 0x09,
    SRC_ALPHA_SATURATE = 0x0A,
    BOTH_SRC_ALPHA = 0x0B,
    BOTH_INV_SRC_ALPHA = 0x0C,
    CONST_COLOR = 0x0D,
    ONE_MINUS_CONST_COLOR = 0x0E,
    SRC1_COLOR = 0x0F,
    INV_SRC1_COLOR = 0x10,
    SRC1_ALPHA = 0x11,
    INV_SRC1_ALPHA = 0x12,
    CONST_ALPHA = 0x13,
    ONE_MINUS_CONST_ALPHA = 0x14
};

enum class GX2_COMBINEFUNC {
    DST_PLUS_SRC = 0,
    SRC_MINUS_DST = 1,
    MIN_DST_SRC = 2,
    MAX_DST_SRC = 3,
    DST_MINUS_SRC = 4
};

// GX2SetBlendControl(target, colorSrc, colorDst, colorCombine, separateAlpha, alphaSrc, alphaDst, alphaCombine)
static void hook_FixUIBlending(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    GX2_BLENDFACTOR colorSrcFactor = (GX2_BLENDFACTOR)hCPU->gpr[4];
    GX2_BLENDFACTOR colorDstFactor = (GX2_BLENDFACTOR)hCPU->gpr[5];
    GX2_COMBINEFUNC colorCombineFunc = (GX2_COMBINEFUNC)hCPU->gpr[6];
    GX2_BLENDFACTOR alphaSrcFactor = (GX2_BLENDFACTOR)hCPU->gpr[8];
    GX2_BLENDFACTOR alphaDstFactor = (GX2_BLENDFACTOR)hCPU->gpr[9];
    GX2_COMBINEFUNC alphaCombineFunc = (GX2_COMBINEFUNC)hCPU->gpr[10];

    bool matchesColorSettings = colorSrcFactor == GX2_BLENDFACTOR::DST_COLOR && colorDstFactor == GX2_BLENDFACTOR::SRC_ALPHA && colorCombineFunc == GX2_COMBINEFUNC::DST_PLUS_SRC;
    bool matchesAlpha = alphaSrcFactor == GX2_BLENDFACTOR::SRC_ALPHA && alphaDstFactor == GX2_BLENDFACTOR::ONE_MINUS_SRC_ALPHA && alphaCombineFunc == GX2_COMBINEFUNC::DST_PLUS_SRC;
    if (matchesColorSettings && matchesAlpha) {
        hCPU->gpr[7] = 1;
        hCPU->gpr[8] = std::to_underlying(GX2_BLENDFACTOR::ZERO);
        hCPU->gpr[9] = std::to_underlying(GX2_BLENDFACTOR::DST_ALPHA);
    }
}

void Interface::RegisterHooks() {
    CemuHost::RegisterHook("hook_FixStaminaGaugeScreenPosition", &hook_FixStaminaGaugeScreenPosition);
    CemuHost::RegisterHook("hook_FixExtraStaminaGaugeIconPositions", &hook_FixExtraStaminaGaugeIconPositions);
    CemuHost::RegisterHook("hook_CreateNewScreen", &hook_CreateNewScreen);
    CemuHost::RegisterHook("hook_FixUIBlending", &hook_FixUIBlending);
}
