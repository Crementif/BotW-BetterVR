#include "pch.h"
#include "interface_hooks.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_state.h"
#include "game/botw/camera/stereo_view.h"
#include "game/botw/player/hand_aim.h"
#include "game/botw/player/roomscale.h"
#include "rendering/renderer.h"
#include "rendering/tracking.h"

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

// sead::Projection::cameraPosToScreenPos (0x030C1068) finishing its screen position, replaces stfs f7, 8(r30)
// the game places name labels and markers for its own flat screen, but the headset shows the HUD on a smaller floating panel,
// so they go where the line from the eyes to the spot crosses that panel instead
static void hook_PlaceScreenMarker(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x030C1138;

    const uint32_t screenPosPtr = hCPU->gpr[30];
    const uint32_t cameraPosPtr = hCPU->gpr[31];
    const uint32_t projectionMtxPtr = hCPU->gpr[3];
    const float depth = (float)hCPU->fpr[7].fp0;
    Guest::WriteBE(screenPosPtr + 0x8, &depth);

    // only the HUD screens, sead::Projection::project also feeds on-screen checks in gameplay code
    const uint32_t returnAddress = Guest::Get<uint32_t>(hCPU->gpr[1] + 0x14).getLE();
    if (returnAddress != 0x02ED3BF0 && returnAddress != 0x02F565AC && returnAddress != 0x02F9743C && returnAddress != 0x02FB24CC && returnAddress != 0x0360A500)
        return;

    if (!Tracking::IsSessionRunning() || !GameState::IsInGame() || GameState::UseMonoFrameBufferTemporarilyDuringMenusOrPictures() || GameState::UseBlackBarsDuringEvents())
        return;

    // only perspective projections, with a last row of (0, 0, -1, 0)
    std::array<float, 4> lastRow = {};
    for (uint32_t i = 0; i < lastRow.size(); i++)
        Guest::ReadBE(projectionMtxPtr + 0x30 + i * 4, &lastRow[i]);
    if (lastRow[0] != 0.0f || lastRow[1] != 0.0f || lastRow[2] != -1.0f || lastRow[3] != 0.0f)
        return;

    const std::optional<glm::mat4> gameCameraWorld = StereoView::GetGameCameraWorld();
    const std::optional<RND_Renderer::HudQuad> hud = RND_Renderer::GetHudQuad();
    const std::optional<glm::fmat4> head = Tracking::HeadPose();
    if (!gameCameraWorld.has_value() || !hud.has_value() || !head.has_value())
        return;

    const glm::vec3 cameraPos = Guest::Get<BEVec3>(cameraPosPtr).getLE();
    const glm::vec3 worldPos = glm::vec3(gameCameraWorld.value() * glm::vec4(cameraPos, 1.0f));
    glm::vec3 stagePos = glm::vec3(glm::inverse(HandAim::ResolveCameraReferenceMtx(-1)) * glm::vec4(worldPos, 1.0f));
    if (GameState::IsFirstPerson()) {
        const glm::vec3 appliedHeadPos = Roomscale::GetAppliedHeadPosition();
        stagePos.x += appliedHeadPos.x;
        stagePos.z += appliedHeadPos.z;
    }

    const glm::vec3 eyePos = glm::vec3(head.value()[3]);
    const glm::vec3 panelNormal = hud->orientation * glm::vec3(0.0f, 0.0f, 1.0f);
    const glm::vec3 toMarker = stagePos - eyePos;
    const float along = glm::dot(toMarker, panelNormal);
    if (along > -0.01f)
        return;

    const glm::vec3 hit = eyePos + toMarker * (glm::dot(hud->position - eyePos, panelNormal) / along);
    const glm::vec3 onPanel = glm::inverse(hud->orientation) * (hit - hud->position);
    const glm::vec2 screenPos = glm::vec2(onPanel.x / (hud->size.x * 0.5f), onPanel.y / (hud->size.y * 0.5f));
    if (!std::isfinite(screenPos.x) || !std::isfinite(screenPos.y))
        return;
    Guest::WriteBE(screenPosPtr, &screenPos.x);
    Guest::WriteBE(screenPosPtr + 0x4, &screenPos.y);
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
    CemuHost::RegisterHook("hook_PlaceScreenMarker", &hook_PlaceScreenMarker);
    CemuHost::RegisterHook("hook_FixUIBlending", &hook_FixUIBlending);
}
