#include "pch.h"
#include "debug_hooks.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_utils.h"
#include "utils/mod_settings.h"
#include "utils/debug_draw.h"

static void hook_OSReportToConsole(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (hCPU->gpr[3] == 0) {
        return;
    }

    std::string_view message = Guest::CString(hCPU->gpr[3]);
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.remove_suffix(1);
    }
    if (!message.empty()) {
        Log::print<PPC>("{}", message);
    }
}

static bool TryGetProfilerSection(uint32_t sectionId, BetterVRProfiler::Section* outSection) {
    if (sectionId >= (uint32_t)BetterVRProfiler::Section::Count) {
        return false;
    }

    *outSection = (BetterVRProfiler::Section)sectionId;
    return true;
}

static void hook_ProfileSectionBegin(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    BetterVRProfiler::Section section = BetterVRProfiler::Section::RoomscaleResolve;
    if (TryGetProfilerSection(hCPU->gpr[3], &section)) {
        BetterVRProfiler::BeginSpan(section);
    }
}

static void hook_ProfileSectionEnd(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    BetterVRProfiler::Section section = BetterVRProfiler::Section::RoomscaleResolve;
    if (TryGetProfilerSection(hCPU->gpr[3], &section)) {
        BetterVRProfiler::EndSpan(section);
    }
}

static void hook_VisualizeRayCastHits(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    if (!GetSettings().ShouldShowRaycastLines()) {
        return;
    }

    ksys::phys::RayCast rayCast = {};
    Guest::Read(hCPU->gpr[3], &rayCast);

    const glm::fvec3 lineOffset = glm::fvec3(0.0f, 0.5f, 0.0f);
    const glm::fvec3 rayStart = rayCast.from.getLE() + lineOffset;
    const glm::fvec3 rayEnd = rayCast.to.getLE() + lineOffset;
    const glm::fvec3 raycastHitPos = Guest::Get<BEVec3>(hCPU->gpr[4]).getLE() + lineOffset;

    DebugDraw::instance().Line(rayStart, raycastHitPos, IM_COL32(255, 0, 255, 255));
    DebugDraw::instance().Line(raycastHitPos, rayEnd, IM_COL32(128, 0, 128, 128));
}

static void hook_DropWeaponLogging(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t actorPtr = hCPU->gpr[3];
    uint32_t actorNamePtr = Guest::Get<uint32_t>(actorPtr + offsetof(ActorWiiU, name) + offsetof(sead::FixedSafeString40, c_str)).getLE();
    if (actorNamePtr == 0)
        return;

    const glm::fvec3 position = Guest::Get<BEVec3>(hCPU->gpr[5]).getLE();
    Log::print<CONTROLS>("{} ({:08X}) is dropping weapon with idx={}, position={}, a4={}, a5={}, a6={}, a7={}", Guest::CString(actorNamePtr), actorPtr, hCPU->gpr[4], position, hCPU->gpr[6], hCPU->gpr[7], hCPU->gpr[8], hCPU->gpr[9]);
}

static void hook_ModifyHandModelAccessSearch(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

#ifdef _DEBUG
    if (hCPU->gpr[3] != 0) {
        Log::print<CONTROLS>("Searching for model handle using {}", Guest::CString(hCPU->gpr[3]));
    }
#endif
}

#define LOG_PLAYER_BODY(...)                                \
    do {                                                    \
        if (GameUtils::IsTrackedPlayerBody(hCPU->gpr[3])) { \
            Log::print<ROOMSCALE>(__VA_ARGS__);             \
        }                                                   \
    } while (0)

static void hook_SetRigidBodyVelocity(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    LOG_PLAYER_BODY("[{:08X}] SetVelocity {}", hCPU->gpr[0], Guest::Get<BEVec3>(hCPU->gpr[4]).getLE());
}

static void hook_SetRigidBodyTransform(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x03486CD8;
    hCPU->gpr[0] = hCPU->sprNew.LR;

    BEMatrix34 transform = Guest::Get<BEMatrix34>(hCPU->gpr[4]);
    LOG_PLAYER_BODY("[{:08X}] SetTransform pos={} rot={}", hCPU->sprNew.LR, transform.getPos().getLE(), transform.getRotLE());
}

static void hook_SetRigidBodyScale(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x03486F50;
    hCPU->gpr[0] = hCPU->sprNew.LR;

    LOG_PLAYER_BODY("[{:08X}] SetScale {}", hCPU->sprNew.LR, (float)hCPU->fpr[1].fp0);
}

static void hook_SetRigidBodyPosition(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x03486D84;
    hCPU->gpr[0] = hCPU->sprNew.LR;

    LOG_PLAYER_BODY("[{:08X}] SetPosition {}", hCPU->sprNew.LR, Guest::Get<BEVec3>(hCPU->gpr[4]).getLE());
}

static void hook_SetRigidBodyPositionAndRotation(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = 0x03489A84;
    hCPU->gpr[11] = hCPU->gpr[1];

    BEMatrix34 transform = Guest::Get<BEMatrix34>(hCPU->gpr[4]);
    LOG_PLAYER_BODY("[{:08X}] SetPositionAndRotation pos={} rot={}", hCPU->sprNew.LR, transform.getPos().getLE(), transform.getRotLE());
}

void DebugHooks::RegisterHooks() {
    CemuHost::RegisterHook("hook_OSReportToConsole", &hook_OSReportToConsole);
    CemuHost::RegisterHook("hook_ProfileSectionBegin", &hook_ProfileSectionBegin);
    CemuHost::RegisterHook("hook_ProfileSectionEnd", &hook_ProfileSectionEnd);
    CemuHost::RegisterHook("hook_VisualizeRayCastHits", &hook_VisualizeRayCastHits);
    CemuHost::RegisterHook("hook_DropWeaponLogging", &hook_DropWeaponLogging);
    CemuHost::RegisterHook("hook_ModifyHandModelAccessSearch", &hook_ModifyHandModelAccessSearch);
    CemuHost::RegisterHook("hook_SetRigidBodyVelocity", &hook_SetRigidBodyVelocity);
    CemuHost::RegisterHook("hook_SetRigidBodyTransform", &hook_SetRigidBodyTransform);
    CemuHost::RegisterHook("hook_SetRigidBodyScale", &hook_SetRigidBodyScale);
    CemuHost::RegisterHook("hook_SetRigidBodyPosition", &hook_SetRigidBodyPosition);
    CemuHost::RegisterHook("hook_SetRigidBodyPositionAndRotation", &hook_SetRigidBodyPositionAndRotation);
}
