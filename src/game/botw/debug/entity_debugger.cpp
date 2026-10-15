#include "pch.h"
#include "entity_debugger.h"
#include "cemu/cemu_host.h"
#include "game/botw/game_utils.h"
#include "game/botw/player/player.h"
#include "instance.h"
#include "rendering/imgui_menus.h"
#include "utils/debug_draw.h"
#include "utils/mod_settings.h"

std::mutex g_actorListMutex;
std::unordered_map<uint32_t, std::pair<std::string, uint32_t>> s_knownActors;

void EntityDebugger::TrackActor(uint32_t listIndex, uint32_t actorPtr, const char* actorName) {
    std::scoped_lock lock(g_actorListMutex);

    // only the entity debugger reads the actor list
    if (!VRManager::instance().Debugger || !GetSettings().enableDebuggerTools.load(std::memory_order_relaxed)) {
        s_knownActors.clear();
        return;
    }

    // the list is walked from the start again every time an actor gets created
    if (listIndex == 0) {
        s_knownActors.clear();
    }

    if (actorName[0] != '\0') {
        s_knownActors.emplace(actorPtr + stringToHash(actorName), std::make_pair(actorName, actorPtr));
    }
}

// ksys::phys::RigidBodyFromShape::create to create a RigidBody from a shape
// use Actor::getRigidBodyByName

std::unordered_map<uint32_t, std::pair<std::string, uint32_t>> s_alreadyAddedActors;

#ifdef _DEBUG
struct ArrowTrail {
    std::vector<glm::vec3> points;
    int framesRemaining = 180 * 20;
};
static std::unordered_map<uint32_t, ArrowTrail> s_activeArrowTrails;
static std::vector<ArrowTrail> s_persistedArrowTrails;
static constexpr float kArrowTrailSeedSpeedSq = 0.0001f;
static constexpr float kArrowTrailSeedDistance = 0.05f;

static void UpdateArrowTrails() {
    std::unordered_set<uint32_t> currentFrameArrowIds;

    for (const auto& [actorId, actorInfo] : s_knownActors) {
        const std::string& actorName = actorInfo.first;
        if (actorName.find("Arrow") != std::string::npos && actorName.find("Quiver") == std::string::npos) {
            uint32_t actorPtr = actorInfo.second;
            BEMatrix34 mtx = Guest::Get<BEMatrix34>(actorPtr + offsetof(ActorWiiU, mtx));
            BEVec3 velocity = Guest::Get<BEVec3>(actorPtr + offsetof(ActorWiiU, velocity));
            glm::vec3 currentPos = mtx.getPos().getLE();
            glm::vec3 currentVelocity = velocity.getLE();

            if (glm::all(glm::isfinite(currentPos)) && glm::all(glm::isfinite(currentVelocity))) {
                currentFrameArrowIds.insert(actorId);
                ArrowTrail& activeTrail = s_activeArrowTrails[actorId];
                const float speedSq = glm::length2(currentVelocity);

                if (activeTrail.points.empty() && speedSq >= kArrowTrailSeedSpeedSq) {
                    activeTrail.points.push_back(currentPos - glm::normalize(currentVelocity) * kArrowTrailSeedDistance);
                }

                if (activeTrail.points.empty() || glm::distance2(activeTrail.points.back(), currentPos) > 0.0001f) {
                    activeTrail.points.push_back(currentPos);
                }
            }
        }
    }

    for (auto it = s_activeArrowTrails.begin(); it != s_activeArrowTrails.end();) {
        uint32_t actorId = it->first;
        if (!currentFrameArrowIds.contains(actorId)) {
            if (it->second.points.size() >= 2) {
                s_persistedArrowTrails.push_back(std::move(it->second));
            }
            it = s_activeArrowTrails.erase(it);
        }
        else {
            ++it;
        }
    }

    for (const auto& trail : s_activeArrowTrails | std::views::values) {
        if (trail.points.size() >= 2) {
            for (size_t i = 1; i < trail.points.size(); ++i) {
                DebugDraw::instance().Line(trail.points[i - 1], trail.points[i], DebugDrawColor(255, 64, 220, 255), 4.0f, false);
            }
            DebugDraw::instance().Dot(trail.points.back(), 0.2f, DebugDrawColor(255, 64, 220, 255), false);
        }
        else if (trail.points.size() == 1) {
            DebugDraw::instance().Dot(trail.points.front(), 0.28f, DebugDrawColor(255, 64, 220, 255), false);
        }
    }

    for (auto it = s_persistedArrowTrails.begin(); it != s_persistedArrowTrails.end();) {
        it->framesRemaining--;
        if (it->framesRemaining <= 0) {
            it = s_persistedArrowTrails.erase(it);
        }
        else {
            const float age = 1.0f - (float)it->framesRemaining / (float)(180 * 20);
            const uint8_t alpha = (uint8_t)glm::clamp((int)std::lround(255.0f * (1.0f - age)), 0, 255);
            const uint32_t fadeColor = DebugDrawColor(255, 64, 220, alpha);

            for (size_t i = 1; i < it->points.size(); ++i) {
                DebugDraw::instance().Line(it->points[i - 1], it->points[i], fadeColor, 4.0f, false);
            }

            for (size_t i = 0; i < it->points.size(); i += 8) {
                DebugDraw::instance().Dot(it->points[i], 0.03f * 4.0f, fadeColor, false);
            }
            if (!it->points.empty()) {
                DebugDraw::instance().Dot(it->points.back(), 0.04f * 4.0f, fadeColor, false);
            }
            ++it;
        }
    }
}
#endif

static bool IsEntityInspectorVisible() {
    return ImGuiMenus::g_isOpen && ImGuiMenus::g_currentPage == ImGuiMenus::DEBUG_PAGE;
}

void EntityDebugger::UpdateEntityMemory() {
    const bool buildInspectorModel = IsEntityInspectorVisible();
    const bool drawWorldBoxes = GetSettings().ShouldShowEntityBoxesIn3DView();
#ifdef _DEBUG
    const bool trackArrowTrails = Log::isLogTypeEnabled<ARROW_SHOT_CAPTURE>();
#else
    constexpr bool trackArrowTrails = false;
#endif

    if (!buildInspectorModel && !drawWorldBoxes && !trackArrowTrails) {
        return;
    }

    std::scoped_lock lock(g_actorListMutex);
    if (buildInspectorModel) {
        for (const auto& hash : s_alreadyAddedActors | std::views::keys) {
            if (!s_knownActors.contains(hash)) {
                RemoveEntity(hash);
            }
        }

        s_alreadyAddedActors = s_knownActors;
    }

    if (buildInspectorModel || drawWorldBoxes) {
        UpdateActorEntities(buildInspectorModel, drawWorldBoxes);
    }

#ifdef _DEBUG
    if (trackArrowTrails) {
        UpdateArrowTrails();
    }
#endif
}

void EntityDebugger::UpdateActorEntities(bool buildInspectorModel, bool drawWorldBoxes) {
    BEMatrix34 playerPos = {};
    for (const auto& actorData : s_knownActors | std::views::values) {
        if (actorData.first == "GameROMPlayer") {
            Guest::Read(actorData.second + offsetof(ActorWiiU, mtx), &playerPos);
            glm::fvec3 newPlayerPos = playerPos.getPos().getLE();
            m_playerPos = newPlayerPos;
        }
        else if (actorData.first == "GameRomCamera") {
            Guest::Read(actorData.second + offsetof(ActorWiiU, mtx), &playerPos);
            glm::fvec3 newPlayerPos = playerPos.getPos().getLE();
        }
        else if (actorData.first.starts_with("Weapon_Sword")) {
        }
    }

    const bool logAnimationSlots = buildInspectorModel && m_logAnimationSlots.load(std::memory_order_relaxed);

    for (auto& [actorId, actorInfo] : s_knownActors) {
        uint32_t actorPtr = actorInfo.second;
        const std::string& actorName = actorInfo.first;

        BEMatrix34 mtx = Guest::Get<BEMatrix34>(actorPtr + offsetof(ActorWiiU, mtx));
        BEVec3 aabbMin = Guest::Get<BEVec3>(actorPtr + offsetof(ActorWiiU, aabb.min));
        BEVec3 aabbMax = Guest::Get<BEVec3>(actorPtr + offsetof(ActorWiiU, aabb.max));

        if (drawWorldBoxes) {
            glm::fvec3 pos = mtx.getPos().getLE();
            float distance = glm::distance(m_playerPos, pos);
            bool matchesFilter = m_filter.empty() || actorName.find(m_filter) != std::string::npos;

            if (matchesFilter && distance >= m_worldAABBMinDistance && distance <= m_worldAABBMaxDistance) {
                glm::fquat rot = mtx.getRotLE();
                glm::fvec3 localMin = aabbMin.getLE();
                glm::fvec3 localMax = aabbMax.getLE();
                glm::fvec3 halfExtents = (localMax - localMin) * 0.5f;

                if (glm::all(glm::greaterThan(halfExtents, glm::vec3(0.0f)))) {
                    glm::fvec3 worldCenter = pos + glm::mat3_cast(rot) * ((localMin + localMax) * 0.5f);
                    DebugDraw::instance().Box(worldCenter, halfExtents, rot, IM_COL32(255, 255, 255, 255 / 1), 1.0f);
                }
                else {
                    DebugDraw::instance().Dot(pos, 0.15f, IM_COL32(255, 255, 255, 255 / 1));
                }
            }
        }

        if (!buildInspectorModel) {
            continue;
        }

        auto addField = [&]<typename T>(std::string_view name, uint32_t offset) -> void {
            uint32_t address = actorPtr + offset;
            AddOrUpdateEntity(actorId, actorName, name, address, Guest::Get<T>(address));
        };

        auto addMemoryRange = [&](std::string_view name, const uint32_t addressPtr, const uint32_t size) -> void {
            if (HasEntityValue(actorId, name)) {
                return;
            }

            uint32_t address = 0;
            if (Guest::ReadBE(addressPtr, &address); address != 0) {
                AddOrUpdateEntity(actorId, actorName, name, address, MemoryRange{ address, address + size, std::make_unique<MemoryEditor>() });
            }
        };

        if (actorName.starts_with("Weapon")) {
            addField.operator()<BEVec3>("Weapon::originalScale", offsetof(Weapon, originalScale));
            addMemoryRange("Weapon::actorAtk.struct7Ptr", actorPtr + offsetof(Weapon, actorAtk.struct7Ptr), 0x2D8);
            addMemoryRange("Weapon::actorAtk.attackSensorStruct7Ptr", actorPtr + offsetof(Weapon, actorAtk.attackSensorPtr), 0x5C);
            addMemoryRange("Weapon::actorAtk.struct8Ptr", actorPtr + offsetof(Weapon, actorAtk.struct8Ptr), 0x714);
            addMemoryRange("Weapon::actorAtk.attackSensorStruct8Ptr", actorPtr + offsetof(Weapon, actorAtk.attackSensorStruct8Ptr), 0x28);
            addField.operator()<BEType<uint16_t>>("Weapon::weaponFlags", offsetof(Weapon, weaponFlags));
            addField.operator()<BEType<uint16_t>>("Weapon::otherFlags", offsetof(Weapon, otherFlags));
            addField.operator()<BEType<int32_t>>("Weapon::requestedActorWeaponsCategory", offsetof(Weapon, requestedActorWeaponsCategory));
        }

        if (actorName.starts_with("GameROMPlayer")) {
            addMemoryRange("ActorWeapons", actorPtr + offsetof(PlayerOrEnemy, weapons), 0x68);

        }

        AddOrUpdateEntity(actorId, actorName, "mtx", actorPtr + offsetof(ActorWiiU, mtx), mtx);
        if (playerPos.pos_x.getLE() != 0.0f) {
            SetPosition(actorId, playerPos.getPos(), mtx.getPos());
        }
        SetRotation(actorId, mtx.getRotLE());

        if (aabbMin.x.getLE() != 0.0f) {
            SetAABB(actorId, aabbMin.getLE(), aabbMax.getLE());
        }

        addField.operator()<BEVec3>("velocity", offsetof(ActorWiiU, velocity));
        addField.operator()<BEVec3>("angularVelocity", offsetof(ActorWiiU, angularVelocity));
        addField.operator()<BEVec3>("scale", offsetof(ActorWiiU, scale));
        addField.operator()<BEVec3>("aabb_min", offsetof(ActorWiiU, aabb.min));
        addField.operator()<BEVec3>("aabb_max", offsetof(ActorWiiU, aabb.max));
        addField.operator()<uint32_t>("flags2", offsetof(ActorWiiU, flags2));
        addField.operator()<uint32_t>("flags2Copy", offsetof(ActorWiiU, flags2Copy));
        addField.operator()<uint32_t>("flags", offsetof(ActorWiiU, flags));
        addField.operator()<uint32_t>("flags3", offsetof(ActorWiiU, flags3));

        uint32_t asListPtr = logAnimationSlots ? Guest::Get<BEType<uint32_t>>(actorPtr + offsetof(ActorWiiU, asListPtr)).getLE() : 0;
        if (asListPtr != 0) {
            std::string animations;
            for (const auto& slot : GameUtils::GetAnimationSlots(asListPtr)) {
                if (!animations.empty()) {
                    animations += '\n';
                }
                fmt::format_to(std::back_inserter(animations), "[{}][{}] {} ({:08X})", slot.groupIndex, slot.slotIndex, slot.name, slot.slotPtr);
            }

            if (animations.empty()) {
                animations = "<none assigned>";
            }
            else {
                Log::print<INFO>("ASList animations of {} (ASList at {:08X}):\n{}", actorName, asListPtr, animations);
            }

            AddOrUpdateEntity(actorId, actorName, "ASList::getAnimation", asListPtr, std::move(animations));
        }

        addField.operator()<uint32_t>("hashId", offsetof(ActorWiiU, hashId));
        addMemoryRange("physics", actorPtr + offsetof(ActorWiiU, actorPhysicsPtr), 0xE0);
        addMemoryRange("actorX6A0", actorPtr + offsetof(ActorWiiU, actorX6A0Ptr), 0x6C);
        addMemoryRange("chemicals", actorPtr + offsetof(ActorWiiU, chemicalsPtr), 0x64);
        addMemoryRange("reactions", actorPtr + offsetof(ActorWiiU, reactionsPtr), 0x0C);
    }
}

void EntityDebugger::DrawWorldSpaceOverlaySettings(bool* changed) {
    auto& settings = GetSettings();

    bool showEntityBoxes = settings.debugShowEntityBoxesIn3DView;
    if (ImGui::Checkbox("Show Entity Boxes In 3D View", &showEntityBoxes)) {
        settings.debugShowEntityBoxesIn3DView = showEntityBoxes;
        *changed = true;
    }

    if (showEntityBoxes) {
        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragFloat("Min 3D Box Distance", &m_worldAABBMinDistance, 1.0f, 0.0f, 10000.0f, "%.0f");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragFloat("Max 3D Box Distance", &m_worldAABBMaxDistance, 1.0f, 0.0f, 10000.0f, "%.0f");
    }

    bool showRaycastLines = settings.debugShowRaycastLines;
    if (ImGui::Checkbox("Show Raycast Lines", &showRaycastLines)) {
        settings.debugShowRaycastLines = showRaycastLines;
        *changed = true;
    }

    bool showWeaponAxes = settings.debugShowWeaponAxes;
    if (ImGui::Checkbox("Show Weapon Axes", &showWeaponAxes)) {
        settings.debugShowWeaponAxes = showWeaponAxes;
        *changed = true;
    }

    bool showRoomscalePhysics = settings.debugShowRoomscalePhysics;
    if (ImGui::Checkbox("Show Roomscale Physics", &showRoomscalePhysics)) {
        settings.debugShowRoomscalePhysics = showRoomscalePhysics;
        *changed = true;
    }
}

void EntityDebugger::DrawEntityInspectorContent() {
    static char buf[256];
    ImGui::InputText("Entity Filter", buf, std::size(buf));
    m_filter = buf;

    bool logAnimationSlots = m_logAnimationSlots.load(std::memory_order_relaxed);
    if (ImGui::Checkbox("Log ASList Animations", &logAnimationSlots)) {
        m_logAnimationSlots.store(logAnimationSlots, std::memory_order_relaxed);
    }

    if (ImGui::CollapsingHeader("Entity List")) {
        std::multimap<float, std::reference_wrapper<Entity>> sortedEntities;
        for (auto& entity : m_entities | std::views::values) {
            if (m_filter.empty() || entity.name.find(m_filter) != std::string::npos) {
                sortedEntities.emplace(entity.priority, entity);
            }
        }

        for (auto& entity : sortedEntities | std::views::values) {
            std::string id = entity.get().name + "##" + std::to_string(entity.get().values[0].value_address);
            ImGui::Text(fmt::format("{}: dist={}", entity.get().name, std::abs(entity.get().priority)).c_str());
            ImGui::PushID(id.c_str());

            for (auto& value : entity.get().values) {
                ImGui::PushID(value.value_name.c_str());

                if (ImGui::Button("Copy")) {
                    ImGui::SetClipboardText(fmt::format("0x{:08x}", value.value_address).c_str());
                }
                ImGui::SameLine();

                std::visit([&]<typename T0>(T0&& arg) {
                    using T = std::decay_t<T0>;

                    if constexpr (std::is_same_v<T, BEType<uint32_t>>) {
                        uint32_t val = std::get<BEType<uint32_t>>(value.value).getLE();
                        if (ImGui::DragScalar(value.value_name.c_str(), ImGuiDataType_U32, &val)) {
                            std::get<BEType<uint32_t>>(value.value) = val;
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEType<int32_t>>) {
                        int32_t val = std::get<BEType<int32_t>>(value.value).getLE();
                        if (ImGui::DragScalar(value.value_name.c_str(), ImGuiDataType_S32, &val)) {
                            std::get<BEType<int32_t>>(value.value) = val;
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEType<float>>) {
                        float val = std::get<BEType<float>>(value.value).getLE();
                        if (ImGui::DragScalar(value.value_name.c_str(), ImGuiDataType_Float, &val)) {
                            std::get<BEType<float>>(value.value) = val;
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEType<uint8_t>>) {
                        uint8_t val = std::get<BEType<uint8_t>>(value.value).getLE();
                        if (ImGui::DragScalar(value.value_name.c_str(), ImGuiDataType_U8, &val)) {
                            std::get<BEType<uint8_t>>(value.value) = val;
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEType<uint16_t>>) {
                        uint16_t val = std::get<BEType<uint16_t>>(value.value).getLE();
                        if (ImGui::DragScalar(value.value_name.c_str(), ImGuiDataType_U16, &val)) {
                            std::get<BEType<uint16_t>>(value.value) = val;
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEVec3>) {
                        float xyz[3] = { std::get<BEVec3>(value.value).x.getLE(), std::get<BEVec3>(value.value).y.getLE(), std::get<BEVec3>(value.value).z.getLE() };
                        if (ImGui::DragFloat3(value.value_name.c_str(), xyz)) {
                            std::get<BEVec3>(value.value).x = xyz[0];
                            std::get<BEVec3>(value.value).y = xyz[1];
                            std::get<BEVec3>(value.value).z = xyz[2];
                        }
                    }
                    else if constexpr (std::is_same_v<T, BEMatrix34>) {
                        if (value.expanded) {
                            auto mtx = std::get<BEMatrix34>(value.value).getLEMatrix();
                            ImGui::Indent();
                            bool row0Changed = ImGui::DragFloat4("Row 0", &mtx[0].x, 10.0f, 0, 0, nullptr, ImGuiSliderFlags_NoRoundToFormat);
                            ImGui::Unindent();
                            ImGui::Indent();
                            bool row1Changed = ImGui::DragFloat4("Row 1", &mtx[1].x, 10.0f, 0, 0, nullptr, ImGuiSliderFlags_NoRoundToFormat);
                            ImGui::Unindent();
                            ImGui::Indent();
                            bool row2Changed = ImGui::DragFloat4("Row 2", &mtx[2].x, 10.0f, 0, 0, nullptr, ImGuiSliderFlags_NoRoundToFormat);
                            ImGui::Unindent();
                            if (row0Changed || row1Changed || row2Changed) {
                                std::get<BEMatrix34>(value.value).setLEMatrix(mtx);
                            }
                        }
                        else {
                            float xyz[3] = { std::get<BEMatrix34>(value.value).pos_x.getLE(), std::get<BEMatrix34>(value.value).pos_y.getLE(), std::get<BEMatrix34>(value.value).pos_z.getLE() };
                            if (ImGui::DragFloat3(value.value_name.c_str(), xyz)) {
                                std::get<BEMatrix34>(value.value).pos_x = xyz[0];
                                std::get<BEMatrix34>(value.value).pos_y = xyz[1];
                                std::get<BEMatrix34>(value.value).pos_z = xyz[2];
                            }
                        }

                        ImGui::SameLine();
                        if (ImGui::Button("...")) {
                            value.expanded = !value.expanded;
                        }
                    }
                    else if constexpr (std::is_same_v<T, MemoryRange>) {
                        if (ImGui::Button(fmt::format("{} '{}' memory at {:08X}", value.expanded ? "Close" : "Open", value.value_name, value.value_address).c_str())) {
                            value.expanded = !value.expanded;
                        }
                        if (value.expanded) {
                            auto& mem_edit = std::get<MemoryRange>(value.value).editor;
                            uint32_t data = std::get<MemoryRange>(value.value).start;
                            uint32_t size = std::get<MemoryRange>(value.value).end - std::get<MemoryRange>(value.value).start;
                            if (ImGui::BeginChild("MemoryEditorInline", ImVec2(0.0f, 300.0f), true)) {
                                mem_edit->DrawContents(Guest::HostPointer<ImU8>((uint32_t)data), size, 0x0);
                            }
                            ImGui::EndChild();
                        }
                    }
                    else if constexpr (std::is_same_v<T, std::string>) {
                        ImGui::TextUnformatted(fmt::format("{}: {}", value.value_name, std::get<std::string>(value.value)).c_str());
                    }
                },
                           value.value);

                ImGui::PopID();
            }

            ImGui::PopID();
        }
    }
}

void EntityDebugger::AddOrUpdateEntity(uint32_t actorId, std::string_view entityName, std::string_view valueName, uint32_t address, ValueVariant&& value) {
    auto entityIt = m_entities.find(actorId);
    if (entityIt == m_entities.end()) {
        entityIt = m_entities.try_emplace(actorId, Entity{ std::string(entityName), 0.0f, {}, {}, {}, {} }).first;
    }

    const auto& valueIt = std::ranges::find_if(entityIt->second.values, [&](EntityValue& val) {
        return val.value_name == valueName;
    });

    if (valueIt == entityIt->second.values.end()) {
        entityIt->second.values.emplace_back(std::string(valueName), false, address, std::move(value));
    }
    else if (!std::holds_alternative<MemoryRange>(value)) {
        valueIt->value = std::move(value);
    }
}

bool EntityDebugger::HasEntityValue(uint32_t actorId, std::string_view valueName) const {
    const auto entityIt = m_entities.find(actorId);
    if (entityIt == m_entities.end()) {
        return false;
    }

    return std::ranges::any_of(entityIt->second.values, [&](const EntityValue& val) {
        return val.value_name == valueName;
    });
}

void EntityDebugger::SetPosition(uint32_t actorId, const BEVec3& ws_playerPos, const BEVec3& ws_entityPos) {
    if (const auto it = m_entities.find(actorId); it != m_entities.end()) {
        it->second.priority = ws_playerPos.DistanceSq(ws_entityPos);
        it->second.position = ws_entityPos;
    }
}

void EntityDebugger::SetRotation(uint32_t actorId, const glm::fquat rotation) {
    if (const auto it = m_entities.find(actorId); it != m_entities.end()) {
        it->second.rotation = rotation;
    }
}

void EntityDebugger::SetAABB(uint32_t actorId, glm::fvec3 min, glm::fvec3 max) {
    if (const auto it = m_entities.find(actorId); it != m_entities.end()) {
        it->second.aabbMin = min;
        it->second.aabbMax = max;
    }
}

void EntityDebugger::RemoveEntity(uint32_t actorId) {
    m_entities.erase(actorId);
}

void EntityDebugger::RemoveEntityValue(uint32_t actorId, const std::string& valueName) {
    if (const auto it = m_entities.find(actorId); it != m_entities.end()) {
        it->second.values.erase(std::ranges::remove_if(it->second.values, [&](const EntityValue& val) {
                                    return val.value_name == valueName;
                                }).begin(),
                                it->second.values.end());
    }
}
// the debug UI reads the keyboard through Win32, so it doesn't work on other platforms yet
#if defined(_WIN32)

std::array<bool, 256> s_pressedKeyState = {};
std::array<bool, ImGuiKey_NamedKey_COUNT> s_pressedNamedKeyState = {};

#define IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN(key, isShiftPressed)    \
    {                                                                              \
        bool isKeyDown = GetAsyncKeyState(key) & 0x8000;                           \
        bool wasKeyDown = s_pressedKeyState[key];                                  \
        s_pressedKeyState[key] = isKeyDown;                                        \
        if (isKeyDown && !wasKeyDown) {                                            \
            ImGui::GetIO().AddInputCharacter(isShiftPressed ? key : tolower(key)); \
        }                                                                          \
    }

#define IS_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN(key, imGuiKey)                          \
    {                                                                                 \
        bool wasKeyDown = s_pressedNamedKeyState[imGuiKey - ImGuiKey_NamedKey_BEGIN]; \
        bool isKeyDown = GetAsyncKeyState(key) & 0x8000;                              \
        s_pressedNamedKeyState[imGuiKey - ImGuiKey_NamedKey_BEGIN] = isKeyDown;       \
        if (isKeyDown && !wasKeyDown) {                                               \
            ImGui::GetIO().AddKeyEvent(imGuiKey, true);                               \
        }                                                                             \
        else if (!isKeyDown && wasKeyDown) {                                          \
            ImGui::GetIO().AddKeyEvent(imGuiKey, false);                              \
        }                                                                             \
    }

void EntityDebugger::UpdateKeyboardControls() {
    ImGui::GetIO().KeyAlt = GetAsyncKeyState(VK_MENU) & 0x8000;
    ImGui::GetIO().KeyCtrl = GetAsyncKeyState(VK_CONTROL) & 0x8000;
    ImGui::GetIO().KeyShift = GetAsyncKeyState(VK_SHIFT) & 0x8000;
    ImGui::GetIO().KeySuper = GetAsyncKeyState(VK_LWIN) & 0x8000;

    bool isShift = GetAsyncKeyState(VK_SHIFT) & 0x8000;
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN(' ', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('0', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('1', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('2', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('3', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('4', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('5', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('6', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('7', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('8', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('9', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('A', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('B', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('C', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('D', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('E', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('F', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('G', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('H', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('I', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('J', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('K', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('L', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('M', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('N', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('O', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('P', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('R', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('Q', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('S', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('T', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('U', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('V', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('W', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('X', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('Y', isShift);
    IS_ALPHANUMERIC_KEY_DOWN_AND_WASNT_PREVIOUSLY_DOWN('Z', isShift);

    bool isKeyDown = GetAsyncKeyState(VK_OEM_MINUS) & 0x8000;
    bool wasKeyDown = s_pressedKeyState[VK_OEM_MINUS];
    s_pressedKeyState[VK_OEM_MINUS] = isKeyDown;
    if (isKeyDown && !wasKeyDown) {
        ImGui::GetIO().AddInputCharacter(isShift ? '_' : '-');
    }

    bool isBackspaceKeyDown = GetAsyncKeyState(VK_BACK) & 0x8000;
    bool wasBackspaceKeyDown = s_pressedKeyState[VK_BACK];
    s_pressedKeyState[VK_BACK] = isBackspaceKeyDown;
    if (isBackspaceKeyDown && !wasBackspaceKeyDown) {
        ImGui::GetIO().AddKeyEvent(ImGuiKey_Backspace, true);
    }
    else if (!isBackspaceKeyDown && wasBackspaceKeyDown) {
        ImGui::GetIO().AddKeyEvent(ImGuiKey_Backspace, false);
    }
}

#else

void EntityDebugger::UpdateKeyboardControls() {
}

#endif
