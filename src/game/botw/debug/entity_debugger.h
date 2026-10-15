#pragma once

struct MemoryRange {
    uint32_t start;
    uint32_t end;
    std::unique_ptr<MemoryEditor> editor;
};

using ValueVariant = std::variant<BEType<uint32_t>, BEType<int32_t>, BEType<uint16_t>, BEType<uint8_t>, BEType<float>, BEVec3, BEMatrix34, MemoryRange, std::string>;


class EntityDebugger {
public:
    static void TrackActor(uint32_t listIndex, uint32_t actorPtr, const char* actorName);

    void AddOrUpdateEntity(uint32_t actorId, std::string_view entityName, std::string_view valueName, uint32_t address, ValueVariant&& value);
    void SetPosition(uint32_t actorId, const BEVec3& ws_playerPos, const BEVec3& ws_entityPos);
    void SetRotation(uint32_t actorId, const glm::fquat rotation);
    void SetAABB(uint32_t actorId, glm::fvec3 min, glm::fvec3 max);
    void RemoveEntity(uint32_t actorId);
    void RemoveEntityValue(uint32_t actorId, const std::string& valueName);
    void UpdateEntityMemory();

    void UpdateKeyboardControls();
    void DrawWorldSpaceOverlaySettings(bool* changed);
    void DrawEntityInspectorContent();

    struct EntityValue {
        std::string value_name;
        bool expanded = false;
        uint32_t value_address;
        ValueVariant value;
    };

    struct Entity {
        std::string name;
        float priority;
        BEVec3 position;
        glm::fquat rotation;
        glm::fvec3 aabbMin;
        glm::fvec3 aabbMax;
        std::vector<EntityValue> values;
    };

    std::unordered_map<uint32_t, Entity> m_entities;
    glm::fvec3 m_playerPos = {};

private:
    void UpdateActorEntities(bool buildInspectorModel, bool drawWorldBoxes);
    bool HasEntityValue(uint32_t actorId, std::string_view valueName) const;

    std::atomic_bool m_logAnimationSlots = false;
    std::string m_filter;
    bool m_showWorldLabels = false;
    float m_worldAABBMinDistance = 1.0f;
    float m_worldAABBMaxDistance = 100.0f;
};
