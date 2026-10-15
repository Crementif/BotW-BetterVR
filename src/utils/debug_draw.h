#pragma once

#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cstdint>
#include <mutex>
#include <span>
#include <vector>

constexpr uint32_t DebugDrawColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16) | ((uint32_t)a << 24);
}

struct DebugDrawVertex {
    glm::vec3 position = glm::vec3(0.0f);
    uint32_t color = 0;
};

using DebugDrawLineVertex = DebugDrawVertex;
using DebugDrawTriangleVertex = DebugDrawVertex;

struct DebugDrawRenderData {
    std::vector<DebugDrawTriangleVertex> triangleVertices;
    std::vector<DebugDrawLineVertex> lineVertices;
    std::vector<DebugDrawTriangleVertex> xrayTriangleVertices;
    std::vector<DebugDrawLineVertex> xrayLineVertices;
    std::array<glm::mat4, 2> viewProjections = { glm::mat4(1.0f), glm::mat4(1.0f) };
    std::array<bool, 2> hasViewProjections = { false, false };
    glm::vec3 cameraPos = glm::vec3(0.0f);

    bool IsEmpty() const { return triangleVertices.empty() && lineVertices.empty() && xrayTriangleVertices.empty() && xrayLineVertices.empty(); }
};

class DebugDraw {
public:
    static DebugDraw& instance() {
        static DebugDraw s_instance;
        return s_instance;
    }

    // -- Submit primitives (thread-safe, callable from any thread) --
    void Line(const glm::vec3& a, const glm::vec3& b, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, bool xray = false);
    void Dot(const glm::vec3& position, float radius = 0.15f, uint32_t color = DebugDrawColor(0, 255, 0), bool xray = false);
    void Sphere(const glm::vec3& position, float radius = 0.15f, uint32_t color = DebugDrawColor(0, 255, 0), int segments = 0, bool xray = false);
    // Y-up wire capsule. halfHeight is the half-height of the full capsule, including the rounded caps.
    void Capsule(const glm::vec3& center, float radius, float halfHeight, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, int segments = 0, bool xray = false);
    void PhysicsBody(const glm::vec3& center, float radius, float halfHeight, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, int segments = 0, bool xray = false);
    // World-space ring on the XZ plane using the game's Y-up convention.
    void Circle(const glm::vec3& position, float radius = 1.0f, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, int segments = 0, bool xray = false);
    void Polyline(std::span<const glm::vec3> points, uint32_t color = DebugDrawColor(255, 220, 64, 220), float thickness = 1.0f, bool xray = false);
    void Polyline(std::span<const glm::vec3> points, std::span<const uint32_t> colors, float thickness = 1.0f, bool xray = false);
    void Arc(const glm::vec3& start, const glm::vec3& initialVelocity, const glm::vec3& acceleration, float duration, uint32_t color = DebugDrawColor(255, 220, 64, 200), int segments = 0, bool xray = false);
    void Box(const glm::vec3& min, const glm::vec3& max, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, bool xray = false);
    void Box(const glm::vec3& center, const glm::vec3& halfExtents, const glm::quat& rotation, uint32_t color = DebugDrawColor(0, 255, 0), float thickness = 1.0f, bool xray = false);
    void Frustum(const glm::mat4& viewProjection, uint32_t color = DebugDrawColor(255, 255, 0), float thickness = 1.0f, bool xray = false);

    void UpdateEyeView(uint32_t eyeIndex, const glm::mat4& view);
    void UpdateEyeProjection(uint32_t eyeIndex, const glm::mat4& projection);
    void SnapshotEyeState(uint32_t eyeIndex, long frameIdx);

    DebugDrawRenderData TakeRenderData(long frameIdx = -1);

    void Clear();

private:
    DebugDraw() = default;

    enum class PrimitiveType : uint8_t {
        LINE,
        DOT,
        SPHERE,
        CAPSULE,
        PHYSICS_BODY,
        CIRCLE,
        POLYLINE,
        ARC,
        AABB,
        ORIENTED_BOX,
        FRUSTUM,
    };

    struct DebugPrimitive {
        PrimitiveType type;
        uint32_t color;
        float thickness;
        bool xray = false;

        glm::vec3 a = {};
        glm::vec3 b = {};
        glm::vec3 c = {};
        glm::quat rotation = glm::identity<glm::quat>();
        glm::mat4 inverseVP = glm::mat4(1.0f);
        std::vector<glm::vec3> points;
        std::vector<uint32_t> pointColors;
        float radius = 1.0f;
        float duration = 0.0f;
        int segments = 0;
    };

    std::mutex m_mutex;
    std::vector<DebugPrimitive> m_primitives;

    // Internal helpers
    static void AddLine(DebugDrawRenderData& renderData, const glm::vec3& a, const glm::vec3& b, uint32_t color, bool xray = false);
    static void AddEdges(DebugDrawRenderData& renderData, const glm::vec3* corners, const int (*edges)[2], int edgeCount, uint32_t color, bool xray = false);
};
