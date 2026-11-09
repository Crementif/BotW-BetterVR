#pragma once

#include "vk_backend.h"
#include "game/game.h"

class WorldGuide {
public:
    struct Billboard {
        glm::fvec3 center;
        glm::fvec3 right;
        glm::fvec3 up;
    };

    explicit WorldGuide(RND_VkBackend* backend);
    ~WorldGuide();

    void InitTextures(VkCommandBuffer cmdBuffer);
    bool Prepare(const std::array<XrView, 2>& views);
    void Render(VkCommandBuffer commandBuffer, const RND_VkBackend::RenderTarget& target, const XrView& view);

private:
    struct Glyph {
        glm::fvec2 uvMin = {};
        glm::fvec2 uvMax = {};
        glm::fvec2 offset = {};
        glm::fvec2 size = {};
        float advance = 0.0f;
        bool valid = false;
    };

    struct Vertex {
        glm::fvec3 position;
        glm::fvec2 uv;
        uint32_t color;
        glm::fvec4 shape;
        float kind;
    };

    void BakeGlyphs();
    void LayoutCards();
    void CreatePipeline();
    const Glyph* FindGlyph(uint32_t codepoint) const;
    float MeasureText(const char* text, float scale) const;

    void AddQuad(const Billboard& frame, glm::fvec2 min, glm::fvec2 max, glm::fvec2 uvMin, glm::fvec2 uvMax, uint32_t color, const glm::fvec4& shape, float kind);
    void AddBox(const Billboard& frame, glm::fvec2 center, glm::fvec2 halfSize, float radius, float outline, uint32_t color);
    void AddText(const Billboard& frame, glm::fvec2 baseline, const char* text, float scale, uint32_t color, float weight);
    void AddIcon(const Billboard& frame, glm::fvec2 center, const char* icon, float scale, uint32_t color);
    void AddRibbon(std::span<const glm::fvec3> points, std::span<const float> halfWidths, std::span<const uint32_t> colors, const glm::fvec3& headPos);
    void AddZone(const glm::fvec3& zone, float radius, const glm::fvec3& headPos, float fade, float pulse);

    RND_VkBackend* m_backend = nullptr;

    std::unique_ptr<VulkanTexture> m_glyphTexture;
    std::vector<uint8_t> m_glyphPixels;
    std::array<Glyph, 96> m_textGlyphs = {};
    std::array<Glyph, 64> m_iconGlyphs = {};
    float m_capHeight = 0.0f;

    VkSampler m_sampler = VK_NULL_HANDLE;
    VkShaderModule m_vertexShader = VK_NULL_HANDLE;
    VkShaderModule m_fragmentShader = VK_NULL_HANDLE;
    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSet = VK_NULL_HANDLE;
    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkRenderPass m_renderPass = VK_NULL_HANDLE;
    VkPipeline m_pipeline = VK_NULL_HANDLE;

    std::array<RND_VkBackend::HostBuffer, RND_VkBackend::kFrameContextCount> m_vertexBuffers = {};
    std::vector<Vertex> m_vertices;
    uint32_t m_vertexCount = 0;

    Game::Guide m_guide = {};
    std::array<glm::fvec2, Game::MaxGuideCards> m_halfSizes = {};
    std::array<float, Game::MaxGuideCards> m_inputColumns = {};
    uint64_t m_generation = 0;
    bool m_visible = false;
    std::chrono::steady_clock::time_point m_fadeStart = {};
};
