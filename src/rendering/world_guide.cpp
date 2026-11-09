#include "pch.h"

#include "world_guide.h"
#include "shaders_spirv.h"
#include "font_roboto.h"
#include "font_material_symbols.h"

#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "imstb_truetype.h"

static constexpr float GlyphPixels = 56.0f;
static constexpr int GlyphPadding = 7;
static constexpr uint32_t AtlasWidth = 1024;
static constexpr uint32_t AtlasHeight = 1024;

static constexpr float TitleHeight = 0.024f;
static constexpr float BodyHeight = 0.0175f;
static constexpr float IconHeight = 0.030f;
static constexpr float CardPadding = 0.016f;
static constexpr float TitleRowHeight = 0.034f;
static constexpr float TitleGap = 0.012f;
static constexpr float RowHeight = 0.027f;
static constexpr float ColumnGap = 0.018f;
static constexpr float CardRadius = 0.012f;

static constexpr float KindGlyph = 0.0f;
static constexpr float KindBox = 1.0f;
static constexpr float KindRibbon = 2.0f;

static uint32_t Rgba(uint8_t r, uint8_t g, uint8_t b, float alpha) {
    const uint32_t a = (uint32_t)std::lround(glm::clamp(alpha, 0.0f, 1.0f) * 255.0f);
    return (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16) | (a << 24);
}

static uint32_t NextCodepoint(const char*& text) {
    const uint8_t lead = (uint8_t)*text++;
    if (lead < 0x80)
        return lead;
    int extra = lead >= 0xF0 ? 3 : lead >= 0xE0 ? 2 : 1;
    uint32_t codepoint = lead & (0x3F >> extra);
    while (extra-- > 0 && (*text & 0xC0) == 0x80)
        codepoint = (codepoint << 6) | ((uint8_t)*text++ & 0x3F);
    return codepoint;
}

static glm::fmat4 ProjectionOf(const XrFovf& fov) {
    const float left = std::tan(fov.angleLeft);
    const float right = std::tan(fov.angleRight);
    const float up = std::tan(fov.angleUp);
    const float down = std::tan(fov.angleDown);
    constexpr float nearZ = 0.05f;
    constexpr float farZ = 100.0f;

    glm::fmat4 projection(0.0f);
    projection[0][0] = 2.0f / (right - left);
    projection[2][0] = (right + left) / (right - left);
    projection[1][1] = 2.0f / (down - up);
    projection[2][1] = (up + down) / (down - up);
    projection[2][2] = -farZ / (farZ - nearZ);
    projection[3][2] = -(farZ * nearZ) / (farZ - nearZ);
    projection[2][3] = -1.0f;
    return projection;
}

static glm::fvec3 HeadPositionOf(const std::array<XrView, 2>& views) {
    return (ToGLM(views[0].pose.position) + ToGLM(views[1].pose.position)) * 0.5f;
}

WorldGuide::WorldGuide(RND_VkBackend* backend): m_backend(backend) {
    checkAssert(m_backend != nullptr, "The world guide is missing its renderer backend!");
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    BakeGlyphs();
    m_glyphTexture = std::make_unique<VulkanTexture>(AtlasWidth, AtlasHeight, VK_FORMAT_R8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT);

    VkSamplerCreateInfo samplerInfo = { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.maxLod = 0.0f;
    checkVkResult(dispatch->CreateSampler(device, &samplerInfo, nullptr, &m_sampler), "Failed to create the world guide sampler!");

    m_vertexShader = m_backend->CreateShaderModule(world_guide_vert_spv, sizeof(world_guide_vert_spv));
    m_fragmentShader = m_backend->CreateShaderModule(world_guide_frag_spv, sizeof(world_guide_frag_spv));

    const VkDescriptorSetLayoutBinding glyphBinding = {
        .binding = 0,
        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
        .descriptorCount = 1,
        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
        .pImmutableSamplers = &m_sampler
    };
    VkDescriptorSetLayoutCreateInfo setLayoutInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    setLayoutInfo.bindingCount = 1;
    setLayoutInfo.pBindings = &glyphBinding;
    checkVkResult(dispatch->CreateDescriptorSetLayout(device, &setLayoutInfo, nullptr, &m_descriptorSetLayout), "Failed to create the world guide descriptor set layout!");

    const VkPushConstantRange pushConstantRange = {
        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        .offset = 0,
        .size = sizeof(worldGuideSettings)
    };
    VkPipelineLayoutCreateInfo pipelineLayoutInfo = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
    checkVkResult(dispatch->CreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout), "Failed to create the world guide pipeline layout!");

    VkDescriptorSetAllocateInfo setAllocateInfo = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    setAllocateInfo.descriptorPool = m_backend->GetDescriptorPool();
    setAllocateInfo.descriptorSetCount = 1;
    setAllocateInfo.pSetLayouts = &m_descriptorSetLayout;
    checkVkResult(dispatch->AllocateDescriptorSets(device, &setAllocateInfo, &m_descriptorSet), "Failed to allocate the world guide descriptor set!");

    const VkDescriptorImageInfo glyphInfo = {
        .sampler = VK_NULL_HANDLE,
        .imageView = m_glyphTexture->GetImageView(),
        .imageLayout = VK_IMAGE_LAYOUT_GENERAL
    };
    VkWriteDescriptorSet write = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    write.dstSet = m_descriptorSet;
    write.dstBinding = 0;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.pImageInfo = &glyphInfo;
    dispatch->UpdateDescriptorSets(device, 1, &write, 0, nullptr);

    m_vertices.reserve(16384);
    for (RND_VkBackend::HostBuffer& vertexBuffer : m_vertexBuffers)
        vertexBuffer = m_backend->CreateHostBuffer((uint32_t)(m_vertices.capacity() * sizeof(Vertex)), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
}

WorldGuide::~WorldGuide() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    m_backend->WaitForSubmittedWork();
    for (RND_VkBackend::HostBuffer& vertexBuffer : m_vertexBuffers)
        m_backend->DestroyHostBuffer(vertexBuffer);
    m_glyphTexture.reset();

    if (m_pipeline != VK_NULL_HANDLE)
        dispatch->DestroyPipeline(device, m_pipeline, nullptr);
    if (m_pipelineLayout != VK_NULL_HANDLE)
        dispatch->DestroyPipelineLayout(device, m_pipelineLayout, nullptr);
    if (m_descriptorSetLayout != VK_NULL_HANDLE)
        dispatch->DestroyDescriptorSetLayout(device, m_descriptorSetLayout, nullptr);
    if (m_vertexShader != VK_NULL_HANDLE)
        dispatch->DestroyShaderModule(device, m_vertexShader, nullptr);
    if (m_fragmentShader != VK_NULL_HANDLE)
        dispatch->DestroyShaderModule(device, m_fragmentShader, nullptr);
    if (m_sampler != VK_NULL_HANDLE)
        dispatch->DestroySampler(device, m_sampler, nullptr);
}

void WorldGuide::BakeGlyphs() {
    const auto bakeStart = std::chrono::steady_clock::now();
    ImFontAtlas fontAtlas;
    fontAtlas.AddFontFromMemoryCompressedTTF(roboto_compressed_data, roboto_compressed_size, GlyphPixels);
    const unsigned char* robotoData = (const unsigned char*)fontAtlas.ConfigData.back().FontData;

    stbtt_fontinfo textFont = {};
    stbtt_fontinfo iconFont = {};
    checkAssert(stbtt_InitFont(&textFont, robotoData, stbtt_GetFontOffsetForIndex(robotoData, 0)) != 0, "Failed to read the world guide's text font!");
    checkAssert(stbtt_InitFont(&iconFont, material_symbols_data, stbtt_GetFontOffsetForIndex(material_symbols_data, 0)) != 0, "Failed to read the world guide's icon font!");
    const float textScale = stbtt_ScaleForPixelHeight(&textFont, GlyphPixels);
    const float iconScale = stbtt_ScaleForMappingEmToPixels(&iconFont, GlyphPixels);

    m_glyphPixels.assign((size_t)AtlasWidth * AtlasHeight, 0);
    int penX = 1;
    int penY = 1;
    int rowHeight = 0;

    auto bake = [&](const stbtt_fontinfo& font, float scale, uint32_t codepoint, Glyph& glyph) {
        int advance = 0;
        int leftBearing = 0;
        stbtt_GetCodepointHMetrics(&font, (int)codepoint, &advance, &leftBearing);
        glyph.advance = (float)advance * scale;
        glyph.valid = true;

        int width = 0;
        int height = 0;
        int offsetX = 0;
        int offsetY = 0;
        unsigned char* field = stbtt_GetCodepointSDF(&font, scale, (int)codepoint, GlyphPadding, 128, 128.0f / (float)GlyphPadding, &width, &height, &offsetX, &offsetY);
        if (field == nullptr)
            return;

        if (penX + width + 1 > (int)AtlasWidth) {
            penX = 1;
            penY += rowHeight + 1;
            rowHeight = 0;
        }
        if (penY + height + 1 > (int)AtlasHeight) {
            Log::print<WARNING>("The world guide's glyph atlas is full, U+{:04X} is left out", codepoint);
            stbtt_FreeSDF(field, nullptr);
            return;
        }

        for (int row = 0; row < height; row++)
            memcpy(&m_glyphPixels[(size_t)(penY + row) * AtlasWidth + penX], field + (size_t)row * width, (size_t)width);
        stbtt_FreeSDF(field, nullptr);

        glyph.uvMin = glm::fvec2((float)penX / AtlasWidth, (float)penY / AtlasHeight);
        glyph.uvMax = glm::fvec2((float)(penX + width) / AtlasWidth, (float)(penY + height) / AtlasHeight);
        glyph.offset = glm::fvec2((float)offsetX, (float)offsetY);
        glyph.size = glm::fvec2((float)width, (float)height);
        penX += width + 1;
        rowHeight = std::max(rowHeight, height);
    };

    for (uint32_t codepoint = 32; codepoint < 128; codepoint++)
        bake(textFont, textScale, codepoint, m_textGlyphs[codepoint - 32]);
    for (uint32_t codepoint = ICON_MIN_MS; codepoint <= ICON_MAX_MS && codepoint - ICON_MIN_MS < m_iconGlyphs.size(); codepoint++) {
        if (stbtt_FindGlyphIndex(&iconFont, (int)codepoint) != 0)
            bake(iconFont, iconScale, codepoint, m_iconGlyphs[codepoint - ICON_MIN_MS]);
    }

    int x0 = 0;
    int y0 = 0;
    int x1 = 0;
    int y1 = 0;
    stbtt_GetCodepointBox(&textFont, 'H', &x0, &y0, &x1, &y1);
    m_capHeight = (float)y1 * textScale;
    Log::print<INFO>("Baked the world guide's glyphs in {:.1f} ms", std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - bakeStart).count());
}

void WorldGuide::InitTextures(VkCommandBuffer cmdBuffer) {
    m_glyphTexture->vkTransitionLayout(cmdBuffer, VK_IMAGE_LAYOUT_GENERAL);
    m_glyphTexture->vkUpload(cmdBuffer, m_glyphPixels.data(), m_glyphPixels.size());
    m_glyphPixels = {};
}

void WorldGuide::CreatePipeline() {
    const auto* dispatch = m_backend->GetDispatch();
    VkDevice device = m_backend->GetDevice();

    if (m_pipeline != VK_NULL_HANDLE) {
        m_backend->WaitForSubmittedWork();
        dispatch->DestroyPipeline(device, m_pipeline, nullptr);
        m_pipeline = VK_NULL_HANDLE;
    }

    const std::array<VkPipelineShaderStageCreateInfo, 2> stages = {
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_VERTEX_BIT, .module = m_vertexShader, .pName = "main" },
        VkPipelineShaderStageCreateInfo{ .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, .stage = VK_SHADER_STAGE_FRAGMENT_BIT, .module = m_fragmentShader, .pName = "main" }
    };

    const VkVertexInputBindingDescription vertexBinding = {
        .binding = 0,
        .stride = sizeof(Vertex),
        .inputRate = VK_VERTEX_INPUT_RATE_VERTEX
    };
    const std::array<VkVertexInputAttributeDescription, 5> vertexAttributes = {
        VkVertexInputAttributeDescription{ .location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(Vertex, position) },
        VkVertexInputAttributeDescription{ .location = 1, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT, .offset = offsetof(Vertex, uv) },
        VkVertexInputAttributeDescription{ .location = 2, .binding = 0, .format = VK_FORMAT_R8G8B8A8_UNORM, .offset = offsetof(Vertex, color) },
        VkVertexInputAttributeDescription{ .location = 3, .binding = 0, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = offsetof(Vertex, shape) },
        VkVertexInputAttributeDescription{ .location = 4, .binding = 0, .format = VK_FORMAT_R32_SFLOAT, .offset = offsetof(Vertex, kind) }
    };
    const VkPipelineVertexInputStateCreateInfo vertexInputState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
        .vertexBindingDescriptionCount = 1,
        .pVertexBindingDescriptions = &vertexBinding,
        .vertexAttributeDescriptionCount = (uint32_t)vertexAttributes.size(),
        .pVertexAttributeDescriptions = vertexAttributes.data()
    };

    const VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
        .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
    };
    const VkPipelineViewportStateCreateInfo viewportState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
        .viewportCount = 1,
        .scissorCount = 1
    };
    const VkPipelineRasterizationStateCreateInfo rasterizationState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
        .polygonMode = VK_POLYGON_MODE_FILL,
        .cullMode = VK_CULL_MODE_NONE,
        .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
        .lineWidth = 1.0f
    };
    const VkPipelineMultisampleStateCreateInfo multisampleState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
        .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
    };
    const VkPipelineDepthStencilStateCreateInfo depthStencilState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
        .depthTestEnable = VK_FALSE,
        .depthWriteEnable = VK_FALSE,
        .depthCompareOp = VK_COMPARE_OP_ALWAYS
    };
    const VkPipelineColorBlendAttachmentState blendAttachment = {
        .blendEnable = VK_TRUE,
        .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
        .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .colorBlendOp = VK_BLEND_OP_ADD,
        .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
        .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
        .alphaBlendOp = VK_BLEND_OP_ADD,
        .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
    };
    const VkPipelineColorBlendStateCreateInfo colorBlendState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
        .attachmentCount = 1,
        .pAttachments = &blendAttachment
    };
    const std::array<VkDynamicState, 2> dynamicStates = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    const VkPipelineDynamicStateCreateInfo dynamicState = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
        .dynamicStateCount = (uint32_t)dynamicStates.size(),
        .pDynamicStates = dynamicStates.data()
    };

    VkGraphicsPipelineCreateInfo createInfo = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    createInfo.stageCount = (uint32_t)stages.size();
    createInfo.pStages = stages.data();
    createInfo.pVertexInputState = &vertexInputState;
    createInfo.pInputAssemblyState = &inputAssemblyState;
    createInfo.pViewportState = &viewportState;
    createInfo.pRasterizationState = &rasterizationState;
    createInfo.pMultisampleState = &multisampleState;
    createInfo.pDepthStencilState = &depthStencilState;
    createInfo.pColorBlendState = &colorBlendState;
    createInfo.pDynamicState = &dynamicState;
    createInfo.layout = m_pipelineLayout;
    createInfo.renderPass = m_renderPass;
    createInfo.subpass = 0;
    checkVkResult(dispatch->CreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &createInfo, nullptr, &m_pipeline), "Failed to create the world guide pipeline!");
}

const WorldGuide::Glyph* WorldGuide::FindGlyph(uint32_t codepoint) const {
    if (codepoint >= 32 && codepoint < 128)
        return m_textGlyphs[codepoint - 32].valid ? &m_textGlyphs[codepoint - 32] : nullptr;
    if (codepoint >= ICON_MIN_MS && codepoint - ICON_MIN_MS < m_iconGlyphs.size())
        return m_iconGlyphs[codepoint - ICON_MIN_MS].valid ? &m_iconGlyphs[codepoint - ICON_MIN_MS] : nullptr;
    return nullptr;
}

float WorldGuide::MeasureText(const char* text, float scale) const {
    float width = 0.0f;
    while (text != nullptr && *text != '\0') {
        if (const Glyph* glyph = FindGlyph(NextCodepoint(text)))
            width += glyph->advance * scale;
    }
    return width;
}

void WorldGuide::AddQuad(const Billboard& frame, glm::fvec2 min, glm::fvec2 max, glm::fvec2 uvMin, glm::fvec2 uvMax, uint32_t color, const glm::fvec4& shape, float kind) {
    auto corner = [&](float x, float y, float u, float v) {
        return Vertex{ frame.center + frame.right * x + frame.up * y, glm::fvec2(u, v), color, shape, kind };
    };
    const Vertex topLeft = corner(min.x, max.y, uvMin.x, uvMin.y);
    const Vertex topRight = corner(max.x, max.y, uvMax.x, uvMin.y);
    const Vertex bottomLeft = corner(min.x, min.y, uvMin.x, uvMax.y);
    const Vertex bottomRight = corner(max.x, min.y, uvMax.x, uvMax.y);
    m_vertices.push_back(topLeft);
    m_vertices.push_back(bottomLeft);
    m_vertices.push_back(topRight);
    m_vertices.push_back(topRight);
    m_vertices.push_back(bottomLeft);
    m_vertices.push_back(bottomRight);
}

void WorldGuide::AddBox(const Billboard& frame, glm::fvec2 center, glm::fvec2 halfSize, float radius, float outline, uint32_t color) {
    const glm::fvec2 extent = halfSize + glm::fvec2(0.003f);
    AddQuad(frame, center - extent, center + extent, glm::fvec2(-extent.x, extent.y), glm::fvec2(extent.x, -extent.y), color, glm::fvec4(halfSize, radius, outline), KindBox);
}

void WorldGuide::AddText(const Billboard& frame, glm::fvec2 baseline, const char* text, float scale, uint32_t color, float weight) {
    float penX = baseline.x;
    while (text != nullptr && *text != '\0') {
        const Glyph* glyph = FindGlyph(NextCodepoint(text));
        if (glyph == nullptr)
            continue;
        if (glyph->size.x > 0.0f) {
            const glm::fvec2 min(penX + glyph->offset.x * scale, baseline.y - (glyph->offset.y + glyph->size.y) * scale);
            const glm::fvec2 max(penX + (glyph->offset.x + glyph->size.x) * scale, baseline.y - glyph->offset.y * scale);
            AddQuad(frame, min, max, glyph->uvMin, glyph->uvMax, color, glm::fvec4(weight, 0.0f, 0.0f, 0.0f), KindGlyph);
        }
        penX += glyph->advance * scale;
    }
}

void WorldGuide::AddIcon(const Billboard& frame, glm::fvec2 center, const char* icon, float scale, uint32_t color) {
    const Glyph* glyph = icon != nullptr ? FindGlyph(NextCodepoint(icon)) : nullptr;
    if (glyph == nullptr || glyph->size.x <= 0.0f)
        return;
    const glm::fvec2 halfSize = glyph->size * scale * 0.5f;
    AddQuad(frame, center - halfSize, center + halfSize, glyph->uvMin, glyph->uvMax, color, glm::fvec4(0.0f), KindGlyph);
}

void WorldGuide::AddRibbon(std::span<const glm::fvec3> points, std::span<const float> halfWidths, std::span<const uint32_t> colors, const glm::fvec3& headPos) {
    constexpr float EdgeMargin = 0.0015f;
    auto edges = [&](size_t i, Vertex& left, Vertex& right) {
        const glm::fvec3 previous = points[i == 0 ? 0 : i - 1];
        const glm::fvec3 next = points[std::min(i + 1, points.size() - 1)];
        glm::fvec3 side = glm::cross(next - previous, headPos - points[i]);
        side = glm::length2(side) < 1e-10f ? glm::fvec3(1.0f, 0.0f, 0.0f) : glm::normalize(side);
        const float halfWidth = halfWidths[i];
        const float across = 1.0f + EdgeMargin / std::max(halfWidth, 0.0005f);
        left = Vertex{ points[i] + side * (halfWidth + EdgeMargin), glm::fvec2(-across, 0.0f), colors[i], glm::fvec4(0.0f), KindRibbon };
        right = Vertex{ points[i] - side * (halfWidth + EdgeMargin), glm::fvec2(across, 0.0f), colors[i], glm::fvec4(0.0f), KindRibbon };
    };

    for (size_t i = 0; i + 1 < points.size(); i++) {
        Vertex startLeft, startRight, endLeft, endRight;
        edges(i, startLeft, startRight);
        edges(i + 1, endLeft, endRight);
        m_vertices.push_back(startLeft);
        m_vertices.push_back(startRight);
        m_vertices.push_back(endLeft);
        m_vertices.push_back(endLeft);
        m_vertices.push_back(startRight);
        m_vertices.push_back(endRight);
    }
}

static WorldGuide::Billboard FacingHead(const glm::fvec3& center, const glm::fvec3& headPos);

void WorldGuide::AddZone(const glm::fvec3& zone, float radius, const glm::fvec3& headPos, float fade, float pulse) {
    const Billboard frame = FacingHead(zone, headPos);

    const glm::fvec2 origin(0.0f);
    AddBox(frame, origin, glm::fvec2(radius), radius, 0.0f, Rgba(110, 215, 255, 0.16f * fade));
    const float pulseRadius = radius * (1.0f + 0.4f * pulse);
    AddBox(frame, origin, glm::fvec2(pulseRadius), pulseRadius, 0.0025f, Rgba(110, 215, 255, (1.0f - pulse) * 0.55f * fade));
    AddBox(frame, origin, glm::fvec2(radius), radius, 0.0035f, Rgba(110, 215, 255, 0.9f * fade));
    AddBox(frame, origin, glm::fvec2(radius * 0.16f), radius * 0.16f, 0.0f, Rgba(235, 248, 255, 0.95f * fade));
}

static glm::fvec2 CardHalfSize(float titleWidth, float inputWidth, float actionWidth, uint32_t rowCount) {
    const float titleRow = IconHeight + 0.010f + titleWidth;
    const float rows = inputWidth + ColumnGap + actionWidth;
    const float width = std::max(titleRow, rows) + CardPadding * 2.0f;
    const float height = CardPadding * 2.0f + TitleRowHeight + (rowCount > 0 ? TitleGap + RowHeight * (float)rowCount : 0.0f);
    return glm::fvec2(width, height) * 0.5f;
}

void WorldGuide::LayoutCards() {
    const float titleScale = TitleHeight / GlyphPixels;
    const float bodyScale = BodyHeight / GlyphPixels;
    for (size_t i = 0; i < m_guide.count; i++) {
        const Game::GuideCard& card = m_guide.cards[i];
        float inputWidth = 0.0f;
        float actionWidth = 0.0f;
        uint32_t rowCount = 0;
        for (const Game::GuideRow& row : card.rows) {
            if (row.input == nullptr)
                continue;
            inputWidth = std::max(inputWidth, MeasureText(row.input, bodyScale));
            actionWidth = std::max(actionWidth, MeasureText(row.action, bodyScale));
            rowCount++;
        }
        m_inputColumns[i] = inputWidth;
        m_halfSizes[i] = CardHalfSize(MeasureText(card.title, titleScale), inputWidth, actionWidth, rowCount);
    }
}

bool WorldGuide::Prepare(const std::array<XrView, 2>& views) {
    m_vertexCount = 0;
    const Game::Guide guide = Game::GetGuide();
    const auto now = std::chrono::steady_clock::now();
    if (guide.generation != m_generation) {
        m_generation = guide.generation;
        if (guide.count > 0) {
            m_guide = guide;
            m_visible = true;
            m_fadeStart = now;
            LayoutCards();
        }
        else if (m_visible) {
            m_visible = false;
            m_fadeStart = now;
        }
    }

    const float elapsed = std::chrono::duration<float>(now - m_fadeStart).count();
    const float fade = glm::smoothstep(0.0f, 1.0f, m_visible ? elapsed / 0.25f : 1.0f - elapsed / 0.3f);
    if (fade <= 0.0f || m_guide.count == 0)
        return false;

    const float pulse = std::fmod(elapsed, 1.6f) / 1.6f;
    const glm::fvec3 headPos = HeadPositionOf(views);
    const float titleScale = TitleHeight / GlyphPixels;
    const float bodyScale = BodyHeight / GlyphPixels;
    const float iconScale = IconHeight / GlyphPixels;

    m_vertices.clear();
    const std::array<glm::fvec2, Game::MaxGuideCards>& halfSizes = m_halfSizes;
    const std::array<float, Game::MaxGuideCards>& inputColumns = m_inputColumns;
    const glm::fvec3 rise(0.0f, m_visible ? (fade - 1.0f) * 0.025f : 0.0f, 0.0f);

    for (size_t i = 0; i < m_guide.count; i++) {
        const Game::GuideCard& card = m_guide.cards[i];
        if (card.zoneRadius <= 0.0f)
            continue;
        AddZone(card.zone, card.zoneRadius, headPos, fade, pulse);

        const Billboard frame = FacingHead(card.position + rise, headPos);
        const glm::fvec3 toZone = card.zone - frame.center;
        const glm::fvec2 planar(glm::dot(toZone, frame.right), glm::dot(toZone, frame.up));
        const float reach = std::min(std::abs(planar.x) > 1e-5f ? halfSizes[i].x / std::abs(planar.x) : std::numeric_limits<float>::max(), std::abs(planar.y) > 1e-5f ? halfSizes[i].y / std::abs(planar.y) : std::numeric_limits<float>::max());
        if (reach >= 1.0f || reach == std::numeric_limits<float>::max())
            continue;

        const glm::fvec3 start = frame.center + (frame.right * planar.x + frame.up * planar.y) * reach;
        const glm::fvec3 end = card.zone - glm::normalize(card.zone - start) * card.zoneRadius * 1.2f;
        const glm::fvec3 middle = (start + end) * 0.5f;
        const glm::fvec3 bulge = glm::normalize(middle - headPos) * glm::length(end - start) * 0.22f;
        const glm::fvec3 control = middle + bulge;
        constexpr float HeadLength = 0.03f;
        const glm::fvec3 endDirection = glm::normalize(end - control);
        const glm::fvec3 shaftEnd = end - endDirection * HeadLength;

        constexpr size_t SegmentCount = 24;
        std::array<glm::fvec3, SegmentCount + 1> points = {};
        std::array<float, SegmentCount + 1> widths = {};
        std::array<uint32_t, SegmentCount + 1> colors = {};
        for (size_t s = 0; s <= SegmentCount; s++) {
            const float t = (float)s / (float)SegmentCount;
            points[s] = (1.0f - t) * (1.0f - t) * start + 2.0f * (1.0f - t) * t * control + t * t * shaftEnd;
            widths[s] = 0.0028f;
            colors[s] = Rgba(110, 215, 255, glm::mix(0.3f, 0.95f, t) * fade);
        }
        AddRibbon(points, widths, colors, headPos);

        const std::array<glm::fvec3, 2> headPoints = { shaftEnd, end };
        const std::array<float, 2> headWidths = { 0.011f, 0.0f };
        const std::array<uint32_t, 2> headColors = { Rgba(110, 215, 255, 0.95f * fade), Rgba(110, 215, 255, 0.95f * fade) };
        AddRibbon(headPoints, headWidths, headColors, headPos);
    }

    for (size_t i = 0; i < m_guide.count; i++) {
        const Game::GuideCard& card = m_guide.cards[i];
        const Billboard frame = FacingHead(card.position + rise, headPos);
        const glm::fvec2 half = halfSizes[i];

        AddBox(frame, glm::fvec2(0.0f), half, CardRadius, 0.0f, Rgba(14, 18, 26, 0.92f * fade));
        AddBox(frame, glm::fvec2(0.0f), half, CardRadius, 0.0016f, Rgba(110, 215, 255, 0.55f * fade));

        const float left = -half.x + CardPadding;
        const float titleCenterY = half.y - CardPadding - TitleRowHeight * 0.5f;
        AddIcon(frame, glm::fvec2(left + IconHeight * 0.5f, titleCenterY), card.icon, iconScale, Rgba(110, 215, 255, fade));
        AddText(frame, glm::fvec2(left + IconHeight + 0.010f, titleCenterY - m_capHeight * titleScale * 0.5f), card.title, titleScale, Rgba(255, 255, 255, fade), 0.06f);

        float rowCenterY = half.y - CardPadding - TitleRowHeight - TitleGap - RowHeight * 0.5f;
        bool hasRows = false;
        for (const Game::GuideRow& row : card.rows) {
            if (row.input == nullptr)
                continue;
            hasRows = true;
            const float baselineY = rowCenterY - m_capHeight * bodyScale * 0.5f;
            AddText(frame, glm::fvec2(left, baselineY), row.input, bodyScale, Rgba(110, 215, 255, fade), 0.03f);
            AddText(frame, glm::fvec2(left + inputColumns[i] + ColumnGap, baselineY), row.action, bodyScale, Rgba(226, 232, 240, fade), 0.0f);
            rowCenterY -= RowHeight;
        }
        if (hasRows) {
            const float dividerY = half.y - CardPadding - TitleRowHeight - TitleGap * 0.5f;
            AddBox(frame, glm::fvec2(0.0f, dividerY), glm::fvec2(half.x - CardPadding, 0.0006f), 0.0f, 0.0f, Rgba(255, 255, 255, 0.14f * fade));
        }
    }

    if (m_vertices.empty())
        return false;

    const uint32_t frameContextIndex = m_backend->GetFrameContextIndex();
    RND_VkBackend::HostBuffer& vertexBuffer = m_vertexBuffers[frameContextIndex];
    const uint32_t requiredBytes = (uint32_t)(m_vertices.size() * sizeof(Vertex));
    if (requiredBytes > vertexBuffer.capacity) {
        if (vertexBuffer.buffer != VK_NULL_HANDLE) {
            m_backend->WaitForSubmittedWork();
            m_backend->DestroyHostBuffer(vertexBuffer);
        }
        vertexBuffer = m_backend->CreateHostBuffer(std::max(requiredBytes, std::max(vertexBuffer.capacity * 2, 65536u)), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    }
    if (vertexBuffer.mapped == nullptr)
        return false;
    memcpy(vertexBuffer.mapped, m_vertices.data(), requiredBytes);
    m_vertexCount = (uint32_t)m_vertices.size();
    return true;
}

static WorldGuide::Billboard FacingHead(const glm::fvec3& center, const glm::fvec3& headPos) {
    const glm::fvec3 toHead = headPos - center;
    const glm::fvec3 normal = glm::length2(toHead) < 1e-6f ? glm::fvec3(0.0f, 0.0f, 1.0f) : glm::normalize(toHead);
    glm::fvec3 right = glm::cross(glm::fvec3(0.0f, 1.0f, 0.0f), normal);
    right = glm::length2(right) < 1e-6f ? glm::fvec3(1.0f, 0.0f, 0.0f) : glm::normalize(right);
    return { center, right, glm::cross(normal, right) };
}

void WorldGuide::Render(VkCommandBuffer commandBuffer, const RND_VkBackend::RenderTarget& target, const XrView& view) {
    if (m_vertexCount == 0)
        return;

    const auto* dispatch = m_backend->GetDispatch();
    if (VkRenderPass renderPass = m_backend->AcquireRenderPass(target.colorFormat, target.depthFormat, true); renderPass != m_renderPass || m_pipeline == VK_NULL_HANDLE) {
        m_renderPass = renderPass;
        CreatePipeline();
    }

    const RND_VkBackend::HostBuffer& vertexBuffer = m_vertexBuffers[m_backend->GetFrameContextIndex()];
    if (vertexBuffer.buffer == VK_NULL_HANDLE)
        return;

    const glm::fmat4 viewMatrix = glm::inverse(ToMat4(ToGLM(view.pose.position), ToGLM(view.pose.orientation)));
    const worldGuideSettings settings = {
        .viewProjection = ProjectionOf(view.fov) * viewMatrix,
        .srgbEncode = target.srgbEncodeInShader ? 1.0f : 0.0f,
    };

    VkRenderPassBeginInfo beginInfo = { VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    beginInfo.renderPass = m_renderPass;
    beginInfo.framebuffer = m_backend->AcquireFramebuffer(m_renderPass, target.colorView, target.depthView, target.extent);
    beginInfo.renderArea = { { 0, 0 }, target.extent };
    dispatch->CmdBeginRenderPass(commandBuffer, &beginInfo, VK_SUBPASS_CONTENTS_INLINE);

    const VkViewport viewport = { 0.0f, 0.0f, (float)target.extent.width, (float)target.extent.height, 0.0f, 1.0f };
    const VkRect2D scissor = { { 0, 0 }, target.extent };
    dispatch->CmdSetViewport(commandBuffer, 0, 1, &viewport);
    dispatch->CmdSetScissor(commandBuffer, 0, 1, &scissor);

    const VkDeviceSize offset = 0;
    dispatch->CmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline);
    dispatch->CmdBindVertexBuffers(commandBuffer, 0, 1, &vertexBuffer.buffer, &offset);
    dispatch->CmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);
    dispatch->CmdPushConstants(commandBuffer, m_pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(settings), &settings);
    dispatch->CmdDraw(commandBuffer, m_vertexCount, 1, 0, 0);
    dispatch->CmdEndRenderPass(commandBuffer);
}
