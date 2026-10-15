#pragma once

#include "utils/vulkan_utils.h"

class Swapchain {
public:
    enum class Kind {
        Color,
        Depth,
    };

    Swapchain(Kind kind, uint32_t width, uint32_t height, uint32_t sampleCount);
    ~Swapchain();

    void PrepareRendering();
    VkImageView StartRendering();
    void FinishRendering();

    XrSwapchain GetHandle() const { return m_swapchain; };
    VkImage GetImage() const { return m_images[m_swapchainImageIdx]; };
    VkImageView GetImageView() const { return m_imageViews[m_swapchainImageIdx]; };

    VkFormat GetFormat() const { return m_format; };
    // some runtimes only offer UNORM formats, then the present shader has to do sRGB itself
    bool NeedsShaderSrgbEncode() const { return m_kind == Kind::Color && !VulkanUtils::IsSrgbFormat(m_format); };
    [[nodiscard]] uint32_t GetWidth() const { return m_width; };
    [[nodiscard]] uint32_t GetHeight() const { return m_height; };
    [[nodiscard]] VkExtent2D GetExtent() const { return { m_width, m_height }; };

private:
    XrSwapchain m_swapchain = XR_NULL_HANDLE;
    Kind m_kind;
    uint32_t m_width;
    uint32_t m_height;
    VkFormat m_format = VK_FORMAT_UNDEFINED;

    std::vector<VkImage> m_images;
    std::vector<VkImageView> m_imageViews;
    uint32_t m_swapchainImageIdx = 0;
};
