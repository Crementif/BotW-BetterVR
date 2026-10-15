#include "swapchain.h"
#include "instance.h"
#include "utils/vulkan_utils.h"

static constexpr VkFormat kPreferredColorFormats[] = {
    VK_FORMAT_R8G8B8A8_SRGB,
    VK_FORMAT_B8G8R8A8_SRGB,
    VK_FORMAT_R8G8B8A8_UNORM,
};

static constexpr VkFormat kPreferredDepthFormats[] = {
    VK_FORMAT_D32_SFLOAT,
    VK_FORMAT_D24_UNORM_S8_UINT,
};

Swapchain::Swapchain(Kind kind, uint32_t width, uint32_t height, uint32_t sampleCount): m_kind(kind), m_width(width), m_height(height) {
    const std::span<const VkFormat> preferredFormats = kind == Kind::Depth
        ? std::span<const VkFormat>(kPreferredDepthFormats)
        : std::span<const VkFormat>(kPreferredColorFormats);

    uint32_t swapchainFormatCount = 0;
    checkXRResult(xrEnumerateSwapchainFormats(VRManager::instance().XR->GetSession(), 0, &swapchainFormatCount, nullptr), "Failed to count the OpenXR swapchain formats!");
    std::vector<int64_t> runtimeFormats(swapchainFormatCount);
    checkXRResult(xrEnumerateSwapchainFormats(VRManager::instance().XR->GetSession(), swapchainFormatCount, &swapchainFormatCount, runtimeFormats.data()), "Failed to enumerate the OpenXR swapchain formats!");

    for (VkFormat preferredFormat : preferredFormats) {
        if (std::ranges::find(runtimeFormats, (int64_t)preferredFormat) != runtimeFormats.end()) {
            m_format = preferredFormat;
            break;
        }
    }
    if (m_format == VK_FORMAT_UNDEFINED) {
        std::string offeredFormats;
        for (int64_t runtimeFormat : runtimeFormats) {
            offeredFormats += fmt::format("{} ", runtimeFormat);
        }
        Log::print<ERROR>("OpenXR runtime offered {} swapchain formats: {}", kind == Kind::Depth ? "depth" : "color", offeredFormats);
        throw std::runtime_error("OpenXR runtime doesn't support any of the Vulkan swapchain formats that the mod can present.");
    }
    Log::print<INFO>("Using Vulkan format {} for the {} swapchain ({}x{})", (int)m_format, kind == Kind::Depth ? "depth" : "color", width, height);

    // the mod's render passes are single-sampled, so MSAA images would fail vkCreateFramebuffer
    if (sampleCount != 1) {
        Log::print<WARNING>("The OpenXR runtime recommends {} samples, but the mod renders single-sampled, so the swapchain is created with 1 sample", sampleCount);
    }

    XrSwapchainCreateInfo swapchainCreateInfo = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
    swapchainCreateInfo.width = width;
    swapchainCreateInfo.height = height;
    swapchainCreateInfo.arraySize = 1;
    swapchainCreateInfo.sampleCount = 1;
    swapchainCreateInfo.format = m_format;
    swapchainCreateInfo.mipCount = 1;
    swapchainCreateInfo.faceCount = 1;
    swapchainCreateInfo.usageFlags = (kind == Kind::Depth ? XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT) | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    swapchainCreateInfo.createFlags = 0;
    checkXRResult(xrCreateSwapchain(VRManager::instance().XR->GetSession(), &swapchainCreateInfo, &m_swapchain), "Failed to create OpenXR swapchain images!");

    uint32_t swapchainImagesCount = 0;
    checkXRResult(xrEnumerateSwapchainImages(m_swapchain, 0, &swapchainImagesCount, nullptr), "Failed to enumerate swapchain images!");
    std::vector<XrSwapchainImageVulkan2KHR> swapchainImages(swapchainImagesCount, { XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR });
    checkXRResult(xrEnumerateSwapchainImages(m_swapchain, swapchainImagesCount, &swapchainImagesCount, reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchainImages.data())), "Failed to enumerate swapchain images!");

    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    for (const XrSwapchainImageVulkan2KHR& swapchainImage : swapchainImages) {
        VkImageViewCreateInfo imageViewCreateInfo = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        imageViewCreateInfo.image = swapchainImage.image;
        imageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        imageViewCreateInfo.format = m_format;
        imageViewCreateInfo.components = {
            .r = VK_COMPONENT_SWIZZLE_IDENTITY,
            .g = VK_COMPONENT_SWIZZLE_IDENTITY,
            .b = VK_COMPONENT_SWIZZLE_IDENTITY,
            .a = VK_COMPONENT_SWIZZLE_IDENTITY
        };
        imageViewCreateInfo.subresourceRange = {
            .aspectMask = VulkanUtils::GetAspectMaskForFormat(m_format),
            .baseMipLevel = 0,
            .levelCount = 1,
            .baseArrayLayer = 0,
            .layerCount = 1
        };

        VkImageView imageView = VK_NULL_HANDLE;
        checkVkResult(dispatch->CreateImageView(VRManager::instance().VK->GetDevice(), &imageViewCreateInfo, nullptr, &imageView), "Failed to create a view for an OpenXR swapchain image!");

        m_images.emplace_back(swapchainImage.image);
        m_imageViews.emplace_back(imageView);
    }
}

void Swapchain::PrepareRendering() {
    checkXRResult(xrAcquireSwapchainImage(m_swapchain, nullptr, &m_swapchainImageIdx), "Can't acquire OpenXR swapchain image!");
}

VkImageView Swapchain::StartRendering() {
    XrSwapchainImageWaitInfo waitSwapchainInfo = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitSwapchainInfo.timeout = XR_INFINITE_DURATION;
    if (XrResult waitResult = xrWaitSwapchainImage(m_swapchain, &waitSwapchainInfo); waitResult == XR_TIMEOUT_EXPIRED || XR_FAILED(waitResult)) {
        checkXRResult(waitResult, "Failed to wait for swapchain image!");
    }

    return m_imageViews[m_swapchainImageIdx];
}

void Swapchain::FinishRendering() {
    XrSwapchainImageReleaseInfo releaseSwapchainInfo = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    checkXRResult(xrReleaseSwapchainImage(m_swapchain, &releaseSwapchainInfo), "Failed to release swapchain image!");
}

Swapchain::~Swapchain() {
    // the backend caches framebuffers built from these views, so they have to go before the views do
    if (VRManager::instance().Backend != nullptr) {
        VRManager::instance().Backend->FlushFramebufferCache();
    }

    const auto* dispatch = VRManager::instance().VK->GetDeviceDispatch();
    for (VkImageView imageView : m_imageViews) {
        dispatch->DestroyImageView(VRManager::instance().VK->GetDevice(), imageView, nullptr);
    }
    m_imageViews.clear();
    m_images.clear();

    if (m_swapchain != XR_NULL_HANDLE) {
        xrDestroySwapchain(m_swapchain);
    }
}
