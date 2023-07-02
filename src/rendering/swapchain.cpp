#include "swapchain.h"
#include "instance.h"
#include "../utils/d3d12_utils.h"

template <DXGI_FORMAT T>
Swapchain<T>::Swapchain(uint32_t width, uint32_t height, uint32_t sampleCount): m_width(width), m_height(height) {
    auto getBestSwapchainFormat = [](const std::vector<DXGI_FORMAT>& applicationSupportedFormats) -> DXGI_FORMAT {
        // Finds the first matching DXGI_FORMAT (int) that matches the int64 from OpenXR
        uint32_t swapchainCount = 0;
        xrEnumerateSwapchainFormats(VRManager::instance().XR->GetSession(), 0, &swapchainCount, nullptr);
        std::vector<int64_t> xrPreferredFormats(swapchainCount);
        xrEnumerateSwapchainFormats(VRManager::instance().XR->GetSession(), swapchainCount, &swapchainCount, xrPreferredFormats.data());

        auto found = std::find_first_of(std::begin(xrPreferredFormats), std::end(xrPreferredFormats), std::begin(applicationSupportedFormats), std::end(applicationSupportedFormats));
        if (found == std::end(xrPreferredFormats)) {
            throw std::runtime_error("OpenXR runtime doesn't support any of the presenting modes that the GPU drivers support.");
        }
        return (DXGI_FORMAT)*found;
    };

    const std::vector<DXGI_FORMAT> preferredColorFormats = {
        // fixme: check if OpenXR prefers sRGB or not
        T
    };
    m_format = getBestSwapchainFormat(preferredColorFormats);

    XrSwapchainCreateInfo swapchainCreateInfo = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
    swapchainCreateInfo.width = width;
    swapchainCreateInfo.height = height;
    swapchainCreateInfo.arraySize = 1;
    swapchainCreateInfo.sampleCount = sampleCount;
    swapchainCreateInfo.format = m_format;
    swapchainCreateInfo.mipCount = 1;
    swapchainCreateInfo.faceCount = 1;
    swapchainCreateInfo.usageFlags = D3D12Utils::IsDepthFormat(T) ? XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    swapchainCreateInfo.createFlags = 0;
    checkXRResult(xrCreateSwapchain(VRManager::instance().XR->GetSession(), &swapchainCreateInfo, &m_swapchain), "Failed to create OpenXR swapchain images!");

    uint32_t swapchainImagesCount = 0;
    checkXRResult(xrEnumerateSwapchainImages(m_swapchain, 0, &swapchainImagesCount, NULL), "Failed to enumerate swapchain images!");
    std::vector<XrSwapchainImageD3D12KHR> swapchainImages(swapchainImagesCount, { XR_TYPE_SWAPCHAIN_IMAGE_D3D12_KHR });
    checkXRResult(xrEnumerateSwapchainImages(m_swapchain, swapchainImagesCount, &swapchainImagesCount, reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchainImages.data())), "Failed to enumerate swapchain images!");

    for (size_t i=0; i<swapchainImages.size(); i++) {
        // D3D12Utils::CreateConstantBuffer(D3D12_HEAP_TYPE_DEFAULT);
        swapchainImages[i].texture->SetName(std::format(L"Swapchain Image {}", i).c_str());
        m_swapchainTextures.emplace_back(swapchainImages[i].texture);
    }
}

template <DXGI_FORMAT T>
void Swapchain<T>::PrepareRendering() {
    checkXRResult(xrAcquireSwapchainImage(m_swapchain, NULL, &m_swapchainImageIdx), "Can't acquire OpenXR swapchain image!");
}

template <DXGI_FORMAT T>
ID3D12Resource* Swapchain<T>::StartRendering() {
    XrSwapchainImageWaitInfo waitSwapchainInfo = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    waitSwapchainInfo.timeout = XR_INFINITE_DURATION;
    if (XrResult waitResult = xrWaitSwapchainImage(m_swapchain, &waitSwapchainInfo); waitResult == XR_TIMEOUT_EXPIRED || XR_FAILED(waitResult)) {
        checkXRResult(waitResult, "Failed to wait for swapchain image!");
    }
    return m_swapchainTextures[m_swapchainImageIdx].Get();
}

template <DXGI_FORMAT T>
void Swapchain<T>::FinishRendering() {
    XrSwapchainImageReleaseInfo releaseSwapchainInfo = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    checkXRResult(xrReleaseSwapchainImage(m_swapchain, &releaseSwapchainInfo), "Failed to release swapchain image!");
}

template <DXGI_FORMAT T>
Swapchain<T>::~Swapchain() {
    if (m_swapchain != XR_NULL_HANDLE) {
        xrDestroySwapchain(m_swapchain);
    }
}

template class Swapchain<DXGI_FORMAT_D32_FLOAT>;
template class Swapchain<DXGI_FORMAT_R8G8B8A8_UNORM_SRGB>;