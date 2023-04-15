#pragma once

class Swapchain {
public:
    Swapchain(uint32_t width, uint32_t height, uint32_t sampleCount);
    ~Swapchain();

    void PrepareRendering();
    ID3D12Resource* StartRendering();
    void FinishRendering();

    XrSwapchain GetHandle() { return m_swapchain; };
    ID3D12Resource* GetTexture() { return m_swapchainTextures[m_swapchainImageIdx].Get(); };

    DXGI_FORMAT GetFormat() { return DXGI_FORMAT_R8G8B8A8_UNORM; };
    uint32_t GetWidth() const { return m_width; };
    uint32_t GetHeight() const { return m_height; };

private:
    XrSwapchain m_swapchain = XR_NULL_HANDLE;
    uint32_t m_width;
    uint32_t m_height;
    DXGI_FORMAT m_format;

    std::vector<ComPtr<ID3D12Resource>> m_swapchainTextures;
    uint32_t m_swapchainImageIdx = 0;

    ComPtr<ID3D12DescriptorHeap> m_colorTarget;
};