#pragma once

template <DXGI_FORMAT T>
class Swapchain {
public:
    Swapchain(uint32_t width, uint32_t height, uint32_t sampleCount);
    ~Swapchain();

    void PrepareRendering();
    ID3D12Resource* StartRendering();
    void FinishRendering();

    XrSwapchain GetHandle() const { return m_swapchain; };
    // The session owns the swapchains made from it, so a session that is already destroyed took this
    // handle with it and xrDestroySwapchain on it would be invalid handle usage. The wrapper still
    // tears down everything it owns itself, it just forgets about the runtime object.
    void AbandonHandle() { m_swapchain = XR_NULL_HANDLE; }
    ID3D12Resource* GetTexture() const { return m_swapchainTextures[m_swapchainImageIdx].Get(); };

    DXGI_FORMAT GetFormat() const { return m_format; };
    [[nodiscard]] uint32_t GetWidth() const { return m_width; };
    [[nodiscard]] uint32_t GetHeight() const { return m_height; };

private:
    XrSwapchain m_swapchain = XR_NULL_HANDLE;
    uint32_t m_width;
    uint32_t m_height;
    DXGI_FORMAT m_format;

    std::vector<ComPtr<ID3D12Resource>> m_swapchainTextures;
    uint32_t m_swapchainImageIdx = 0;
};