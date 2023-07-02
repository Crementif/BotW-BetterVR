#pragma once

class SharedTexture;

class Texture {
public:
    Texture(uint32_t width, uint32_t height, DXGI_FORMAT format);
    ~Texture();

    void d3d12SignalFence(uint64_t value);
    void d3d12WaitForFence(uint64_t value);
    void d3d12TransitionLayout(ID3D12GraphicsCommandList* cmdList, D3D12_RESOURCE_STATES state);
    bool CopyFromSharedTexture(ID3D12GraphicsCommandList* cmdList, SharedTexture* texture);

    ID3D12Resource* d3d12GetTexture() const { return m_d3d12Texture.Get(); }
    DXGI_FORMAT d3d12GetFormat() const { return m_d3d12Format; }

protected:
    DXGI_FORMAT m_d3d12Format;
    HANDLE m_d3d12TextureHandle = nullptr;
    ComPtr<ID3D12Resource> m_d3d12Texture;
    D3D12_RESOURCE_STATES m_currState = D3D12_RESOURCE_STATE_COMMON;

    HANDLE m_d3d12FenceHandle = nullptr;
    ComPtr<ID3D12Fence> m_d3d12Fence;
    uint64_t m_fenceValue = 0;
};

class SharedTexture : public Texture {
    friend class SharedTexture;

public:
    SharedTexture(uint32_t width, uint32_t height, VkFormat vkFormat, DXGI_FORMAT d3d12Format);
    ~SharedTexture();

    void vkTransitionLayout(VkCommandBuffer cmdBuffer, VkImageLayout layout);
    void CopyFromVkImage(VkCommandBuffer cmdBuffer, VkImage srcImage);

    const VkSemaphore& GetSemaphore() const { return m_vkSemaphore; }

private:
    VkImage m_vkImage = VK_NULL_HANDLE;
    VkDeviceMemory m_vkMemory = VK_NULL_HANDLE;
    VkSemaphore m_vkSemaphore = VK_NULL_HANDLE;
    VkImageLayout m_currLayout = VK_IMAGE_LAYOUT_UNDEFINED;
};