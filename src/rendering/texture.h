#pragma once

class SharedTexture;

class BaseVulkanTexture {
    friend class SharedTexture;
    friend class VulkanTexture;
public:
    BaseVulkanTexture(uint32_t width, uint32_t height, VkFormat vkFormat): m_width(width), m_height(height), m_vkFormat(vkFormat) {}
    virtual ~BaseVulkanTexture();

    void vkPipelineBarrier(VkCommandBuffer cmdBuffer);
    void vkTransitionLayout(VkCommandBuffer cmdBuffer, VkImageLayout newLayout);

    void vkClear(VkCommandBuffer cmdBuffer, VkClearColorValue color);
    void vkClearDepth(VkCommandBuffer cmdBuffer, float depth, uint32_t stencil = 0);
    void vkCopyToImage(VkCommandBuffer cmdBuffer, VkImage dstImage);
    // AMD GPU FIX: srcLayout parameter to specify the actual source image layout
    // If srcLayout is TRANSFER_SRC_OPTIMAL, assume caller has already transitioned and skip internal transitions
    void vkCopyFromImage(VkCommandBuffer cmdBuffer, VkImage srcImage, VkImageLayout srcLayout = VK_IMAGE_LAYOUT_GENERAL);
    uint32_t GetWidth() const { return m_width; }
    uint32_t GetHeight() const { return m_height; }
    VkFormat GetFormat() const { return m_vkFormat; }
    VkImageAspectFlags GetAspectMask() const;

protected:
    VkImage m_vkImage = VK_NULL_HANDLE;
    VkDeviceMemory m_vkMemory = VK_NULL_HANDLE;
    VkImageLayout m_vkCurrLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    uint32_t m_width;
    uint32_t m_height;
    VkFormat m_vkFormat;
};

class VulkanTexture : public BaseVulkanTexture {
    friend class VulkanFramebuffer;
public:
    VulkanTexture(uint32_t width, uint32_t height, VkFormat format, VkImageUsageFlags usage, bool disableAlphaThroughSwizzling);
    VulkanTexture(uint32_t width, uint32_t height, VkFormat format, VkImageUsageFlags usage): VulkanTexture(width, height, format, usage, false) {
    }
    ~VulkanTexture() override;

    VkImageView GetImageView() const { return m_vkImageView; }

private:
    VkImageView m_vkImageView = VK_NULL_HANDLE;
};

class VulkanFramebuffer : public VulkanTexture {
public:
    VulkanFramebuffer(uint32_t width, uint32_t height, VkFormat format, VkRenderPass renderPass);
    ~VulkanFramebuffer() override;

    VkFramebuffer GetFramebuffer() const { return m_framebuffer; }
private:
    VkFramebuffer m_framebuffer = VK_NULL_HANDLE;
};

class Texture {
public:
    Texture(uint32_t width, uint32_t height, DXGI_FORMAT format);
    virtual ~Texture();

    void d3d12SignalFence(uint64_t value);
    void d3d12WaitForFence(uint64_t value);
    void d3d12TransitionLayout(ID3D12GraphicsCommandList* cmdList, D3D12_RESOURCE_STATES state);

    ID3D12Resource* d3d12GetTexture() const { return m_d3d12Texture.Get(); }
    DXGI_FORMAT d3d12GetFormat() const { return m_d3d12Format; }

    uint64_t GetLastSignalledValue() const { return m_fenceLastSignaledValue; }
    uint64_t GetLastAwaitedValue() const { return m_fenceLastAwaitedValue; }

protected:
    void SetLastSignalledValue(uint64_t value) {
        // Track signal/wait pattern for debugging
        static uint32_t s_signalCount = 0;
        s_signalCount++;
        if (s_signalCount % 500 == 0 || m_fenceLastSignaledValue == value) {
            Log::print<INFO>("Semaphore signal #{}: texture={}, value {} -> {} (last waited={})",
                s_signalCount, (void*)this, m_fenceLastSignaledValue, value, m_fenceLastAwaitedValue);
        }
        if (m_fenceLastSignaledValue == value && value != 0) {
            Log::print<WARNING>("Double signal detected! texture={}, value={}", (void*)this, value);
        }
        m_fenceLastSignaledValue = value;
    }
    void SetLastAwaitedValue(uint64_t value) {
        static uint32_t s_waitCount = 0;
        s_waitCount++;
        if (s_waitCount % 500 == 0 || m_fenceLastAwaitedValue == value) {
            Log::print<INFO>("Semaphore wait #{}: texture={}, value {} -> {} (last signaled={})",
                s_waitCount, (void*)this, m_fenceLastAwaitedValue, value, m_fenceLastSignaledValue);
        }
        if (m_fenceLastAwaitedValue == value && value != 0) {
            Log::print<WARNING>("Double wait detected! texture={}, value={}", (void*)this, value);
        }
        m_fenceLastAwaitedValue = value;
    }

    DXGI_FORMAT m_d3d12Format;
    HANDLE m_d3d12TextureHandle = nullptr;
    ComPtr<ID3D12Resource> m_d3d12Texture;
    D3D12_RESOURCE_STATES m_currState = D3D12_RESOURCE_STATE_COMMON;

    HANDLE m_d3d12FenceHandle = nullptr;
    ComPtr<ID3D12Fence> m_d3d12Fence;
    uint64_t m_fenceLastSignaledValue = 0;
    uint64_t m_fenceLastAwaitedValue = 0;
};

class SharedTexture : public Texture, public BaseVulkanTexture {
public:
    SharedTexture(uint32_t width, uint32_t height, VkFormat vkFormat, DXGI_FORMAT d3d12Format);
    ~SharedTexture() override;

    // srcImageLayout: the ACTUAL current layout of srcImage (e.g., from Cemu's CmdClearColorImage hook)
    void CopyFromVkImage(VkCommandBuffer cmdBuffer, VkImage srcImage, VkImageLayout srcImageLayout = VK_IMAGE_LAYOUT_GENERAL);
    const VkSemaphore& GetSemaphore() const { return m_vkSemaphore; }

    // AMD GPU FIX: Timeline semaphores require strictly increasing values.
    // Instead of ping-ponging between 0 and 1, we use a monotonically increasing counter.
    // The flow is:
    //   1. Vulkan waits for value N (last D3D12 signal, or 0 initially)
    //   2. Vulkan copies, then signals N+1
    //   3. D3D12 waits for N+1
    //   4. D3D12 uses texture, then signals N+2
    //   5. Next frame: Vulkan waits for N+2, signals N+3, etc.

    // Get the value Vulkan should wait for (the last value D3D12 signaled)
    uint64_t GetVulkanWaitValue() const { return m_fenceCounter.load(); }

    // Get the value Vulkan should signal (increments counter)
    uint64_t GetVulkanSignalValue() { return ++m_fenceCounter; }

    // Get the value D3D12 should wait for (the last value Vulkan signaled)
    uint64_t GetD3D12WaitValue() const { return m_fenceCounter.load(); }

    // Get the value D3D12 should signal (increments counter)
    uint64_t GetD3D12SignalValue() { return ++m_fenceCounter; }

    const VkSemaphore& GetSemaphoreForSignal(uint64_t dbg_SignalTo = 0) {
        SetLastSignalledValue(dbg_SignalTo);
        return m_vkSemaphore;
    }
    const VkSemaphore& GetSemaphoreForWait(uint64_t dbg_WaitFor = 0) {
        SetLastAwaitedValue(dbg_WaitFor);
        return m_vkSemaphore;
    }

private:
    VkSemaphore m_vkSemaphore = VK_NULL_HANDLE;
    std::atomic_bool m_activeOperation = false;
    std::atomic<uint64_t> m_fenceCounter{0};  // Monotonically increasing fence value
};