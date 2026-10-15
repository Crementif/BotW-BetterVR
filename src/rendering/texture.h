#pragma once

class BaseVulkanTexture {
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
    void vkCopyFromImage(VkCommandBuffer cmdBuffer, VkImage srcImage, uint32_t srcLayer = 0);

    // the staging buffer lives until vkReleaseStaging, which the caller may only call once the copy has executed
    void vkUpload(VkCommandBuffer cmdBuffer, const void* data, size_t size);
    void vkReleaseStaging();

    uint32_t GetWidth() const { return m_width; }
    uint32_t GetHeight() const { return m_height; }
    VkFormat GetFormat() const { return m_vkFormat; }
    VkImageAspectFlags GetAspectMask() const;
    VkImage GetImage() const { return m_vkImage; }

protected:
    VkImage m_vkImage = VK_NULL_HANDLE;
    VkDeviceMemory m_vkMemory = VK_NULL_HANDLE;
    VkImageLayout m_vkCurrLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    uint32_t m_width;
    uint32_t m_height;
    VkFormat m_vkFormat;

    VkBuffer m_stagingBuffer = VK_NULL_HANDLE;
    VkDeviceMemory m_stagingMemory = VK_NULL_HANDLE;
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

// waiters take the last signaled value and signalers the next one
class FrameSync {
public:
    FrameSync();
    ~FrameSync();

    FrameSync(const FrameSync&) = delete;
    FrameSync& operator=(const FrameSync&) = delete;

    const VkSemaphore& GetSemaphore() const { return m_vkSemaphore; }

    uint64_t GetWaitValue() const { return m_counter.load(); }
    uint64_t GetSignalValue() { return ++m_counter; }

    const VkSemaphore& GetSemaphoreForSignal(uint64_t dbg_SignalTo = 0) {
        SetLastSignalledValue(dbg_SignalTo);
        return m_vkSemaphore;
    }
    const VkSemaphore& GetSemaphoreForWait(uint64_t dbg_WaitFor = 0) {
        SetLastAwaitedValue(dbg_WaitFor);
        return m_vkSemaphore;
    }

private:
    void SetLastSignalledValue(uint64_t value) {
        static uint32_t s_signalCount = 0;
        s_signalCount++;
        if (s_signalCount % 500 == 0 || m_lastSignalledValue == value) {
            Log::print<INTEROP>("Semaphore signal #{}: sync={}, value {} -> {} (last waited={})", s_signalCount, (void*)this, m_lastSignalledValue, value, m_lastAwaitedValue);
        }
        if (m_lastSignalledValue == value && value != 0) {
            Log::print<WARNING>("Double signal detected! sync={}, value={}", (void*)this, value);
        }
        m_lastSignalledValue = value;
    }
    void SetLastAwaitedValue(uint64_t value) {
        static uint32_t s_waitCount = 0;
        s_waitCount++;
        if (s_waitCount % 500 == 0 || m_lastAwaitedValue == value) {
            Log::print<INTEROP>("Semaphore wait #{}: sync={}, value {} -> {} (last signaled={})", s_waitCount, (void*)this, m_lastAwaitedValue, value, m_lastSignalledValue);
        }
        if (m_lastAwaitedValue == value && value != 0) {
            Log::print<WARNING>("Double wait detected! sync={}, value={}", (void*)this, value);
        }
        m_lastAwaitedValue = value;
    }

    VkSemaphore m_vkSemaphore = VK_NULL_HANDLE;
    std::atomic<uint64_t> m_counter{ 0 };
    uint64_t m_lastSignalledValue = 0;
    uint64_t m_lastAwaitedValue = 0;
};
