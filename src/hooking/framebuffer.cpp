#include "framebuffer.h"
#include "instance.h"
#include "layer.h"
#include "utils/d3d12_utils.h"

std::array<CaptureTexture, 3> captureTextures = {
    CaptureTexture{ false, { 0, 0 }, VK_NULL_HANDLE, VK_FORMAT_B10G11R11_UFLOAT_PACK32, { 1280, 720 }, { nullptr, nullptr }, { false, false}, VK_NULL_HANDLE },
    CaptureTexture{ false, { 0, 0 }, VK_NULL_HANDLE, VK_FORMAT_D32_SFLOAT, { 1280, 720 }, { nullptr, nullptr },  { false, false}, VK_NULL_HANDLE },
    CaptureTexture{ false, { 0, 0 }, VK_NULL_HANDLE, VK_FORMAT_A2B10G10R10_UNORM_PACK32, { 1280, 720 }, { nullptr, nullptr },  { false, false}, VK_NULL_HANDLE }
};
std::atomic_size_t foundResolutions = captureTextures.size();

std::mutex lockImageResolutions;
std::unordered_map<VkImage, std::pair<VkExtent2D, VkFormat>> imageResolutions;

using namespace VRLayer;

VkResult VkDeviceOverrides::CreateImage(const vkroots::VkDeviceDispatch* pDispatch, VkDevice device, const VkImageCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkImage* pImage) {
    VkResult res = pDispatch->CreateImage(device, pCreateInfo, pAllocator, pImage);

    //static uint32_t lowestFoundWidth = std::min_element(captureTextures.begin(), captureTextures.end(), [](const CaptureTexture& a, const CaptureTexture& b){ return a.foundSize.width < b.foundSize.width; })->foundSize.width;
    if (foundResolutions > 0 && pCreateInfo->extent.width >= 1280 && pCreateInfo->extent.height >= 720) {
        lockImageResolutions.lock();
        checkAssert(imageResolutions.try_emplace(*pImage, std::make_pair(VkExtent2D{ pCreateInfo->extent.width, pCreateInfo->extent.height }, pCreateInfo->format)).second, "Couldn't insert image resolution into map!");
        lockImageResolutions.unlock();
    }
    return res;
}

void VkDeviceOverrides::DestroyImage(const vkroots::VkDeviceDispatch* pDispatch, VkDevice device, VkImage image, const VkAllocationCallbacks* pAllocator) {
    pDispatch->DestroyImage(device, image, pAllocator);
    if (foundResolutions > 0) {
        lockImageResolutions.lock();
        imageResolutions.erase(image);
        lockImageResolutions.unlock();
    }
}

void VkDeviceOverrides::CmdClearColorImage(const vkroots::VkDeviceDispatch* pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearColorValue* pColor, uint32_t rangeCount, const VkImageSubresourceRange* pRanges) {
    // check for magical clear values
    if (pColor->float32[1] >= 0.12 && pColor->float32[1] <= 0.13 && pColor->float32[2] >= 0.97 && pColor->float32[2] <= 0.99) {
        // r value in magical clear value is the capture idx after rounding down
        const long captureIdx = std::lroundf(pColor->float32[0]*32.0f);
        auto& capture = captureTextures[captureIdx];

        // this is a hack since the game clears the 2D layer twice, and we want to only capture the first one
        if (capture.format == VK_FORMAT_A2B10G10R10_UNORM_PACK32 && VRManager::instance().XR->GetRenderer()->m_layer2D.GetStatus() == RND_Renderer::Layer::Status::BINDING) {
            // clear it to black for a better visualization since Cemu's output is only the HUD at this point
            const_cast<VkClearColorValue*>(pColor)[0] = { 0.0f, 0.0f, 0.0f, 0.0f };
            return pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
        }

        // this is a hack since the game will eventually overlap multiple textures on top of
        if (capture.initialized && capture.foundImage != image) {
            // clear the image to be transparent to allow for the HUD to be rendered on top of it which results in a transparent HUD layer
            const_cast<VkClearColorValue*>(pColor)[0] = { 0.0f, 0.0f, 0.0f, 0.0f };
            return pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
        }

        checkAssert(capture.captureCmdBuffer == VK_NULL_HANDLE, "This texture already got captured in a previous command buffer, but never got submitted!");

        // initialize textures if not already done
        if (!capture.initialized) {
            lockImageResolutions.lock();

            auto it = imageResolutions.find(image);
            checkAssert(it != imageResolutions.end(), "Couldn't find the resolution for an image. Is the graphic pack not active?");

            if (capture.format != it->second.second) {
                lockImageResolutions.unlock();
                return pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
            }
            capture.initialized = true;
            capture.foundSize = it->second.first;
            checkAssert(capture.format == it->second.second, std::format("[WARNING] Got {} as VkFormat instead of the expected {}", it->second.second, capture.format).c_str());

            capture.sharedTextures[OpenXR::EyeSide::LEFT] = std::make_unique<SharedTexture>(capture.foundSize.width, capture.foundSize.height, capture.format, D3D12Utils::ToDXGIFormat(capture.format));
            capture.sharedTextures[OpenXR::EyeSide::RIGHT] = std::make_unique<SharedTexture>(capture.foundSize.width, capture.foundSize.height, capture.format, D3D12Utils::ToDXGIFormat(capture.format));
            capture.sharedTextures[OpenXR::EyeSide::LEFT]->d3d12GetTexture()->SetName(std::format(L"CaptureTexture #{} - Left", captureIdx).c_str());
            capture.sharedTextures[OpenXR::EyeSide::RIGHT]->d3d12GetTexture()->SetName(std::format(L"captureTexture #{} - Right", captureIdx).c_str());
            // note: The VkImage for both the 3D and 2D layer are the same so don't remove it from the pool of possible capture resolutions
            //imageResolutions.erase(it);
            foundResolutions--;
            Log::print("Found capture texture {}: res={}x{}, format={}", captureIdx, capture.foundSize.width, capture.foundSize.height, capture.format);

            ComPtr<ID3D12CommandAllocator> cmdAllocator;
            {
                ID3D12Device* d3d12Device = VRManager::instance().D3D12->GetDevice();
                ID3D12CommandQueue* d3d12Queue = VRManager::instance().D3D12->GetCommandQueue();
                d3d12Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&cmdAllocator));
                auto& sharedTextures = capture.sharedTextures;

                RND_D3D12::CommandContext<true> transitionInitialTextures(d3d12Device, d3d12Queue, cmdAllocator.Get(), [&sharedTextures, captureIdx](RND_D3D12::CommandContext<true>* context) {
                    context->GetRecordList()->SetName(L"transitionInitialTextures");
                    sharedTextures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
                    sharedTextures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
                    context->Signal(sharedTextures[OpenXR::EyeSide::LEFT].get(), 0);
                    context->Signal(sharedTextures[OpenXR::EyeSide::RIGHT].get(), 0);
                    Log::print("[left] texture {}: Initializing semaphore to value 0", captureIdx);
                    Log::print("[right] texture {}: Initializing semaphore to value 0", captureIdx);
                });
            }

            lockImageResolutions.unlock();
        }

        capture.foundImage = image;
        capture.captureCmdBuffer = commandBuffer;

        if (capture.format == VK_FORMAT_A2B10G10R10_UNORM_PACK32) {
            capture.isCapturingSharedTextures[VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide()] = true;
            Log::print("[{}] texture {}: Waiting for semaphore value {}", VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx, 0);
            Log::print("[{}] texture {}: Copying texture", VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx);
            capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide()]->CopyFromVkImage(commandBuffer, image);
            VRManager::instance().XR->GetRenderer()->m_layer2D.AddTexture(VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide(), capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer2D.GetCurrentSide()].get());
            VRManager::instance().XR->GetRenderer()->m_layer2D.FlipSide();

            const_cast<VkClearColorValue*>(pColor)[0] = { 0.0f, 0.0f, 0.0f, 0.0f };
            pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
        }
        else {
            capture.isCapturingSharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()] = true;
            Log::print("[{}] texture {}: Waiting for semaphore value {}", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx, 0);
            Log::print("[{}] texture {}: Copying texture", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx);

            capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()]->CopyFromVkImage(commandBuffer, image);
            VRManager::instance().XR->GetRenderer()->m_layer3D.AddTexture(VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide(), capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()].get());
            // delay flipping until depth buffer is also captured

            // clear the image to be transparent to allow for the HUD to be rendered on top of it which results in a transparent HUD layer
            const_cast<VkClearColorValue*>(pColor)[0] = { 0.0f, 0.0f, 0.0f, 0.0f };
            pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
        }
        return;
    }
    else {
        return pDispatch->CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
    }
}

void VRLayer::VkDeviceOverrides::CmdClearDepthStencilImage(const vkroots::VkDeviceDispatch* pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearDepthStencilValue* pDepthStencil, uint32_t rangeCount, const VkImageSubresourceRange* pRanges) {
    // check for magical clear values
    if (rangeCount == 1 && pDepthStencil->depth >= 0.011456789 && pDepthStencil->depth <= 0.013456789) {
        // stencil value is the capture idx
        const uint32_t captureIdx = pDepthStencil->stencil;
        auto& capture = captureTextures[captureIdx];

        if (capture.initialized && capture.foundImage != image) {
            return pDispatch->CmdClearDepthStencilImage(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
        }

        checkAssert(capture.captureCmdBuffer == VK_NULL_HANDLE, "This texture already got captured in a previous command buffer, but never got submitted!");

        // initialize textures if not already done
        if (!capture.initialized) {
            lockImageResolutions.lock();

            auto it = imageResolutions.find(image);
            if (it == imageResolutions.end()) {
                lockImageResolutions.unlock();
                return pDispatch->CmdClearDepthStencilImage(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
            }
            checkAssert(it != imageResolutions.end(), "Couldn't find the resolution for an image. Is the graphic pack not active?");

            capture.initialized = true;
            capture.foundSize = it->second.first;
            checkAssert(capture.format == it->second.second, std::format("Got {} as VkFormat instead of the expected {}", it->second.second, capture.format).c_str());

            capture.sharedTextures[OpenXR::EyeSide::LEFT] = std::make_unique<SharedTexture>(capture.foundSize.width, capture.foundSize.height, capture.format, D3D12Utils::ToDXGIFormat(capture.format));
            capture.sharedTextures[OpenXR::EyeSide::RIGHT] = std::make_unique<SharedTexture>(capture.foundSize.width, capture.foundSize.height, capture.format, D3D12Utils::ToDXGIFormat(capture.format));
            capture.sharedTextures[OpenXR::EyeSide::LEFT]->d3d12GetTexture()->SetName(std::format(L"CaptureTexture #{} - Left", captureIdx).c_str());
            capture.sharedTextures[OpenXR::EyeSide::RIGHT]->d3d12GetTexture()->SetName(std::format(L"captureTexture #{} - Right", captureIdx).c_str());
            imageResolutions.erase(it);
            foundResolutions--;
            Log::print("Found depth capture texture {}: res={}x{}, format={}", captureIdx, capture.foundSize.width, capture.foundSize.height, capture.format);

            ComPtr<ID3D12CommandAllocator> cmdAllocator;
            {
                ID3D12Device* d3d12Device = VRManager::instance().D3D12->GetDevice();
                ID3D12CommandQueue* d3d12Queue = VRManager::instance().D3D12->GetCommandQueue();
                d3d12Device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&cmdAllocator));
                auto& sharedTextures = capture.sharedTextures;

                RND_D3D12::CommandContext<true> transitionInitialTextures(d3d12Device, d3d12Queue, cmdAllocator.Get(), [&sharedTextures, captureIdx](RND_D3D12::CommandContext<true>* context) {
                    context->GetRecordList()->SetName(L"transitionInitialTextures");
                    sharedTextures[OpenXR::EyeSide::LEFT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
                    sharedTextures[OpenXR::EyeSide::RIGHT]->d3d12TransitionLayout(context->GetRecordList(), D3D12_RESOURCE_STATE_COPY_DEST);
                    Log::print("[left] texture {}: Initializing semaphore to value 0", captureIdx);
                    Log::print("[right] texture {}: Initializing semaphore to value 0", captureIdx);
                    context->Signal(sharedTextures[OpenXR::EyeSide::LEFT].get(), 0);
                    context->Signal(sharedTextures[OpenXR::EyeSide::RIGHT].get(), 0);
                });
            }

            lockImageResolutions.unlock();
        }
        capture.foundImage = image;
        Log::print("[{}] texture {}: Waiting for semaphore value {}", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx, 0);
        Log::print("[{}] texture {}: Copying texture", VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide() == OpenXR::EyeSide::LEFT ? "left" : "right", captureIdx);
        capture.isCapturingSharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()] = true;
        capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()]->CopyFromVkImage(commandBuffer, image);
        capture.captureCmdBuffer = commandBuffer;
        VRManager::instance().XR->GetRenderer()->m_layer3D.AddDepthTexture(VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide(), capture.sharedTextures[VRManager::instance().XR->GetRenderer()->m_layer3D.GetCurrentSide()].get());
        VRManager::instance().XR->GetRenderer()->m_layer3D.FlipSide();
        return;
    }
    else {
        return pDispatch->CmdClearDepthStencilImage(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
    }
}

VkResult VkDeviceOverrides::EndCommandBuffer(const vkroots::VkDeviceDispatch* pDispatch, VkCommandBuffer commandBuffer) {
    return pDispatch->EndCommandBuffer(commandBuffer);
}

VkResult VkDeviceOverrides::QueueSubmit(const vkroots::VkDeviceDispatch* pDispatch, VkQueue queue, uint32_t submitCount, const VkSubmitInfo* pSubmits, VkFence fence) {
    // insert pipeline barriers if an active capture is ongoing
    bool activeCapture = std::any_of(captureTextures.begin(), captureTextures.end(), [](const CaptureTexture& capture) { return capture.captureCmdBuffer != VK_NULL_HANDLE; });

    struct ModifiedSubmitInfo_t {
        std::vector<VkSemaphore> waitSemaphores;
        std::vector<uint64_t> timelineWaitValues;
        std::vector<VkPipelineStageFlags> waitDstStageMasks;
        std::vector<VkSemaphore> signalSemaphores;
        std::vector<uint64_t> timelineSignalValues;

        VkTimelineSemaphoreSubmitInfo timelineSemaphoreSubmitInfo = { VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO };
    };

    if (activeCapture) {
        std::vector<ModifiedSubmitInfo_t> modifiedSubmitInfos(submitCount);
        for (uint32_t i = 0; i < submitCount; i++) {
            const VkSubmitInfo& submitInfo = pSubmits[i];
            ModifiedSubmitInfo_t& modifiedSubmitInfo = modifiedSubmitInfos[i];

            // copy old semaphores into new vectors
            for (uint32_t j = 0; j < submitInfo.waitSemaphoreCount; j++) {
                modifiedSubmitInfo.waitSemaphores.emplace_back(submitInfo.pWaitSemaphores[j]);
                modifiedSubmitInfo.waitDstStageMasks.emplace_back(submitInfo.pWaitDstStageMask[j]);
                modifiedSubmitInfo.timelineWaitValues.emplace_back(0);
            }

            for (uint32_t j = 0; j < submitInfo.signalSemaphoreCount; j++) {
                modifiedSubmitInfo.signalSemaphores.emplace_back(submitInfo.pSignalSemaphores[j]);
                modifiedSubmitInfo.timelineSignalValues.emplace_back(0);
            }

            // find timeline semaphore submit info if already present
            const VkTimelineSemaphoreSubmitInfo* timelineSemaphoreSubmitInfoPtr = &modifiedSubmitInfo.timelineSemaphoreSubmitInfo;

            const VkBaseInStructure* pNextIt = static_cast<const VkBaseInStructure*>(submitInfo.pNext);
            while (pNextIt) {
                if (pNextIt->sType == VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO) {
                    timelineSemaphoreSubmitInfoPtr = reinterpret_cast<const VkTimelineSemaphoreSubmitInfo*>(pNextIt);
                    break;
                }
                pNextIt = pNextIt->pNext;
            }
            if (pNextIt == nullptr)
                const_cast<VkSubmitInfo&>(submitInfo).pNext = &modifiedSubmitInfo.timelineSemaphoreSubmitInfo;

            // copy any existing timeline values into new vectors
            for (uint32_t j = 0; j < timelineSemaphoreSubmitInfoPtr->waitSemaphoreValueCount; j++) {
                modifiedSubmitInfo.timelineWaitValues.emplace_back(timelineSemaphoreSubmitInfoPtr->pWaitSemaphoreValues[j]);
            }
            for (uint32_t j = 0; j < timelineSemaphoreSubmitInfoPtr->signalSemaphoreValueCount; j++) {
                modifiedSubmitInfo.timelineSignalValues.emplace_back(timelineSemaphoreSubmitInfoPtr->pSignalSemaphoreValues[j]);
            }

            // insert timeline semaphores
            for (uint32_t j = 0; j < submitInfo.commandBufferCount; j++) {
                for (uint32_t l = 0; l < captureTextures.size(); l++) { // fixme: revert to auto& capture : captureTextures
                    auto& capture = captureTextures[l];
                    if (submitInfo.pCommandBuffers[j] == capture.captureCmdBuffer) {
                        for (size_t k=0; k<capture.isCapturingSharedTextures.size(); k++) {
                            if (capture.isCapturingSharedTextures[k]) {
                                // wait for D3D12/XR to finish with previous shared texture render
                                modifiedSubmitInfo.waitSemaphores.emplace_back(capture.sharedTextures[k]->GetSemaphore());
                                modifiedSubmitInfo.waitDstStageMasks.emplace_back(VK_PIPELINE_STAGE_ALL_COMMANDS_BIT); // check which stage is applicable; used to be set to
                                modifiedSubmitInfo.timelineWaitValues.emplace_back(0);

                                // signal to D3D12/XR rendering that the shared texture can be rendered to VR headset
                                modifiedSubmitInfo.signalSemaphores.emplace_back(capture.sharedTextures[k]->GetSemaphore());
                                modifiedSubmitInfo.timelineSignalValues.emplace_back(1);
                                Log::print("[{}] texture {}: Signaling semaphore to value {}", k == 0 ? "left" : "right", l, modifiedSubmitInfo.timelineSignalValues.back());
                                capture.isCapturingSharedTextures[k] = false;
                            }
                        }
                        capture.captureCmdBuffer = VK_NULL_HANDLE;
                    }
                }
            }

            // update the VkTimelineSemaphoreSubmitInfo struct
            const_cast<VkTimelineSemaphoreSubmitInfo*>(timelineSemaphoreSubmitInfoPtr)->waitSemaphoreValueCount = (uint32_t)modifiedSubmitInfo.timelineWaitValues.size();
            const_cast<VkTimelineSemaphoreSubmitInfo*>(timelineSemaphoreSubmitInfoPtr)->pWaitSemaphoreValues = modifiedSubmitInfo.timelineWaitValues.data();
            const_cast<VkTimelineSemaphoreSubmitInfo*>(timelineSemaphoreSubmitInfoPtr)->signalSemaphoreValueCount = (uint32_t)modifiedSubmitInfo.timelineSignalValues.size();
            const_cast<VkTimelineSemaphoreSubmitInfo*>(timelineSemaphoreSubmitInfoPtr)->pSignalSemaphoreValues = modifiedSubmitInfo.timelineSignalValues.data();

            // add wait and signal semaphores to the submit info
            const_cast<VkSubmitInfo&>(submitInfo).waitSemaphoreCount = (uint32_t)modifiedSubmitInfo.waitSemaphores.size();
            const_cast<VkSubmitInfo&>(submitInfo).pWaitSemaphores = modifiedSubmitInfo.waitSemaphores.data();
            const_cast<VkSubmitInfo&>(submitInfo).pWaitDstStageMask = modifiedSubmitInfo.waitDstStageMasks.data();
            const_cast<VkSubmitInfo&>(submitInfo).signalSemaphoreCount = (uint32_t)modifiedSubmitInfo.signalSemaphores.size();
            const_cast<VkSubmitInfo&>(submitInfo).pSignalSemaphores = modifiedSubmitInfo.signalSemaphores.data();
        }

        return pDispatch->QueueSubmit(queue, submitCount, pSubmits, fence);
    }
    else {
        return pDispatch->QueueSubmit(queue, submitCount, pSubmits, fence);
    }
}

VkResult VkDeviceOverrides::QueuePresentKHR(const vkroots::VkDeviceDispatch* pDispatch, VkQueue queue, const VkPresentInfoKHR* pPresentInfo) {
    VRManager::instance().XR->ProcessEvents();
    if (VRManager::instance().XR->GetRenderer() && ((VRManager::instance().XR->GetRenderer()->m_layer2D.ShouldRender() && VRManager::instance().XR->GetRenderer()->m_layer3D.GetStatus() == RND_Renderer::Layer::Status::PREPARING) || VRManager::instance().XR->GetRenderer()->m_layer3D.ShouldRender())) {
        VRManager::instance().XR->GetRenderer()->EndFrame();
        VRManager::instance().XR->GetRenderer()->StartFrame();
    }
    return pDispatch->QueuePresentKHR(queue, pPresentInfo);
}