#include "pch.h"

#include "framebuffer.h"
#include "instance.h"
#include "layer.h"
#include "utils/vulkan_utils.h"
#include "utils/debug_draw.h"
#include "utils/render_utils.h"


std::mutex lockImageResolutions;
std::unordered_map<VkImage, std::pair<VkExtent2D, VkFormat>> imageResolutions;

struct PendingCopyOperation {
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    SharedTexture* texture = nullptr;
};

std::mutex s_pendingCopyMutex;
std::vector<PendingCopyOperation> s_pendingCopyOperations;

VkImage s_curr3DColorImage = VK_NULL_HANDLE;
VkImage s_curr3DDepthImage = VK_NULL_HANDLE;

using namespace VRLayer;

static bool QueuePendingCopyOperation(VkCommandBuffer commandBuffer, SharedTexture* texture) {
    if (commandBuffer == VK_NULL_HANDLE || texture == nullptr) {
        return false;
    }

    std::lock_guard lk(s_pendingCopyMutex);
    const auto duplicateIt = std::ranges::find_if(s_pendingCopyOperations, [commandBuffer, texture](const PendingCopyOperation& operation) {
        return operation.commandBuffer == commandBuffer && operation.texture == texture;
    });
    if (duplicateIt != s_pendingCopyOperations.end()) {
        Log::print<INTEROP>("Ignoring duplicate pending copy registration for commandBuffer={} texture={}", (void*)commandBuffer, (void*)texture);
        return false;
    }

    s_pendingCopyOperations.push_back({ commandBuffer, texture });
    return true;
}

VkResult VkDeviceOverrides::CreateImage(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, const VkImageCreateInfo* pCreateInfo, const VkAllocationCallbacks* pAllocator, VkImage* pImage) {
    VkResult res = pDispatch.CreateImage(device, pCreateInfo, pAllocator, pImage);

    if (pCreateInfo->extent.width >= 1280 && pCreateInfo->extent.height >= 720) {
        lockImageResolutions.lock();
        checkAssert(imageResolutions.try_emplace(*pImage, std::make_pair(VkExtent2D{ pCreateInfo->extent.width, pCreateInfo->extent.height }, pCreateInfo->format)).second, "Couldn't insert image resolution into map!");
        lockImageResolutions.unlock();
    }
    return res;
}

void VkDeviceOverrides::DestroyImage(const vkroots::VkDeviceDispatch& pDispatch, VkDevice device, VkImage image, const VkAllocationCallbacks* pAllocator) {
    lockImageResolutions.lock();
    imageResolutions.erase(image);
    if (s_curr3DColorImage == image) {
        s_curr3DColorImage = VK_NULL_HANDLE;
    }
    else if (s_curr3DDepthImage == image) {
        s_curr3DDepthImage = VK_NULL_HANDLE;
    }
    lockImageResolutions.unlock();

    pDispatch.DestroyImage(device, image, pAllocator);
}


void CemuHooks::hook_FixCameraSaveFilesAndInventory(PPCInterpreter_t* hCPU) {
    hCPU->instructionPointer = hCPU->sprNew.LR;

    uint32_t originCaller = hCPU->gpr[0];
    uint32_t tempLayerCopyObject = hCPU->gpr[3];
    uint32_t frameIdx = hCPU->gpr[4];
    EyeSide side = (EyeSide)hCPU->gpr[5];

    Log::print<PPC>("[{:08X}] hook_FixCameraSaveFilesAndInventory: tempLayerCopyObject={:08X}, side={}, frameIdx={}", originCaller, tempLayerCopyObject, side, frameIdx);
    if (auto* renderer = VRManager::instance().XR->GetRenderer(); renderer != nullptr) {
        renderer->SignalGameCapturing3DFrameBuffer((long)frameIdx);
    }
}


void VkDeviceOverrides::CmdClearColorImage(const vkroots::VkCommandBufferDispatch& pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearColorValue* pColor, uint32_t rangeCount, const VkImageSubresourceRange* pRanges) {
    // check whether the magic values are there, and which order they are in to determine which eye
    OpenXR::EyeSide side = (OpenXR::EyeSide)-1;
    if (pColor->float32[1] >= 0.12 && pColor->float32[1] <= 0.13 && pColor->float32[2] >= 0.97 && pColor->float32[2] <= 0.99) {
        side = OpenXR::EyeSide::LEFT;
    }
    else if (pColor->float32[2] >= 0.12 && pColor->float32[2] <= 0.13 && pColor->float32[1] >= 0.97 && pColor->float32[1] <= 0.99) {
        side = OpenXR::EyeSide::RIGHT;
    }

    if (!VRManager::instance().VK) {
        auto* dispatch = pDispatch.pDeviceDispatch;
        VRManager::instance().Init(dispatch->pPhysicalDeviceDispatch->pInstanceDispatch->Instance, dispatch->PhysicalDevice, dispatch->Device);
        VRManager::instance().InitSession();
    }

    if (side != (OpenXR::EyeSide)-1) {
        // r value in magical clear value is the capture idx after rounding down
        const long captureIdx = std::lroundf(pColor->float32[0] * 32.0f);
        const long frameIdx = pColor->float32[3] < 0.5f ? 0 : 1;
        checkAssert(captureIdx == 0 || captureIdx == 2, "Invalid capture index!");

        Log::print<RENDERING>("[{}] Clearing color image for {} layer for {} side", frameIdx, captureIdx == 0 ? "3D" : "2D", side == OpenXR::EyeSide::LEFT ? "left" : "right");

        auto* renderer = VRManager::instance().XR->GetRenderer();
        if (!renderer) {
            Log::print<RENDERING>("Renderer is not initialized yet!");
            return pDispatch.CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
        }
        auto& layer3D = renderer->m_layer3D;
        auto& layer2D = renderer->m_layer2D;
        auto& imguiOverlay = renderer->m_imguiOverlay;

        // initialize the textures of both 2D and 3D layer if either is found since they share the same VkImage and resolution
        if (captureIdx == 0 || captureIdx == 2) {
            if (!layer2D) {
                lockImageResolutions.lock();
                if (const auto it = imageResolutions.find(image); it != imageResolutions.end()) {
                    auto viewConfs = VRManager::instance().XR->GetViewConfigurations();

                    VkExtent2D renderRes = it->second.first;
                    VkExtent2D swapchainRes = it->second.first;
                    if (VRManager::instance().XR->m_capabilities.isMetaSimulator) {
                        swapchainRes = VkExtent2D{ viewConfs[0].recommendedImageRectWidth, viewConfs[0].recommendedImageRectHeight };
                    }

                    renderer->m_gameRenderAspectRatio = (float)renderRes.width / (float)renderRes.height;
                    layer3D = std::make_unique<RND_Renderer::Layer3D>(renderer, renderRes, swapchainRes);
                    layer2D = std::make_unique<RND_Renderer::Layer2D>(renderer, renderRes, swapchainRes);
                    for (auto& textures : layer3D->GetSharedTextures()) {
                        for (auto& texture : textures) {
                            texture->Init(commandBuffer);
                        }
                    }
                    for (auto& textures : layer3D->GetDepthSharedTextures()) {
                        for (auto& texture : textures) {
                            texture->Init(commandBuffer);
                        }
                    }
                    for (auto& texture : layer2D->GetSharedTextures()) {
                        texture->Init(commandBuffer);
                    }

                    Log::print<INFO>("Found rendering resolution {}x{} @ {} using capture #{}", renderRes.width, renderRes.height, it->second.second, captureIdx);
                    // ImGui keeps one context per process, so an overlay that was carried over from a
                    // replaced session stays: only a renderer that never had one builds a new one.
                    if (!imguiOverlay)
                        imguiOverlay = std::make_unique<RND_Renderer::ImGuiOverlay>(renderer, commandBuffer, renderRes, VK_FORMAT_A2B10G10R10_UNORM_PACK32);
                    VRManager::instance().Hooks->m_entityDebugger = std::make_unique<EntityDebugger>();
                }
                else {
                    checkAssert(false, "Couldn't find image resolution in map!");
                }
                lockImageResolutions.unlock();
            }
        }

        if (!renderer->IsInitialized()) {
            return;
        }

        checkAssert(layer3D && layer2D, "Couldn't find 3D or 2D layer!");

        // change source image to GENERAL layout
        VulkanUtils::TransitionLayout(commandBuffer, image, imageLayout, VK_IMAGE_LAYOUT_GENERAL);
        VulkanUtils::DebugPipelineBarrier(commandBuffer);

        auto returnToLayout = [&]() {
            VulkanUtils::TransitionLayout(commandBuffer, image, VK_IMAGE_LAYOUT_GENERAL, imageLayout);
            VulkanUtils::DebugPipelineBarrier(commandBuffer);
        };

        RND_Renderer::RenderFrame& frame = renderer->GetFrame(frameIdx);

        auto clearFramebuffer = [&](bool disableAlpha) -> void {
            VkClearColorValue clearColor = disableAlpha ? VkClearColorValue{ { 0.0f, 0.0f, 0.0f, 1.0f } } : VkClearColorValue{ { 0.0f, 0.0f, 0.0f, 0.0f } };
            pDispatch.CmdClearColorImage(commandBuffer, image, imageLayout, &clearColor, rangeCount, pRanges);
        };
        const bool useMonoCapture = CemuHooks::UseMonoFrameBufferTemporarilyDuringMenusOrPictures();

        // 3D layer - color texture for 3D rendering
        if (captureIdx == 0) {
            auto skip3DColorCapture = [&](bool disableAlpha) -> void {
                returnToLayout();
                if (useMonoCapture) {
                    return;
                }
                clearFramebuffer(disableAlpha);
            };

            // check if the color texture has the appropriate texture format
            if (s_curr3DColorImage == VK_NULL_HANDLE) {
                lockImageResolutions.lock();
                if (const auto it = imageResolutions.find(image); it != imageResolutions.end()) {
                    if (it->second.second == VK_FORMAT_A2B10G10R10_UNORM_PACK32) {
                        s_curr3DColorImage = it->first;
                    }
                }
                lockImageResolutions.unlock();
            }

            // don't clear the image if we're in the faux 2D mode
            if (CemuHooks::UseBlackBarsDuringEvents()) {
                returnToLayout();
                return;
            }

            if (image != s_curr3DColorImage) {
                Log::print<RENDERING>("Color image is not the same as the current 3D color image! ({} != {})", (void*)image, (void*)s_curr3DColorImage);
                return skip3DColorCapture(!renderer->IsRendering3D(frameIdx));
            }

            if (side == EyeSide::LEFT) {
                if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
                    renderer->BeginStereoCaptureGeneration(frameIdx);
                }
            }
            else if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
                frame.AddIssue(RND_Renderer::CaptureIssue_UnexpectedSequence);
                Log::print<RENDERING>("[{}] Ignoring right-eye 3D color before stereo generation start", frameIdx);
                return skip3DColorCapture(!renderer->IsRendering3D(frameIdx));
            }

            if (frame.HasFatalIssue()) {
                Log::print<RENDERING>("[{}] Ignoring 3D color capture because the stereo slot is already invalid", frameIdx);
                return skip3DColorCapture(!renderer->IsRendering3D(frameIdx));
            }

            const uint32_t colorMask = side == EyeSide::LEFT ? RND_Renderer::CaptureMask_ColorLeft : RND_Renderer::CaptureMask_ColorRight;
            if (!frame.TryAcceptCapture(colorMask, image, frame.acceptedColorImages[side], frameIdx, side == EyeSide::LEFT ? "left-eye 3D color" : "right-eye 3D color")) {
                renderer->NoteDuplicateCaptureDropped();
                return skip3DColorCapture(false);
            }
            frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();

            SharedTexture* texture = layer3D->CopyColorToLayer(side, commandBuffer, image, frameIdx);
            renderer->On3DColorCopied(side, frameIdx);
            QueuePendingCopyOperation(commandBuffer, texture);

            if (useMonoCapture) {
                returnToLayout();
                return;
            }

            // imgui needs only one eye to render Cemu's 2D output, so use right side since it looks better
            if (side == EyeSide::RIGHT) {
                // note: Uses vkCmdCopyImage to copy the (right-eye-only) image to the imgui overlay's texture
                float desktopAspectRatio = layer3D->GetAspectRatio(side);
                const RenderUtils::UvTransform& desktopUvTransform = layer3D->GetPresentUvTransform(side);
                imguiOverlay->Draw3DLayerAsBackground(commandBuffer, image, desktopAspectRatio, desktopUvTransform, frameIdx);
            }

            // clear the image to be transparent to allow for the HUD to be rendered on top of it which results in a transparent HUD layer
            returnToLayout();
            clearFramebuffer(false);
            return;
        }

        // 2D layer - color texture for HUD rendering
        if (captureIdx == 2) {
            bool hudCopied = frame.Is2DComplete();

            if (side == EyeSide::LEFT) {
                if (!frame.IsStereoRecord() && frame.recordKind == RND_Renderer::CaptureRecordKind::None) {
                    renderer->BeginHudCaptureGeneration(frameIdx);
                }

                if (!frame.TryAcceptCapture(RND_Renderer::CaptureMask_Hud, image, frame.acceptedHudImage, frameIdx, "HUD", false)) {
                    renderer->NoteDuplicateCaptureDropped();
                    returnToLayout();
                    clearFramebuffer(false);
                    return;
                }
                frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();

                if (imguiOverlay) {
                    imguiOverlay->DrawHUDLayerAsBackground(commandBuffer, image, frameIdx);
                    VulkanUtils::DebugPipelineBarrier(commandBuffer);
                }

                if (imguiOverlay) {
                    imguiOverlay->Update();
                    imguiOverlay->Render(frameIdx, false, false);
                    imguiOverlay->DrawAndCopyToImage(commandBuffer, image, frameIdx, false);
                    VulkanUtils::DebugPipelineBarrier(commandBuffer);
                }

                SharedTexture* texture = layer2D->CopyColorToLayer(commandBuffer, image, frameIdx);
                renderer->On2DCopied(frameIdx);

                returnToLayout();
                QueuePendingCopyOperation(commandBuffer, texture);
                return;
            }
            if (side == EyeSide::RIGHT) {
                // render the imgui overlay on the right side
                if (imguiOverlay) {
                    // render imgui, and then copy the framebuffer to the 2D layer
                    imguiOverlay->Render(frameIdx, true, true);
                    imguiOverlay->Update();
                    imguiOverlay->DrawAndCopyToImage(commandBuffer, image, frameIdx, true);

                    returnToLayout();
                    return;
                }

                if (hudCopied) {
                    returnToLayout();
                    return clearFramebuffer(false);
                }
            }
        }
        returnToLayout();
        return;
    }
    else {
        return pDispatch.CmdClearColorImage(commandBuffer, image, imageLayout, pColor, rangeCount, pRanges);
    }
}

void VkDeviceOverrides::CmdClearDepthStencilImage(const vkroots::VkCommandBufferDispatch& pDispatch, VkCommandBuffer commandBuffer, VkImage image, VkImageLayout imageLayout, const VkClearDepthStencilValue* pDepthStencil, uint32_t rangeCount, const VkImageSubresourceRange* pRanges) {
    // check for magical clear values
    // check order and whether there's a match with the magical clear value
    OpenXR::EyeSide side = (OpenXR::EyeSide)-1;
    if (pDepthStencil->depth >= 0.011456789 && pDepthStencil->depth <= 0.013456789) { // 0.0123456789
        side = OpenXR::EyeSide::LEFT;
    }
    else if (pDepthStencil->depth >= 0.153987654 && pDepthStencil->depth <= 0.173987654) { // 0.163987654
        side = OpenXR::EyeSide::RIGHT;
    }

    if (rangeCount == 1 && side != (OpenXR::EyeSide)-1) {
        // stencil value is the frame counter
        const uint32_t frameCounter = pDepthStencil->stencil;
        checkAssert(frameCounter == 0 || frameCounter == 1, "Invalid frame counter for depth clear!");

        auto* renderer = VRManager::instance().XR->GetRenderer();
        if (renderer == nullptr) {
            return pDispatch.CmdClearDepthStencilImage(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
        }
        auto& layer3D = renderer->m_layer3D;

        if (!renderer->IsInitialized()) {
            return;
        }

        Log::print<RENDERING>("[{}] Clearing depth image for 3D layer for {} side", frameCounter, side == OpenXR::EyeSide::LEFT ? "left" : "right");

        // change source image to GENERAL layout
        VulkanUtils::TransitionLayout(commandBuffer, image, imageLayout, VK_IMAGE_LAYOUT_GENERAL);
        VulkanUtils::DebugPipelineBarrier(commandBuffer);

        auto returnToLayout = [&]() {
            VulkanUtils::TransitionLayout(commandBuffer, image, VK_IMAGE_LAYOUT_GENERAL, imageLayout);
            VulkanUtils::DebugPipelineBarrier(commandBuffer);
        };

        RND_Renderer::RenderFrame& frame = renderer->GetFrame(frameCounter);

        if (side == OpenXR::EyeSide::LEFT || side == OpenXR::EyeSide::RIGHT) {
            // 3D layer - depth texture for 3D rendering
            if (s_curr3DDepthImage == VK_NULL_HANDLE) {
                lockImageResolutions.lock();
                if (const auto it = imageResolutions.find(image); it != imageResolutions.end()) {
                    if (it->second.second == VK_FORMAT_D32_SFLOAT) {
                        s_curr3DDepthImage = it->first;
                    }
                }
                lockImageResolutions.unlock();
            }

            if (image != s_curr3DDepthImage) {
                Log::print<RENDERING>("Depth image is not the same as the current 3D depth image! ({} != {})", (void*)image, (void*)s_curr3DDepthImage);
                returnToLayout();
                return;
            }

            if (!frame.IsStereoRecord() || frame.activeStereoGeneration == 0) {
                frame.AddIssue(RND_Renderer::CaptureIssue_UnexpectedSequence);
                Log::print<RENDERING>("[{}] Ignoring {} depth capture before stereo generation start", frameCounter, side == EyeSide::LEFT ? "left-eye" : "right-eye");
                returnToLayout();
                return;
            }

            if (frame.HasFatalIssue()) {
                Log::print<RENDERING>("[{}] Ignoring 3D depth capture because the stereo slot is already invalid", frameCounter);
                returnToLayout();
                return;
            }

            const uint32_t depthMask = side == EyeSide::LEFT ? RND_Renderer::CaptureMask_DepthLeft : RND_Renderer::CaptureMask_DepthRight;
            if (!frame.TryAcceptCapture(depthMask, image, frame.acceptedDepthImages[side], frameCounter, side == EyeSide::LEFT ? "left-eye 3D depth" : "right-eye 3D depth")) {
                renderer->NoteDuplicateCaptureDropped();
                returnToLayout();
                return;
            }
            frame.lastActivityOrdinal = renderer->NextCaptureActivityOrdinal();

            SharedTexture* texture = layer3D->CopyDepthToLayer(side, commandBuffer, image, frameCounter);
            renderer->On3DDepthCopied(side, frameCounter);
            QueuePendingCopyOperation(commandBuffer, texture);
            returnToLayout();
            return;
        }
    }
    else {
        return pDispatch.CmdClearDepthStencilImage(commandBuffer, image, imageLayout, pDepthStencil, rangeCount, pRanges);
    }
}

VkResult VkDeviceOverrides::QueueSubmit(const vkroots::VkQueueDispatch& pDispatch, VkQueue queue, uint32_t submitCount, const VkSubmitInfo* pSubmits, VkFence fence) {
    VkResult result = VK_SUCCESS;

    size_t pendingCopyCount = 0;
    {
        std::lock_guard lk(s_pendingCopyMutex);
        pendingCopyCount = s_pendingCopyOperations.size();
    }

    if (pendingCopyCount == 0) {
        result = pDispatch.QueueSubmit(queue, submitCount, pSubmits, fence);
    }
    else {
        struct ModifiedSubmitInfo_t {
            VkSubmitInfo submitInfoCopy; // Shadow copy of VkSubmitInfo
            std::vector<VkSemaphore> waitSemaphores;
            std::vector<uint64_t> timelineWaitValues;
            std::vector<VkPipelineStageFlags> waitDstStageMasks;
            std::vector<VkSemaphore> signalSemaphores;
            std::vector<uint64_t> timelineSignalValues;

            VkTimelineSemaphoreSubmitInfo timelineSemaphoreSubmitInfo = { VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO };
        };

        // insert (possible) pipeline barriers for any active copy operations
        std::vector<std::vector<PendingCopyOperation>> matchedOperations(submitCount);
        {
            std::lock_guard lk(s_pendingCopyMutex);

            for (auto it = s_pendingCopyOperations.begin(); it != s_pendingCopyOperations.end();) {
                bool matched = false;

                for (uint32_t submitIdx = 0; submitIdx < submitCount && !matched; ++submitIdx) {
                    const VkSubmitInfo& submitInfo = pSubmits[submitIdx];
                    for (uint32_t commandBufferIdx = 0; commandBufferIdx < submitInfo.commandBufferCount; ++commandBufferIdx) {
                        if (submitInfo.pCommandBuffers[commandBufferIdx] == it->commandBuffer) {
                            matchedOperations[submitIdx].push_back(*it);
                            matched = true;
                            break;
                        }
                    }
                }

                if (matched) {
                    it = s_pendingCopyOperations.erase(it);
                }
                else {
                    ++it;
                }
            }
        }

        std::vector<ModifiedSubmitInfo_t> modifiedSubmitInfos{ submitCount };
        std::vector<VkSubmitInfo> shadowSubmits{ submitCount };

        for (uint32_t i = 0; i < submitCount; i++) {
            const VkSubmitInfo& submitInfo = pSubmits[i];
            ModifiedSubmitInfo_t& modifiedSubmitInfo = modifiedSubmitInfos[i];

            // AMD GPU FIX: Create shadow copy of original VkSubmitInfo
            modifiedSubmitInfo.submitInfoCopy = submitInfo;

            // copy old semaphores into new vectors
            if (submitInfo.waitSemaphoreCount > 0 && submitInfo.pWaitSemaphores != nullptr) {
                modifiedSubmitInfo.waitSemaphores.assign(submitInfo.pWaitSemaphores, submitInfo.pWaitSemaphores + submitInfo.waitSemaphoreCount);
            }
            if (submitInfo.waitSemaphoreCount > 0 && submitInfo.pWaitDstStageMask != nullptr) {
                modifiedSubmitInfo.waitDstStageMasks.assign(submitInfo.pWaitDstStageMask, submitInfo.pWaitDstStageMask + submitInfo.waitSemaphoreCount);
            }
            modifiedSubmitInfo.timelineWaitValues.resize(submitInfo.waitSemaphoreCount, 0);

            if (submitInfo.signalSemaphoreCount > 0 && submitInfo.pSignalSemaphores != nullptr) {
                modifiedSubmitInfo.signalSemaphores.assign(submitInfo.pSignalSemaphores, submitInfo.pSignalSemaphores + submitInfo.signalSemaphoreCount);
            }
            modifiedSubmitInfo.timelineSignalValues.resize(submitInfo.signalSemaphoreCount, 0);

            // find timeline semaphore submit info if already present
            const VkTimelineSemaphoreSubmitInfo* existingTimelineInfo = nullptr;

            const VkBaseInStructure* pNextIt = static_cast<const VkBaseInStructure*>(submitInfo.pNext);
            while (pNextIt) {
                if (pNextIt->sType == VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO) {
                    existingTimelineInfo = reinterpret_cast<const VkTimelineSemaphoreSubmitInfo*>(pNextIt);
                    break;
                }
                pNextIt = pNextIt->pNext;
            }

            // copy any existing timeline values into new vectors
            if (existingTimelineInfo) {
                for (uint32_t j = 0; j < existingTimelineInfo->waitSemaphoreValueCount; j++) {
                    modifiedSubmitInfo.timelineWaitValues[j] = existingTimelineInfo->pWaitSemaphoreValues[j];
                }
                for (uint32_t j = 0; j < existingTimelineInfo->signalSemaphoreValueCount; j++) {
                    modifiedSubmitInfo.timelineSignalValues[j] = existingTimelineInfo->pSignalSemaphoreValues[j];
                }
            }

            // Insert timeline semaphores for active copy operations
            for (const PendingCopyOperation& operation : matchedOperations[i]) {
                // Wait for D3D12/XR to finish with the previous shared texture render
                uint64_t waitValue = operation.texture->GetVulkanWaitValue();
                modifiedSubmitInfo.waitSemaphores.emplace_back(operation.texture->GetSemaphoreForWait(waitValue));
                modifiedSubmitInfo.waitDstStageMasks.emplace_back(VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);
                modifiedSubmitInfo.timelineWaitValues.emplace_back(waitValue);

                // Signal to D3D12/XR rendering that the shared texture can be rendered to VR headset
                uint64_t signalValue = operation.texture->GetVulkanSignalValue();
                modifiedSubmitInfo.signalSemaphores.emplace_back(operation.texture->GetSemaphoreForSignal(signalValue));
                modifiedSubmitInfo.timelineSignalValues.emplace_back(signalValue);
            }

            // Update timeline semaphore submit info
            modifiedSubmitInfo.timelineSemaphoreSubmitInfo.waitSemaphoreValueCount = (uint32_t)modifiedSubmitInfo.timelineWaitValues.size();
            modifiedSubmitInfo.timelineSemaphoreSubmitInfo.pWaitSemaphoreValues = modifiedSubmitInfo.timelineWaitValues.data();
            modifiedSubmitInfo.timelineSemaphoreSubmitInfo.signalSemaphoreValueCount = (uint32_t)modifiedSubmitInfo.timelineSignalValues.size();
            modifiedSubmitInfo.timelineSemaphoreSubmitInfo.pSignalSemaphoreValues = modifiedSubmitInfo.timelineSignalValues.data();

            // AMD GPU FIX: Preserve existing pNext chain - prepend our timeline struct
            modifiedSubmitInfo.timelineSemaphoreSubmitInfo.pNext = submitInfo.pNext;

            modifiedSubmitInfo.submitInfoCopy.pNext = &modifiedSubmitInfo.timelineSemaphoreSubmitInfo;
            modifiedSubmitInfo.submitInfoCopy.waitSemaphoreCount = (uint32_t)modifiedSubmitInfo.waitSemaphores.size();
            modifiedSubmitInfo.submitInfoCopy.pWaitSemaphores = modifiedSubmitInfo.waitSemaphores.data();
            modifiedSubmitInfo.submitInfoCopy.pWaitDstStageMask = modifiedSubmitInfo.waitDstStageMasks.data();
            modifiedSubmitInfo.submitInfoCopy.signalSemaphoreCount = (uint32_t)modifiedSubmitInfo.signalSemaphores.size();
            modifiedSubmitInfo.submitInfoCopy.pSignalSemaphores = modifiedSubmitInfo.signalSemaphores.data();

            shadowSubmits[i] = modifiedSubmitInfo.submitInfoCopy;
        }
        result = pDispatch.QueueSubmit(queue, submitCount, shadowSubmits.data(), fence);
    }

    if (result != VK_SUCCESS) {
        Log::print<ERROR>("QueueSubmit failed with error {}", result);
        if (result == VK_ERROR_DEVICE_LOST) {
            LogDeviceFaultInfo();
        }
    }

    return result;
}

VkResult VkDeviceOverrides::QueuePresentKHR(const vkroots::VkQueueDispatch& pDispatch, VkQueue queue, const VkPresentInfoKHR* pPresentInfo) {
    VRManager::instance().XR->ProcessEvents();

    auto* renderer = VRManager::instance().XR->GetRenderer();
    if (renderer && renderer->m_layer3D && renderer->m_layer2D && renderer->m_imguiOverlay) {
        if (renderer->IsInitialized()) {
            renderer->EndFrame();
        }
        renderer->StartFrame();
    }

    VkResult result = pDispatch.QueuePresentKHR(queue, pPresentInfo);
    if (result == VK_ERROR_DEVICE_LOST) {
        Log::print<ERROR>("QueuePresentKHR failed with error {}", result);
        LogDeviceFaultInfo();
    }

    return result;
}
