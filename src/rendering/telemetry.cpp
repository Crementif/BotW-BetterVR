#include "pch.h"

#include "telemetry.h"
#include "d3d12.h"
#include "instance.h"

#include <filesystem>

EyeTelemetry& EyeTelemetry::instance() {
    static EyeTelemetry s_instance;
    return s_instance;
}

void EyeTelemetry::EnsureSlotBuffer(ID3D12Device* device, Slot& slot, int side) {
    if (slot.readback[side] != nullptr) {
        return;
    }

    D3D12_HEAP_PROPERTIES heapProps = {};
    heapProps.Type = D3D12_HEAP_TYPE_READBACK;

    D3D12_RESOURCE_DESC desc = {};
    desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    desc.Width = kEyeBufferSize;
    desc.Height = 1;
    desc.DepthOrArraySize = 1;
    desc.MipLevels = 1;
    desc.Format = DXGI_FORMAT_UNKNOWN;
    desc.SampleDesc.Count = 1;
    desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    checkHResult(device->CreateCommittedResource(&heapProps, D3D12_HEAP_FLAG_NONE, &desc,
                     D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&slot.readback[side])),
        "Failed to create telemetry readback buffer!");

    // readback buffers can stay persistently mapped
    void* mapped = nullptr;
    const D3D12_RANGE readRange = { 0, kEyeBufferSize };
    checkHResult(slot.readback[side]->Map(0, &readRange, &mapped), "Failed to map telemetry readback buffer!");
    slot.mapped[side] = (uint8_t*)mapped;
}

void EyeTelemetry::RecordSample(ID3D12Device* device, ID3D12GraphicsCommandList* cmdList, int side, ID3D12Resource* texture) {
    if (texture == nullptr || side < 0 || side > 1) {
        return;
    }

    Slot& slot = m_slots[m_writeSlot];
    if (slot.pending) {
        // ring wrapped before the GPU caught up; drop this frame's sample
        ++m_stats[side].droppedFrames;
        if (side == 0) {
            ++m_droppedRingFrames;
        }
        return;
    }

    EnsureSlotBuffer(device, slot, side);

    const D3D12_RESOURCE_DESC texDesc = texture->GetDesc();
    const uint32_t texW = (uint32_t)texDesc.Width;
    const uint32_t texH = (uint32_t)texDesc.Height;
    if (texW < kRegionSize * 2 || texH < kRegionSize * 2) {
        return;
    }
    slot.format[side] = texDesc.Format;

    // center + the centers of the four quadrants
    const uint32_t half = kRegionSize / 2;
    const struct { uint32_t x, y; } centers[kRegions] = {
        { texW / 2, texH / 2 },
        { texW / 4, texH / 4 },
        { (3 * texW) / 4, texH / 4 },
        { texW / 4, (3 * texH) / 4 },
        { (3 * texW) / 4, (3 * texH) / 4 },
    };

    for (uint32_t region = 0; region < kRegions; ++region) {
        const uint32_t x = std::min(centers[region].x - half, texW - kRegionSize);
        const uint32_t y = std::min(centers[region].y - half, texH - kRegionSize);

        D3D12_TEXTURE_COPY_LOCATION src = {};
        src.pResource = texture;
        src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
        src.SubresourceIndex = 0;

        D3D12_TEXTURE_COPY_LOCATION dst = {};
        dst.pResource = slot.readback[side].Get();
        dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
        dst.PlacedFootprint.Offset = (uint64_t)region * kRegionSize * kRowPitch;
        dst.PlacedFootprint.Footprint.Format = texDesc.Format;
        dst.PlacedFootprint.Footprint.Width = kRegionSize;
        dst.PlacedFootprint.Footprint.Height = kRegionSize;
        dst.PlacedFootprint.Footprint.Depth = 1;
        dst.PlacedFootprint.Footprint.RowPitch = kRowPitch;

        const D3D12_BOX srcBox = { x, y, 0, x + kRegionSize, y + kRegionSize, 1 };
        cmdList->CopyTextureRegion(&dst, 0, 0, 0, &src, &srcBox);
    }

    slot.sampled[side] = true;
}

void EyeTelemetry::OnFrameSubmitted(uint64_t fenceValue, uint32_t frameNo, uint32_t skipMask) {
    Slot& slot = m_slots[m_writeSlot];
    if (!slot.sampled[0] && !slot.sampled[1]) {
        return;
    }

    slot.pending = true;
    slot.fenceValue = fenceValue;
    slot.frameNo = frameNo;
    slot.skipMask = skipMask;
    m_writeSlot = (m_writeSlot + 1) % kSlots;
}

static float DecodePixelLuma(uint32_t v, DXGI_FORMAT format) {
    float r, g, b;
    if (format == DXGI_FORMAT_R10G10B10A2_UNORM) {
        r = (float)(v & 0x3FF) / 1023.0f;
        g = (float)((v >> 10) & 0x3FF) / 1023.0f;
        b = (float)((v >> 20) & 0x3FF) / 1023.0f;
    }
    else if (format == DXGI_FORMAT_B8G8R8A8_UNORM || format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB) {
        b = (float)(v & 0xFF) / 255.0f;
        g = (float)((v >> 8) & 0xFF) / 255.0f;
        r = (float)((v >> 16) & 0xFF) / 255.0f;
    }
    else {
        r = (float)(v & 0xFF) / 255.0f;
        g = (float)((v >> 8) & 0xFF) / 255.0f;
        b = (float)((v >> 16) & 0xFF) / 255.0f;
    }
    return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

void EyeTelemetry::DecodeSlot(const Slot& slot) {
    constexpr size_t kPixelsPerEye = kRegions * kRegionSize * kRegionSize;
    std::array<std::array<float, kPixelsPerEye>, 2> currentLuma = {};
    std::array<float, 2> temporalDelta = {};
    std::array<bool, 2> flickered = {};

    for (int side = 0; side < 2; ++side) {
        if (!slot.sampled[side] || slot.mapped[side] == nullptr) {
            ++m_stats[side].droppedFrames;
            continue;
        }

        float regionLuma[kRegions];
        float mean = 0.0f, minLuma = 1.0f, maxLuma = 0.0f, edgeEnergy = 0.0f;
        size_t sampleIndex = 0;
        for (uint32_t region = 0; region < kRegions; ++region) {
            const uint8_t* regionBase = slot.mapped[side] + (uint64_t)region * kRegionSize * kRowPitch;
            double regionSum = 0.0;
            for (uint32_t row = 0; row < kRegionSize; ++row) {
                const uint32_t* pixels = reinterpret_cast<const uint32_t*>(regionBase + row * kRowPitch);
                for (uint32_t px = 0; px < kRegionSize; ++px) {
                    const float luma = DecodePixelLuma(pixels[px], slot.format[side]);
                    currentLuma[side][sampleIndex++] = luma;
                    regionSum += luma;
                    if (px > 0) {
                        edgeEnergy += std::abs(luma - DecodePixelLuma(pixels[px - 1], slot.format[side]));
                    }
                    if (row > 0) {
                        const uint32_t* previousRow = reinterpret_cast<const uint32_t*>(regionBase + (row - 1) * kRowPitch);
                        edgeEnergy += std::abs(luma - DecodePixelLuma(previousRow[px], slot.format[side]));
                    }
                }
            }
            regionLuma[region] = (float)(regionSum / (kRegionSize * kRegionSize));
            mean += regionLuma[region];
            minLuma = std::min(minLuma, regionLuma[region]);
            maxLuma = std::max(maxLuma, regionLuma[region]);
        }
        mean /= (float)kRegions;
        edgeEnergy /= (float)(kRegions * (2 * kRegionSize * kRegionSize - 2 * kRegionSize));

        if (m_hasPrevious[side]) {
            double delta = 0.0;
            for (size_t i = 0; i < kPixelsPerEye; ++i) {
                delta += std::abs(currentLuma[side][i] - m_previousLuma[side][i]);
            }
            temporalDelta[side] = (float)(delta / kPixelsPerEye);
        }

        FrameState newState = FrameState::NORMAL;
        if (mean > 0.90f && minLuma > 0.80f) {
            newState = FrameState::WHITE;
        }
        else if (mean < 0.02f && maxLuma < 0.05f) {
            newState = FrameState::BLACK;
        }

        EyeStats& stats = m_stats[side];
        const FrameState previousState = stats.state;
        switch (newState) {
            case FrameState::WHITE: ++stats.whiteFrames; break;
            case FrameState::BLACK: ++stats.blackFrames; break;
            default: ++stats.normalFrames; break;
        }
        stats.meanLuma = mean;
        stats.minRegionLuma = minLuma;
        stats.maxRegionLuma = maxLuma;
        stats.temporalDelta = temporalDelta[side];
        stats.edgeEnergy = edgeEnergy;
        stats.lastSampleFrame = slot.frameNo;
        ++stats.sampledFrames;

        // A large full-region temporal jump is the signal humans describe as flicker.
        // Blank/non-blank transitions are always incidents; ordinary scene motion uses
        // a higher delta threshold to avoid turning camera movement into false alarms.
        flickered[side] = m_hasPrevious[side] &&
            ((newState != previousState && (newState != FrameState::NORMAL || previousState != FrameState::UNKNOWN)) ||
             temporalDelta[side] > 0.22f);
        if (flickered[side]) {
            ++stats.flickerEvents;
            stats.lastFlickerFrame = slot.frameNo;
        }
        if (m_hasPrevious[side] && temporalDelta[side] < 0.00025f) {
            ++stats.frozenFrames;
        }

        if (newState != stats.state) {
            ++stats.transitions;
            stats.lastTransitionFrame = slot.frameNo;
            Log::print<INFO>("[telemetry] eye={} {}->{} at frame {} (mask=0x{:X}) luma mean={:.3f} min={:.3f} max={:.3f}",
                side == 0 ? "L" : "R", StateName(stats.state), StateName(newState), slot.frameNo, slot.skipMask,
                mean, minLuma, maxLuma);
            stats.state = newState;
        }
        m_previousLuma[side] = currentLuma[side];
        m_hasPrevious[side] = true;
    }

    if (slot.sampled[0] && slot.sampled[1]) {
        double difference = 0.0;
        uint64_t nearIdentical = 0;
        for (size_t i = 0; i < kPixelsPerEye; ++i) {
            const float delta = std::abs(currentLuma[0][i] - currentLuma[1][i]);
            difference += delta;
            nearIdentical += delta < (1.0f / 255.0f) ? 1 : 0;
        }
        m_stereoStats.meanAbsDifference = (float)(difference / kPixelsPerEye);
        m_stereoStats.nearIdenticalFraction = (float)nearIdentical / kPixelsPerEye;
        ++m_stereoStats.sampledFrames;

        if (m_stereoStats.nearIdenticalFraction > 0.995f) {
            ++m_stereoStats.nearIdenticalFrames;
        }
        if (flickered[0] != flickered[1] || std::abs(temporalDelta[0] - temporalDelta[1]) > 0.16f) {
            ++m_stereoStats.asymmetricFlickerEvents;
            m_stereoStats.lastAnomalyFrame = slot.frameNo;
            m_stereoStats.lastAnomaly = "ASYMMETRIC_FLICKER";
            Log::print<WARNING>("[telemetry] stereo anomaly={} frame={} deltaL={:.4f} deltaR={:.4f} eyeMAD={:.4f}",
                m_stereoStats.lastAnomaly, slot.frameNo, temporalDelta[0], temporalDelta[1], m_stereoStats.meanAbsDifference);
        }
        else if ((temporalDelta[0] < 0.00025f && temporalDelta[1] > 0.02f) ||
                 (temporalDelta[1] < 0.00025f && temporalDelta[0] > 0.02f)) {
            ++m_stereoStats.frozenEyeEvents;
            m_stereoStats.lastAnomalyFrame = slot.frameNo;
            m_stereoStats.lastAnomaly = "FROZEN_EYE";
        }
    }
    else {
        ++m_stereoStats.incompleteFrames;
    }
}

void EyeTelemetry::Collect(uint64_t completedFenceValue) {
    for (auto& slot : m_slots) {
        if (!slot.pending || slot.fenceValue > completedFenceValue) {
            continue;
        }
        DecodeSlot(slot);
        slot.pending = false;
        slot.sampled[0] = false;
        slot.sampled[1] = false;
    }
}

// ---------------------------------------------------------------------------
// Full-texture BMP dumps

static void WriteBmp24(const std::filesystem::path& path, uint32_t width, uint32_t height, const std::vector<uint8_t>& bgr) {
    const uint32_t rowSize = ((width * 3 + 3) / 4) * 4;
    const uint32_t imageSize = rowSize * height;

#pragma pack(push, 1)
    struct {
        uint16_t magic = 0x4D42;
        uint32_t fileSize = 0;
        uint32_t reserved = 0;
        uint32_t dataOffset = 54;
        uint32_t headerSize = 40;
        int32_t width = 0;
        int32_t height = 0;
        uint16_t planes = 1;
        uint16_t bpp = 24;
        uint32_t compression = 0;
        uint32_t imageSize = 0;
        int32_t ppmX = 2835;
        int32_t ppmY = 2835;
        uint32_t colors = 0;
        uint32_t importantColors = 0;
    } header;
#pragma pack(pop)
    header.fileSize = 54 + imageSize;
    header.width = (int32_t)width;
    header.height = (int32_t)height; // positive: bottom-up
    header.imageSize = imageSize;

    FILE* file = nullptr;
    if (_wfopen_s(&file, path.wstring().c_str(), L"wb") != 0 || file == nullptr) {
        Log::print<ERROR>("[telemetry] failed to open dump file {}", path.string());
        return;
    }
    fwrite(&header, sizeof(header), 1, file);
    std::vector<uint8_t> padRow(rowSize, 0);
    for (int32_t row = (int32_t)height - 1; row >= 0; --row) {
        memcpy(padRow.data(), bgr.data() + (size_t)row * width * 3, (size_t)width * 3);
        fwrite(padRow.data(), rowSize, 1, file);
    }
    fclose(file);
}

int EyeTelemetry::DumpEyeTextures(RND_D3D12* d3d12, ID3D12Resource* colorL, ID3D12Resource* colorR,
    uint32_t frameNo, D3D12_RESOURCE_STATES initialState, const char* namePrefix) {
    const std::filesystem::path dumpDirectory = std::filesystem::path("BetterVR_dumps") /
        std::format("session_{}", GetCurrentProcessId());
    std::error_code ec;
    std::filesystem::create_directories(dumpDirectory, ec);

    int written = 0;
    const std::pair<ID3D12Resource*, std::string> textures[] = {
        { colorL, std::format("{}_L", namePrefix) }, { colorR, std::format("{}_R", namePrefix) }
    };

    for (const auto& [texture, name] : textures) {
        if (texture == nullptr) {
            continue;
        }

        const D3D12_RESOURCE_DESC desc = texture->GetDesc();
        const uint32_t width = (uint32_t)desc.Width;
        const uint32_t height = desc.Height;

        // Ask D3D12 for the exact linear footprint instead of relying on hand-rolled
        // alignment assumptions. (Depth is intentionally not in the texture list;
        // runtime-owned D32 swapchains need a shader conversion before readback.)
        D3D12_PLACED_SUBRESOURCE_FOOTPRINT footprint = {};
        UINT64 bufferSize = 0;
        d3d12->GetDevice()->GetCopyableFootprints(&desc, 0, 1, 0, &footprint, nullptr, nullptr, &bufferSize);
        const uint32_t rowPitch = footprint.Footprint.RowPitch;

        D3D12_HEAP_PROPERTIES heapProps = {};
        heapProps.Type = D3D12_HEAP_TYPE_READBACK;
        D3D12_RESOURCE_DESC bufDesc = {};
        bufDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
        bufDesc.Width = bufferSize;
        bufDesc.Height = 1;
        bufDesc.DepthOrArraySize = 1;
        bufDesc.MipLevels = 1;
        bufDesc.SampleDesc.Count = 1;
        bufDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

        ComPtr<ID3D12Resource> readback;
        if (FAILED(d3d12->GetDevice()->CreateCommittedResource(&heapProps, D3D12_HEAP_FLAG_NONE, &bufDesc,
                D3D12_RESOURCE_STATE_COPY_DEST, nullptr, IID_PPV_ARGS(&readback)))) {
            Log::print<ERROR>("[telemetry] dump: failed to create readback buffer for {}", name);
            continue;
        }

        {
            RND_D3D12::CommandContext<true> copyContext(d3d12, [&](RND_D3D12::CommandContext<true>* context) {
                const bool isDepth = desc.Format == DXGI_FORMAT_D32_FLOAT || desc.Format == DXGI_FORMAT_R32_FLOAT;
                D3D12_RESOURCE_BARRIER transition = {};
                transition.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
                transition.Transition.pResource = texture;
                transition.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
                transition.Transition.StateBefore = initialState;
                transition.Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
                context->GetRecordList()->ResourceBarrier(1, &transition);

                D3D12_TEXTURE_COPY_LOCATION src = {};
                src.pResource = texture;
                src.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;
                src.SubresourceIndex = 0;

                D3D12_TEXTURE_COPY_LOCATION dst = {};
                dst.pResource = readback.Get();
                dst.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
                dst.PlacedFootprint = footprint;

                context->GetRecordList()->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);

                std::swap(transition.Transition.StateBefore, transition.Transition.StateAfter);
                context->GetRecordList()->ResourceBarrier(1, &transition);
            });
        } // blocking context: copy has completed here

        uint8_t* mapped = nullptr;
        const D3D12_RANGE readRange = { 0, bufferSize };
        if (FAILED(readback->Map(0, &readRange, (void**)&mapped)) || mapped == nullptr) {
            Log::print<ERROR>("[telemetry] dump: failed to map readback buffer for {}", name);
            continue;
        }

        std::vector<uint8_t> bgr((size_t)width * height * 3);
        const bool isDepth = desc.Format == DXGI_FORMAT_D32_FLOAT || desc.Format == DXGI_FORMAT_R32_FLOAT;
        for (uint32_t row = 0; row < height; ++row) {
            const uint32_t* srcRow = (const uint32_t*)(mapped + (uint64_t)row * rowPitch);
            uint8_t* dstRow = bgr.data() + (size_t)row * width * 3;
            for (uint32_t px = 0; px < width; ++px) {
                uint8_t r, g, b;
                if (isDepth) {
                    float depth = 0.0f;
                    memcpy(&depth, &srcRow[px], sizeof(float));
                    // game depth is reversed and heavily front-loaded; gamma stretch for visibility
                    const uint8_t gray = (uint8_t)(glm::clamp(std::pow((double)depth, 0.22), 0.0, 1.0) * 255.0);
                    r = g = b = gray;
                }
                else {
                    const uint32_t v = srcRow[px];
                    if (desc.Format == DXGI_FORMAT_R10G10B10A2_UNORM) {
                        r = (uint8_t)(((v >> 0) & 0x3FF) >> 2);
                        g = (uint8_t)(((v >> 10) & 0x3FF) >> 2);
                        b = (uint8_t)(((v >> 20) & 0x3FF) >> 2);
                    }
                    else if (desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM || desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB) {
                        b = (uint8_t)(v & 0xFF);
                        g = (uint8_t)((v >> 8) & 0xFF);
                        r = (uint8_t)((v >> 16) & 0xFF);
                    }
                    else {
                        r = (uint8_t)(v & 0xFF);
                        g = (uint8_t)((v >> 8) & 0xFF);
                        b = (uint8_t)((v >> 16) & 0xFF);
                    }
                }
                dstRow[px * 3 + 0] = b;
                dstRow[px * 3 + 1] = g;
                dstRow[px * 3 + 2] = r;
            }
        }
        readback->Unmap(0, nullptr);

        const std::filesystem::path fileName = dumpDirectory / std::format("frame{}_{}.bmp", frameNo, name);
        WriteBmp24(fileName, width, height, bgr);
        ++written;
    }

    Log::print<INFO>("[telemetry] dumped {} final textures for frame {} to {}", written, frameNo, dumpDirectory.string());
    return written;
}
