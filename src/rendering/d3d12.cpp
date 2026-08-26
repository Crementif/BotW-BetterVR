#include "pch.h"

#include "d3d12.h"
#include "instance.h"
#include "utils/d3d12_utils.h"

#include "shader.h"

#define ENABLE_VALIDATION_LAYER FALSE

RND_D3D12::RND_D3D12() {
    UINT dxgiFactoryFlags = 0;
#if ENABLE_VALIDATION_LAYER
    ComPtr<ID3D12Debug> debugController0;
    ComPtr<ID3D12Debug1> debugController1;
    D3D12GetDebugInterface(IID_PPV_ARGS(&debugController0));
    debugController0->QueryInterface(IID_PPV_ARGS(&debugController1));
    debugController0->EnableDebugLayer();
    debugController1->SetEnableGPUBasedValidation(true);
    debugController1->SetEnableSynchronizedCommandQueueValidation(true);
    debugController1->EnableDebugLayer();
    dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
#endif

    ComPtr<IDXGIFactory6> dxgiFactory;
    checkHResult(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&dxgiFactory)), "Failed to create the DXGIFactory!");

    ComPtr<IDXGIAdapter1> dxgiAdapter;
    for (UINT idx = 0; SUCCEEDED(dxgiFactory->EnumAdapterByGpuPreference(idx, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&dxgiAdapter))); idx++) {
        DXGI_ADAPTER_DESC1 adapterDesc;
        dxgiAdapter->GetDesc1(&adapterDesc);
        if (memcmp(&adapterDesc.AdapterLuid, &VRManager::instance().XR->m_capabilities.adapter, sizeof(LUID)) == 0) {
            char descriptionStr[256 + 1];
            WideCharToMultiByte(CP_UTF8, 0, adapterDesc.Description, -1, descriptionStr, 256, NULL, NULL);
            Log::print<INFO>("Using {} as the VR GPU!", descriptionStr);
            break;
        }
    }
    checkAssert(dxgiAdapter, "Failed to find the VR GPU selected by OpenXR!");

    checkHResult(D3D12CreateDevice(dxgiAdapter.Get(), VRManager::instance().XR->m_capabilities.minFeatureLevel, IID_PPV_ARGS(&m_device)), "Failed to create D3D12 device!");

#if ENABLE_VALIDATION_LAYER
    // Disable specific validation messages to hide spam caused by SteamVR
    ComPtr<ID3D12InfoQueue> pInfoQueue;
    checkHResult(m_device->QueryInterface(IID_PPV_ARGS(&pInfoQueue)), "Failed to get D3D12 info queue!");

    D3D12_MESSAGE_SEVERITY severities[] = {
        D3D12_MESSAGE_SEVERITY_CORRUPTION,
        D3D12_MESSAGE_SEVERITY_ERROR,
        D3D12_MESSAGE_SEVERITY_WARNING,
        //D3D12_MESSAGE_SEVERITY_INFO,
        D3D12_MESSAGE_SEVERITY_MESSAGE,
    };

    D3D12_MESSAGE_ID denyIds[] = { D3D12_MESSAGE_ID_REFLECTSHAREDPROPERTIES_INVALIDOBJECT };
    D3D12_INFO_QUEUE_FILTER filter = {};
    filter.DenyList.NumIDs = (UINT)std::size(denyIds);
    filter.DenyList.pIDList = denyIds;
    filter.AllowList.NumSeverities = (UINT)std::size(severities);
    filter.AllowList.pSeverityList = severities;
    pInfoQueue->PushStorageFilter(&filter);
#endif

    D3D12_COMMAND_QUEUE_DESC queueDesc = {
        .Type = D3D12_COMMAND_LIST_TYPE_DIRECT,
        .Flags = D3D12_COMMAND_QUEUE_FLAG_NONE
    };
    checkHResult(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_queue)), "Failed to create D3D12 command queue!");

    checkHResult(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)), "Failed to create D3D12 queue fence!");
    m_fenceEvent = CreateEventA(nullptr, FALSE, FALSE, nullptr);
    checkAssert(m_fenceEvent != NULL, "Failed to create D3D12 queue event!");

    auto createReusableCommandList = [this](ID3D12CommandAllocator* allocator, ComPtr<ID3D12GraphicsCommandList>& commandList) {
        checkHResult(m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, allocator, nullptr, IID_PPV_ARGS(commandList.ReleaseAndGetAddressOf())), "Failed to create reusable D3D12 command list!");
        checkHResult(commandList->Close(), "Failed to close reusable D3D12 command list after creation!");
    };

    for (auto& frameContext : m_frameContexts) {
        checkHResult(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(frameContext.allocator.ReleaseAndGetAddressOf())), "Failed to create frame command allocator!");
        for (auto& commandList : frameContext.commandLists) {
            createReusableCommandList(frameContext.allocator.Get(), commandList);
        }
    }

    checkHResult(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(m_immediateContext.allocator.ReleaseAndGetAddressOf())), "Failed to create immediate command allocator!");
    createReusableCommandList(m_immediateContext.allocator.Get(), m_immediateContext.commandList);
}

RND_D3D12::~RND_D3D12() {
    if (m_fenceEvent != nullptr) {
        CloseHandle(m_fenceEvent);
        m_fenceEvent = nullptr;
    }
}

void RND_D3D12::StartFrame() {
    m_currentFrameContextIndex = (m_currentFrameContextIndex + 1) % kFrameContextCount;

    FrameContext& frameContext = GetCurrentFrameContext();
    if (frameContext.completionFenceValue != 0) {
        WaitForQueueFence(frameContext.completionFenceValue);
        frameContext.completionFenceValue = 0;
    }

    checkHResult(frameContext.allocator->Reset(), "Failed to reset frame command allocator!");
    frameContext.nextCommandListIndex = 0;
}

void RND_D3D12::EndFrame() {
    GetCurrentFrameContext().completionFenceValue = SignalQueueFence();
}

RND_D3D12::FrameContext& RND_D3D12::GetCurrentFrameContext() {
    return m_frameContexts[m_currentFrameContextIndex];
}

ID3D12GraphicsCommandList* RND_D3D12::AcquireFrameCommandList() {
    FrameContext& frameContext = GetCurrentFrameContext();
    checkAssert(frameContext.nextCommandListIndex < frameContext.commandLists.size(), "Exceeded the reusable D3D12 frame command list pool!");

    ID3D12GraphicsCommandList* commandList = frameContext.commandLists[frameContext.nextCommandListIndex++].Get();
    checkHResult(commandList->Reset(frameContext.allocator.Get(), nullptr), "Failed to reset reusable frame command list!");
    return commandList;
}

ID3D12GraphicsCommandList* RND_D3D12::AcquireImmediateCommandList() {
    checkHResult(m_immediateContext.allocator->Reset(), "Failed to reset immediate command allocator!");
    checkHResult(m_immediateContext.commandList->Reset(m_immediateContext.allocator.Get(), nullptr), "Failed to reset immediate command list!");
    return m_immediateContext.commandList.Get();
}

void RND_D3D12::ExecuteCommandList(ID3D12GraphicsCommandList* commandList) {
    ID3D12CommandList* collectedLists[] = { commandList };
    m_queue->ExecuteCommandLists((UINT)std::size(collectedLists), collectedLists);
}

uint64_t RND_D3D12::SignalQueueFence() {
    const uint64_t fenceValue = m_nextFenceValue++;
    checkHResult(m_queue->Signal(m_fence.Get(), fenceValue), "Failed to signal the reusable D3D12 queue fence!");
    return fenceValue;
}

void RND_D3D12::WaitForQueueFence(uint64_t fenceValue) {
    if (m_fence->GetCompletedValue() >= fenceValue) {
        return;
    }

    checkHResult(m_fence->SetEventOnCompletion(fenceValue, m_fenceEvent), "Failed to arm the reusable D3D12 queue event!");
    WaitForSingleObject(m_fenceEvent, INFINITE);
}

template <bool depth>
RND_D3D12::PresentPipeline<depth>::PresentPipeline(RND_Renderer* pRenderer): m_renderer(pRenderer) {
    // This needs to know the format of the swapchain images, thus needs to wait until the swapchain images are created
    m_vertexShader = D3D12Utils::CompileShader(depth ? presentDepthHLSL : presentHLSL, "VSMain", "vs_5_1");
    m_pixelShader = D3D12Utils::CompileShader(depth ? presentDepthHLSL : presentHLSL, "PSMain", "ps_5_1");

    auto createSignature = [this]() {
        // clang-format off
        D3D12_DESCRIPTOR_RANGE pixelRange[] = {
            // Input textures
            {
                .RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV,
                .NumDescriptors = (UINT)this->m_attachmentHandles.size(),
                .BaseShaderRegister = 0,
                .RegisterSpace = 0,
                .OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND
            }
        };

        D3D12_ROOT_PARAMETER rootParams[2] = {};
        rootParams[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        rootParams[0].DescriptorTable.NumDescriptorRanges = (UINT)std::size(pixelRange);
        rootParams[0].DescriptorTable.pDescriptorRanges = pixelRange;
        rootParams[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParams[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParams[1].Descriptor.ShaderRegister = 1;
        rootParams[1].Descriptor.RegisterSpace = 0;
        rootParams[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
        // clang-format on

        D3D12_STATIC_SAMPLER_DESC textureSampler = {
            .Filter = D3D12_FILTER_MIN_MAG_MIP_POINT,
            .AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .MipLODBias = 0,
            .MaxAnisotropy = 0,
            .ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER,
            .BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
            .MinLOD = 0.0f,
            .MaxLOD = D3D12_FLOAT32_MAX,
            .ShaderRegister = 0,
            .RegisterSpace = 0,
            .ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL
        };

        D3D12_ROOT_SIGNATURE_DESC rootSigDesc = {
            .NumParameters = (UINT)std::size(rootParams),
            .pParameters = rootParams,
            .NumStaticSamplers = 1,
            .pStaticSamplers = &textureSampler,
            .Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT
        };

        ComPtr<ID3DBlob> serializedBlob;
        ComPtr<ID3DBlob> error;
        if (HRESULT res = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1_0, &serializedBlob, &error); FAILED(res)) {
            checkHResult(res, std::format("Failed to serialize root signature! {}", std::string((const char*)error->GetBufferPointer(), error->GetBufferSize())).c_str());
        }

        ComPtr<ID3D12RootSignature> rootSigBlob;
        checkHResult(VRManager::instance().D3D12->GetDevice()->CreateRootSignature(0, serializedBlob->GetBufferPointer(), serializedBlob->GetBufferSize(), IID_PPV_ARGS(&rootSigBlob)), "Failed to create root signature!");
        return rootSigBlob;
    };

    m_attachmentHeap = D3D12Utils::CreateDescriptorHeap(VRManager::instance().D3D12->GetDevice(), D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, true, (UINT)m_attachmentHandles.size());
    m_targetHeap = D3D12Utils::CreateDescriptorHeap(VRManager::instance().D3D12->GetDevice(), D3D12_DESCRIPTOR_HEAP_TYPE_RTV, false, (UINT)m_targetHandles.size());
    if constexpr (depth) {
        m_depthHeap = D3D12Utils::CreateDescriptorHeap(VRManager::instance().D3D12->GetDevice(), D3D12_DESCRIPTOR_HEAP_TYPE_DSV, false, (UINT)m_depthTargetHandles.size());
    }

    for (uint32_t i = 0; i < m_attachmentHandles.size(); i++) {
        m_attachmentHandles[i] = m_attachmentHeap->GetCPUDescriptorHandleForHeapStart();
        m_attachmentHandles[i].ptr += (i * VRManager::instance().D3D12->GetDevice()->GetDescriptorHandleIncrementSize(m_attachmentHeap->GetDesc().Type));
    }

    for (uint32_t i = 0; i < m_targetHandles.size(); i++) {
        m_targetHandles[i] = m_targetHeap->GetCPUDescriptorHandleForHeapStart();
        m_targetHandles[i].ptr += (i * VRManager::instance().D3D12->GetDevice()->GetDescriptorHandleIncrementSize(m_targetHeap->GetDesc().Type));
    }

    for (uint32_t i = 0; i < m_depthTargetHandles.size(); i++) {
        m_depthTargetHandles[i] = m_depthHeap->GetCPUDescriptorHandleForHeapStart();
        m_depthTargetHandles[i].ptr += (i * VRManager::instance().D3D12->GetDevice()->GetDescriptorHandleIncrementSize(m_depthHeap->GetDesc().Type));
    }

    m_signature = createSignature();

    // upload screen indices
    ComPtr<ID3D12Resource> screenIndicesStaging;
    {
        ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
        RND_D3D12::CommandContext<true> uploadBufferContext(VRManager::instance().D3D12.get(), [this, device, &screenIndicesStaging](RND_D3D12::CommandContext<true>* context) {
            m_screenIndicesBuffer = D3D12Utils::CreateConstantBuffer(device, D3D12_HEAP_TYPE_DEFAULT, sizeof(screenIndices));

            screenIndicesStaging = D3D12Utils::CreateConstantBuffer(device, D3D12_HEAP_TYPE_UPLOAD, sizeof(screenIndices));
            void* data;
            const D3D12_RANGE readRange = { .Begin = 0, .End = 0 };
            checkHResult(screenIndicesStaging->Map(0, &readRange, &data), "Failed to map memory for screen indices buffer!");
            memcpy(data, screenIndices, sizeof(screenIndices));
            screenIndicesStaging->Unmap(0, nullptr);

            context->GetRecordList()->CopyBufferRegion(m_screenIndicesBuffer.Get(), 0, screenIndicesStaging.Get(), 0, sizeof(screenIndices));
            m_screenIndicesView = {
                .BufferLocation = m_screenIndicesBuffer->GetGPUVirtualAddress(),
                .SizeInBytes = sizeof(screenIndices),
                .Format = DXGI_FORMAT_R16_UINT
            };
        });
    }
}


// These change the CPU handles that'll later be used for binding the actual assets
template <bool depth>
void RND_D3D12::PresentPipeline<depth>::BindAttachment(uint32_t attachmentIdx, ID3D12Resource* srcTexture, DXGI_FORMAT overwriteFormat) {
    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = overwriteFormat != DXGI_FORMAT_UNKNOWN ? overwriteFormat : srcTexture->GetDesc().Format;
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Texture2D.MipLevels = 1;
    VRManager::instance().D3D12->GetDevice()->CreateShaderResourceView(srcTexture, &srvDesc, m_attachmentHandles[attachmentIdx]);
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::BindTarget(uint32_t targetIdx, ID3D12Resource* dstTexture, DXGI_FORMAT overwriteFormat) {
    D3D12_RENDER_TARGET_VIEW_DESC rtvDesc = {};
    rtvDesc.Format = overwriteFormat != DXGI_FORMAT_UNKNOWN ? overwriteFormat : dstTexture->GetDesc().Format;
    rtvDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    VRManager::instance().D3D12->GetDevice()->CreateRenderTargetView(dstTexture, &rtvDesc, m_targetHandles[targetIdx]);

    if (rtvDesc.Format != m_targetFormats[targetIdx]) {
        m_targetFormats[targetIdx] = rtvDesc.Format;
        RecreatePipeline();
    }
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::BindDepthTarget(ID3D12Resource* dstTexture, DXGI_FORMAT overwriteFormat) {
    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = overwriteFormat != DXGI_FORMAT_UNKNOWN ? overwriteFormat : dstTexture->GetDesc().Format;
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Flags = D3D12_DSV_FLAG_NONE;
    VRManager::instance().D3D12->GetDevice()->CreateDepthStencilView(dstTexture, &dsvDesc, m_depthTargetHandles[0]);

    if (dsvDesc.Format != m_targetFormats.back()) {
        m_targetFormats.back() = dsvDesc.Format;
        RecreatePipeline();
    }
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::BindSettings(float screenWidth, float screenHeight, const RenderUtils::UvTransform& uvTransform) {
    m_renderWidth = screenWidth;
    m_renderHeight = screenHeight;
    m_uvTransform = uvTransform;
    m_settingsBuffer = D3D12Utils::CreateConstantBuffer(VRManager::instance().D3D12->GetDevice(), D3D12_HEAP_TYPE_UPLOAD, sizeof(presentSettings));
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::UpdateSettingsBuffer(ID3D12Resource* swapchain) {
    presentSettings settings = {
        .renderWidth = m_renderWidth,
        .renderHeight = m_renderHeight,
        .swapchainWidth = static_cast<float>(swapchain->GetDesc().Width),
        .swapchainHeight = static_cast<float>(swapchain->GetDesc().Height),
        .uvOffsetX = m_uvTransform.offsetX,
        .uvOffsetY = m_uvTransform.offsetY,
        .uvScaleX = m_uvTransform.scaleX,
        .uvScaleY = m_uvTransform.scaleY,
        .customFadeAmount = 0.0f,
        .customFadeColorR = 0.0f,
        .customFadeColorG = 0.0f,
        .customFadeColorB = 0.0f,
        .isFadeActive = 0.0f,
        .hudMaxAlpha = 1.0f,
    };

    if constexpr (depth) {
        const RND_Renderer::CustomFade fade = m_renderer != nullptr ? m_renderer->GetCustomFade() : RND_Renderer::CustomFade{};
        settings.customFadeAmount = fade.amount;
        settings.customFadeColorR = fade.color.x;
        settings.customFadeColorG = fade.color.y;
        settings.customFadeColorB = fade.color.z;
        settings.isFadeActive = m_renderer != nullptr && m_renderer->IsFadeActive() ? 1.0f : 0.0f;
    }
    else {
        settings.hudMaxAlpha = m_renderer != nullptr ? m_renderer->GetHudMaxAlpha() : 1.0f;
    }

    void* data = nullptr;
    const D3D12_RANGE readRange = { .Begin = 0, .End = 0 };
    checkHResult(m_settingsBuffer->Map(0, &readRange, &data), "Failed to map memory for present settings buffer!");
    memcpy(data, &settings, sizeof(settings));
    m_settingsBuffer->Unmap(0, nullptr);
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::RecreatePipeline() {
    // AMD GPU FIX: Don't declare SV_InstanceID/SV_VertexID in the input layout.
    // These are system-generated values, not vertex buffer inputs.
    // AMD strictly enforces this - it will try to read from an unbound vertex buffer.
    // NVIDIA ignores the "SV_" prefix and provides system values directly.

    D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
    psoDesc.InputLayout = { nullptr, 0 };  // Empty input layout - shader generates system values
    psoDesc.pRootSignature = m_signature.Get();
    psoDesc.VS = { m_vertexShader->GetBufferPointer(), m_vertexShader->GetBufferSize() };
    psoDesc.PS = { m_pixelShader->GetBufferPointer(), m_pixelShader->GetBufferSize() };
    psoDesc.BlendState = {
        .AlphaToCoverageEnable = false,
        .IndependentBlendEnable = false
    };
    for (size_t i = 0; i < std::size(psoDesc.BlendState.RenderTarget); i++) {
        psoDesc.BlendState.RenderTarget[i] = {
            .BlendEnable = false,

            .SrcBlend = D3D12_BLEND_ONE,
            .DestBlend = D3D12_BLEND_ZERO,
            .BlendOp = D3D12_BLEND_OP_ADD,

            .SrcBlendAlpha = D3D12_BLEND_ONE,
            .DestBlendAlpha = D3D12_BLEND_ZERO,
            .BlendOpAlpha = D3D12_BLEND_OP_ADD,

            .LogicOp = D3D12_LOGIC_OP_NOOP,
            .RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL
        };
    }
    psoDesc.SampleMask = UINT_MAX;
    psoDesc.RasterizerState = {
        .FillMode = D3D12_FILL_MODE_SOLID,
        .CullMode = D3D12_CULL_MODE_BACK,
        .FrontCounterClockwise = false,
        .DepthBias = D3D12_DEFAULT_DEPTH_BIAS,
        .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
        .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
        .DepthClipEnable = true,
        .MultisampleEnable = false,
        .AntialiasedLineEnable = false,
        .ForcedSampleCount = 0,
        .ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF
    };
    // clang-format off
    psoDesc.DepthStencilState = {
        .DepthEnable = depth,
        .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL,
        .DepthFunc = D3D12_COMPARISON_FUNC_ALWAYS,
        .StencilEnable = false,
        .StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK,
        .StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK,
        .FrontFace = {
            .StencilFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilPassOp = D3D12_STENCIL_OP_KEEP,
            .StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS
        }
    };
    // clang-format on
    psoDesc.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_0xFFFF;
    psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    psoDesc.NumRenderTargets = 1;
    for (uint32_t i = 0; i < m_targetHandles.size(); i++) {
        psoDesc.RTVFormats[i] = m_targetFormats[i];
    }
    psoDesc.DSVFormat = m_targetFormats.back();
    psoDesc.SampleDesc.Count = 1;
    psoDesc.SampleDesc.Quality = 0;
    psoDesc.NodeMask = 0;
    psoDesc.CachedPSO = { nullptr, 0 };
    psoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&m_pipelineState)), "Failed to create graphics pipeline state!");
}

template <bool depth>
void RND_D3D12::PresentPipeline<depth>::Render(ID3D12GraphicsCommandList* cmdList, ID3D12Resource* swapchain) {
    cmdList->SetPipelineState(m_pipelineState.Get());
    cmdList->SetGraphicsRootSignature(m_signature.Get());

    // set framebuffer
    D3D12_VIEWPORT viewportSize = { 0.0f, 0.0f, (float)swapchain->GetDesc().Width, (float)swapchain->GetDesc().Height, 0.0f, 1.0f };
    cmdList->RSSetViewports(1, &viewportSize);

    D3D12_RECT scissorRect = { 0, 0, (LONG)swapchain->GetDesc().Width, (LONG)swapchain->GetDesc().Height };
    cmdList->RSSetScissorRects(1, &scissorRect);

    // set settings
    checkAssert(m_settingsBuffer != nullptr, "Failed to present texture since graphics pipeline hasn't bound some settings yet!");
    UpdateSettingsBuffer(swapchain);
    cmdList->SetGraphicsRootConstantBufferView(1, m_settingsBuffer->GetGPUVirtualAddress());

    // set shared texture
    ID3D12DescriptorHeap* heaps[] = { m_attachmentHeap.Get() };
    cmdList->SetDescriptorHeaps((UINT)std::size(heaps), heaps);

    cmdList->SetGraphicsRootDescriptorTable(0, m_attachmentHeap->GetGPUDescriptorHandleForHeapStart());

    // set render target
    cmdList->OMSetRenderTargets(1, &m_targetHandles[0], true, depth ? &m_depthTargetHandles[0] : nullptr);

    // draw
    //float clearColor[4] = { textureIdx == 0 ? 0.0f, 0.2f, 0.4f, 1.0f : 0.4f, 0.2f, 0.0f, 1.0f };
    //cmdList->ClearRenderTargetView(renderTargetView, clearColor, 0, nullptr);

    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    cmdList->IASetIndexBuffer(&m_screenIndicesView);
    cmdList->DrawIndexedInstanced((UINT)std::size(screenIndices), 1, 0, 0, 0);
}

RND_D3D12::DebugDrawPipeline::DebugDrawPipeline() {
    m_vertexShader = D3D12Utils::CompileShader(debugDrawLineHLSL, "VSMain", "vs_5_1");
    m_pixelShader = D3D12Utils::CompileShader(debugDrawLineHLSL, "PSMain", "ps_5_1");

    auto createSignature = []() {
        D3D12_DESCRIPTOR_RANGE sceneDepthRange[] = {
            {
                .RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV,
                .NumDescriptors = 1,
                .BaseShaderRegister = 0,
                .RegisterSpace = 0,
                .OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND,
            }
        };

        D3D12_ROOT_PARAMETER rootParams[2] = {};
        rootParams[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
        rootParams[0].DescriptorTable.NumDescriptorRanges = (UINT)std::size(sceneDepthRange);
        rootParams[0].DescriptorTable.pDescriptorRanges = sceneDepthRange;
        rootParams[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
        rootParams[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV;
        rootParams[1].Descriptor.ShaderRegister = 0;
        rootParams[1].Descriptor.RegisterSpace = 0;
        rootParams[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;

        D3D12_STATIC_SAMPLER_DESC sampler = {
            .Filter = D3D12_FILTER_MIN_MAG_MIP_POINT,
            .AddressU = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressV = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP,
            .MipLODBias = 0.0f,
            .MaxAnisotropy = 0,
            .ComparisonFunc = D3D12_COMPARISON_FUNC_NEVER,
            .BorderColor = D3D12_STATIC_BORDER_COLOR_TRANSPARENT_BLACK,
            .MinLOD = 0.0f,
            .MaxLOD = D3D12_FLOAT32_MAX,
            .ShaderRegister = 0,
            .RegisterSpace = 0,
            .ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL,
        };

        D3D12_ROOT_SIGNATURE_DESC rootSigDesc = {
            .NumParameters = (UINT)std::size(rootParams),
            .pParameters = rootParams,
            .NumStaticSamplers = 1,
            .pStaticSamplers = &sampler,
            .Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT,
        };

        ComPtr<ID3DBlob> serializedBlob;
        ComPtr<ID3DBlob> error;
        if (HRESULT res = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1_0, &serializedBlob, &error); FAILED(res)) {
            checkHResult(res, std::format("Failed to serialize debug draw root signature! {}", std::string((const char*)error->GetBufferPointer(), error->GetBufferSize())).c_str());
        }

        ComPtr<ID3D12RootSignature> rootSignature;
        checkHResult(VRManager::instance().D3D12->GetDevice()->CreateRootSignature(0, serializedBlob->GetBufferPointer(), serializedBlob->GetBufferSize(), IID_PPV_ARGS(&rootSignature)), "Failed to create debug draw root signature!");
        return rootSignature;
    };

    ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
    m_sceneDepthHeap = D3D12Utils::CreateDescriptorHeap(device, D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV, true, 2);
    m_targetHeap = D3D12Utils::CreateDescriptorHeap(device, D3D12_DESCRIPTOR_HEAP_TYPE_RTV, false, 2);
    m_depthHeap = D3D12Utils::CreateDescriptorHeap(device, D3D12_DESCRIPTOR_HEAP_TYPE_DSV, false, 2);
    const UINT sceneDepthDescriptorStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    const UINT targetDescriptorStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
    const UINT depthDescriptorStride = device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);
    const D3D12_CPU_DESCRIPTOR_HANDLE sceneDepthHeapStart = m_sceneDepthHeap->GetCPUDescriptorHandleForHeapStart();
    const D3D12_GPU_DESCRIPTOR_HANDLE sceneDepthGpuHeapStart = m_sceneDepthHeap->GetGPUDescriptorHandleForHeapStart();
    const D3D12_CPU_DESCRIPTOR_HANDLE targetHeapStart = m_targetHeap->GetCPUDescriptorHandleForHeapStart();
    const D3D12_CPU_DESCRIPTOR_HANDLE depthHeapStart = m_depthHeap->GetCPUDescriptorHandleForHeapStart();
    for (int eye = 0; eye < 2; ++eye) {
        m_sceneDepthHandles[eye] = sceneDepthHeapStart;
        m_sceneDepthHandles[eye].ptr += eye * sceneDepthDescriptorStride;
        m_sceneDepthGpuHandles[eye] = sceneDepthGpuHeapStart;
        m_sceneDepthGpuHandles[eye].ptr += eye * sceneDepthDescriptorStride;

        m_targetHandles[eye] = targetHeapStart;
        m_targetHandles[eye].ptr += eye * targetDescriptorStride;

        m_depthHandles[eye] = depthHeapStart;
        m_depthHandles[eye].ptr += eye * depthDescriptorStride;
    }
    m_signature = createSignature();
    for (uint32_t eye = 0; eye < 2; ++eye) {
        for (uint32_t settingsIndex = 0; settingsIndex < 2; ++settingsIndex) {
            m_sceneSettingsBuffers[eye][settingsIndex] = D3D12Utils::CreateConstantBuffer(device, D3D12_HEAP_TYPE_UPLOAD, sizeof(debugDrawLineSettings));
            void* mappedData = nullptr;
            const D3D12_RANGE readRange = { .Begin = 0, .End = 0 };
            checkHResult(m_sceneSettingsBuffers[eye][settingsIndex]->Map(0, &readRange, &mappedData), "Failed to map memory for debug draw scene settings!");
            m_sceneSettingsMappedData[eye][settingsIndex] = (uint8_t*)mappedData;
        }
    }
}

RND_D3D12::DebugDrawPipeline::~DebugDrawPipeline() {
    for (uint32_t eye = 0; eye < m_sceneSettingsBuffers.size(); ++eye) {
        for (uint32_t settingsIndex = 0; settingsIndex < m_sceneSettingsBuffers[eye].size(); ++settingsIndex) {
            if (m_sceneSettingsBuffers[eye][settingsIndex] != nullptr && m_sceneSettingsMappedData[eye][settingsIndex] != nullptr) {
                m_sceneSettingsBuffers[eye][settingsIndex]->Unmap(0, nullptr);
                m_sceneSettingsMappedData[eye][settingsIndex] = nullptr;
            }
        }
    }

    if (m_vertexBuffer != nullptr && m_mappedVertexBuffer != nullptr) {
        m_vertexBuffer->Unmap(0, nullptr);
        m_mappedVertexBuffer = nullptr;
    }
}

void RND_D3D12::DebugDrawPipeline::EnsureVertexBuffer(uint32_t requiredBytes) {
    if (requiredBytes <= m_vertexBufferCapacity) {
        return;
    }

    if (m_vertexBuffer != nullptr && m_mappedVertexBuffer != nullptr) {
        m_vertexBuffer->Unmap(0, nullptr);
        m_mappedVertexBuffer = nullptr;
    }

    m_vertexBufferCapacity = std::max(requiredBytes, std::max(m_vertexBufferCapacity * 2, 4096u));
    m_vertexBuffer = D3D12Utils::CreateConstantBuffer(VRManager::instance().D3D12->GetDevice(), D3D12_HEAP_TYPE_UPLOAD, m_vertexBufferCapacity);
    void* mappedData = nullptr;
    const D3D12_RANGE readRange = { .Begin = 0, .End = 0 };
    checkHResult(m_vertexBuffer->Map(0, &readRange, &mappedData), "Failed to map memory for debug draw vertex buffer!");
    m_mappedVertexBuffer = (DebugDrawVertex*)mappedData;
}

void RND_D3D12::DebugDrawPipeline::UploadRenderData(const DebugDrawRenderData& renderData) {
    const uint32_t triangleVertexCount = (uint32_t)renderData.triangleVertices.size();
    const uint32_t lineVertexCount = (uint32_t)renderData.lineVertices.size();
    const uint32_t xrayTriangleVertexCount = (uint32_t)renderData.xrayTriangleVertices.size();
    const uint32_t xrayLineVertexCount = (uint32_t)renderData.xrayLineVertices.size();
    const uint32_t totalVertexCount = triangleVertexCount + lineVertexCount + xrayTriangleVertexCount + xrayLineVertexCount;
    const uint32_t vertexBufferBytes = totalVertexCount * sizeof(DebugDrawVertex);
    EnsureVertexBuffer(vertexBufferBytes);

    if (m_mappedVertexBuffer == nullptr) {
        return;
    }

    DebugDrawVertex* dstVertices = m_mappedVertexBuffer;
    if (triangleVertexCount > 0) {
        memcpy(dstVertices, renderData.triangleVertices.data(), triangleVertexCount * sizeof(DebugDrawVertex));
    }
    if (lineVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount, renderData.lineVertices.data(), lineVertexCount * sizeof(DebugDrawVertex));
    }
    if (xrayTriangleVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount + lineVertexCount, renderData.xrayTriangleVertices.data(), xrayTriangleVertexCount * sizeof(DebugDrawVertex));
    }
    if (xrayLineVertexCount > 0) {
        memcpy(dstVertices + triangleVertexCount + lineVertexCount + xrayTriangleVertexCount, renderData.xrayLineVertices.data(), xrayLineVertexCount * sizeof(DebugDrawVertex));
    }

    m_vertexBufferView = {
        .BufferLocation = m_vertexBuffer->GetGPUVirtualAddress(),
        .SizeInBytes = vertexBufferBytes,
        .StrideInBytes = sizeof(DebugDrawVertex),
    };
}

void RND_D3D12::DebugDrawPipeline::UpdateSceneSettings(OpenXR::EyeSide side, uint32_t settingsIndex, ID3D12Resource* colorTarget, const glm::mat4& viewProjection, float xrayAlphaScale) {
    debugDrawLineSettings settings = {
        .viewProjection = viewProjection,
        .targetWidth = (float)colorTarget->GetDesc().Width,
        .targetHeight = (float)colorTarget->GetDesc().Height,
        .depthBias = 0.0005f,
        .xrayAlphaScale = xrayAlphaScale,
    };

    checkAssert(settingsIndex < m_sceneSettingsBuffers[side].size(), "Debug draw scene settings index is out of range!");
    checkAssert(m_sceneSettingsMappedData[side][settingsIndex] != nullptr, "Debug draw scene settings mapping is missing!");
    memcpy(m_sceneSettingsMappedData[side][settingsIndex], &settings, sizeof(settings));
}

void RND_D3D12::DebugDrawPipeline::RecreatePipeline() {
    D3D12_INPUT_ELEMENT_DESC inputLayout[] = {
        {
            .SemanticName = "POSITION",
            .SemanticIndex = 0,
            .Format = DXGI_FORMAT_R32G32B32_FLOAT,
            .InputSlot = 0,
            .AlignedByteOffset = 0,
            .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            .InstanceDataStepRate = 0,
        },
        {
            .SemanticName = "COLOR",
            .SemanticIndex = 0,
            .Format = DXGI_FORMAT_R8G8B8A8_UNORM,
            .InputSlot = 0,
            .AlignedByteOffset = 12,
            .InputSlotClass = D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            .InstanceDataStepRate = 0,
        },
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC baseDesc = {};
    baseDesc.InputLayout = { inputLayout, (UINT)std::size(inputLayout) };
    baseDesc.pRootSignature = m_signature.Get();
    baseDesc.VS = { m_vertexShader->GetBufferPointer(), m_vertexShader->GetBufferSize() };
    baseDesc.PS = { m_pixelShader->GetBufferPointer(), m_pixelShader->GetBufferSize() };
    baseDesc.BlendState = {
        .AlphaToCoverageEnable = false,
        .IndependentBlendEnable = false,
    };
    baseDesc.BlendState.RenderTarget[0] = {
        .BlendEnable = true,
        .LogicOpEnable = false,
        .SrcBlend = D3D12_BLEND_SRC_ALPHA,
        .DestBlend = D3D12_BLEND_INV_SRC_ALPHA,
        .BlendOp = D3D12_BLEND_OP_ADD,
        .SrcBlendAlpha = D3D12_BLEND_ONE,
        .DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA,
        .BlendOpAlpha = D3D12_BLEND_OP_ADD,
        .LogicOp = D3D12_LOGIC_OP_NOOP,
        .RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL,
    };
    baseDesc.SampleMask = UINT_MAX;
    baseDesc.RasterizerState = {
        .FillMode = D3D12_FILL_MODE_SOLID,
        .CullMode = D3D12_CULL_MODE_NONE,
        .FrontCounterClockwise = true,
        .DepthBias = D3D12_DEFAULT_DEPTH_BIAS,
        .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
        .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
        .DepthClipEnable = true,
        .MultisampleEnable = false,
        .AntialiasedLineEnable = false,
        .ForcedSampleCount = 0,
        .ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF,
    };
    baseDesc.DepthStencilState = {
        .DepthEnable = true,
        .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL,
        .DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL,
        .StencilEnable = false,
        .StencilReadMask = D3D12_DEFAULT_STENCIL_READ_MASK,
        .StencilWriteMask = D3D12_DEFAULT_STENCIL_WRITE_MASK,
        .FrontFace = {
            .StencilFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilPassOp = D3D12_STENCIL_OP_KEEP,
            .StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS,
        },
        .BackFace = {
            .StencilFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilDepthFailOp = D3D12_STENCIL_OP_KEEP,
            .StencilPassOp = D3D12_STENCIL_OP_KEEP,
            .StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS,
        },
    };
    baseDesc.IBStripCutValue = D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED;
    baseDesc.NumRenderTargets = 1;
    baseDesc.RTVFormats[0] = m_targetFormats[0];
    baseDesc.DSVFormat = m_targetFormats[1];
    baseDesc.SampleDesc.Count = 1;
    baseDesc.SampleDesc.Quality = 0;
    baseDesc.NodeMask = 0;
    baseDesc.CachedPSO = { nullptr, 0 };
    baseDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC triangleDesc = baseDesc;
    triangleDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateGraphicsPipelineState(&triangleDesc, IID_PPV_ARGS(&m_trianglePipelineState)), "Failed to create debug draw triangle pipeline state!");

    D3D12_GRAPHICS_PIPELINE_STATE_DESC lineDesc = baseDesc;
    lineDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateGraphicsPipelineState(&lineDesc, IID_PPV_ARGS(&m_linePipelineState)), "Failed to create debug draw line pipeline state!");

    D3D12_GRAPHICS_PIPELINE_STATE_DESC xrayBaseDesc = baseDesc;
    xrayBaseDesc.DepthStencilState.DepthEnable = false;
    xrayBaseDesc.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ZERO;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC xrayTriangleDesc = xrayBaseDesc;
    xrayTriangleDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateGraphicsPipelineState(&xrayTriangleDesc, IID_PPV_ARGS(&m_xrayTrianglePipelineState)), "Failed to create debug draw x-ray triangle pipeline state!");

    D3D12_GRAPHICS_PIPELINE_STATE_DESC xrayLineDesc = xrayBaseDesc;
    xrayLineDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;
    checkHResult(VRManager::instance().D3D12->GetDevice()->CreateGraphicsPipelineState(&xrayLineDesc, IID_PPV_ARGS(&m_xrayLinePipelineState)), "Failed to create debug draw x-ray line pipeline state!");
}

void RND_D3D12::DebugDrawPipeline::RenderVertices(ID3D12GraphicsCommandList* commandList, D3D12_PRIMITIVE_TOPOLOGY topology, ID3D12PipelineState* pipelineState, uint32_t vertexOffset, uint32_t vertexCount) {
    if (pipelineState == nullptr || vertexCount == 0) {
        return;
    }

    commandList->SetPipelineState(pipelineState);
    commandList->IASetPrimitiveTopology(topology);
    commandList->IASetVertexBuffers(0, 1, &m_vertexBufferView);
    commandList->DrawInstanced(vertexCount, 1, vertexOffset, 0);
}

void RND_D3D12::DebugDrawPipeline::Render(OpenXR::EyeSide side, ID3D12GraphicsCommandList* commandList, ID3D12Resource* sceneDepthTexture, ID3D12Resource* colorTarget, DXGI_FORMAT colorFormat, ID3D12Resource* depthTarget, DXGI_FORMAT depthFormat, const DebugDrawRenderData& renderData, const glm::mat4& viewProjection) {
    if (renderData.IsEmpty()) {
        return;
    }

    if (colorFormat != m_targetFormats[0] || depthFormat != m_targetFormats[1] || m_trianglePipelineState == nullptr || m_linePipelineState == nullptr) {
        m_targetFormats[0] = colorFormat;
        m_targetFormats[1] = depthFormat;
        RecreatePipeline();
    }

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.Format = D3D12Utils::ToSRVFormat(sceneDepthTexture->GetDesc().Format);
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Texture2D.MipLevels = 1;
    VRManager::instance().D3D12->GetDevice()->CreateShaderResourceView(sceneDepthTexture, &srvDesc, m_sceneDepthHandles[side]);

    D3D12_RENDER_TARGET_VIEW_DESC rtvDesc = {};
    rtvDesc.Format = colorFormat;
    rtvDesc.ViewDimension = D3D12_RTV_DIMENSION_TEXTURE2D;
    VRManager::instance().D3D12->GetDevice()->CreateRenderTargetView(colorTarget, &rtvDesc, m_targetHandles[side]);

    D3D12_DEPTH_STENCIL_VIEW_DESC dsvDesc = {};
    dsvDesc.Format = depthFormat;
    dsvDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
    dsvDesc.Flags = D3D12_DSV_FLAG_NONE;
    VRManager::instance().D3D12->GetDevice()->CreateDepthStencilView(depthTarget, &dsvDesc, m_depthHandles[side]);

    const uint32_t triangleVertexCount = (uint32_t)renderData.triangleVertices.size();
    const uint32_t lineVertexCount = (uint32_t)renderData.lineVertices.size();
    const uint32_t xrayTriangleVertexCount = (uint32_t)renderData.xrayTriangleVertices.size();
    const uint32_t xrayLineVertexCount = (uint32_t)renderData.xrayLineVertices.size();
    const uint32_t totalVertexCount = triangleVertexCount + lineVertexCount + xrayTriangleVertexCount + xrayLineVertexCount;
    if (totalVertexCount > 0 && m_vertexBufferView.SizeInBytes < totalVertexCount * sizeof(DebugDrawVertex)) {
        UploadRenderData(renderData);
    }

    commandList->SetGraphicsRootSignature(m_signature.Get());

    D3D12_VIEWPORT viewport = { 0.0f, 0.0f, (float)colorTarget->GetDesc().Width, (float)colorTarget->GetDesc().Height, 0.0f, 1.0f };
    D3D12_RECT scissorRect = { 0, 0, (LONG)colorTarget->GetDesc().Width, (LONG)colorTarget->GetDesc().Height };
    commandList->RSSetViewports(1, &viewport);
    commandList->RSSetScissorRects(1, &scissorRect);

    UpdateSceneSettings(side, 0, colorTarget, viewProjection, 0.0f);
    commandList->SetGraphicsRootConstantBufferView(1, m_sceneSettingsBuffers[side][0]->GetGPUVirtualAddress());

    ID3D12DescriptorHeap* heaps[] = { m_sceneDepthHeap.Get() };
    commandList->SetDescriptorHeaps((UINT)std::size(heaps), heaps);
    commandList->SetGraphicsRootDescriptorTable(0, m_sceneDepthGpuHandles[side]);
    commandList->OMSetRenderTargets(1, &m_targetHandles[side], true, &m_depthHandles[side]);

    RenderVertices(commandList, D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST, m_trianglePipelineState.Get(), 0, triangleVertexCount);
    RenderVertices(commandList, D3D_PRIMITIVE_TOPOLOGY_LINELIST, m_linePipelineState.Get(), triangleVertexCount, lineVertexCount);

    if (xrayTriangleVertexCount > 0 || xrayLineVertexCount > 0) {
        const uint32_t xrayTriangleOffset = triangleVertexCount + lineVertexCount;
        const uint32_t xrayLineOffset = xrayTriangleOffset + xrayTriangleVertexCount;
        UpdateSceneSettings(side, 1, colorTarget, viewProjection, 0.35f);
        commandList->SetGraphicsRootConstantBufferView(1, m_sceneSettingsBuffers[side][1]->GetGPUVirtualAddress());
        RenderVertices(commandList, D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST, m_xrayTrianglePipelineState.Get(), xrayTriangleOffset, xrayTriangleVertexCount);
        RenderVertices(commandList, D3D_PRIMITIVE_TOPOLOGY_LINELIST, m_xrayLinePipelineState.Get(), xrayLineOffset, xrayLineVertexCount);
    }
}

template class RND_D3D12::PresentPipeline<false>;
template class RND_D3D12::PresentPipeline<true>;
