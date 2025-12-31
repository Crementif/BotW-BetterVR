#include "d3d12.h"
#include "instance.h"
#include "texture.h"
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
}

RND_D3D12::~RND_D3D12() {
}

template <bool depth>
RND_D3D12::PresentPipeline<depth>::PresentPipeline(RND_Renderer* pRenderer) {
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

        D3D12_ROOT_PARAMETER rootParams[] = {
            {
                D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE,
                {
                    (UINT)std::size(pixelRange),
                    pixelRange,
                },
                D3D12_SHADER_VISIBILITY_PIXEL
            },
            {
                D3D12_ROOT_PARAMETER_TYPE_CBV,
                {
                    1,
                    0
                },
                D3D12_SHADER_VISIBILITY_ALL
            }
        };
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
    ComPtr<ID3D12CommandAllocator> uploadBufferAllocator;
    {
        ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
        ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
        device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&uploadBufferAllocator));
        RND_D3D12::CommandContext<true> uploadBufferContext(device, queue, uploadBufferAllocator.Get(), [this, device, &screenIndicesStaging](RND_D3D12::CommandContext<true>* context) {
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
void RND_D3D12::PresentPipeline<depth>::BindSettings(float screenWidth, float screenHeight) {
    ComPtr<ID3D12Resource> newSettingsStaging;
    ComPtr<ID3D12CommandAllocator> newSettingsAllocator;
    {
        ID3D12Device* device = VRManager::instance().D3D12->GetDevice();
        ID3D12CommandQueue* queue = VRManager::instance().D3D12->GetCommandQueue();
        device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&newSettingsAllocator));
        RND_D3D12::CommandContext<true> uploadBufferContext(device, queue, newSettingsAllocator.Get(), [this, device, &newSettingsStaging, screenWidth, screenHeight](RND_D3D12::CommandContext<true>* context) {
            m_settingsBuffer = D3D12Utils::CreateConstantBuffer(device, D3D12_HEAP_TYPE_DEFAULT, sizeof(presentSettings));

            newSettingsStaging = D3D12Utils::CreateConstantBuffer(device, D3D12_HEAP_TYPE_UPLOAD, sizeof(presentSettings));
            void* data;
            const D3D12_RANGE readRange = { .Begin = 0, .End = 0 };
            checkHResult(newSettingsStaging->Map(0, &readRange, &data), "Failed to map memory for screen indices buffer!");
            presentSettings settings = {
                .renderWidth = screenWidth,
                .renderHeight = screenHeight,
                .swapchainWidth = screenWidth,
                .swapchainHeight = screenHeight,
            };
            memcpy(data, &settings, sizeof(presentSettings));
            newSettingsStaging->Unmap(0, nullptr);

            context->GetRecordList()->CopyBufferRegion(m_settingsBuffer.Get(), 0, newSettingsStaging.Get(), 0, sizeof(presentSettings));
        });
    }
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

template class RND_D3D12::PresentPipeline<false>;
template class RND_D3D12::PresentPipeline<true>;