// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.


#include "pch.h"
#include "OpenXrProgram.h"
#include "DxUtility.h"
#include "BotWHook.h"

namespace {
    namespace CubeShader {
        struct Vertex {
            XrVector3f Position;
            XrVector3f Color;
        };

        // Winding order is clockwise. Each side uses a different color.
        constexpr unsigned short screenIndices[] = {
            0,  1,  2,  2,  1,  3,
        };

        struct ViewProjectionConstantBuffer {
            DirectX::XMFLOAT4 viewportSize;
        };

        constexpr uint32_t MaxViewInstance = 2;

        // Separate entrypoints for the vertex and pixel shader functions.
        constexpr char ShaderHlsl[] = R"_(
            struct VSInput {
                float3 Pos : POSITION;
                uint instId : SV_InstanceID;
                uint vertexId : SV_VertexID;
            };
            struct PSInput {
                float2 uv0 : TEXCOORD0;
                float4 Pos : SV_POSITION;
                uint viewId : SV_RenderTargetArrayIndex;
            };

            cbuffer ViewportSizeConstantBuffer : register(b0) {                                                                                                       
                float4 viewportSize;
            };

            SamplerState screenSampler : register(s0);
            Texture2D screen : register(t0);

            PSInput MainVS(VSInput input) {
                float screenX = (float)(input.vertexId / 2) * 2.0 - 1.0 + 0.5;
                float screenY = (float)(input.vertexId % 2) * 2.0 - 1.0;
            
                PSInput output;
                output.uv0.x = input.instId;
                output.uv0.y = 0.0;

                output.Pos.x = screenX * 1.25 - 0.75 + (float)input.instId;
                output.Pos.y = screenY;
                output.Pos.z = 1.0;
                output.Pos.w = 1.0;

                if (input.instId == 1) {
                    output.Pos.x -= 0.5;
                }

                output.viewId = input.instId;
                return output;
            }

            float4 MainPS(PSInput input) : SV_TARGET {
                float4 screenColor;
                if (input.uv0.x == 0.0) {
                    screenColor = screen.Sample(screenSampler, input.Pos.xy / float2(viewportSize.x * 2.0, viewportSize.y));
                }
                else {
                    float2 xyPos = input.Pos.xy / float2(viewportSize.x * 2.0, viewportSize.y);
                    xyPos.x += 0.5;
                    screenColor = screen.Sample(screenSampler, xyPos);
                }
                return float4(screenColor.rgb, 1);
            }
            )_";

    } // namespace CubeShader

    struct CubeGraphics : sample::IGraphicsPluginD3D11 {
        ID3D11Device* InitializeDevice(LUID adapterLuid, const std::vector<D3D_FEATURE_LEVEL>& featureLevels) override {
            m_adapter = sample::dx::GetAdapter(adapterLuid);

            sample::dx::CreateD3D11DeviceAndContext(m_adapter.get(), featureLevels, m_device.put(), m_deviceContext.put());

            InitializeD3DResources(m_adapter.get());

            return m_device.get();
        }

        void InitializeD3DResources(IDXGIAdapter1 *adapter) {
            const winrt::com_ptr<ID3DBlob> vertexShaderBytes = sample::dx::CompileShader(CubeShader::ShaderHlsl, "MainVS", "vs_5_0");
            CHECK_HRCMD(m_device->CreateVertexShader(vertexShaderBytes->GetBufferPointer(), vertexShaderBytes->GetBufferSize(), nullptr, m_vertexShader.put()));

            const winrt::com_ptr<ID3DBlob> pixelShaderBytes = sample::dx::CompileShader(CubeShader::ShaderHlsl, "MainPS", "ps_5_0");
            CHECK_HRCMD(m_device->CreatePixelShader(pixelShaderBytes->GetBufferPointer(), pixelShaderBytes->GetBufferSize(), nullptr, m_pixelShader.put()));

            const D3D11_INPUT_ELEMENT_DESC vertexDesc[] = {
                {"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
                {"COLOR", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0},
            };

            CHECK_HRCMD(m_device->CreateInputLayout(vertexDesc, (UINT)std::size(vertexDesc), vertexShaderBytes->GetBufferPointer(), vertexShaderBytes->GetBufferSize(), m_inputLayout.put()));

            const CD3D11_BUFFER_DESC viewportSizeConstantBufferDesc(sizeof(CubeShader::ViewProjectionConstantBuffer), D3D11_BIND_CONSTANT_BUFFER);
            CHECK_HRCMD(m_device->CreateBuffer(&viewportSizeConstantBufferDesc, nullptr, m_viewportSizeCBuffer.put()));

            const D3D11_SUBRESOURCE_DATA indexBufferData{CubeShader::screenIndices};
            const CD3D11_BUFFER_DESC indexBufferDesc(sizeof(CubeShader::screenIndices), D3D11_BIND_INDEX_BUFFER);
            CHECK_HRCMD(m_device->CreateBuffer(&indexBufferDesc, &indexBufferData, m_cubeIndexBuffer.put()));

            D3D11_FEATURE_DATA_D3D11_OPTIONS3 options;
            m_device->CheckFeatureSupport(D3D11_FEATURE_D3D11_OPTIONS3, &options, sizeof(options));
            CHECK_MSG(options.VPAndRTArrayIndexFromAnyShaderFeedingRasterizer, "This sample requires VPRT support. Adjust sample shaders on GPU without VRPT.");

            m_initialTextureDesc.Height = 1080;
            m_initialTextureDesc.Width = 1080;
            m_initialTextureDesc.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
            m_initialTextureDesc.ArraySize = 1;
            m_initialTextureDesc.BindFlags = D3D11_BIND_FLAG::D3D11_BIND_SHADER_RESOURCE;
            m_initialTextureDesc.MiscFlags = 0;
            m_initialTextureDesc.SampleDesc.Count = 1;
            m_initialTextureDesc.SampleDesc.Quality = 0;
            m_initialTextureDesc.MipLevels = 1;
            m_initialTextureDesc.CPUAccessFlags = 0;
            m_initialTextureDesc.Usage = D3D11_USAGE_DEFAULT;
            CHECK_HRCMD(m_device->CreateTexture2D(&m_initialTextureDesc, NULL, m_initialTexture.put()));

            m_setup_not_done = true;
        }

        const std::vector<DXGI_FORMAT>& SupportedColorFormats() const override {
            const static std::vector<DXGI_FORMAT> SupportedColorFormats = {
                DXGI_FORMAT_R8G8B8A8_UNORM,
                DXGI_FORMAT_B8G8R8A8_UNORM,
                DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
                DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
            };
            return SupportedColorFormats;
        }

        void RenderView(const XrRect2Di& imageRect, const std::vector<xr::math::ViewProjection>& viewProjections, DXGI_FORMAT colorSwapchainFormat, ID3D11Texture2D* colorTexture) override {
            const uint32_t viewInstanceCount = (uint32_t)viewProjections.size();
            CHECK_MSG(viewInstanceCount <= CubeShader::MaxViewInstance, "Sample shader supports 2 or fewer view instances. Adjust shader to accommodate more.")

            CD3D11_VIEWPORT viewport((float)imageRect.offset.x, (float)imageRect.offset.y, (float)imageRect.extent.width, (float)imageRect.extent.height);
            m_deviceContext->RSSetViewports(1, &viewport);

            // Create RenderTargetView with the original swapchain format (swapchain image is typeless).
            winrt::com_ptr<ID3D11RenderTargetView> renderTargetView;
            const CD3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc(D3D11_RTV_DIMENSION_TEXTURE2DARRAY, colorSwapchainFormat);
            CHECK_HRCMD(m_device->CreateRenderTargetView(colorTexture, &renderTargetViewDesc, renderTargetView.put()));

            // Clear swapchain. NOTE: This will clear the entire render target view, not just the specified view.
            constexpr DirectX::XMVECTORF32 transparent = { 1.000000000f, 1.000000000f, 1.000000000f, 1.000000000f };
            m_deviceContext->ClearRenderTargetView(renderTargetView.get(), transparent);

            ID3D11RenderTargetView* renderTargets[] = {renderTargetView.get()};
            m_deviceContext->OMSetRenderTargets((UINT)std::size(renderTargets), renderTargets, NULL);

            if (getCemuHWND() != NULL && getHookMode() == HOOK_MODE::GFX_PACK_ENABLED || getHookMode() == HOOK_MODE::BOTH_ENABLED_PRECALC || getHookMode() == HOOK_MODE::BOTH_ENABLED_GFX_CALC) {
                if (m_setup_not_done) {
                    m_setup_not_done = false;

                    // Find the correct monitor to mirror
                    HMONITOR cemuMonitorHandle = MonitorFromWindow(getCemuHWND(), MONITOR_DEFAULTTOPRIMARY);

                    UINT i = 0;
                    IDXGIOutput* pOutput;
                    while (m_adapter->EnumOutputs(i, &pOutput) != DXGI_ERROR_NOT_FOUND) {
                        DXGI_OUTPUT_DESC outputDesc;
                        pOutput->GetDesc(&outputDesc);
                        if (outputDesc.Monitor == cemuMonitorHandle) {
                            break;
                        }
                        i++;
                    }

                    // Initialize dublication
                    IDXGIOutput1* output1;
                    pOutput->QueryInterface(IID_PPV_ARGS(&output1));
                    pOutput->Release();

                    output1->DuplicateOutput(m_device.get(), m_dxgiOutputApplication.put());
                    m_dxgiOutputApplication->GetDesc(&m_outputDesc);
                    output1->Release();
                }
                IDXGIResource *dxgiResource = nullptr;
                ID3D11Resource *outputResource = nullptr;

                DXGI_OUTDUPL_FRAME_INFO dxgiFrameInfo;
                HRESULT hr = m_dxgiOutputApplication->AcquireNextFrame(INFINITE, &dxgiFrameInfo, &dxgiResource);

                if (hr == DXGI_ERROR_ACCESS_LOST || hr == DXGI_ERROR_WAIT_TIMEOUT) {
                    // When you go into full-screen, this'll prevent the game from erroring out
                    D3D11_SHADER_RESOURCE_VIEW_DESC shaderResourceView = {};
                    shaderResourceView.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
                    shaderResourceView.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                    shaderResourceView.Texture2D.MostDetailedMip = 0;
                    shaderResourceView.Texture2D.MipLevels = 1;

                    ID3D11ShaderResourceView* outputResourceView[1];

                    m_device->CreateShaderResourceView(m_initialTexture.get(), &shaderResourceView, outputResourceView);
                    m_deviceContext->PSSetShaderResources(3, 1, outputResourceView);

                    if (hr == DXGI_ERROR_ACCESS_LOST) {
                        // Release and reset everything so that it'll be recreated in the next frame
                        m_dxgiOutputApplication->ReleaseFrame();
                        m_dxgiOutputApplication->Release();
                        m_dxgiOutputApplication = nullptr;
                        m_setup_not_done = true;
                    }
                }
                else {
                    dxgiResource->QueryInterface(IID_PPV_ARGS(&outputResource));
                    dxgiResource->Release();
                    
                    D3D11_SHADER_RESOURCE_VIEW_DESC shaderResourceView;
                    shaderResourceView.Format = m_outputDesc.ModeDesc.Format;
                    shaderResourceView.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                    shaderResourceView.Texture2D.MostDetailedMip = 0;
                    shaderResourceView.Texture2D.MipLevels = 1;
                    
                    ID3D11ShaderResourceView* outputResourceView[1];
                    
                    m_device->CreateShaderResourceView(outputResource, &shaderResourceView, outputResourceView);
                    m_deviceContext->PSSetShaderResources(0, 1, outputResourceView);
                }
            }
            else {
                D3D11_SHADER_RESOURCE_VIEW_DESC shaderResourceView = {};
                shaderResourceView.Format = DXGI_FORMAT_B8G8R8A8_UNORM;
                shaderResourceView.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
                shaderResourceView.Texture2D.MostDetailedMip = 0;
                shaderResourceView.Texture2D.MipLevels = 1;

                ID3D11ShaderResourceView* outputResourceView[1];

                m_device->CreateShaderResourceView(m_initialTexture.get(), &shaderResourceView, outputResourceView);
                m_deviceContext->PSSetShaderResources(3, 1, outputResourceView);
            }

            ID3D11Buffer* const constantBuffers[] = { m_viewportSizeCBuffer.get() };
            m_deviceContext->PSSetConstantBuffers(0, (UINT)std::size(constantBuffers), constantBuffers);

            CubeShader::ViewProjectionConstantBuffer viewportSizeCBufferData{};
            DirectX::XMStoreFloat4(&viewportSizeCBufferData.viewportSize, { (float)imageRect.extent.width, (float)imageRect.extent.height, 0, 0 });
            m_deviceContext->UpdateSubresource(m_viewportSizeCBuffer.get(), 0, nullptr, &viewportSizeCBufferData, 0, 0);

            m_deviceContext->VSSetShader(m_vertexShader.get(), nullptr, 0);
            m_deviceContext->PSSetShader(m_pixelShader.get(), nullptr, 0);

            m_deviceContext->IASetIndexBuffer(m_cubeIndexBuffer.get(), DXGI_FORMAT_R16_UINT, 0);
            m_deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            m_deviceContext->IASetInputLayout(m_inputLayout.get());

            m_deviceContext->DrawIndexedInstanced((UINT)std::size(CubeShader::screenIndices), viewInstanceCount, 0, 0, 0);

            if (!m_setup_not_done) m_dxgiOutputApplication->ReleaseFrame();
        }

    private:
        winrt::com_ptr<IDXGIAdapter1> m_adapter;
        winrt::com_ptr<ID3D11Device> m_device;
        winrt::com_ptr<ID3D11DeviceContext> m_deviceContext;
        winrt::com_ptr<ID3D11VertexShader> m_vertexShader;
        winrt::com_ptr<ID3D11PixelShader> m_pixelShader;
        winrt::com_ptr<ID3D11InputLayout> m_inputLayout;
        winrt::com_ptr<ID3D11Buffer> m_viewportSizeCBuffer;
        winrt::com_ptr<ID3D11Buffer> m_cubeIndexBuffer;

        winrt::com_ptr<IDXGIOutputDuplication> m_dxgiOutputApplication;
        winrt::com_ptr<ID3D11Texture2D> m_initialTexture;
        winrt::com_ptr<ID3D11SamplerState> m_outputSampler;
        winrt::com_ptr<ID3D11VertexShader> m_outputVertexShader;
        winrt::com_ptr<ID3D11PixelShader> m_outputPixelShader;
        winrt::com_ptr<ID3D11InputLayout> m_outputInputLayout;
        DXGI_OUTDUPL_DESC m_outputDesc;
        D3D11_TEXTURE2D_DESC m_initialTextureDesc;
        winrt::com_ptr<ID3D11Texture2D> m_placeholderTexture;
        DXGI_OUTDUPL_FRAME_INFO m_outputFrameInfo;
        D3D11_TEXTURE2D_DESC m_placeholderFrameDesc;
        bool m_setup_not_done;
    };
} // namespace

namespace sample {
    std::unique_ptr<sample::IGraphicsPluginD3D11> CreateCubeGraphics() {
        return std::make_unique<CubeGraphics>();
    }
} // namespace sample
