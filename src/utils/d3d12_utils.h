#pragma once

namespace D3D12Utils {
    static ComPtr<ID3DBlob> CompileShader(const char* sourceHLSL, const char* entryPoint, const char* version) {
        DWORD shaderCompileFlags = D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR | D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_WARNINGS_ARE_ERRORS;
#ifdef _DEBUG
        shaderCompileFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
        shaderCompileFlags |= D3DCOMPILE_DEBUG;
#else
        shaderCompileFlags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
        ComPtr<ID3DBlob> shaderBytes;
        ID3DBlob* hlslCompilationErrors;
        if (FAILED(D3DCompile(sourceHLSL, strlen(sourceHLSL), nullptr, nullptr, nullptr, entryPoint, version, shaderCompileFlags, 0, &shaderBytes, &hlslCompilationErrors))) {
            std::string errorMessage((const char*)hlslCompilationErrors->GetBufferPointer(), hlslCompilationErrors->GetBufferSize());
            Log::print<ERROR>("Vertex Shader Compilation Error:");
            Log::print<ERROR>(errorMessage.c_str());
            throw std::runtime_error("Error during the vertex shader compilation!");
        }
        return shaderBytes;
    };

    static ComPtr<ID3D12Resource> CreateConstantBuffer(ID3D12Device* device, D3D12_HEAP_TYPE heapType, uint32_t size) {
        auto findAlignedSize = [](uint32_t size) -> UINT {
            static_assert((D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT & (D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT - 1)) == 0, "The alignment must be power-of-two");
            return (size + D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT - 1) & ~(D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT - 1);
        };

        D3D12_RESOURCE_STATES bufferType = (heapType == D3D12_HEAP_TYPE_UPLOAD) ? D3D12_RESOURCE_STATE_GENERIC_READ : D3D12_RESOURCE_STATE_COMMON;

        D3D12_HEAP_PROPERTIES heapProp = {
            .Type = heapType,
            .CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN,
            .MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN
        };

        // clang-format off
        D3D12_RESOURCE_DESC buffDesc = {
            .Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
            .Alignment = 0,
            .Width = (heapType == D3D12_HEAP_TYPE_UPLOAD) ? findAlignedSize(size) : size,
            .Height = 1,
            .DepthOrArraySize = 1,
            .MipLevels = 1,
            .Format = DXGI_FORMAT_UNKNOWN,
            .SampleDesc = {
                .Count = 1,
                .Quality = 0
            },
            .Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
            .Flags = D3D12_RESOURCE_FLAG_NONE
        };
        // clang-format on

        ComPtr<ID3D12Resource> buffer;
        checkHResult(device->CreateCommittedResource(&heapProp, D3D12_HEAP_FLAG_NONE, &buffDesc, bufferType, nullptr, IID_PPV_ARGS(&buffer)), "Failed to create buffer!");
        return buffer;
    }

    static ComPtr<ID3D12DescriptorHeap> CreateDescriptorHeap(ID3D12Device* device, D3D12_DESCRIPTOR_HEAP_TYPE descType, bool shaderVisible, UINT numDescriptors) {
        ComPtr<ID3D12DescriptorHeap> descriptorHeap;
        D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {
            .Type = descType,
            .NumDescriptors = numDescriptors,
            .Flags = shaderVisible ? D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE : D3D12_DESCRIPTOR_HEAP_FLAG_NONE
        };
        checkHResult(device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&descriptorHeap)), "Failed to create descriptor heap!");
        return descriptorHeap;
    };

    static constexpr bool IsDepthFormat(DXGI_FORMAT format) {
        switch (format) {
            case DXGI_FORMAT_D16_UNORM:
            case DXGI_FORMAT_D32_FLOAT:
            case DXGI_FORMAT_D24_UNORM_S8_UINT:
            case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
            case DXGI_FORMAT_R32_TYPELESS:  // Typeless can be used as depth
            case DXGI_FORMAT_R24G8_TYPELESS:
            case DXGI_FORMAT_R32G8X24_TYPELESS:
            case DXGI_FORMAT_R16_TYPELESS:
                return true;
            default:
                return false;
        }
    }

    // Convert depth format to typeless equivalent for creating resources that need both DSV and SRV
    static constexpr DXGI_FORMAT ToTypelessDepthFormat(DXGI_FORMAT format) {
        switch (format) {
            case DXGI_FORMAT_D32_FLOAT:
                return DXGI_FORMAT_R32_TYPELESS;
            case DXGI_FORMAT_D24_UNORM_S8_UINT:
                return DXGI_FORMAT_R24G8_TYPELESS;
            case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
                return DXGI_FORMAT_R32G8X24_TYPELESS;
            case DXGI_FORMAT_D16_UNORM:
                return DXGI_FORMAT_R16_TYPELESS;
            default:
                return format;  // Return as-is if not a depth format
        }
    }

    // Get the DSV format for a typeless depth resource
    static constexpr DXGI_FORMAT ToDSVFormat(DXGI_FORMAT format) {
        switch (format) {
            case DXGI_FORMAT_R32_TYPELESS:
                return DXGI_FORMAT_D32_FLOAT;
            case DXGI_FORMAT_R24G8_TYPELESS:
                return DXGI_FORMAT_D24_UNORM_S8_UINT;
            case DXGI_FORMAT_R32G8X24_TYPELESS:
                return DXGI_FORMAT_D32_FLOAT_S8X24_UINT;
            case DXGI_FORMAT_R16_TYPELESS:
                return DXGI_FORMAT_D16_UNORM;
            default:
                return format;
        }
    }

    // Get the SRV format for a typeless depth resource
    static constexpr DXGI_FORMAT ToSRVFormat(DXGI_FORMAT format) {
        switch (format) {
            case DXGI_FORMAT_R32_TYPELESS:
            case DXGI_FORMAT_D32_FLOAT:
                return DXGI_FORMAT_R32_FLOAT;
            case DXGI_FORMAT_R24G8_TYPELESS:
            case DXGI_FORMAT_D24_UNORM_S8_UINT:
                return DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
            case DXGI_FORMAT_R32G8X24_TYPELESS:
            case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
                return DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS;
            case DXGI_FORMAT_R16_TYPELESS:
            case DXGI_FORMAT_D16_UNORM:
                return DXGI_FORMAT_R16_UNORM;
            default:
                return format;
        }
    }

    // Based on https://github.com/doitsujin/dxvk/blob/master/src/dxgi/dxgi_format.cpp
    constexpr std::pair<DXGI_FORMAT, VkFormat> DxgiToVkFormat[] = {
        { DXGI_FORMAT_R32G32B32A32_FLOAT, VK_FORMAT_R32G32B32A32_SFLOAT },
        { DXGI_FORMAT_R32G32B32A32_UINT, VK_FORMAT_R32G32B32A32_UINT },
        { DXGI_FORMAT_R32G32B32A32_SINT, VK_FORMAT_R32G32B32A32_SINT },
        { DXGI_FORMAT_R32G32B32_FLOAT, VK_FORMAT_R32G32B32_SFLOAT },
        { DXGI_FORMAT_R32G32B32_UINT, VK_FORMAT_R32G32B32_UINT },
        { DXGI_FORMAT_R32G32B32_SINT, VK_FORMAT_R32G32B32_SINT },
        { DXGI_FORMAT_R16G16B16A16_FLOAT, VK_FORMAT_R16G16B16A16_SFLOAT },
        { DXGI_FORMAT_R16G16B16A16_UNORM, VK_FORMAT_R16G16B16A16_UNORM },
        { DXGI_FORMAT_R16G16B16A16_UINT, VK_FORMAT_R16G16B16A16_UINT },
        { DXGI_FORMAT_R16G16B16A16_SNORM, VK_FORMAT_R16G16B16A16_SNORM },
        { DXGI_FORMAT_R16G16B16A16_SINT, VK_FORMAT_R16G16B16A16_SINT },
        { DXGI_FORMAT_R32G32_FLOAT, VK_FORMAT_R32G32_SFLOAT },
        { DXGI_FORMAT_R32G32_UINT, VK_FORMAT_R32G32_UINT },
        { DXGI_FORMAT_R32G32_SINT, VK_FORMAT_R32G32_SINT },
        { DXGI_FORMAT_D32_FLOAT_S8X24_UINT, VK_FORMAT_D32_SFLOAT_S8_UINT },
        { DXGI_FORMAT_R10G10B10A2_UNORM, VK_FORMAT_A2B10G10R10_UNORM_PACK32 },
        { DXGI_FORMAT_R10G10B10A2_UINT, VK_FORMAT_A2B10G10R10_UINT_PACK32 },
        { DXGI_FORMAT_R11G11B10_FLOAT, VK_FORMAT_B10G11R11_UFLOAT_PACK32 },
        { DXGI_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_R8G8B8A8_UNORM },
        { DXGI_FORMAT_R8G8B8A8_UNORM_SRGB, VK_FORMAT_R8G8B8A8_SRGB },
        { DXGI_FORMAT_R8G8B8A8_UINT, VK_FORMAT_R8G8B8A8_UINT },
        { DXGI_FORMAT_R8G8B8A8_SNORM, VK_FORMAT_R8G8B8A8_SNORM },
        { DXGI_FORMAT_R8G8B8A8_SINT, VK_FORMAT_R8G8B8A8_SINT },
        { DXGI_FORMAT_R16G16_FLOAT, VK_FORMAT_R16G16_SFLOAT },
        { DXGI_FORMAT_R16G16_UNORM, VK_FORMAT_R16G16_UNORM },
        { DXGI_FORMAT_R16G16_UINT, VK_FORMAT_R16G16_UINT },
        { DXGI_FORMAT_R16G16_SNORM, VK_FORMAT_R16G16_SNORM },
        { DXGI_FORMAT_R16G16_SINT, VK_FORMAT_R16G16_SINT },
        { DXGI_FORMAT_D32_FLOAT, VK_FORMAT_D32_SFLOAT },
        { DXGI_FORMAT_R32_FLOAT, VK_FORMAT_R32_SFLOAT },
        { DXGI_FORMAT_R32_UINT, VK_FORMAT_R32_UINT },
        { DXGI_FORMAT_R32_SINT, VK_FORMAT_R32_SINT },
        { DXGI_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
        { DXGI_FORMAT_R8G8_UNORM, VK_FORMAT_R8G8_UNORM },
        { DXGI_FORMAT_R8G8_UINT, VK_FORMAT_R8G8_UINT },
        { DXGI_FORMAT_R8G8_SNORM, VK_FORMAT_R8G8_SNORM },
        { DXGI_FORMAT_R8G8_SINT, VK_FORMAT_R8G8_SINT },
        { DXGI_FORMAT_R16_FLOAT, VK_FORMAT_R16_SFLOAT },
        { DXGI_FORMAT_D16_UNORM, VK_FORMAT_D16_UNORM },
        { DXGI_FORMAT_R16_UNORM, VK_FORMAT_R16_UNORM },
        { DXGI_FORMAT_R16_UINT, VK_FORMAT_R16_UINT },
        { DXGI_FORMAT_R16_SNORM, VK_FORMAT_R16_SNORM },
        { DXGI_FORMAT_R16_SINT, VK_FORMAT_R16_SINT },
        { DXGI_FORMAT_R8_UNORM, VK_FORMAT_R8_UNORM },
        { DXGI_FORMAT_R8_UINT, VK_FORMAT_R8_UINT },
        { DXGI_FORMAT_R8_SNORM, VK_FORMAT_R8_SNORM },
        { DXGI_FORMAT_R8_SINT, VK_FORMAT_R8_SINT },
        { DXGI_FORMAT_A8_UNORM, VK_FORMAT_R8_UNORM },
        { DXGI_FORMAT_R9G9B9E5_SHAREDEXP, VK_FORMAT_E5B9G9R9_UFLOAT_PACK32 },
        { DXGI_FORMAT_R8G8_B8G8_UNORM, VK_FORMAT_B8G8R8G8_422_UNORM },
        { DXGI_FORMAT_G8R8_G8B8_UNORM, VK_FORMAT_G8B8G8R8_422_UNORM },
        { DXGI_FORMAT_BC1_UNORM, VK_FORMAT_BC1_RGBA_UNORM_BLOCK },
        { DXGI_FORMAT_BC1_UNORM_SRGB, VK_FORMAT_BC1_RGBA_SRGB_BLOCK },
        { DXGI_FORMAT_BC2_UNORM, VK_FORMAT_BC2_UNORM_BLOCK },
        { DXGI_FORMAT_BC2_UNORM_SRGB, VK_FORMAT_BC2_SRGB_BLOCK },
        { DXGI_FORMAT_BC3_UNORM, VK_FORMAT_BC3_UNORM_BLOCK },
        { DXGI_FORMAT_BC3_UNORM_SRGB, VK_FORMAT_BC3_SRGB_BLOCK },
        { DXGI_FORMAT_BC4_UNORM, VK_FORMAT_BC4_UNORM_BLOCK },
        { DXGI_FORMAT_BC4_SNORM, VK_FORMAT_BC4_SNORM_BLOCK },
        { DXGI_FORMAT_BC5_UNORM, VK_FORMAT_BC5_UNORM_BLOCK },
        { DXGI_FORMAT_BC5_SNORM, VK_FORMAT_BC5_SNORM_BLOCK },
        { DXGI_FORMAT_B5G6R5_UNORM, VK_FORMAT_R5G6B5_UNORM_PACK16 },
        { DXGI_FORMAT_B5G5R5A1_UNORM, VK_FORMAT_A1R5G5B5_UNORM_PACK16 },
        { DXGI_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_B8G8R8A8_UNORM },
        { DXGI_FORMAT_B8G8R8X8_UNORM, VK_FORMAT_B8G8R8A8_UNORM },
        { DXGI_FORMAT_B8G8R8A8_UNORM_SRGB, VK_FORMAT_B8G8R8A8_SRGB },
        { DXGI_FORMAT_B8G8R8X8_UNORM_SRGB, VK_FORMAT_B8G8R8A8_SRGB },
        { DXGI_FORMAT_BC6H_UF16, VK_FORMAT_BC6H_UFLOAT_BLOCK },
        { DXGI_FORMAT_BC6H_SF16, VK_FORMAT_BC6H_SFLOAT_BLOCK },
        { DXGI_FORMAT_BC7_UNORM, VK_FORMAT_BC7_UNORM_BLOCK },
        { DXGI_FORMAT_BC7_UNORM_SRGB, VK_FORMAT_BC7_SRGB_BLOCK },
        { DXGI_FORMAT_AYUV, VK_FORMAT_R8G8B8A8_UNORM },
        { DXGI_FORMAT_Y410, VK_FORMAT_G8_B8R8_2PLANE_420_UNORM },
        { DXGI_FORMAT_420_OPAQUE, VK_FORMAT_G8_B8R8_2PLANE_420_UNORM },
        { DXGI_FORMAT_YUY2, VK_FORMAT_G8B8G8R8_422_UNORM },
        { DXGI_FORMAT_B4G4R4A4_UNORM, VK_FORMAT_A4R4G4B4_UNORM_PACK16_EXT },
        { DXGI_FORMAT_R10G10B10A2_UNORM, VK_FORMAT_A2R10G10B10_UNORM_PACK32 },
        { DXGI_FORMAT_R10G10B10A2_UINT, VK_FORMAT_A2R10G10B10_UINT_PACK32 },
        { DXGI_FORMAT_R10G10B10A2_UNORM, VK_FORMAT_A2B10G10R10_UNORM_PACK32 }
    };

    static DXGI_FORMAT ToDXGIFormat(VkFormat format) {
        for (size_t i = 0; i < std::size(DxgiToVkFormat); i++) {
            if (DxgiToVkFormat[i].second == format) {
                return DxgiToVkFormat[i].first;
            }
        }
        Log::print<ERROR>("Couldn't find a known DXGI_FORMAT mapping for {}", format);
        return DXGI_FORMAT_UNKNOWN;
    }

    static VkFormat ToVkFormat(DXGI_FORMAT format) {
        for (size_t i = 0; i < std::size(DxgiToVkFormat); i++) {
            if (DxgiToVkFormat[i].first == format) {
                return DxgiToVkFormat[i].second;
            }
        }
        Log::print<ERROR>("Couldn't find a known VkFormat mapping for {}", format);
        return VK_FORMAT_UNDEFINED;
    }

    static std::string_view DXGIFormatToString(const DXGI_FORMAT format) {
        switch (format) {
            case DXGI_FORMAT_UNKNOWN: return "DXGI_FORMAT_UNKNOWN";
            case DXGI_FORMAT_R32G32B32A32_TYPELESS: return "DXGI_FORMAT_R32G32B32A32_TYPELESS";
            case DXGI_FORMAT_R32G32B32A32_FLOAT: return "DXGI_FORMAT_R32G32B32A32_FLOAT";
            case DXGI_FORMAT_R32G32B32A32_UINT: return "DXGI_FORMAT_R32G32B32A32_UINT";
            case DXGI_FORMAT_R32G32B32A32_SINT: return "DXGI_FORMAT_R32G32B32A32_SINT";
            case DXGI_FORMAT_R32G32B32_TYPELESS: return "DXGI_FORMAT_R32G32B32_TYPELESS";
            case DXGI_FORMAT_R32G32B32_FLOAT: return "DXGI_FORMAT_R32G32B32_FLOAT";
            case DXGI_FORMAT_R32G32B32_UINT: return "DXGI_FORMAT_R32G32B32_UINT";
            case DXGI_FORMAT_R32G32B32_SINT: return "DXGI_FORMAT_R32G32B32_SINT";
            case DXGI_FORMAT_R16G16B16A16_TYPELESS: return "DXGI_FORMAT_R16G16B16A16_TYPELESS";
            case DXGI_FORMAT_R16G16B16A16_FLOAT: return "DXGI_FORMAT_R16G16B16A16_FLOAT";
            case DXGI_FORMAT_R16G16B16A16_UNORM: return "DXGI_FORMAT_R16G16B16A16_UNORM";
            case DXGI_FORMAT_R16G16B16A16_UINT: return "DXGI_FORMAT_R16G16B16A16_UINT";
            case DXGI_FORMAT_R16G16B16A16_SNORM: return "DXGI_FORMAT_R16G16B16A16_SNORM";
            case DXGI_FORMAT_R16G16B16A16_SINT: return "DXGI_FORMAT_R16G16B16A16_SINT";
            case DXGI_FORMAT_R32G32_TYPELESS: return "DXGI_FORMAT_R32G32_TYPELESS";
            case DXGI_FORMAT_R32G32_FLOAT: return "DXGI_FORMAT_R32G32_FLOAT";
            case DXGI_FORMAT_R32G32_UINT: return "DXGI_FORMAT_R32G32_UINT";
            case DXGI_FORMAT_R32G32_SINT: return "DXGI_FORMAT_R32G32_SINT";
            case DXGI_FORMAT_R32G8X24_TYPELESS: return "DXGI_FORMAT_R32G8X24_TYPELESS";
            case DXGI_FORMAT_D32_FLOAT_S8X24_UINT: return "DXGI_FORMAT_D32_FLOAT_S8X24_UINT";
            case DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS: return "DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS";
            case DXGI_FORMAT_X32_TYPELESS_G8X24_UINT: return "DXGI_FORMAT_X32_TYPELESS_G8X24_UINT";
            case DXGI_FORMAT_R10G10B10A2_TYPELESS: return "DXGI_FORMAT_R10G10B10A2_TYPELESS";
            case DXGI_FORMAT_R10G10B10A2_UNORM: return "DXGI_FORMAT_R10G10B10A2_UNORM";
            case DXGI_FORMAT_R10G10B10A2_UINT: return "DXGI_FORMAT_R10G10B10A2_UINT";
            case DXGI_FORMAT_R11G11B10_FLOAT: return "DXGI_FORMAT_R11G11B10_FLOAT";
            case DXGI_FORMAT_R8G8B8A8_TYPELESS: return "DXGI_FORMAT_R8G8B8A8_TYPELESS";
            case DXGI_FORMAT_R8G8B8A8_UNORM: return "DXGI_FORMAT_R8G8B8A8_UNORM";
            case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return "DXGI_FORMAT_R8G8B8A8_UNORM_SRGB";
            case DXGI_FORMAT_R8G8B8A8_UINT: return "DXGI_FORMAT_R8G8B8A8_UINT";
            case DXGI_FORMAT_R8G8B8A8_SNORM: return "DXGI_FORMAT_R8G8B8A8_SNORM";
            case DXGI_FORMAT_R8G8B8A8_SINT: return "DXGI_FORMAT_R8G8B8A8_SINT";
            case DXGI_FORMAT_R16G16_TYPELESS: return "DXGI_FORMAT_R16G16_TYPELESS";
            case DXGI_FORMAT_R16G16_FLOAT: return "DXGI_FORMAT_R16G16_FLOAT";
            case DXGI_FORMAT_R16G16_UNORM: return "DXGI_FORMAT_R16G16_UNORM";
            case DXGI_FORMAT_R16G16_UINT: return "DXGI_FORMAT_R16G16_UINT";
            case DXGI_FORMAT_R16G16_SNORM: return "DXGI_FORMAT_R16G16_SNORM";
            case DXGI_FORMAT_R16G16_SINT: return "DXGI_FORMAT_R16G16_SINT";
            case DXGI_FORMAT_R32_TYPELESS: return "DXGI_FORMAT_R32_TYPELESS";
            case DXGI_FORMAT_D32_FLOAT: return "DXGI_FORMAT_D32_FLOAT";
            case DXGI_FORMAT_R32_FLOAT: return "DXGI_FORMAT_R32_FLOAT";
            case DXGI_FORMAT_R32_UINT: return "DXGI_FORMAT_R32_UINT";
            case DXGI_FORMAT_R32_SINT: return "DXGI_FORMAT_R32_SINT";
            case DXGI_FORMAT_R24G8_TYPELESS: return "DXGI_FORMAT_R24G8_TYPELESS";
            case DXGI_FORMAT_D24_UNORM_S8_UINT: return "DXGI_FORMAT_D24_UNORM_S8_UINT";
            case DXGI_FORMAT_R24_UNORM_X8_TYPELESS: return "DXGI_FORMAT_R24_UNORM_X8_TYPELESS";
            case DXGI_FORMAT_X24_TYPELESS_G8_UINT: return "DXGI_FORMAT_X24_TYPELESS_G8_UINT";
            case DXGI_FORMAT_R8G8_TYPELESS: return "DXGI_FORMAT_R8G8_TYPELESS";
            case DXGI_FORMAT_R8G8_UNORM: return "DXGI_FORMAT_R8G8_UNORM";
            case DXGI_FORMAT_R8G8_UINT: return "DXGI_FORMAT_R8G8_UINT";
            case DXGI_FORMAT_R8G8_SNORM: return "DXGI_FORMAT_R8G8_SNORM";
            case DXGI_FORMAT_R8G8_SINT: return "DXGI_FORMAT_R8G8_SINT";
            case DXGI_FORMAT_R16_TYPELESS: return "DXGI_FORMAT_R16_TYPELESS";
            case DXGI_FORMAT_R16_FLOAT: return "DXGI_FORMAT_R16_FLOAT";
            case DXGI_FORMAT_D16_UNORM: return "DXGI_FORMAT_D16_UNORM";
            case DXGI_FORMAT_R16_UNORM: return "DXGI_FORMAT_R16_UNORM";
            case DXGI_FORMAT_R16_UINT: return "DXGI_FORMAT_R16_UINT";
            case DXGI_FORMAT_R16_SNORM: return "DXGI_FORMAT_R16_SNORM";
            case DXGI_FORMAT_R16_SINT: return "DXGI_FORMAT_R16_SINT";
            case DXGI_FORMAT_R8_TYPELESS: return "DXGI_FORMAT_R8_TYPELESS";
            case DXGI_FORMAT_R8_UNORM: return "DXGI_FORMAT_R8_UNORM";
            case DXGI_FORMAT_R8_UINT: return "DXGI_FORMAT_R8_UINT";
            case DXGI_FORMAT_R8_SNORM: return "DXGI_FORMAT_R8_SNORM";
            case DXGI_FORMAT_R8_SINT: return "DXGI_FORMAT_R8_SINT";
            case DXGI_FORMAT_A8_UNORM: return "DXGI_FORMAT_A8_UNORM";
            case DXGI_FORMAT_R1_UNORM: return "DXGI_FORMAT_R1_UNORM";
            case DXGI_FORMAT_R9G9B9E5_SHAREDEXP: return "DXGI_FORMAT_R9G9B9E5_SHAREDEXP";
            case DXGI_FORMAT_R8G8_B8G8_UNORM: return "DXGI_FORMAT_R8G8_B8G8_UNORM";
            case DXGI_FORMAT_G8R8_G8B8_UNORM: return "DXGI_FORMAT_G8R8_G8B8_UNORM";
            case DXGI_FORMAT_BC1_TYPELESS: return "DXGI_FORMAT_BC1_TYPELESS";
            case DXGI_FORMAT_BC1_UNORM: return "DXGI_FORMAT_BC1_UNORM";
            case DXGI_FORMAT_BC1_UNORM_SRGB: return "DXGI_FORMAT_BC1_UNORM_SRGB";
            case DXGI_FORMAT_BC2_TYPELESS: return "DXGI_FORMAT_BC2_TYPELESS";
            case DXGI_FORMAT_BC2_UNORM: return "DXGI_FORMAT_BC2_UNORM";
            case DXGI_FORMAT_BC2_UNORM_SRGB: return "DXGI_FORMAT_BC2_UNORM_SRGB";
            case DXGI_FORMAT_BC3_TYPELESS: return "DXGI_FORMAT_BC3_TYPELESS";
            case DXGI_FORMAT_BC3_UNORM: return "DXGI_FORMAT_BC3_UNORM";
            case DXGI_FORMAT_BC3_UNORM_SRGB: return "DXGI_FORMAT_BC3_UNORM_SRGB";
            case DXGI_FORMAT_BC4_TYPELESS: return "DXGI_FORMAT_BC4_TYPELESS";
            case DXGI_FORMAT_BC4_UNORM: return "DXGI_FORMAT_BC4_UNORM";
            case DXGI_FORMAT_BC4_SNORM: return "DXGI_FORMAT_BC4_SNORM";
            case DXGI_FORMAT_BC5_TYPELESS: return "DXGI_FORMAT_BC5_TYPELESS";
            case DXGI_FORMAT_BC5_UNORM: return "DXGI_FORMAT_BC5_UNORM";
            case DXGI_FORMAT_BC5_SNORM: return "DXGI_FORMAT_BC5_SNORM";
            case DXGI_FORMAT_B5G6R5_UNORM: return "DXGI_FORMAT_B5G6R5_UNORM";
            case DXGI_FORMAT_B5G5R5A1_UNORM: return "DXGI_FORMAT_B5G5R5A1_UNORM";
            case DXGI_FORMAT_B8G8R8A8_UNORM: return "DXGI_FORMAT_B8G8R8A8_UNORM";
            case DXGI_FORMAT_B8G8R8X8_UNORM: return "DXGI_FORMAT_B8G8R8X8_UNORM";
            case DXGI_FORMAT_B8G8R8A8_TYPELESS: return "DXGI_FORMAT_B8G8R8A8_TYPELESS";
            case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB: return "DXGI_FORMAT_B8G8R8A8_UNORM_SRGB";
            case DXGI_FORMAT_B8G8R8X8_TYPELESS: return "DXGI_FORMAT_B8G8R8X8_TYPELESS";
            case DXGI_FORMAT_B8G8R8X8_UNORM_SRGB: return "DXGI_FORMAT_B8G8R8X8_UNORM_SRGB";
            case DXGI_FORMAT_BC6H_TYPELESS: return "DXGI_FORMAT_BC6H_TYPELESS";
            case DXGI_FORMAT_BC6H_UF16: return "DXGI_FORMAT_BC6H_UF16";
            case DXGI_FORMAT_BC6H_SF16: return "DXGI_FORMAT_BC6H_SF16";
            case DXGI_FORMAT_BC7_TYPELESS: return "DXGI_FORMAT_BC7_TYPELESS";
            case DXGI_FORMAT_BC7_UNORM: return "DXGI_FORMAT_BC7_UNORM";
            case DXGI_FORMAT_BC7_UNORM_SRGB: return "DXGI_FORMAT_BC7_UNORM_SRGB";
            case DXGI_FORMAT_AYUV: return "DXGI_FORMAT_AYUV";
            case DXGI_FORMAT_Y410: return "DXGI_FORMAT_Y410";
            case DXGI_FORMAT_Y416: return "DXGI_FORMAT_Y416";
            case DXGI_FORMAT_NV12: return "DXGI_FORMAT_NV12";
            case DXGI_FORMAT_P010: return "DXGI_FORMAT_P010";
            case DXGI_FORMAT_P016: return "DXGI_FORMAT_P016";
            case DXGI_FORMAT_420_OPAQUE: return "DXGI_FORMAT_420_OPAQUE";
            case DXGI_FORMAT_YUY2: return "DXGI_FORMAT_YUY2";
            case DXGI_FORMAT_Y210: return "DXGI_FORMAT_Y210";
            case DXGI_FORMAT_Y216: return "DXGI_FORMAT_Y216";
            case DXGI_FORMAT_NV11: return "DXGI_FORMAT_NV11";
            case DXGI_FORMAT_AI44: return "DXGI_FORMAT_AI44";
            case DXGI_FORMAT_IA44: return "DXGI_FORMAT_IA44";
            case DXGI_FORMAT_P8: return "DXGI_FORMAT_P8";
            case DXGI_FORMAT_A8P8: return "DXGI_FORMAT_A8P8";
            case DXGI_FORMAT_B4G4R4A4_UNORM: return "DXGI_FORMAT_B4G4R4A4_UNORM";
            case DXGI_FORMAT_P208: return "DXGI_FORMAT_P208";
            case DXGI_FORMAT_V208: return "DXGI_FORMAT_V208";
            case DXGI_FORMAT_V408: return "DXGI_FORMAT_V408";
            case DXGI_FORMAT_FORCE_UINT: return "DXGI_FORMAT_FORCE_UINT";
            default: return "Unknown DXGI_FORMAT";
        }
    }
}