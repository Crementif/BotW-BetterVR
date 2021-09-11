#include "layer.h"

XrInstance xrInstanceHandle = XR_NULL_HANDLE;
XrSession xrSessionHandle = XR_NULL_HANDLE;
XrSystemId xrSystemId = XR_NULL_SYSTEM_ID;
XrSpace xrSpaceHandle = XR_NULL_HANDLE;
XrDebugUtilsMessengerEXT xrDebugMessengerHandle = XR_NULL_HANDLE;
XrGraphicsBindingD3D11KHR  xrVulkanBindings = {};
uint32_t hmdViewCount = 0;

XrSwapchain xrSwapchainHandle = XR_NULL_HANDLE;
DXGI_FORMAT xrSwapchainFormat;
XrRect2Di xrSwapchainSize = { {0, 0}, {0, 0} };
std::vector<XrSwapchainImageD3D11KHR> xrSwapchainImages;

winrt::com_ptr<IDXGIFactory1> dxgiFactory;
winrt::com_ptr<IDXGIAdapter1> dxgiAdapter;
ID3D11Device* device;
ID3D11DeviceContext* deviceContext;
IDXGIAdapter1* adapter;
ID3D11PixelShader* pixelShader;
ID3D11VertexShader* vertexShader;
ID3D11Buffer* screenIndicesBuffer;
ID3D11Buffer* constantBuffer;
ID3D11Resource* constantBufferResource;

ID3D11SamplerState* textureSampler;
XrView leftView;
XrView rightView;
sideTextureResources leftEyeResources = {};
sideTextureResources rightEyeResources = {};

struct constantBufferShader {
	float renderWidth;
	float renderHeight;
	float swapchainWidth;
	float swapchainHeight;
	float eyeSeparation;
	float showWholeScreen; // this mode could be used to show each display a part of the screen
	float showSingleScreen; // this mode shows the same picture in each eye
	float singleScreenScale;
	float zoomOutLevel;
	float gap0;
	float gap1;
	float gap2;
};
constexpr unsigned short screenIndices[] = {
	0,  1,  2,  2,  1,  3,
};

constexpr XrPosef xrIdentityPose = { {0, 0, 0, 1}, {0, 0, 0} };
std::vector<XrViewConfigurationView> viewConfigurations;

constexpr char shaderHLSL[] = R"_(
	struct VSInput {
	    uint instId : SV_InstanceID;
	    uint vertexId : SV_VertexID;
	};
	struct PSInput {
	    float4 Pos : SV_POSITION;
		float2 Tex : TEXCOORD0;
	    uint viewId : SV_RenderTargetArrayIndex;
	};
	
	cbuffer ViewportSizeConstantBuffer : register(b0) {
		float renderWidth;
		float renderHeight;
	    float swapchainWidth;
		float swapchainHeight;
		float eyeSeparation;
		float showWholeScreen;
		float showSingleScreen;
		float singleScreenScale;
		float zoomOutLevel;
		float gap0;
		float gap1;
		float gap2;
	};
	
	SamplerState textureSampler : register(s0);
	Texture2D leftTexture : register(t0);
	Texture2D rightTexture : register(t1);
	
	PSInput VSMain(VSInput input) {
		PSInput output;
		output.Tex = float2(input.vertexId%2, input.vertexId%4/2);
		output.Pos = float4((output.Tex.x-0.5f)*2.0, -(output.Tex.y-0.5f)*2, 0.0, 1.0);
	    output.viewId = input.instId;
	    return output;
	}
	
	float4 PSMain(PSInput input) : SV_TARGET {
		float4 renderColor = float4(0.0, 1.0, 1.0, 1.0);
		float2 samplePosition = input.Tex;
		if (!showWholeScreen) {
			float xScale = (swapchainWidth/swapchainHeight) / (renderWidth/renderHeight);
			float leftRemainsWidth = (1.0 - xScale) / 2.0;
			samplePosition.x = samplePosition.x * xScale + leftRemainsWidth;

			if (showSingleScreen) {
				float stretchBackScale = 1.0 / ((16.0f/9.0f) / (renderWidth/renderHeight));
				samplePosition.x = samplePosition.x * stretchBackScale - (stretchBackScale * 0.5) + 0.5;
			}

			if (showSingleScreen) {
				samplePosition = (samplePosition - 0.5) * (2.0 * (1.0+(singleScreenScale*0.5))) + 0.5;
				if (input.viewId == 0) {
					samplePosition.x -= (eyeSeparation * 2.0);
				}
				else {
					samplePosition.x += (eyeSeparation * 2.0);
				}
			}
			else {
				samplePosition.x = input.Tex.x;
				if (input.viewId == 0) {
					samplePosition.x -= (eyeSeparation);
				}
				else {
					samplePosition.x += (eyeSeparation);
				}
			}
			if (input.viewId == 0) {
		    	renderColor = leftTexture.Sample(textureSampler, samplePosition);
			}
			else {
				renderColor = rightTexture.Sample(textureSampler, samplePosition);
			}
        }
		else {
			samplePosition.x /= 2.0;
			renderColor = leftTexture.Sample(textureSampler, samplePosition);
		}
        return renderColor;
	}
)_";

void dx11LogDebugUtilsMessenger(XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageType, const XrDebugUtilsMessengerCallbackDataEXT* callbackData, void* userData) {
	logPrint(std::string(callbackData->functionName) + std::string(": ") + std::string(callbackData->message));
}

void dx11Initialize() {
	uint32_t xrExtensionCount = 0;
	xrEnumerateInstanceExtensionProperties(NULL, 0, &xrExtensionCount, NULL);
	std::vector<XrExtensionProperties> instanceExtensions;
	instanceExtensions.resize(xrExtensionCount, { XR_TYPE_EXTENSION_PROPERTIES });

	checkXRResult(xrEnumerateInstanceExtensionProperties(NULL, xrExtensionCount, &xrExtensionCount, instanceExtensions.data()), "Failed to enumerate OpenXR instance extension properties!");

	bool d3d11Supported = false;
	bool convertTimeSupported = false;
	bool debugUtilsSupported = false;
	for (XrExtensionProperties& extensionProperties : instanceExtensions) {
		if (strcmp(extensionProperties.extensionName, XR_KHR_D3D11_ENABLE_EXTENSION_NAME) == 0) {
			d3d11Supported = true;
		}
		else if (strcmp(extensionProperties.extensionName, XR_EXT_DEBUG_UTILS_EXTENSION_NAME) == 0) {
			convertTimeSupported = true;
		}
		else if (strcmp(extensionProperties.extensionName, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME) == 0) {
			debugUtilsSupported = true;
		}
	}

	if (!d3d11Supported) {
		logPrint("OpenXR runtime doesn't support Direct3D 11!");
		throw std::runtime_error("Current OpenXR runtime doesn't support Direct3D 11. See the Github page's troubleshooting section for a solution!");
	}

	if (!convertTimeSupported || !debugUtilsSupported) {
		logPrint("OpenXR runtime doesn't support required extensions!");
		throw std::runtime_error("Current OpenXR runtime doesn't support the required extensions (XR_EXT_debug_utils or XR_KHR_win32_convert_performance_counter_time)");
	}

	std::vector<const char*> enabledExtensions = { XR_KHR_D3D11_ENABLE_EXTENSION_NAME, XR_EXT_DEBUG_UTILS_EXTENSION_NAME, XR_KHR_WIN32_CONVERT_PERFORMANCE_COUNTER_TIME_EXTENSION_NAME };

	XrInstanceCreateInfo xrInstanceCreateInfo = { XR_TYPE_INSTANCE_CREATE_INFO };
	xrInstanceCreateInfo.createFlags = 0;
	xrInstanceCreateInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
	xrInstanceCreateInfo.enabledExtensionNames = enabledExtensions.data();
	xrInstanceCreateInfo.enabledApiLayerCount = 0;
	xrInstanceCreateInfo.enabledApiLayerNames = NULL;
	xrInstanceCreateInfo.applicationInfo = { "BotW BetterVR", 1, "Cemu", 1, XR_CURRENT_API_VERSION };

	checkXRResult(xrCreateInstance(&xrInstanceCreateInfo, &xrInstanceHandle), "Failed to create the OpenXR instance!");

	checkXRResult(xrGetInstanceProcAddr(xrInstanceHandle, "xrGetD3D11GraphicsRequirementsKHR", (PFN_xrVoidFunction*)&func_xrGetD3D11GraphicsRequirementsKHR), "Failed to get the function pointer of xrGetD3D11GraphicsRequirementsKHR!");
	checkXRResult(xrGetInstanceProcAddr(xrInstanceHandle, "xrConvertTimeToWin32PerformanceCounterKHR", (PFN_xrVoidFunction*)&func_xrConvertTimeToWin32PerformanceCounterKHR), "Failed to get the function pointer of xrConvertTimeToWin32PerformanceCounterKHR!");
	checkXRResult(xrGetInstanceProcAddr(xrInstanceHandle, "xrConvertWin32PerformanceCounterToTimeKHR", (PFN_xrVoidFunction*)&func_xrConvertWin32PerformanceCounterToTimeKHR), "Failed to get the function pointer of xrConvertWin32PerformanceCounterToTimeKHR!");
	checkXRResult(xrGetInstanceProcAddr(xrInstanceHandle, "xrCreateDebugUtilsMessengerEXT", (PFN_xrVoidFunction*)&func_xrCreateDebugUtilsMessengerEXT), "Failed to get the function pointer of xrCreateDebugUtilsMessengerEXT!");

	XrDebugUtilsMessengerCreateInfoEXT utilsMessengerCreateInfo = { XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
	utilsMessengerCreateInfo.messageSeverities = XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	utilsMessengerCreateInfo.messageTypes = XR_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT | XR_DEBUG_UTILS_MESSAGE_TYPE_CONFORMANCE_BIT_EXT;
	utilsMessengerCreateInfo.userCallback = (PFN_xrDebugUtilsMessengerCallbackEXT)&dx11LogDebugUtilsMessenger;
	func_xrCreateDebugUtilsMessengerEXT(xrInstanceHandle, &utilsMessengerCreateInfo, &xrDebugMessengerHandle);

	XrSystemGetInfo xrSystemGetInfo = { XR_TYPE_SYSTEM_GET_INFO, NULL, XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
	checkXRResult(xrGetSystem(xrInstanceHandle, &xrSystemGetInfo, &xrSystemId), "Failed to get the OpenXR system!");

	xrEnumerateViewConfigurationViews(xrInstanceHandle, xrSystemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &hmdViewCount, NULL);
	viewConfigurations.resize(hmdViewCount, { XR_TYPE_VIEW_CONFIGURATION_VIEW });
	checkXRResult(xrEnumerateViewConfigurationViews(xrInstanceHandle, xrSystemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, hmdViewCount, &hmdViewCount, viewConfigurations.data()), "Failed to enumerate view configurations of this VR system!");

	XrViewConfigurationProperties viewConfigurationProperties = { XR_TYPE_VIEW_CONFIGURATION_PROPERTIES };
	checkXRResult(xrGetViewConfigurationProperties(xrInstanceHandle, xrSystemId, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, &viewConfigurationProperties), "The current headset doesn't support stereo rendering!");

	xrSwapchainSize.extent.width = viewConfigurations[0].recommendedImageRectWidth;
	xrSwapchainSize.extent.height = viewConfigurations[0].recommendedImageRectHeight;

	XrGraphicsRequirementsD3D11KHR graphicsRequirements = { XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR };
	func_xrGetD3D11GraphicsRequirementsKHR(xrInstanceHandle, xrSystemId, &graphicsRequirements);

	checkHResult(CreateDXGIFactory1(winrt::guid_of<IDXGIFactory1>(), dxgiFactory.put_void()), "Failed to create the DXGIFactory!");

	for (UINT adapterIndex = 0;; adapterIndex++) {
		winrt::com_ptr<IDXGIAdapter1> tempAdapter;
		dxgiFactory->EnumAdapters1(adapterIndex, tempAdapter.put());
		DXGI_ADAPTER_DESC1 adapterDesc;
		tempAdapter->GetDesc1(&adapterDesc);
		if (memcmp(&adapterDesc.AdapterLuid, &graphicsRequirements.adapterLuid, sizeof(graphicsRequirements.adapterLuid)) == 0) {
			char gpuString[256];
			snprintf(gpuString, 256, "Using %ws as the VR GPU!", adapterDesc.Description);
			logPrint(gpuString);
			dxgiAdapter = tempAdapter;
			break;
		}
	}

	UINT creationFlags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
#ifdef _DEBUG
	creationFlags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

	if (graphicsRequirements.minFeatureLevel >= D3D_FEATURE_LEVEL_11_1)
		throw std::runtime_error("Runtime doesn't support Direct3D feature level 11_0 or 11_1!");

	std::vector<D3D_FEATURE_LEVEL> featureLevels = {
		D3D_FEATURE_LEVEL_11_0,
		D3D_FEATURE_LEVEL_11_1
	};

	checkHResult(D3D11CreateDevice(adapter, D3D_DRIVER_TYPE_HARDWARE, 0, creationFlags, featureLevels.data(), (UINT)featureLevels.size(), D3D11_SDK_VERSION, &device, nullptr, &deviceContext), "Failed to create the DX11 device!");

	logPrint("Initialized D3D11 app!");

	{
		ID3DBlob* shaderBytes;
		DWORD shaderCompileFlags = D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR | D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_WARNINGS_ARE_ERRORS;
#ifdef _DEBUG
		shaderCompileFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
		shaderCompileFlags |= D3DCOMPILE_DEBUG;
#else
		shaderCompileFlags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
		ID3DBlob* hlslCompilationErrors;
		if (FAILED(D3DCompile(shaderHLSL, strlen(shaderHLSL), nullptr, nullptr, nullptr, "VSMain", "vs_5_0", shaderCompileFlags, 0, &shaderBytes, &hlslCompilationErrors))) {
			std::string errorMessage((const char*)hlslCompilationErrors->GetBufferPointer(), hlslCompilationErrors->GetBufferSize());
			logPrint("Vertex Shader Compilation Error:");
			logPrint(errorMessage);
			throw std::runtime_error("Error during the vertex shader compilation!");
		}
		checkHResult(device->CreateVertexShader(shaderBytes->GetBufferPointer(), shaderBytes->GetBufferSize(), nullptr, &vertexShader), "Failed to create the vertex shader!");
	}
	{
		ID3DBlob* shaderBytes;
		DWORD shaderCompileFlags = D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR | D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_WARNINGS_ARE_ERRORS;
#ifdef _DEBUG
		shaderCompileFlags |= D3DCOMPILE_SKIP_OPTIMIZATION;
		shaderCompileFlags |= D3DCOMPILE_DEBUG;
#else
		shaderCompileFlags |= D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
		ID3DBlob* hlslCompilationErrors;
		if (FAILED(D3DCompile(shaderHLSL, strlen(shaderHLSL), nullptr, nullptr, nullptr, "PSMain", "ps_5_0", shaderCompileFlags, 0, &shaderBytes, &hlslCompilationErrors))) {
			std::string errorMessage((const char*)hlslCompilationErrors->GetBufferPointer(), hlslCompilationErrors->GetBufferSize());
			logPrint("Pixel Shader Compilation Error:");
			logPrint(errorMessage);
			throw std::runtime_error("Error during the pixel shader compilation!");
		}
		checkHResult(device->CreatePixelShader(shaderBytes->GetBufferPointer(), shaderBytes->GetBufferSize(), nullptr, &pixelShader), "Failed to create the pixel shader!");
	}

	D3D11_SAMPLER_DESC textureSamplerDesc;
	textureSamplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	textureSamplerDesc.AddressU = D3D11_TEXTURE_ADDRESS_BORDER;
	textureSamplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
	textureSamplerDesc.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
	textureSamplerDesc.MipLODBias = 0;
	textureSamplerDesc.MaxAnisotropy = 1;
	textureSamplerDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
	textureSamplerDesc.BorderColor[0] = 0.0f;
	textureSamplerDesc.BorderColor[1] = 0.0f;
	textureSamplerDesc.BorderColor[2] = 0.0f;
	textureSamplerDesc.BorderColor[3] = 1.0f;
	textureSamplerDesc.MinLOD = -FLT_MAX;
	textureSamplerDesc.MaxLOD = FLT_MAX;
	device->CreateSamplerState(&textureSamplerDesc, &textureSampler);

	const CD3D11_BUFFER_DESC constantBufferDesc(sizeof(constantBufferShader), D3D11_BIND_CONSTANT_BUFFER);
	device->CreateBuffer(&constantBufferDesc, NULL, &constantBuffer);
	constantBuffer->QueryInterface(__uuidof(ID3D11Resource), (void**)&constantBufferResource);
	deviceContext->PSSetConstantBuffers(0, 1, &constantBuffer);
	deviceContext->VSSetConstantBuffers(0, 1, &constantBuffer);

	const D3D11_SUBRESOURCE_DATA indexBufferData = { screenIndices };
	const CD3D11_BUFFER_DESC indexBufferDesc(sizeof(screenIndices), D3D11_BIND_INDEX_BUFFER);
	device->CreateBuffer(&indexBufferDesc, &indexBufferData, &screenIndicesBuffer);

	logPrint("Initialized D3D11 resources!");

	XrGraphicsBindingD3D11KHR graphicsBinding = { XR_TYPE_GRAPHICS_BINDING_D3D11_KHR };
	graphicsBinding.device = device;

	XrSessionCreateInfo createInfo = { XR_TYPE_SESSION_CREATE_INFO };
	createInfo.systemId = xrSystemId;
	createInfo.next = &graphicsBinding;
	checkXRResult(xrCreateSession(xrInstanceHandle, &createInfo, &xrSessionHandle), "Failed to create OpenXR session!");

	logPrint("Successfully created OpenXR session!");

	uint32_t swapchainFormatCount;
	xrEnumerateSwapchainFormats(xrSessionHandle, 0, &swapchainFormatCount, nullptr);
	std::vector<int64_t> swapchainFormats(swapchainFormatCount);
	xrEnumerateSwapchainFormats(xrSessionHandle, (uint32_t)swapchainFormats.size(), &swapchainFormatCount, swapchainFormats.data());

	std::vector<DXGI_FORMAT> preferredColorFormats = {
		DXGI_FORMAT_R8G8B8A8_UNORM,
		DXGI_FORMAT_B8G8R8A8_UNORM,
		DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,
		DXGI_FORMAT_B8G8R8A8_UNORM_SRGB,
	};

	std::vector<int64_t>::iterator foundFormat = std::find_first_of(std::begin(swapchainFormats), std::end(swapchainFormats), std::begin(preferredColorFormats), std::end(preferredColorFormats));
	if (foundFormat == std::end(swapchainFormats)) {
		throw std::runtime_error("Failed to find an applicable OpenXR swapchain format that is also supported by the runtime.");
	}
	xrSwapchainFormat = (DXGI_FORMAT)*foundFormat;

	XrSwapchainCreateInfo swapchainCreateInfo = { XR_TYPE_SWAPCHAIN_CREATE_INFO };
	swapchainCreateInfo.width = xrSwapchainSize.extent.width;
	swapchainCreateInfo.height = xrSwapchainSize.extent.height;
	swapchainCreateInfo.arraySize = hmdViewCount;
	swapchainCreateInfo.sampleCount = viewConfigurations[0].recommendedSwapchainSampleCount;
	swapchainCreateInfo.format = xrSwapchainFormat;
	swapchainCreateInfo.mipCount = 1;
	swapchainCreateInfo.faceCount = 1;
	swapchainCreateInfo.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
	checkXRResult(xrCreateSwapchain(xrSessionHandle, &swapchainCreateInfo, &xrSwapchainHandle), "Failed to create OpenXR swapchain images!");

	uint32_t swapchainChainCount = 0;
	xrEnumerateSwapchainImages(xrSwapchainHandle, 0, &swapchainChainCount, NULL);
	xrSwapchainImages.resize(swapchainChainCount, { XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR });
	checkXRResult(xrEnumerateSwapchainImages(xrSwapchainHandle, swapchainChainCount, &swapchainChainCount, reinterpret_cast<XrSwapchainImageBaseHeader*>(xrSwapchainImages.data())), "Failed to enumerate OpenXR swapchain images!");
	logPrint("Successfully created OpenXR swapchains!");

	XrReferenceSpaceCreateInfo spaceCreateInfo = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO };
	spaceCreateInfo.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
	spaceCreateInfo.poseInReferenceSpace = xrIdentityPose;
	checkXRResult(xrCreateReferenceSpace(xrSessionHandle, &spaceCreateInfo, &xrSpaceHandle), "Failed to create reference space!");
	logPrint("Successfully created OpenXR reference space!");

	XrSessionBeginInfo sessionBeginInfo = { XR_TYPE_SESSION_BEGIN_INFO };
	sessionBeginInfo.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
	checkXRResult(xrBeginSession(xrSessionHandle, &sessionBeginInfo), "Failed to begin OpenXR session!");
	logPrint("Successfully begun OpenXR session!");
}

void dx11CreateSideTexture(sideTextureResources* texture) {
	D3D11_TEXTURE2D_DESC externalTextureDesc = {};
	externalTextureDesc.Format = DXGI_FORMAT_R10G10B10A2_UNORM;
	externalTextureDesc.Width = texture->width;
	externalTextureDesc.Height = texture->height;
	externalTextureDesc.MipLevels = 1;
	externalTextureDesc.ArraySize = 1;
	externalTextureDesc.SampleDesc.Count = 1;
	externalTextureDesc.SampleDesc.Quality = 0;
	externalTextureDesc.Usage = D3D11_USAGE_DEFAULT;
	externalTextureDesc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
	externalTextureDesc.CPUAccessFlags = 0;
	externalTextureDesc.MiscFlags = D3D11_RESOURCE_MISC_SHARED_NTHANDLE | D3D11_RESOURCE_MISC_SHARED_KEYEDMUTEX;

	checkHResult(device->CreateTexture2D(&externalTextureDesc, NULL, &texture->dx11Texture), "Failed to create external texture on D3D11 side!");

	D3D11_SHADER_RESOURCE_VIEW_DESC textureResourceViewDesc;
	textureResourceViewDesc.Format = DXGI_FORMAT_R10G10B10A2_UNORM;
	textureResourceViewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	textureResourceViewDesc.Texture2D.MostDetailedMip = 0;
	textureResourceViewDesc.Texture2D.MipLevels = 1;

	texture->dx11Texture->QueryInterface(__uuidof(ID3D11Resource), (void**)&texture->dx11TextureResource);
	checkHResult(device->CreateShaderResourceView(texture->dx11TextureResource, &textureResourceViewDesc, &texture->dx11ResourceView));

	IDXGIResource1* tempResource = NULL;
	texture->dx11Texture->QueryInterface(__uuidof(IDXGIResource1), (void**)&tempResource);
	texture->dx11Texture->QueryInterface(__uuidof(IDXGIKeyedMutex), (void**)&texture->keyedMutex);
	checkHResult(tempResource->CreateSharedHandle(NULL, DXGI_SHARED_RESOURCE_READ | DXGI_SHARED_RESOURCE_WRITE, NULL, &texture->externalHandle), "Failed to create a shared handle to the external texture");
	tempResource->Release();

	checkHResult(texture->keyedMutex->AcquireSync(0, INFINITE), "Failed to initialize external texture's keyed mutex!");
	checkHResult(texture->keyedMutex->ReleaseSync(1), "Failed to init release the external texture's keyed mutex!");
}

void dx11Shutdown() {
	//leftTexture->Release();
	//leftTextureResource->Release();
	//leftTextureView->Release();
	//rightTexture->Release();
	//rightTextureResource->Release();
	//rightTextureView->Release();

	//externalTexture->Release();
	//externalTextureKeyedMutex->Release();
	//externalTextureResource->Release();
	//externalTextureView->Release();
	//textureSampler->Release();

	screenIndicesBuffer->Release();
	constantBuffer->Release();
	vertexShader->Release();
	pixelShader->Release();
	adapter->Release();
	deviceContext->Release();
	device->Release();
	dxgiAdapter->Release();
	dxgiFactory->Release();

	xrDestroySwapchain(xrSwapchainHandle);
	xrDestroySpace(xrSpaceHandle);
	xrEndSession(xrSessionHandle);
	xrDestroySession(xrSessionHandle);
	xrDestroyInstance(xrInstanceHandle);
}

void dx11Thread() {
	SetThreadDescription(GetCurrentThread(), L"BetterVR DX11 Rendering Thread");
	while (true) {
		dx11Render();
	}
}

bool shouldRender = false;


void dx11Render() {
	XrFrameWaitInfo frameWaitInfo = { XR_TYPE_FRAME_WAIT_INFO };
	XrFrameState frameState = { XR_TYPE_FRAME_STATE };
	checkXRResult(xrWaitFrame(xrSessionHandle, &frameWaitInfo, &frameState), "Failed to wait for the OpenXR frame!");

	LARGE_INTEGER predictedWin32Time;
	func_xrConvertTimeToWin32PerformanceCounterKHR(xrInstanceHandle, frameState.predictedDisplayTime, &predictedWin32Time);
	//dx11PrintQPCTime("Predicted display time is ", predictedWin32Time);

	XrFrameBeginInfo frameBeginInfo = { XR_TYPE_FRAME_BEGIN_INFO };
	checkXRResult(xrBeginFrame(xrSessionHandle, &frameBeginInfo), "Failed to begin the OpenXR frame!");

	std::vector<XrCompositionLayerBaseHeader*> layers;
	std::vector<XrCompositionLayerProjectionView> projectionLayerViews(hmdViewCount, { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW });

	shouldRender = false;
	if (/*frameState.shouldRender*/true) { // Oculus runtime will never render anything otherwise
		XrViewLocateInfo viewLocateInfo = { XR_TYPE_VIEW_LOCATE_INFO };
		viewLocateInfo.displayTime = frameState.predictedDisplayTime;
		viewLocateInfo.space = xrSpaceHandle;
		viewLocateInfo.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

		XrViewState viewState = { XR_TYPE_VIEW_STATE };
		XrView views[2] = { { XR_TYPE_VIEW }, { XR_TYPE_VIEW } };
		uint32_t viewCountOutput;
		checkXRResult(xrLocateViews(xrSessionHandle, &viewLocateInfo, &viewState, (uint32_t)viewConfigurations.size(), &viewCountOutput, views), "Failed to locate views in OpenXR!");

		if (viewState.viewStateFlags & (XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_VALID_BIT)) {
			leftView = views[0];
			rightView = views[1];
			XrSwapchainImageAcquireInfo acquireInfo = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
			uint32_t swapchainImageIndex = 0;
			if (XR_FAILED(xrAcquireSwapchainImage(xrSwapchainHandle, &acquireInfo, &swapchainImageIndex))) {
				throw std::runtime_error("Failed to acquire OpenXR swapchain image!");
			}

			XrCompositionLayerProjection layer = { XR_TYPE_COMPOSITION_LAYER_PROJECTION };
			layer.layerFlags = 0;

			XrSwapchainImageWaitInfo waitInfo = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
			waitInfo.timeout = XR_INFINITE_DURATION;
			checkXRResult(xrWaitSwapchainImage(xrSwapchainHandle, &waitInfo));

			for (uint32_t i = 0; i < hmdViewCount; i++) {
				projectionLayerViews[i].pose = views[i].pose;
				projectionLayerViews[i].subImage.swapchain = xrSwapchainHandle;
				projectionLayerViews[i].subImage.imageRect = xrSwapchainSize;
				projectionLayerViews[i].subImage.imageArrayIndex = i;
			}

			shouldRender = true;
			dx11RenderLayer(xrSwapchainImages[swapchainImageIndex].texture, &leftEyeResources);

			XrSwapchainImageReleaseInfo releaseInfo = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
			checkXRResult(xrReleaseSwapchainImage(xrSwapchainHandle, &releaseInfo), "Failed to release the OpenXR swapchain image!");

			layer.space = xrSpaceHandle;
			layer.viewCount = (uint32_t)projectionLayerViews.size();
			layer.views = projectionLayerViews.data();
			layers.emplace_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&layer));
		}
	}

	XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO };
	frameEndInfo.displayTime = frameState.predictedDisplayTime;
	frameEndInfo.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
	frameEndInfo.layerCount = (uint32_t)layers.size();
	frameEndInfo.layers = layers.data();
	checkXRResult(xrEndFrame(xrSessionHandle, &frameEndInfo), "Failed to end the OpenXR frame!");
}

void dx11RenderLayer(ID3D11Texture2D* swapchainTargetTexture, sideTextureResources* sideTexture) {
	CD3D11_VIEWPORT viewportSize((float)xrSwapchainSize.offset.x, (float)xrSwapchainSize.offset.y, (float)xrSwapchainSize.extent.width, (float)xrSwapchainSize.extent.height);
	deviceContext->RSSetViewports(1, &viewportSize);

	ID3D11RenderTargetView* renderTargetView;
	const CD3D11_RENDER_TARGET_VIEW_DESC renderTargetViewDesc(D3D11_RTV_DIMENSION_TEXTURE2DARRAY, xrSwapchainFormat);
	device->CreateRenderTargetView(swapchainTargetTexture, &renderTargetViewDesc, &renderTargetView);

	deviceContext->ClearRenderTargetView(renderTargetView, DirectX::XMVECTORF32{ 1.000000000f, 1.000000000f, 1.000000000f, 1.000000000f });

	glm::vec3 leftEyePos(leftView.pose.position.x, leftView.pose.position.y, leftView.pose.position.z);
	glm::vec3 rightEyePos(rightView.pose.position.x, rightView.pose.position.y, rightView.pose.position.z);

	//logPrint("Eye separation reported by OpenXR = " + std::to_string(glm::distance(leftEyePos, rightEyePos)));
	//logPrint("FOV left reported by OpenXR = " + std::to_string(leftView.fov.angleLeft));
	//logPrint("FOV right reported by OpenXR = " + std::to_string(leftView.fov.angleRight));
	//logPrint("FOV up reported by OpenXR = " + std::to_string(leftView.fov.angleUp));
	//logPrint("FOV down reported by OpenXR = " + std::to_string(leftView.fov.angleDown));

	constantBufferShader renderData;
	renderData.renderWidth = (float)sideTexture->width;
	renderData.renderHeight = (float)sideTexture->height;
	renderData.swapchainWidth = (float)xrSwapchainSize.extent.width;
	renderData.swapchainHeight = (float)xrSwapchainSize.extent.height;
	renderData.eyeSeparation = glm::distance(leftEyePos, rightEyePos);
	renderData.showWholeScreen = !sideBySideRenderingMode;
	renderData.showSingleScreen = (float)!cameraIsInGame();
	renderData.singleScreenScale = cameraGetMenuZoom();
	renderData.zoomOutLevel = 1;
	deviceContext->UpdateSubresource(constantBufferResource, 0, NULL, &renderData, sizeof(constantBufferShader), 0);

	LARGE_INTEGER beforeAcquire;
	QueryPerformanceCounter(&beforeAcquire);
	checkMutexHResult(leftEyeResources.keyedMutex->AcquireSync(1, INFINITE));
	checkMutexHResult(rightEyeResources.keyedMutex->AcquireSync(1, INFINITE));
	//logTimeElapsed("Acquiring the lock on the dx11 side took ", beforeAcquire);

	deviceContext->PSSetShaderResources(0, 1, cameraUseSwappedFlipMode() ? &rightEyeResources.dx11ResourceView : &leftEyeResources.dx11ResourceView);
	deviceContext->PSSetShaderResources(1, 1, cameraUseSwappedFlipMode() ? &leftEyeResources.dx11ResourceView : &rightEyeResources.dx11ResourceView);
	deviceContext->PSSetSamplers(0, 1, &textureSampler);

	deviceContext->OMSetRenderTargets(1, &renderTargetView, NULL);

	deviceContext->VSSetShader(vertexShader, nullptr, 0);
	deviceContext->PSSetShader(pixelShader, nullptr, 0);

	deviceContext->IASetIndexBuffer(screenIndicesBuffer, DXGI_FORMAT_R16_UINT, 0);
	deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	deviceContext->DrawIndexedInstanced((UINT)std::size(screenIndices), hmdViewCount, 0, 0, 0);

	checkMutexHResult(leftEyeResources.keyedMutex->ReleaseSync(1));
	checkMutexHResult(rightEyeResources.keyedMutex->ReleaseSync(1));
}

bool dx11ShouldRender() {
	return shouldRender;
}

bool dx11IsRunning() {
	return xrInstanceHandle != XR_NULL_HANDLE;
}