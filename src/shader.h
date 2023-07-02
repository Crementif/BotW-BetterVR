constexpr char presentDepthHLSL[] = R"hlsl(
struct VSInput {
    uint instId : SV_InstanceID;
    uint vertexId : SV_VertexID;
};

struct PSInput {
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD;
};

struct PSOutput {
    float4 Color : SV_TARGET;
    float Depth : SV_DEPTH;
};

cbuffer g_settings : register(b1) {
    float renderWidth;
    float renderHeight;
    float swapchainWidth;
    float swapchainHeight;
};

Texture2D g_colorTexture : register(t0);
Texture2D<float> g_depthTexture : register(t1);
SamplerState g_sampler : register(s0);

PSInput VSMain(VSInput input) {
	PSInput output;
	output.uv = float2(input.vertexId%2, input.vertexId%4/2);

    //float swapchainAspectRatio = swapchainWidth / swapchainHeight;
	output.position = float4((output.uv.x-0.5f)*2.0f, -(output.uv.y-0.5f)*2.0f, 0.0, 1.0);

	return output;
}

PSOutput PSMain(PSInput input) {
	float4 renderColor = float4(0.0, 1.0, 1.0, 1.0);
	float2 samplePosition = input.uv;

    float4 colorTexture = g_colorTexture.Sample(g_sampler, samplePosition);
    float depthTexture = g_depthTexture.Sample(g_sampler, samplePosition);

    PSOutput output;
    output.Color = float4(colorTexture.x, colorTexture.y, colorTexture.z, colorTexture.w);
    output.Depth = depthTexture;
    return output;
}
)hlsl";

constexpr char presentHLSL[] = R"hlsl(
struct VSInput {
    uint instId : SV_InstanceID;
    uint vertexId : SV_VertexID;
};

struct PSInput {
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD;
};

struct PSOutput {
    float4 Color : SV_TARGET;
};

cbuffer g_settings : register(b1) {
    float renderWidth;
    float renderHeight;
    float swapchainWidth;
    float swapchainHeight;
};

Texture2D g_colorTexture : register(t0);
SamplerState g_sampler : register(s0);

PSInput VSMain(VSInput input) {
	PSInput output;
	output.uv = float2(input.vertexId%2, input.vertexId%4/2);

    //float swapchainAspectRatio = swapchainWidth / swapchainHeight;
	output.position = float4((output.uv.x-0.5f)*2.0f, -(output.uv.y-0.5f)*2.0f, 0.0, 1.0);

	return output;
}

PSOutput PSMain(PSInput input) {
	float4 renderColor = float4(0.0, 1.0, 1.0, 1.0);
	float2 samplePosition = input.uv;

    float4 colorTexture = g_colorTexture.Sample(g_sampler, samplePosition);

    PSOutput output;
	//output.Color = float4(0.0f, 0.0f, 0.0f, 0.0f);
    output.Color = float4(colorTexture.x, colorTexture.y, colorTexture.z, colorTexture.w);
    return output;
}
)hlsl";


struct presentSettings {
    float renderWidth;
    float renderHeight;
    float swapchainWidth;
    float swapchainHeight;
//    float eyeSeparation;
//    float showWholeScreen;  // this mode could be used to show each display a part of the screen
//    float showSingleScreen; // this mode shows the same picture in each eye
//    float singleScreenScale;
//    float zoomOutLevel;
//    float gap0;
//    float gap1;
//    float gap2;
};

// clang-format off
constexpr unsigned short screenIndices[] = {
    0, 1, 2,
    2, 1, 3,
};
// clang-format on