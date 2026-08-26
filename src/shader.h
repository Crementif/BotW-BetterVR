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
    float uvOffsetX;
    float uvOffsetY;
    float uvScaleX;
    float uvScaleY;
    float customFadeAmount;
    float customFadeColorR;
    float customFadeColorG;
    float customFadeColorB;
    float isFadeActive;
    float hudMaxAlpha;
};

Texture2D g_colorTexture : register(t0);
Texture2D<float> g_depthTexture : register(t1);
Texture2D g_fadeSampleTexture : register(t2);
SamplerState g_sampler : register(s0);

PSInput VSMain(VSInput input) {
	PSInput output;
	output.uv = float2(input.vertexId%2, input.vertexId%4/2);

    //float swapchainAspectRatio = swapchainWidth / swapchainHeight;
	output.position = float4((output.uv.x-0.5f)*2.0f, -(output.uv.y-0.5f)*2.0f, 0.0, 1.0);

	return output;
}

PSOutput PSMain(PSInput input) {
	float2 samplePosition = float2(uvOffsetX, uvOffsetY) + input.uv * float2(uvScaleX, uvScaleY);

    float4 colorTexture = g_colorTexture.Sample(g_sampler, samplePosition);
    float depthTexture = g_depthTexture.Sample(g_sampler, samplePosition);

    // Game fade: sample the corner texel directly from the 2D layer texture
    float4 fadeSample = g_fadeSampleTexture.Load(int3(0, 0, 0));
    // The 2-bit alpha from A2B10G10R10 gives us 0.0, 0.33, 0.67, 1.0
    float gameFade = isFadeActive > 0.5 ? fadeSample.a : 0.0;
    float3 gameFadeColor = fadeSample.rgb;

    // Custom fade (programmatic, e.g. for turn snapping)
    float customFade = saturate(customFadeAmount);
    float3 customFadeCol = float3(customFadeColorR, customFadeColorG, customFadeColorB);

    // Composite: whichever fade is stronger wins
    float finalFade = max(gameFade, customFade);
    float3 finalFadeColor = gameFade >= customFade ? gameFadeColor : customFadeCol;

    PSOutput output;
    output.Color = float4(lerp(colorTexture.xyz, finalFadeColor, finalFade), colorTexture.w);
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
    float uvOffsetX;
    float uvOffsetY;
    float uvScaleX;
    float uvScaleY;
    float customFadeAmount;
    float customFadeColorR;
    float customFadeColorG;
    float customFadeColorB;
    float isFadeActive;
    float hudMaxAlpha;
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
	float2 samplePosition = float2(uvOffsetX, uvOffsetY) + input.uv * float2(uvScaleX, uvScaleY);

    float4 colorTexture = g_colorTexture.Sample(g_sampler, samplePosition);

    // the dpad quick-menu's darkening backdrop is baked into the HUD capture as opaque black
    // (correct in flatscreen where it draws into the same buffer as the 3D scene, but it would
    // fully occlude our separately-composited HUD layer), so cap its alpha instead of letting it through
    PSOutput output;
	//output.Color = float4(0.0f, 0.0f, 0.0f, 0.0f);
    output.Color = float4(colorTexture.x, colorTexture.y, colorTexture.z, min(colorTexture.w, hudMaxAlpha));
    return output;
}
)hlsl";

constexpr char debugDrawLineHLSL[] = R"hlsl(
struct VSInput {
    float3 position : POSITION;
    float4 color : COLOR0;
};

struct PSInput {
    float4 position : SV_POSITION;
    float4 color : COLOR0;
};

struct PSOutput {
    float4 Color : SV_TARGET;
    float Depth : SV_DEPTH;
};

cbuffer g_scene : register(b0) {
    float4x4 viewProjection;
    float targetWidth;
    float targetHeight;
    float depthBias;
    float xrayAlphaScale;
};

Texture2D<float> g_sceneDepthTexture : register(t0);
SamplerState g_sampler : register(s0);

PSInput VSMain(VSInput input) {
    PSInput output;
    output.position = mul(viewProjection, float4(input.position, 1.0f));
    output.color = input.color;
    return output;
}

PSOutput PSMain(PSInput input) {
    float2 uv = float2(input.position.x / targetWidth, input.position.y / targetHeight);
    float sceneDepth = g_sceneDepthTexture.Sample(g_sampler, uv);
    float overlayDepth = saturate(input.position.z - depthBias);

    if (xrayAlphaScale <= 0.0f && overlayDepth > sceneDepth + depthBias) {
        discard;
    }

    PSOutput output;
    output.Color = input.color;
    output.Color.a *= xrayAlphaScale > 0.0f ? xrayAlphaScale : 1.0f;
    output.Depth = overlayDepth;
    return output;
}
)hlsl";


struct presentSettings {
    float renderWidth;
    float renderHeight;
    float swapchainWidth;
    float swapchainHeight;
    float uvOffsetX;
    float uvOffsetY;
    float uvScaleX;
    float uvScaleY;
    float customFadeAmount;
    float customFadeColorR;
    float customFadeColorG;
    float customFadeColorB;
    float isFadeActive;
    float hudMaxAlpha;
};

struct debugDrawLineSettings {
    glm::mat4 viewProjection;
    float targetWidth;
    float targetHeight;
    float depthBias;
    float xrayAlphaScale;
};

// clang-format off
constexpr unsigned short screenIndices[] = {
    0, 1, 2,
    2, 1, 3,
};
// clang-format on
