#version 450

layout(location = 0) in vec2 vUv;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D g_colorTexture;
layout(set = 0, binding = 1) uniform sampler2D g_depthTexture;
layout(set = 0, binding = 2) uniform sampler2D g_fadeSampleTexture;

layout(push_constant) uniform PresentSettings {
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
    float srgbEncode;
} settings;

vec3 EncodeSrgb(vec3 color) {
    vec3 clamped = clamp(color, vec3(0.0), vec3(1.0));
    vec3 low = clamped * 12.92;
    vec3 high = 1.055 * pow(clamped, vec3(1.0 / 2.4)) - 0.055;
    return mix(low, high, greaterThan(clamped, vec3(0.0031308)));
}

void main() {
    vec2 samplePosition = vec2(settings.uvOffsetX, settings.uvOffsetY) + vUv * vec2(settings.uvScaleX, settings.uvScaleY);

    vec4 colorTexture = texture(g_colorTexture, samplePosition);
    float depthTexture = texture(g_depthTexture, samplePosition).r;

    // Game fade: sample the corner texel directly from the 2D layer texture
    vec4 fadeSample = texelFetch(g_fadeSampleTexture, ivec2(0, 0), 0);
    // The 2-bit alpha from A2B10G10R10 gives us 0.0, 0.33, 0.67, 1.0
    float gameFade = settings.isFadeActive > 0.5 ? fadeSample.a : 0.0;
    vec3 gameFadeColor = fadeSample.rgb;

    // Custom fade (programmatic, e.g. for turn snapping)
    float customFade = clamp(settings.customFadeAmount, 0.0, 1.0);
    vec3 customFadeCol = vec3(settings.customFadeColorR, settings.customFadeColorG, settings.customFadeColorB);

    // Composite: whichever fade is stronger wins
    float finalFade = max(gameFade, customFade);
    vec3 finalFadeColor = gameFade >= customFade ? gameFadeColor : customFadeCol;

    vec3 color = mix(colorTexture.rgb, finalFadeColor, finalFade);
    outColor = vec4(settings.srgbEncode > 0.5 ? EncodeSrgb(color) : color, colorTexture.a);
    gl_FragDepth = depthTexture;
}
