#version 450

layout(location = 0) in vec2 vUv;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D g_colorTexture;

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

    vec3 color = settings.srgbEncode > 0.5 ? EncodeSrgb(colorTexture.rgb) : colorTexture.rgb;
    outColor = vec4(color, colorTexture.a);
}
