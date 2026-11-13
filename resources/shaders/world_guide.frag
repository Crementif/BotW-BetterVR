#version 450

layout(location = 0) in vec2 vUv;
layout(location = 1) in vec4 vColor;
layout(location = 2) in vec4 vShape;
layout(location = 3) flat in float vKind;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D g_glyphs;

layout(push_constant) uniform GuideSettings {
    mat4 viewProjection;
    float srgbEncode;
} settings;

vec3 DecodeSrgb(vec3 color) {
    return mix(color / 12.92, pow((color + 0.055) / 1.055, vec3(2.4)), greaterThan(color, vec3(0.04045)));
}

float RoundedBoxDistance(vec2 position, vec2 halfSize, float radius) {
    vec2 q = abs(position) - (halfSize - radius);
    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - radius;
}

const float GlowAdditive = 0.5;

void main() {
    float alpha;
    float additive = 0.0;
    if (vKind < 0.5) {
        float field = texture(g_glyphs, vUv).r;
        float width = max(fwidth(field), 1e-4) * 0.7;
        float edge = 0.5 - vShape.x;
        alpha = smoothstep(edge - width, edge + width, field);
    }
    else if (vKind < 2.5) {
        float falloff = clamp(1.0 - abs(vUv.x), 0.0, 1.0);
        alpha = falloff * falloff * (0.35 + 0.65 * falloff * falloff);
        additive = GlowAdditive;
    }
    else if (vKind < 3.5) {
        float falloff = clamp(1.0 - dot(vUv, vUv), 0.0, 1.0);
        alpha = falloff * falloff * falloff * 0.6 + pow(falloff, 24.0) * 0.4;
        additive = GlowAdditive;
    }
    else {
        float distance = RoundedBoxDistance(vUv, vShape.xy, vShape.z);
        alpha = 1.0 - smoothstep(-vShape.w, vShape.w, distance);
        additive = vKind > 4.5 ? GlowAdditive : 0.0;
    }

    alpha *= vColor.a;
    if (alpha <= 0.002) {
        discard;
    }
    vec3 color = settings.srgbEncode > 0.5 ? vColor.rgb : DecodeSrgb(vColor.rgb);
    outColor = vec4(color * alpha, alpha * (1.0 - additive));
}
