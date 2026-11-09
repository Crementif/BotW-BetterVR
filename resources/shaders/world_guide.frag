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

float Coverage(float distance) {
    return clamp(0.5 - distance / max(fwidth(distance), 1e-6), 0.0, 1.0);
}

vec3 DecodeSrgb(vec3 color) {
    return mix(color / 12.92, pow((color + 0.055) / 1.055, vec3(2.4)), greaterThan(color, vec3(0.04045)));
}

void main() {
    float alpha;
    if (vKind < 0.5) {
        float field = texture(g_glyphs, vUv).r;
        float width = max(fwidth(field), 1e-4) * 0.7;
        float edge = 0.5 - vShape.x;
        alpha = smoothstep(edge - width, edge + width, field);
    }
    else if (vKind < 1.5) {
        vec2 q = abs(vUv) - (vShape.xy - vShape.z);
        float distance = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - vShape.z;
        if (vShape.w > 0.0) {
            distance = abs(distance + vShape.w * 0.5) - vShape.w * 0.5;
        }
        alpha = Coverage(distance);
    }
    else {
        alpha = Coverage(abs(vUv.x) - 1.0);
    }

    alpha *= vColor.a;
    if (alpha <= 0.002) {
        discard;
    }
    vec3 color = settings.srgbEncode > 0.5 ? vColor.rgb : DecodeSrgb(vColor.rgb);
    outColor = vec4(color, alpha);
}
