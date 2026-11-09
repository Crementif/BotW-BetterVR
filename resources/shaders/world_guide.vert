#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec2 inUv;
layout(location = 2) in vec4 inColor;
layout(location = 3) in vec4 inShape;
layout(location = 4) in float inKind;

layout(location = 0) out vec2 vUv;
layout(location = 1) out vec4 vColor;
layout(location = 2) out vec4 vShape;
layout(location = 3) flat out float vKind;

layout(push_constant) uniform GuideSettings {
    mat4 viewProjection;
    float srgbEncode;
} settings;

void main() {
    gl_Position = settings.viewProjection * vec4(inPosition, 1.0);
    vUv = inUv;
    vColor = inColor;
    vShape = inShape;
    vKind = inKind;
}
