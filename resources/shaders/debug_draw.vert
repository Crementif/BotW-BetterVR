#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 vColor;

layout(push_constant) uniform SceneSettings {
    mat4 viewProjection;
    float targetWidth;
    float targetHeight;
    float depthBias;
    float xrayAlphaScale;
} scene;

void main() {
    vec4 clipPosition = scene.viewProjection * vec4(inPosition, 1.0);

    // the game's projection matrix targets a y-up NDC, Vulkan clip space is y-down
    clipPosition.y = -clipPosition.y;

    gl_Position = clipPosition;
    vColor = inColor;
}
