#version 450

layout(location = 0) in vec4 vColor;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D g_sceneDepthTexture;

layout(push_constant) uniform SceneSettings {
    mat4 viewProjection;
    float targetWidth;
    float targetHeight;
    float depthBias;
    float xrayAlphaScale;
} scene;

void main() {
    vec2 uv = vec2(gl_FragCoord.x / scene.targetWidth, gl_FragCoord.y / scene.targetHeight);
    float sceneDepth = texture(g_sceneDepthTexture, uv).r;
    float overlayDepth = clamp(gl_FragCoord.z - scene.depthBias, 0.0, 1.0);

    if (scene.xrayAlphaScale <= 0.0 && overlayDepth > sceneDepth + scene.depthBias) {
        discard;
    }

    outColor = vec4(vColor.rgb, vColor.a * (scene.xrayAlphaScale > 0.0 ? scene.xrayAlphaScale : 1.0));
    gl_FragDepth = overlayDepth;
}
