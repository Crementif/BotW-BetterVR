#version 450

layout(location = 0) out vec2 vUv;

void main() {
    const uint quadIndices[6] = uint[6](0u, 1u, 2u, 2u, 1u, 3u);
    uint vertexId = quadIndices[gl_VertexIndex];
    vUv = vec2(float(vertexId % 2u), float((vertexId % 4u) / 2u));

    // Vulkan clip space is y-down, so the HLSL original's y negation is dropped here
    gl_Position = vec4((vUv.x - 0.5) * 2.0, (vUv.y - 0.5) * 2.0, 0.0, 1.0);
}
