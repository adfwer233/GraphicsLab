#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 outColor;

layout(set = 0, binding = 0) uniform Ubo {
    float zoom;
    float offset_x;
    float offset_y;
} ubo;

void main() {
    gl_Position = vec4((inPosition.x - 0.5) * 2 * 0.5, (inPosition.y - 0.5) * 2 * 0.5, 0.0, 1.0);
    outColor = inColor;
}