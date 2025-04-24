#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 fragColor;

layout(push_constant) uniform PushConstants {
    float zoom;
    float shift_x;
    float shift_y;
} pushConstants;

void main() {

    gl_PointSize = 5.0;
    gl_Position = vec4((inPosition.x - 0.5 + pushConstants.shift_x) * 2 * pushConstants.zoom,
                       (inPosition.y - 0.5 + pushConstants.shift_y) * 2 * pushConstants.zoom,
                       0.0, 1.0);
    fragColor = vec3(1.0, 0.0, 0.0);
}