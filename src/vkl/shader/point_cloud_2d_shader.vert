#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 fragColor;

void main() {

    gl_PointSize = 50.0;
    gl_Position = vec4((inPosition.xy - 0.5) * 2, 0.0, 1.0);
    fragColor = inColor.rgb;
}