#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 outNormal;

void main() {
    gl_Position = vec4(inPosition.xyz, 1.0);
    outNormal = inNormal;
}