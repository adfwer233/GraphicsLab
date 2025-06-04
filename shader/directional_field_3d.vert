#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inDirection;
layout(location = 3) in vec3 inValue;

layout(location = 0) out vec3 outPosition;
layout(location = 1) out vec3 outColor;
layout(location = 2) out vec3 outDirection;
layout(location = 3) out vec3 outValue;

void main() {
    // Pass position to the next stage
    gl_Position = vec4(inPosition, 1.0);

    outPosition = inPosition;
    outColor = inColor;
    outDirection = inDirection;
    outValue = inValue;
}