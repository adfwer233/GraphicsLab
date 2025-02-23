#version 450 core

layout(set = 0, binding = 0) uniform sampler2D densityTexture;

layout(location = 0) out vec4 fragColor;

void main() {
    float density = texture(densityTexture, gl_FragCoord.xy / vec2(2048, 2048)).r;

    fragColor = vec4(density, 0, 0, 1.0);
}