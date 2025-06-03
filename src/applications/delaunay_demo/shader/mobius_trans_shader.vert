#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 outColor;

layout(push_constant) uniform PushConstants {
    vec2 a;
    vec2 b;
    vec2 c;
    vec2 d;
} pushConstants;

vec2 complexMul(vec2 z1, vec2 z2) {
    return vec2(
        z1.x * z2.x - z1.y * z2.y,
        z1.x * z2.y + z1.y * z2.x
    );
}

vec2 complexAdd(vec2 z1, vec2 z2) {
    return z1 + z2;
}

vec2 complexDiv(vec2 a, vec2 b) {
    float denom = b.x * b.x + b.y * b.y;
    return vec2(
        (a.x * b.x + a.y * b.y) / denom,
        (a.y * b.x - a.x * b.y) / denom
    );
}

void main() {
    vec2 z = inPosition;
    gl_PointSize = 15.0;

    vec2 numerator = complexAdd(complexMul(pushConstants.a, z), pushConstants.b);
    vec2 denominator = complexAdd(complexMul(pushConstants.c, z), pushConstants.d);
    vec2 transformed = complexDiv(numerator, denominator);

    gl_Position = vec4(transformed, 0.0, 1.0);
    outColor = inColor;
}