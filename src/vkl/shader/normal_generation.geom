#version 450

layout(triangles) in;
layout(line_strip, max_vertices = 6) out;

layout(location = 0) in vec3 inNormal[];

layout(location = 0) out vec3 outColor;

struct PointLight {
    vec4 position;
    vec4 color;
};

layout(push_constant) uniform PushConstants {
    float normalStrength;
    vec3 normalColor;
} pushConstants;

layout(set = 0, binding = 0) uniform GlobalUbo {
    mat4 model;
    mat4 view;
    mat4 proj;

    PointLight pointLight;
    vec3 cameraPos;
} ubo;

void main(void) {
    float normalLength = pushConstants.normalStrength;

    for (int i = 0; i < gl_in.length(); i++) {
        vec3 pos = gl_in[i].gl_Position.xyz;
        vec3 normal = inNormal[i];

        gl_Position = ubo.proj * ubo.view * ubo.model * vec4(pos, 1.0);
        outColor = pushConstants.normalColor;
        EmitVertex();

        gl_Position = ubo.proj * ubo.view * ubo.model * (vec4(pos + normalLength * normal, 1.0));
        outColor = pushConstants.normalColor;
        EmitVertex();

        EndPrimitive();
    }
}