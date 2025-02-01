#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 32) out;

layout(push_constant) uniform PushConstants {
    mat4 mvp;
} pushConstants;

layout (location = 0) out vec3 vertColor;

void main() {
    gl_Position = pushConstants.mvp * vec4(0.0, 0.0, 0.0, 1.0);
    vertColor = vec3(1.0, 0.0, 0.0);
    EmitVertex();

    gl_Position = pushConstants.mvp * vec4(10.0, 0.0, 0.0, 1.0);
    vertColor = vec3(1.0, 0.0, 0.0);
    EmitVertex();

    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(0.0, 0.0, 0.0, 1.0);
    vertColor = vec3(0.0, 1.0, 0.0);
    EmitVertex();

    gl_Position = pushConstants.mvp * vec4(0.0, 10.0, 0.0, 1.0);
    vertColor = vec3(0.0, 1.0, 0.);
    EmitVertex();

    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(0.0, 0.0, 0.0, 1.0);
    vertColor = vec3(0.0, 0.0, 1.0);
    EmitVertex();

    gl_Position = pushConstants.mvp * vec4(0.0, 0.0, 10.0, 1.0);
    vertColor = vec3(0.0, 0.0, 1.0);
    EmitVertex();

    EndPrimitive();
}