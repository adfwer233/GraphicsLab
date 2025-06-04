#version 450 core

layout (points) in;
layout (line_strip, max_vertices = 32) out; // 12 edges for a box, but we're emitting 16 lines for completeness

layout(push_constant) uniform PushConstants {
    mat4 mvp;
    vec4 min_pos, max_pos;
} pushConstants;

void main() {
    // Emit the AABB box vertex points (8 vertices)

    vec3 vertices[8];
    vertices[0] = pushConstants.min_pos.xyz;  // min_pos
    vertices[1] = vec3(pushConstants.min_pos.x, pushConstants.min_pos.y, pushConstants.max_pos.z); // front-bottom-left
    vertices[2] = vec3(pushConstants.min_pos.x, pushConstants.max_pos.y, pushConstants.min_pos.z); // back-bottom-left
    vertices[3] = vec3(pushConstants.min_pos.x, pushConstants.max_pos.y, pushConstants.max_pos.z); // back-top-left
    vertices[4] = vec3(pushConstants.max_pos.x, pushConstants.min_pos.y, pushConstants.min_pos.z); // front-bottom-right
    vertices[5] = vec3(pushConstants.max_pos.x, pushConstants.min_pos.y, pushConstants.max_pos.z); // front-top-right
    vertices[6] = vec3(pushConstants.max_pos.x, pushConstants.max_pos.y, pushConstants.min_pos.z); // back-bottom-right
    vertices[7] = pushConstants.max_pos.xyz;  // max_pos

    // Emit edges between the vertices to form the AABB
    // Front face edges (4 edges)
    gl_Position = pushConstants.mvp * vec4(vertices[0], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[1], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[1], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[3], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[3], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[2], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[2], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[0], 1.0);
    EmitVertex();
    EndPrimitive();

    // Back face edges (4 edges)
    gl_Position = pushConstants.mvp * vec4(vertices[4], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[5], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[5], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[7], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[7], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[6], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[6], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[4], 1.0);
    EmitVertex();
    EndPrimitive();

    // Connecting edges between front and back faces (4 edges)
    gl_Position = pushConstants.mvp * vec4(vertices[0], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[4], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[1], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[5], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[2], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[6], 1.0);
    EmitVertex();
    EndPrimitive();

    gl_Position = pushConstants.mvp * vec4(vertices[3], 1.0);
    EmitVertex();
    gl_Position = pushConstants.mvp * vec4(vertices[7], 1.0);
    EmitVertex();
    EndPrimitive();
}