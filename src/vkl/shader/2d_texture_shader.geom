#version 450 core

layout (points) in;  // The input primitive type is a single point
layout (triangle_strip, max_vertices = 4) out;  // Output a triangle strip (4 vertices)

void main() {
    // Define the positions of the fullscreen quad vertices in NDC
    vec2 positions[4] = vec2[4](
    vec2(-1.0,  1.0),  // Top-left
    vec2( 1.0,  1.0),  // Top-right
    vec2(-1.0, -1.0),  // Bottom-left
    vec2( 1.0, -1.0)   // Bottom-right
    );

    // Emit vertices for the quad, each with its corresponding texture coordinates
//    for (int i = 0; i < 3; i++) {
//        gl_Position = vec4(positions[i], 0.0, 1.0);
//        EmitVertex();
//    }
    gl_Position = vec4(positions[0], 0.0, 1.0);
    EmitVertex();
    gl_Position = vec4(positions[1], 0.0, 1.0);
    EmitVertex();
    gl_Position = vec4(positions[2], 0.0, 1.0);
    EmitVertex();
    gl_Position = vec4(positions[3], 0.0, 1.0);
    EmitVertex();
    EndPrimitive();
}