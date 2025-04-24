#version 450 core

layout (points) in; // Input is a single point
layout (line_strip, max_vertices = 5) out; // Output rectangle edges as a closed loop

layout(push_constant) uniform PushConstants {
    float zoom;
    float offset_x;
    float offset_y;
} pushConstants;

vec4 transform(vec2 v) {
    return vec4((v.x - 0.5 + pushConstants.offset_x) * 2 * pushConstants.zoom, (v.y - 0.5 + pushConstants.offset_x) * 2 * pushConstants.zoom, 0.0, 1.0);
}

void main() {
    // Define the four corners of the rectangle in NDC (screen space)
    vec4 bottomLeft  = transform(vec2(0.0, 0.0));
    vec4 bottomRight = transform(vec2(1.0, 0.0));
    vec4 topRight    = transform(vec2(1.0, 1.0));
    vec4 topLeft     = transform(vec2(0.0, 1.0));

    // Emit edges of the rectangle
    gl_Position = bottomLeft;
    EmitVertex();

    gl_Position = bottomRight;
    EmitVertex();

    gl_Position = topRight;
    EmitVertex();

    gl_Position = topLeft;
    EmitVertex();

    // Close the loop by connecting back to the first vertex
    gl_Position = bottomLeft;
    EmitVertex();

    EndPrimitive();
}