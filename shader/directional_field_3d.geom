#version 450

layout(points) in;
layout(line_strip, max_vertices = 6) out; // 6 vertices for a basic arrow

layout (location = 0) in vec3 inPosition[];
layout (location = 1) in vec3 inColor[];
layout (location = 2) in vec3 inDirection[];
layout (location = 3) in vec3 inValue[];


layout (location = 0) out vec3 fragColor; // Color to pass to the fragment shader

struct PointLight {
    vec4 position;
    vec4 color;
};

layout(set = 0, binding = 0) uniform GlobalUbo {
    mat4 model;
    mat4 view;
    mat4 proj;

    PointLight pointLight;
    vec3 cameraPos;
} ubo;

void main() {
    mat4 trans = ubo.proj * ubo.view * ubo.model;

    // Calculate the direction of the arrow
    vec3 direction = normalize(inDirection[0]); // Assumes all input points have the same direction

    // Scale the direction to control arrow length
    float length = 0.01; // Set an appropriate length for the arrows
    vec3 arrowBase = inPosition[0];

    // Create the shaft of the arrow
    vec3 arrowShaft = arrowBase + direction * length;

    // Generate the arrowhead (simple cone shape)
    float arrowheadLength = 0.0003;
    vec3 arrowhead1 = arrowShaft + direction * arrowheadLength + cross(direction, vec3(0.0, 0.0, 1.0)) * 0.02;
    vec3 arrowhead2 = arrowShaft + direction * arrowheadLength - cross(direction, vec3(0.0, 0.0, 1.0)) * 0.02;

    // Output vertices
    gl_Position = trans * vec4(arrowBase, 1.0);
    fragColor = vec3(1.0, 0.0, 0.0);
    EmitVertex();

    gl_Position = trans * vec4(arrowShaft, 1.0);
    fragColor = vec3(0.0, 0.0, 1.0);
    EmitVertex();

//    gl_Position = trans * vec4(arrowhead1, 1.0);
//    EmitVertex();
//
//    gl_Position = trans * vec4(arrowhead2, 1.0);
//    EmitVertex();
//
//    gl_Position = trans * vec4(arrowShaft, 1.0);
//    EmitVertex();
//
//    gl_Position = trans * vec4(arrowBase, 1.0);
//    EmitVertex();

    EndPrimitive();
}
