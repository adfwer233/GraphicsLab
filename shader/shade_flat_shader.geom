#version 450

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

layout(location = 0) in vec3 inColor[];
layout(location = 1) in vec3 inPosWorld[];
layout(location = 2) in vec3 inNormalWorld[];
layout(location = 3) in vec2 inTexCoord[];

layout(location = 0) out vec3 outColor;
layout(location = 1) out vec3 outPosWorld;
layout(location = 2) out vec3 outNormalWorld;
layout(location = 3) out vec2 outTexCoord;

void main() {
    vec3 edge1 = inPosWorld[1] - inPosWorld[0];
    vec3 edge2 = inPosWorld[2] - inPosWorld[0];

    vec3 faceNormal = normalize(cross(edge1, edge2));
    vec3 referenceNormal = normalize(inNormalWorld[0] + inNormalWorld[1] + inNormalWorld[2]);
    if (dot(faceNormal, referenceNormal) < 0.0) {
        faceNormal = -faceNormal;
    }

    for (int i = 0; i < 3; ++i) {
        gl_Position = gl_in[i].gl_Position;
        outColor = inColor[i];
        outPosWorld = inPosWorld[i];
        outNormalWorld = faceNormal;
        outTexCoord = inTexCoord[i];
        EmitVertex();
    }

    EndPrimitive();
}

