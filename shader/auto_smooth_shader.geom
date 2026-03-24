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
    // Average neighboring normals per vertex to get a smooth normal per corner.
    // Similar normals are blended; sharp differences are preserved.
    vec3 smoothNormal[3];
    const float smoothThreshold = 0.5;

    for (int i = 0; i < 3; i++) {
        vec3 normalSum = inNormalWorld[i];
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                continue;
            }

            if (dot(inNormalWorld[i], inNormalWorld[j]) >= smoothThreshold) {
                normalSum += inNormalWorld[j];
            }
        }

        smoothNormal[i] = normalize(normalSum);
    }

    for (int i = 0; i < 3; i++) {
        gl_Position = gl_in[i].gl_Position;
        outColor = inColor[i];
        outPosWorld = inPosWorld[i];

        outNormalWorld = smoothNormal[i];

        outTexCoord = inTexCoord[i];
        EmitVertex();
    }
    EndPrimitive();
}
