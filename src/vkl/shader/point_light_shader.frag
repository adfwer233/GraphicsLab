#version 450

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormalWorld;
layout(location = 3) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

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

    vec3 lightPosition = vec3(0.0, 5.0, 0.0);
    vec3 lightColor = ubo.pointLight.color.rgb;
//lightPosition.y = -lightPosition.y;
    // ambient lighting
    float ambientStrength = 0.2;
    vec3 ambient = ambientStrength * ubo.pointLight.color.rgb;

    // diffuse lighting
    vec3 norm = normalize(fragNormalWorld);
    vec3 lightDirection = normalize(lightPosition - fragPosWorld);
    float diff = max(dot(norm, lightDirection), 0.0);
    vec3 diffuse = diff * lightColor;

    // specular lighting
    float specularStrength = 0.5;
    vec3 viewDir = normalize(ubo.cameraPos - fragPosWorld);
    vec3 reflectDir = reflect(-lightDirection, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;

//    outColor = vec4(fragColor, 1.0f);
//    if (dot(norm, lightDirection) > 0)
        outColor = vec4((ambient + diffuse + specular) * vec3(0.9, 0.9, 0.9), 1.0f);
//    else
//        outColor = vec4(ambient, 1.0f);

//    if (fragTexCoord.x < 0.005)
//        outColor = vec4(1.0, 0.0, 0.0, 1.0);
}
