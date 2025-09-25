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

    vec3 lightPosition = ubo.pointLight.position.xyz;
    vec3 lightColor = ubo.pointLight.color.rgb;

    // ambient lighting
    float ambientStrength = 0.2;
    float specularStrength = 0.5;

    // ambient
    vec3 ambient = ambientStrength * lightColor * fragColor;

    // diffuse
    vec3 norm = normalize(fragNormalWorld);
    vec3 lightDir = normalize(lightPosition - fragPosWorld);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor * fragColor;

    // specular
    vec3 viewDir = normalize(ubo.cameraPos.xyz - fragPosWorld);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * lightColor;

    // combine
    vec3 lighting = ambient + diffuse + specular;
    outColor = vec4(lighting, 1.0);
}
