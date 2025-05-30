#pragma once

#include "glm/glm.hpp"
#include "language/reflection/static_reflector.hpp"

namespace GraphicsLab::Material {

/**
 * @brief Data of Principled BRDF
 *
 * c.f. https://disneyanimation.com/publications/physically-based-shading-at-disney/
 */
struct Material {
    glm::vec3 base_color;
    float metallic;
    float roughness;
    float specular;
    float specularTint;
    float anisotropic;
    float sheen;
    float sheenTint;
    float clearcoat;
    float clearcoatGloss;
    float subsurface;
};

} // namespace GraphicsLab::Material