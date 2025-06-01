#pragma once

#include "glm/glm.hpp"
#include "language/reflection/static_reflector.hpp"

namespace SceneTree {

/**
 * @brief Data of Principled BRDF
 *
 * c.f. https://disneyanimation.com/publications/physically-based-shading-at-disney/
 */
struct MaterialData {
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

    REFLECT(
        Property{"base_color", &MaterialData::base_color},
        Property{"metallic", &MaterialData::metallic},
        Property{"roughness", &MaterialData::roughness},
        Property{"specular", &MaterialData::specular},
        Property{"specularTint", &MaterialData::specularTint},
        Property{"anisotropic", &MaterialData::anisotropic},
        Property{"sheen", &MaterialData::sheen},
        Property{"sheenTint", &MaterialData::sheenTint},
        Property{"clearcoat", &MaterialData::clearcoat},
        Property{"clearcoatGloss", &MaterialData::clearcoatGloss},
        Property{"subsurface", &MaterialData::subsurface}
    )
};

struct MaterialMetaInfo {
    std::string name;

    REFLECT(
        Property{"name", &MaterialMetaInfo::name}
    )
};

struct Material {
    MaterialMetaInfo meta;
    MaterialData data;

    REFLECT(
        Property{"meta", &Material::meta},
        Property{"data", &Material::data}
    )
};

}