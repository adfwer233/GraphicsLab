#pragma once

#include "glm/glm.hpp"
#include "language/reflection/static_reflector.hpp"

namespace SceneTree {

enum class MaterialType: int {
    Light,
    Object
};

/**
 * @brief Data of Principled BRDF
 *
 * c.f. https://disneyanimation.com/publications/physically-based-shading-at-disney/
 */
struct alignas(16) MaterialData {
    alignas(16) glm::vec3 base_color = glm::vec3(1.0f);
    float metallic = 0.0f;
    float roughness = 0.5f;
    float specular = 0.5f;
    float specularTint = 0.0f;
    float anisotropic = 0.0f;
    float sheen = 0.0f;
    float sheenTint = 0.5f;
    float clearcoat = 0.0f;
    float clearcoatGloss = 1.0f;
    float subsurface = 0.0f;

    MaterialType materialType = MaterialType::Object;

    REFLECT(Property{"base_color", &MaterialData::base_color}, Property{"metallic", &MaterialData::metallic},
            Property{"roughness", &MaterialData::roughness}, Property{"specular", &MaterialData::specular},
            Property{"specularTint", &MaterialData::specularTint}, Property{"anisotropic", &MaterialData::anisotropic},
            Property{"sheen", &MaterialData::sheen}, Property{"sheenTint", &MaterialData::sheenTint},
            Property{"clearcoat", &MaterialData::clearcoat}, Property{"clearcoatGloss", &MaterialData::clearcoatGloss},
            Property{"subsurface", &MaterialData::subsurface})
};

struct MaterialMetaInfo {
    std::string name;

    REFLECT(Property{"name", &MaterialMetaInfo::name})
};

struct Material {
    MaterialMetaInfo meta;
    MaterialData data;

    std::vector<std::string> textures;

    REFLECT(Property{"meta", &Material::meta}, Property{"data", &Material::data})
};

} // namespace SceneTree