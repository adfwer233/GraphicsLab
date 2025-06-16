#pragma once

#include "material.hpp"

namespace SceneTree {

struct MaterialManager : Reflectable {
    std::vector<Material> materials;

    explicit MaterialManager() {

        // default object material
        Material default_material;
        default_material.meta.name = "default";
        materials.push_back(default_material);

        // default color material
        Material default_light_material;
        default_light_material.meta.name = "default_light";
        default_light_material.data.materialType = MaterialType::Light;
        materials.push_back(default_light_material);
    }

    REFLECT(PROPERTY(materials, &MaterialManager::materials))

    void create_material(const std::string &name) {
        materials.emplace_back();
        materials.back().meta.name = name;
    }

    ReflectDataType reflect() override {
        return {
            {"create_material", TypeErasedValue(&MaterialManager::create_material, this, {"default name"}, {"name"})}};
    }
};

} // namespace SceneTree