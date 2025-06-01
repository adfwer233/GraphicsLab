#pragma once

#include "material.hpp"

namespace SceneTree {

struct MaterialManager {
    std::vector<Material> materials;

    void create_material(const std::string& name) {
        materials.emplace_back();
        materials.back().meta.name = name;
    }

    REFLECT(
        Property{"create_material", &MaterialManager::create_material}
    )
};

}