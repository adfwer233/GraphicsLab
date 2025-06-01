#pragma once

#include "material.hpp"

namespace SceneTree {

struct MaterialManager: Reflectable {
    std::vector<Material> materials;

    void test() {
        spdlog::info("test called");
    }
    void create_material(const std::string& name) {
        materials.emplace_back();
        materials.back().meta.name = name;
    }

    ReflectDataType reflect() override {
        return {
            {"test", TypeErasedValue(&MaterialManager::test, this)},
            {"create_material", TypeErasedValue(&MaterialManager::create_material, this, {"default name"}, {"name"})}
        };
    }
};

}