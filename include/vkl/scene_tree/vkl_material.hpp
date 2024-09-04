#pragma once

#include "vkl/core/vkl_texture.hpp"
#include <assimp/scene.h>

namespace SceneTree {
struct Material {
    aiColor4D diffuse;
    aiColor4D specular;
    aiColor4D ambient;
    aiColor4D emissive;
    float shininess{};

    std::vector<VklTexture *> textures;
};
}; // namespace SceneTree