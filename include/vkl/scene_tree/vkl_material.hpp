#pragma once

#include <assimp/scene.h>
#include "vkl/core/vkl_texture.hpp"

namespace SceneTree {
    struct Material {
        aiColor4D diffuse;
        aiColor4D specular;
        aiColor4D ambient;
        aiColor4D emissive;
        float shininess{};

        std::vector<VklTexture*> textures;
    };
};