#pragma once

#include <map>

#include "vkl/core/vkl_device.hpp"

struct RenderResources {
    std::vector<VkDescriptorSet> sceneRenderTexture;
    std::vector<VkDescriptorSet> projectRenderTexture;

    std::map<std::string, std::vector<VkDescriptorSet>> imguiImages;

    std::map<std::string, std::vector<VkDescriptorSet>> textureImguiImages;
};