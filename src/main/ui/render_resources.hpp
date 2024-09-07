#pragma once

#include "vkl/core/vkl_device.hpp"

struct RenderResources {
    std::vector<VkDescriptorSet> sceneRenderTexture;
};