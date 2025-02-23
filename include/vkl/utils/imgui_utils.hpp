#pragma once

#include "vkl/core/vkl_texture.hpp"

namespace vkl {

struct SRGBColor4 {
    uint8_t r, g, b, a;
};

struct ImguiUtils {

    static auto getImguiTextureFromVklTexture(VklTexture *texture) {
        std::vector<VkDescriptorSet> result;
        auto tex = ImGui_ImplVulkan_AddTexture(texture->getTextureSampler(), texture->getTextureImageView(),
                                               texture->getImageLayout());
        result.push_back(tex);
        return result;
    }
};
} // namespace vkl