#pragma once

#include "stb_image.h"
#include "vkl/core/vkl_texture.hpp"

#include <vkl/core/vkl_buffer.hpp>

namespace vkl {

struct TextureManager {
    VklDevice &device;
    std::vector<std::unique_ptr<VklTexture>> textures;

    struct TextureMetadata {
        std::string name;
        std::string import_path;
        size_t texture_index;
    };

    std::map<std::string, TextureMetadata> textures_metadata;

    explicit TextureManager(VklDevice &device) : device(device) {
    }
    ~TextureManager() {
    }

    void load_texture(const std::string &texture_path) {
        // get the file name
        std::string texture_name = texture_path.substr(texture_path.find_last_of('/') + 1);
        texture_name = texture_name.substr(0, texture_name.find_last_of('.'));

        // check if the texture is already loaded
        if (textures_metadata.find(texture_name) != textures_metadata.end()) {
            return;
        }

        auto texture = createTextureImage(texture_path);
        textures.push_back(std::move(texture));

        TextureMetadata metadata;
        metadata.name = texture_name;
        metadata.import_path = texture_path;
        metadata.texture_index = textures.size() - 1;
        textures_metadata[texture_name] = metadata;
    }

    [[nodiscard]] auto get_texture(const std::string &texture_name) -> VklTexture * {
        if (textures_metadata.find(texture_name) == textures_metadata.end()) {
            throw std::runtime_error("texture not found");
        }

        return textures[textures_metadata[texture_name].texture_index].get();
    }

  private:
    auto createTextureImage(const std::string &texturePath) const -> std::unique_ptr<VklTexture> {
        int texWidth, texHeight, texChannels;
        stbi_uc *pixels = stbi_load(texturePath.c_str(), &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);
        VkDeviceSize imageSize = texWidth * texHeight * texChannels;

        if (!pixels) {
            throw std::runtime_error("failed to load texture image!");
        }

        if (texChannels == 3) {
            throw std::runtime_error("unsupported texture info");
        }

        VklBuffer stagingBuffer{device, imageSize, 1, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT};
        stagingBuffer.map();
        stagingBuffer.writeToBuffer((void *)pixels);
        stagingBuffer.unmap();

        auto texture = std::make_unique<VklTexture>(device, texWidth, texHeight, texChannels);

        device.transitionImageLayout(texture->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        device.copyBufferToImage(stagingBuffer.getBuffer(), texture->image_, static_cast<uint32_t>(texWidth),
                                 static_cast<uint32_t>(texHeight), 1);
        device.transitionImageLayout(texture->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

        stbi_image_free(pixels);

        return texture;
    }
};

} // namespace vkl