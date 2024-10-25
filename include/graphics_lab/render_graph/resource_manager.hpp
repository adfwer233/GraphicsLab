#pragma once

#include "render_pass_reflection.hpp"
#include "resources.hpp"

namespace GraphicsLab {
namespace RenderGraph {

struct ResourceManager {
    explicit ResourceManager(VklDevice &device) : device_(device) {
    }

    Resource *add_resource(const RenderPassReflection::Field& field) {
        VkSampleCountFlagBits sampleBits = VK_SAMPLE_COUNT_1_BIT;

        if (field.get_sample_count() == 8)
            sampleBits = VK_SAMPLE_COUNT_8_BIT;
        if (field.get_type() == RenderPassReflection::Field::Type::Texture2D) {
            auto color_texture = std::make_unique<ColorTextureResource>(field.get_name());
            color_texture->create_instance(device_, field.get_width(), field.get_height(), 4,
                                           VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT  | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                                           VK_IMAGE_LAYOUT_UNDEFINED, VK_FORMAT_R8G8B8A8_SRGB, sampleBits);
            resources_.push_back(std::move(color_texture));
            return resources_.back().get();
        } else if (field.get_type() == RenderPassReflection::Field::Type::TextureDepth) {
            auto depth_texture = std::make_unique<DepthTextureResource>(field.get_name());
            depth_texture->create_instance(device_, field.get_width(), field.get_height(), 4, VK_IMAGE_LAYOUT_UNDEFINED,
                                           VK_FORMAT_R8G8B8A8_SRGB, sampleBits);
            resources_.push_back(std::move(depth_texture));
            return resources_.back().get();
        }

        return nullptr;
    }

    [[nodiscard]] const Resource *get_resource(const std::string &name) const {
        for (const auto &r : resources_) {
            if (r->get_name() == name) {
                return r.get();
            }
        }
        return nullptr;
    }

    Resource *get_resource(const std::string &name) {
        for (auto &r : resources_) {
            if (r->get_name() == name) {
                return r.get();
            }
        }
        return nullptr;
    }

  private:
    VklDevice &device_;

    std::vector<std::unique_ptr<Resource>> resources_;
};

} // namespace RenderGraph
} // namespace GraphicsLab