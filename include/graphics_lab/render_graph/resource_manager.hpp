#pragma once

#include "language/coroutine/generator.hpp"

#include "render_pass_reflection.hpp"
#include "resources.hpp"

namespace GraphicsLab::RenderGraph {

struct ImGuiResources {
    std::map<std::string, std::vector<VkDescriptorSet>> imguiImages;
};

struct ResourceManager {
    explicit ResourceManager(VklDevice &device) : device_(device) {
    }

    ~ResourceManager() {
        for (auto &r : resources_) {
            try {
                if (not r->has_annotation("do_not_release"))
                    r.reset();
                else
                    r.release();
            } catch (std::exception &e) {
                spdlog::warn(e.what());
            }
        }
    }

    Resource *add_resource(const RenderPassReflection::Field &field) {
        VkSampleCountFlagBits sampleBits = VK_SAMPLE_COUNT_1_BIT;

        if (auto cur = get_resource(field.get_name()))
            return cur;

        if (field.get_sample_count() == 8)
            sampleBits = VK_SAMPLE_COUNT_8_BIT;
        if (field.get_type() == RenderPassReflection::Field::Type::Texture2D) {
            auto color_texture = std::make_unique<ColorTextureResource>(field.get_name());
            color_texture->create_instance(device_, field.get_width(), field.get_height(), 4,
                                           VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                                               VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                                               VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                                           field.get_layout(), field.get_format(), sampleBits);
            color_texture->copy_annotation(field);
            resources_.push_back(std::move(color_texture));

            return resources_.back().get();
        } else if (field.get_type() == RenderPassReflection::Field::Type::TextureDepth) {
            auto depth_texture = std::make_unique<DepthTextureResource>(field.get_name());
            VkFormat format = device_.findSupportedFormat(
                {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT},
                VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
            depth_texture->create_instance(device_, field.get_width(), field.get_height(), 4,
                                           VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, format, sampleBits);
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

    Generator<Resource *> get_resource_with_annotation(const std::string annotation) {
        for (auto &r : resources_) {
            if (r->has_annotation(annotation)) {
                co_yield r.get();
            }
        }
    }

  private:
    VklDevice &device_;

    std::vector<std::unique_ptr<Resource>> resources_;
};

} // namespace GraphicsLab::RenderGraph
