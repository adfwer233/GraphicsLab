#pragma once

#include <memory>

#include "graphics_lab/core/annotation.hpp"
#include "vkl/core/vkl_texture.hpp"

namespace GraphicsLab {
namespace RenderGraph {

struct Resource : public AnnotatedClass {
    enum class Type {
        Undefined,
        ColorTexture,
        DepthTexture,
        Buffer
    };

    const std::string get_name() const {
        return name_;
    }

    [[nodiscard]] Type get_type() const {
        return type_;
    }

    virtual ~Resource() = default;

  protected:
    std::string name_;
    Type type_ = Type::Undefined;
};

struct ColorTextureResource : public Resource {
    explicit ColorTextureResource(std::string name) {
        type_ = Type::ColorTexture;
        name_ = name;
    }

    VklTexture *getTexture() {
        return texture_.get();
    }
    VklTexture *get_resolved_texture() {
        if (resolved_texture_ == nullptr)
            return texture_.get();
        return resolved_texture_.get();
    }

    /**
     * @brief generate the instance of images
     *
     * If the sample count is not 1, a resolved texture will be generated
     */
    void create_instance(VklDevice &device_, int texWidth, int texHeight, int texChannels,
                         VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                                   VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
                         VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED, VkFormat format = VK_FORMAT_R8G8B8A8_SRGB,
                         VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT) {
        texture_ =
            std::make_unique<VklTexture>(device_, texWidth, texHeight, texChannels, usage, layout, format, samples);
        if (samples != VK_SAMPLE_COUNT_1_BIT) {
            resolved_texture_ = std::make_unique<VklTexture>(device_, texWidth, texHeight, texChannels, usage, layout,
                                                             format, VK_SAMPLE_COUNT_1_BIT);
        }
    }

  private:
    std::unique_ptr<VklTexture> texture_ = nullptr;
    std::unique_ptr<VklTexture> resolved_texture_ = nullptr;
};

struct DepthTextureResource : public Resource {
    explicit DepthTextureResource(std::string name) {
        type_ = Type::ColorTexture;
        name_ = name;
    }

    VklTexture *getTexture() {
        return texture_.get();
    }
    VklTexture *get_resolved_texture() {
        return resolved_texture_.get();
    }

    /**
     * @brief generate the instance of images
     *
     * If the sample count is not 1, a resolved texture will be generated
     */
    void create_instance(VklDevice &device_, int texWidth, int texHeight, int texChannels,
                         VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED, VkFormat format = VK_FORMAT_R8G8B8A8_SRGB,
                         VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT) {
        VkImageUsageFlags usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

        texture_ =
            std::make_unique<VklTexture>(device_, texWidth, texHeight, texChannels, usage, layout, format, samples);
        if (samples != VK_SAMPLE_COUNT_1_BIT) {
            resolved_texture_ = std::make_unique<VklTexture>(device_, texWidth, texHeight, texChannels, usage, layout,
                                                             format, VK_SAMPLE_COUNT_1_BIT);
        }
    }

  private:
    std::unique_ptr<VklTexture> texture_ = nullptr;
    std::unique_ptr<VklTexture> resolved_texture_ = nullptr;
};

} // namespace RenderGraph
} // namespace GraphicsLab