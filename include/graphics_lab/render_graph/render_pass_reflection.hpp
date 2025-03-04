#pragma once

#include <map>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"
#include "vkl/core/vkl_device.hpp"

#include "graphics_lab/core/annotation.hpp"
#include "graphics_lab/core/macros.hpp"

namespace GraphicsLab {
namespace RenderGraph {

struct RenderPassReflection {
    struct Field : public AnnotatedClass {
        enum class Visibility {
            Undefined = 0x0,
            Input = 0x1,
            Output = 0x2,
            InputOutput = 0x3,
            Internal = 0x4
        };

        enum class Type {
            Texture2D,
            TextureDepth,
            RawBuffer
        };

        Field() = default;
        Field(const std::string &name, const std::string &description, Visibility v)
            : name_(name), description_(description), visibility_(v) {};

        Field &name(const std::string &name) {
            name_ = name;
            return *this;
        }
        Field &description(const std::string &desc) {
            description_ = desc;
            return *this;
        }
        Field &visibility(Visibility v) {
            visibility_ = v;
            return *this;
        }
        Field &type(Type t) {
            type_ = t;
            return *this;
        }
        Field &extent(uint32_t width, uint32_t height) {
            width_ = width;
            height_ = height;
            return *this;
        }
        Field &sample_count(uint32_t sample_count) {
            sample_count_ = sample_count;
            return *this;
        }
        Field &format(VkFormat format) {
            format_ = format;
            return *this;
        }
        Field &layout(VkImageLayout layout) {
            texture_layout_ = layout;
            return *this;
        }

        [[nodiscard]] std::string get_name() const {
            return name_;
        }
        [[nodiscard]] std::string get_description() const {
            return description_;
        }
        [[nodiscard]] Type get_type() const {
            return type_;
        }
        [[nodiscard]] uint32_t get_width() const {
            return width_;
        }
        [[nodiscard]] uint32_t get_height() const {
            return height_;
        }
        [[nodiscard]] uint32_t get_sample_count() const {
            return sample_count_;
        }
        [[nodiscard]] VkImageLayout get_layout() const {
            return texture_layout_;
        }

        [[nodiscard]] VkSampleCountFlagBits get_vk_sample_count_flag_bits() const {
            std::map<uint32_t, VkSampleCountFlagBits> transfer_map{
                {1, VK_SAMPLE_COUNT_1_BIT},
                {4, VK_SAMPLE_COUNT_4_BIT},
                {8, VK_SAMPLE_COUNT_8_BIT},
                {16, VK_SAMPLE_COUNT_16_BIT},
            };

            if (not transfer_map.contains(sample_count_)) {
                spdlog::warn("invalid sample count {}", sample_count_);
                return VK_SAMPLE_COUNT_1_BIT;
            }

            return transfer_map[sample_count_];
        };

        [[nodiscard]] Visibility get_visibility() const {
            return visibility_;
        }
        [[nodiscard]] VkFormat get_format() const {
            return format_;
        }
        [[nodiscard]] bool need_to_resolve() const {
            return sample_count_ != 1;
        }

      private:
        std::string name_;
        std::string description_;
        Type type_ = Type::Texture2D;

        uint32_t width_, height_;
        uint32_t sample_count_;

        VkFormat format_;

        Visibility visibility_ = Visibility::Undefined;

        VkImageLayout texture_layout_ = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    };

    Field &add_field(const Field &field) {
        /**
         * @todo: check existing field
         */

        fields_.push_back(field);
        return fields_.back();
    }

    Field &add_input(const std::string &name, const std::string &description) {
        return add_field(Field(name, description, Field::Visibility::Input));
    }
    Field &add_output(const std::string &name, const std::string &description) {
        return add_field(Field(name, description, Field::Visibility::Output));
    }
    Field &add_input_output(const std::string &name, const std::string &description) {
        return add_field(Field(name, description, Field::Visibility::InputOutput));
    }

    size_t field_count() const {
        return fields_.size();
    }

    const Field *get_field(const std::string &name) const {
        for (const auto &f : fields_) {
            if (f.get_name() == name) {
                return &f;
            }
        }
        return nullptr;
    }

    Field *get_field(const std::string &name) {
        for (auto &f : fields_) {
            if (f.get_name() == name) {
                return &f;
            }
        }
        return nullptr;
    }

    /**
     * iterators
     */

    auto begin() {
        return fields_.begin();
    }
    auto end() {
        return fields_.end();
    }

    auto begin() const {
        return fields_.begin();
    }
    auto end() const {
        return fields_.end();
    }

  private:
    std::vector<Field> fields_;
};

ENUM_CLASS_OPERATORS(RenderPassReflection::Field::Visibility)

} // namespace RenderGraph
} // namespace GraphicsLab