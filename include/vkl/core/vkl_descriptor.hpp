#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "spirv_cross.hpp"
#include "vkl_device.hpp"

/**
 * @page devicePage Vulkan Resource Binding
 *
 * ## DescriptorSetLayout
 *
 * ## DescriptorSet
 *
 * ## DescriptorPool
 */

template <typename T> struct VklDescriptor {
    T *data;
    uint32_t binding;
    VkShaderStageFlags shaderStageFlags;
};

struct VklUniformBufferDescriptor {
    bool operator<(const VklUniformBufferDescriptor &other) const {
        return std::tie(binding, size, typeName) < std::tie(other.binding, other.size, other.typeName);
    }
    uint32_t binding;
    uint32_t size;
    std::string typeName;
};

struct VklSampledImageBufferDescriptor {
    bool operator<(const VklSampledImageBufferDescriptor &other) const {
        return std::tie(binding, name) < std::tie(other.binding, other.name);
    }
    uint32_t binding;
    std::string name;
};

struct VklDescriptorSetLayoutKey {
    bool operator<(const VklDescriptorSetLayoutKey &other) const {
        return std::tie(uniformDescriptors, sampledImageBufferDescriptors) <
               std::tie(other.uniformDescriptors, other.sampledImageBufferDescriptors);
    }
    std::vector<VklUniformBufferDescriptor> uniformDescriptors;
    std::vector<VklSampledImageBufferDescriptor> sampledImageBufferDescriptors;
};

class VklDescriptorSetLayout {
  public:
    class Builder {
      private:
        VklDevice &device_;
        std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings;
        VklDescriptorSetLayoutKey descriptorSetLayoutKey;

      public:
        explicit Builder(VklDevice &device) : device_(device) {};

        Builder &addBinding(uint32_t binding, VkDescriptorType descriptorType, VkShaderStageFlags stageFlags,
                            uint32_t count = 1);

        Builder &addBindingsFromResource(spirv_cross::Compiler &compiler, spirv_cross::ShaderResources &resources);

        [[nodiscard]] std::unique_ptr<VklDescriptorSetLayout> build();

        friend class VklDescriptorSetLayout;
    };

  private:
    VklDevice &device_;
    VkDescriptorSetLayout descriptorSetLayout_;
    std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings_;

  public:
    VklDescriptorSetLayoutKey descriptorSetLayoutKey;

    VklDescriptorSetLayout(VklDevice &device, Builder &builder);
    ~VklDescriptorSetLayout();
    VklDescriptorSetLayout(const VklDescriptorSetLayout &) = delete;
    VklDescriptorSetLayout &operator=(const VklDescriptorSetLayout &) = delete;
    [[nodiscard]] VkDescriptorSetLayout getDescriptorSetLayout() const {
        return descriptorSetLayout_;
    };

    friend class VklDescriptorWriter;
};

class VklDescriptorPool {
  public:
    class Builder {
      private:
        VklDevice &device_;
        std::vector<VkDescriptorPoolSize> poolSizes{};
        uint32_t maxSets = 1000;
        VkDescriptorPoolCreateFlags poolFlags = 0;

      public:
        explicit Builder(VklDevice &device) : device_(device) {
        }

        Builder &addPoolSize(VkDescriptorType descriptorType, uint32_t count);
        Builder &setPoolFlags(VkDescriptorPoolCreateFlags flags);
        Builder &setMaxSets(uint32_t count);
        [[nodiscard]] std::unique_ptr<VklDescriptorPool> build() const;
    };

  private:
    VklDevice &device_;
    VkDescriptorPool descriptorPool;

  public:
    VklDescriptorPool(VklDevice &device, uint32_t maxSets, VkDescriptorPoolCreateFlags poolFlags,
                      const std::vector<VkDescriptorPoolSize> &poolSizes);
    ~VklDescriptorPool();
    VklDescriptorPool(const VklDescriptorPool &) = delete;
    VklDescriptorPool &operator=(const VklDescriptorPool &) = delete;

    VkDescriptorPool getDescriptorPool() {
        return descriptorPool;
    }

    bool allocateDescriptor(const VkDescriptorSetLayout descriptorSetLayout, VkDescriptorSet &descriptor) const;

    void freeDescriptors(std::vector<VkDescriptorSet> &descriptors) const;

    void resetPool();

    friend class VklDescriptorWriter;
};

class VklDescriptorWriter {
  private:
    VklDescriptorSetLayout &setLayout_;
    VklDescriptorPool &pool_;
    std::vector<VkWriteDescriptorSet> writes_;

  public:
    VklDescriptorWriter(VklDescriptorSetLayout &setLayout, VklDescriptorPool &pool);

    VklDescriptorWriter &writeBuffer(uint32_t binding, VkDescriptorBufferInfo *bufferInfo);
    VklDescriptorWriter &writeImage(uint32_t binding, VkDescriptorImageInfo *imageInfo);

    bool build(VkDescriptorSet &set);
    void overwrite(VkDescriptorSet &set);
};
