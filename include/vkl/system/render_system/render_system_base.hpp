#pragma once

#include "vkl/core/vkl_descriptor.hpp"

class BaseRenderSystem {
  public:
    std::unique_ptr<VklDescriptorSetLayout> descriptorSetLayout;
    std::unique_ptr<VklDescriptorPool> descriptorPool;

    std::vector<VkVertexInputBindingDescription> bindingDescriptions;
    std::vector<VkVertexInputAttributeDescription> attributeDescriptions;

    virtual ~BaseRenderSystem() = default;
};