#pragma once

#include <stdexcept>

#include "vkl_device.hpp"

class VklRenderPass {
  private:
    VklDevice &device_;

  public:
    VkRenderPass renderPass;

    VklRenderPass(VklDevice &device, VkRenderPassCreateInfo createInfo) : device_(device) {
        if (vkCreateRenderPass(device_.device(), &createInfo, nullptr, &renderPass) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create render pass in render graph.");
        }
    }

    ~VklRenderPass() {
        vkDestroyRenderPass(device_.device(), renderPass, nullptr);
    }
};