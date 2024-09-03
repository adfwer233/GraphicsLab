#pragma once
#include "vkl_device.hpp"

class VklFramebuffer {
  private:
    VklDevice &device_;

  public:
    VkFramebuffer framebuffer;

    VklFramebuffer(VklDevice &device, VkRenderPass renderPass, size_t attachmentSize, const VkImageView *pAttachments,
                   int width, int height)
        : device_(device) {
        VkFramebufferCreateInfo framebufferCreateInfo{};
        framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferCreateInfo.renderPass = renderPass;
        framebufferCreateInfo.attachmentCount = attachmentSize;
        framebufferCreateInfo.pAttachments = pAttachments;
        framebufferCreateInfo.width = width;
        framebufferCreateInfo.height = height;
        framebufferCreateInfo.layers = 1;

        if (vkCreateFramebuffer(device_.device(), &framebufferCreateInfo, nullptr, &framebuffer) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create framebuffer!");
        }
    }

    ~VklFramebuffer() {
        vkDestroyFramebuffer(device_.device(), framebuffer, nullptr);
    }
};