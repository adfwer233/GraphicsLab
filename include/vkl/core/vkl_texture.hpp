#pragma once

#include "vkl_device.hpp"

class VklTexture {
  private:
    int texWidth_, texHeight_, texChannels_;

    VklDevice &device_;
    VkSampler textureSampler_;
    VkImageView textureImageView;

    VkImageUsageFlags usage_;
  public:
    VkImage image_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;

    VklTexture(VklDevice &device_, int texWidth, int texHeight, int texChannels,
               VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    VklTexture(VklDevice &device, VkImage image);
    ~VklTexture();

    VkSampler getTextureSampler() {
        return textureSampler_;
    }
    VkImageView getTextureImageView() {
        return textureImageView;
    }
    VkDescriptorImageInfo descriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
};