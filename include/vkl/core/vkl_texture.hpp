#pragma once

#include "vkl_device.hpp"

class VklTexture {
  private:
    int texWidth_, texHeight_, texChannels_;

    VklDevice &device_;
    VkSampler textureSampler_;
    VkImageView textureImageView;

    VkImageUsageFlags usage_;
    VkImageLayout layout_;

  public:
    VkImage image_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;

    VklTexture(VklDevice &device_, int texWidth, int texHeight, int texChannels,
               VkImageUsageFlags usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
               VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED, VkFormat format = VK_FORMAT_R8G8B8A8_SRGB,
               VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT);
    VklTexture(VklDevice &device, VkImage image);
    ~VklTexture();

    VkSampler getTextureSampler() {
        return textureSampler_;
    }
    VkImageView getTextureImageView() {
        return textureImageView;
    }
    VkImageLayout getImageLayout() {
        return layout_;
    }
    VkDescriptorImageInfo descriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
};