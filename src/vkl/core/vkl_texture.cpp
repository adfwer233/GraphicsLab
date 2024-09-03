#include <stdexcept>

#include "vkl/core/vkl_texture.hpp"

VklTexture::VklTexture(VklDevice &device, int texWidth, int texHeight, int texChannels, VkImageUsageFlags usage, VkImageLayout layout)
    : texWidth_(texWidth), texHeight_(texHeight), texChannels_(texChannels), device_(device), layout_(layout) {
    if (texChannels == 3) {
        throw std::runtime_error("unsupported texture type \n");
    } else if (texChannels == 4) {
        usage_ = usage;
        device_.createImage(texWidth, texHeight, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL, usage,
                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, this->image_, this->memory_);
        device_.createSampler(this->textureSampler_);

        if (layout != VK_IMAGE_LAYOUT_UNDEFINED)
            device_.transitionImageLayout(this->image_, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, layout);
        this->textureImageView = device_.createImageView(image_, VK_FORMAT_R8G8B8A8_SRGB);
    } else {
        throw std::runtime_error("wrong texture channel number \n");
    }
}

VklTexture::VklTexture(VklDevice &device, VkImage image) : device_(device) {
    this->image_ = image;
    device.createSampler(this->textureSampler_);
    this->textureImageView = device.createImageView(image, VK_FORMAT_R8G8B8A8_UNORM);
}

VklTexture::~VklTexture() {
    vkDestroyImageView(device_.device(), this->textureImageView, nullptr);
    vkDestroySampler(device_.device(), this->textureSampler_, nullptr);
    vkDestroyImage(device_.device(), this->image_, nullptr);
    vkFreeMemory(device_.device(), this->memory_, nullptr);
}

VkDescriptorImageInfo VklTexture::descriptorInfo(VkDeviceSize size, VkDeviceSize offset) {
    return VkDescriptorImageInfo(textureSampler_, textureImageView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}
