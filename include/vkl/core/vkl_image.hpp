#pragma once

#include "vkl_device.hpp"

namespace VklImageUtils {
VkImageMemoryBarrier ReadOnlyToGeneralBarrier(const VkImage &image);

VkImageMemoryBarrier ReadOnlyToDstBarrier(const VkImage &image);

VkImageMemoryBarrier ColorToTransferDstBarrier(const VkImage &image);

VkImageMemoryBarrier readOnlyToSrcBarrier(const VkImage &image);

VkImageMemoryBarrier generalToTransferDstBarrier(const VkImage &image);

VkImageMemoryBarrier generalToTransferSrcBarrier(const VkImage &image);

VkImageMemoryBarrier transferDstToGeneralBarrier(const VkImage &image);

VkImageMemoryBarrier transferDstToReadOnlyBarrier(const VkImage &image);

VkImageMemoryBarrier transferDstToColorBarrier(const VkImage &image);

VkImageMemoryBarrier transferSrcToReadOnlyBarrier(const VkImage &image);

VkImageCopy imageCopyRegion(uint32_t width, uint32_t height);
} // namespace VklImageUtils

class VklImage {
  private:
    VklDevice &device_;

  public:
    VkImage image_ = VK_NULL_HANDLE;
    VkDeviceMemory memory_ = VK_NULL_HANDLE;
    VkImageView imageView;
    uint32_t width;
    uint32_t height;

    VklImage(VklDevice &device, uint32_t width, uint32_t height, VkImageUsageFlags usage) : device_(device) {
        device_.createImage(width, height, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_TILING_OPTIMAL, usage,
                            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, this->image_, this->memory_);
        this->imageView = device_.createImageView(image_, VK_FORMAT_R8G8B8A8_UNORM);
    }

    VkDescriptorImageInfo descriptorInfo(VkImageLayout imageLayout) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = imageLayout;
        imageInfo.imageView = imageView;

        return imageInfo;
    }

    ~VklImage() {
        vkFreeMemory(device_.device(), memory_, nullptr);
        vkDestroyImageView(device_.device(), imageView, nullptr);
        if (image_ != VK_NULL_HANDLE)
            vkDestroyImage(device_.device(), image_, nullptr);
    }
};