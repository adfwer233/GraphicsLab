#pragma once

#include "vkl/core/vkl_device.hpp"

#include "vk_mem_alloc.h"

#include <cstring>
#include <stdexcept>

namespace vkl {

struct Buffer {
    Buffer() = delete;
    Buffer(const Buffer &) = delete;
    Buffer(Buffer &&other) = default;
    Buffer &operator=(const Buffer &) = delete;
    Buffer &operator=(Buffer &&) = default;

    Buffer(VklDevice &device, VkDeviceSize size, VkBufferUsageFlags buffer_usage, VmaMemoryUsage memory_usage,
           VmaAllocationCreateFlags flags = VMA_ALLOCATION_CREATE_MAPPED_BIT |
                                            VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT,
           const std::vector<uint32_t> &queue_family_indices = {})
        : device(device), size(size) {

        VkBufferCreateInfo buffer_info{};
        buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        buffer_info.size = size;
        buffer_info.usage = buffer_usage;
        buffer_info.sharingMode = queue_family_indices.empty() ? VK_SHARING_MODE_EXCLUSIVE : VK_SHARING_MODE_CONCURRENT;
        buffer_info.queueFamilyIndexCount = static_cast<uint32_t>(queue_family_indices.size());
        buffer_info.pQueueFamilyIndices = queue_family_indices.data();

        VmaAllocationCreateInfo alloc_info{};
        alloc_info.usage = memory_usage;
        alloc_info.flags = flags;

        if (vmaCreateBuffer(device.getAllocator(), &buffer_info, &alloc_info, &buffer, &allocation, nullptr) !=
            VK_SUCCESS) {
            throw std::runtime_error("Failed to create buffer!");
        }
    }

    void update(const void *data, VkDeviceSize dataSize, VkDeviceSize offset = 0) {
        void *mappedData;
        vmaMapMemory(device.getAllocator(), allocation, &mappedData);
        std::memcpy(static_cast<char *>(mappedData) + offset, data, dataSize);
        vmaUnmapMemory(device.getAllocator(), allocation);
    }

    VkBuffer get_handle() const {
        return buffer;
    }

    ~Buffer() {
        if (buffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(device.getAllocator(), buffer, allocation);
        }
    }

    VkBuffer get() const {
        return buffer;
    }

  private:
    VklDevice &device;
    VkBuffer buffer = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE;
    VkDeviceSize size;
};

} // namespace vkl

class VklBuffer {

  private:
    static VkDeviceSize getAlignment(VkDeviceSize instanceSize, VkDeviceSize minOffsetAlignment);

    VklDevice &device_;
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;

    void *mapped = nullptr;

    VkDeviceSize bufferSize;
    uint32_t instanceCount;
    VkDeviceSize instanceSize;
    VkDeviceSize alignmentSize;
    VkBufferUsageFlags usageFlags;
    VkMemoryPropertyFlags memoryPropertyFlags;

  public:
    VklBuffer(VklDevice &device, VkDeviceSize instanceSize, uint32_t instanceCount, VkBufferUsageFlags usageFlags,
              VkMemoryPropertyFlags memoryPropertyFlags, VkDeviceSize minOffsetAlignment = 1);

    ~VklBuffer();

    VklBuffer(const VklBuffer &) = delete;
    VklBuffer &operator=(const VklBuffer &) = delete;

    [[nodiscard]] VkBuffer getBuffer() const {
        return buffer;
    }

    /**
     * @brief Map a memory range (VkDeviceMemory) to the buffer (void*)
     * @param size
     * @param offset
     * @return
     */
    VkResult map(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);

    /**
     * @brief unmap the buffer
     */
    void unmap();

    /**
     * @brief write data to the mapped buffer
     * @param data
     * @param size
     * @param offset
     */
    void writeToBuffer(void *data, VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);

    /**
     * @brief flush buffer
     * @param size
     * @param offset
     * @return
     */
    VkResult flush(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);

    /**
     * @brief
     * @param size
     * @param offset
     * @return
     */
    VkDescriptorBufferInfo descriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
};