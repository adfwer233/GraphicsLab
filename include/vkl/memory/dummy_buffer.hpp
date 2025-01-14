#pragma once

#include <memory>

#include "vkl/core/vkl_buffer.hpp"

namespace Vkl {

struct DummyBuffer {
    static VkBuffer getDummyBuffer(VklDevice &device) {
        if (buffer == nullptr) {
            buffer = new VklBuffer(device, 12, 1, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                   VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        }
        return buffer->getBuffer();
    }

  private:
    static inline VklBuffer *buffer = nullptr;
};

} // namespace Vkl