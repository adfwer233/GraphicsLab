#pragma once

#include "resource_manager.hpp"

namespace GraphicsLab {
namespace RenderGraph {
struct RenderContext {
    ResourceManager resource_manager;

    uint32_t get_width() const {
        return width_;
    }
    uint32_t get_height() const {
        return height_;
    }

  private:
    uint32_t width_, height_;
};
} // namespace RenderGraph
} // namespace GraphicsLab