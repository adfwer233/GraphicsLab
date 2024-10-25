#pragma once

#include "resource_manager.hpp"

#include "vkl/core/vkl_swap_chain.hpp"

namespace GraphicsLab {
namespace RenderGraph {

/**
 * @brief RenderContext manages swap chain and window
 */
struct RenderContext {
    ResourceManager resource_manager;

    uint32_t get_width() const {
        return width_;
    }
    uint32_t get_height() const {
        return height_;
    }

    explicit RenderContext(VklDevice& device, GLFWwindow* window, VkExtent2D extent): resource_manager(device), device_(device) {

    }

    VkExtent2D get_full_screen_size() const {
        if (swap_chain_ == nullptr) {
            throw std::runtime_error("swap chain is not created in rendering context");
        }
        return swap_chain_->getSwapChainExtent();
    }

  private:
    uint32_t width_, height_;
    VklDevice& device_;

    std::unique_ptr<VklSwapChain> swap_chain_ = nullptr;
    GLFWwindow* window_ = nullptr;
};
} // namespace RenderGraph
} // namespace GraphicsLab