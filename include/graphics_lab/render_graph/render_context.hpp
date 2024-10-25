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

    explicit RenderContext(VklDevice& device, VklWindow& window, VkExtent2D extent): resource_manager(device), device_(device),
                                                                                      window_(window) {
        width_ = window.getExtent().width;
        height_ = window.getExtent().height;

        recreate_swap_chain();
        create_command_buffers();
    }

    VkExtent2D get_full_screen_size() const {
        if (swap_chain_ == nullptr) {
            throw std::runtime_error("swap chain is not created in rendering context");
        }
        return swap_chain_->getSwapChainExtent();
    }

    VkCommandBuffer get_current_command_buffer() {
        return command_buffers_[current_frame_index_];
    }

    VkRenderPass get_swap_chain_render_pass() {
        return swap_chain_->getRenderPass();
    }

    [[nodiscard]] GLFWwindow* get_glfw_window() { return window_.getGLFWwindow(); }

    void recreate_swap_chain() {
        auto extent = window_.getExtent();
        while (extent.width == 0 || extent.height == 0) {
            extent = window_.getExtent();
            glfwWaitEvents();
        }
        vkDeviceWaitIdle(device_.device());

        if (swap_chain_ == nullptr) {
            swap_chain_ = std::make_unique<VklSwapChain>(device_, extent);
        } else {
            std::shared_ptr<VklSwapChain> oldSwapChain = std::move(swap_chain_);
            swap_chain_ = std::make_unique<VklSwapChain>(device_, extent, oldSwapChain);

            if (!oldSwapChain->compareSwapFormats(*swap_chain_.get())) {
                throw std::runtime_error("Swap chain image(or depth) format has changed!");
            }
        }
    }

    VkCommandBuffer begin_frame() {
        assert(!is_frame_started_ && "Can't call beginFrame while already in progress");

        auto result = swap_chain_->acquireNextImage(&current_image_index_);
        if (result == VK_ERROR_OUT_OF_DATE_KHR) {
            recreate_swap_chain();
            return nullptr;
        }

        if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        is_frame_started_ = true;

        auto commandBuffer = get_current_command_buffer();
        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording command buffer!");
        }
        return commandBuffer;
    }

    VkResult end_frame() {
        vkDeviceWaitIdle(device_.device());
        assert(is_frame_started_ && "Can't call endFrame while frame is not in progress");
        auto commandBuffer = get_current_command_buffer();
        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to record command buffer!");
        }

        std::vector<VkCommandBuffer> command_buffer_to_submit{commandBuffer};
        auto result = swap_chain_->submitCommandBuffers(command_buffer_to_submit, &current_image_index_);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
            recreate_swap_chain();
        } else if (result != VK_SUCCESS) {
            throw std::runtime_error("failed to present swap chain image!");
        }

        is_frame_started_ = false;
        current_frame_index_ = (current_frame_index_ + 1) % VklSwapChain::MAX_FRAMES_IN_FLIGHT;

        return result;
    }

    [[nodiscard]] uint32_t get_current_frame_index() const { return current_frame_index_; }

    ~RenderContext() {
        vkDeviceWaitIdle(device_.device());
    }

  private:
    void create_command_buffers() {
        command_buffers_.resize(VklSwapChain::MAX_FRAMES_IN_FLIGHT);

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = device_.getCommandPool();
        allocInfo.commandBufferCount = static_cast<uint32_t>(command_buffers_.size());

        if (vkAllocateCommandBuffers(device_.device(), &allocInfo, command_buffers_.data()) !=
            VK_SUCCESS) {
            throw std::runtime_error("failed to allocate command buffers!");
        }
    }


    uint32_t width_, height_;
    VklDevice& device_;

    std::unique_ptr<VklSwapChain> swap_chain_ = nullptr;
    VklWindow& window_;

    std::vector<VkCommandBuffer> command_buffers_;

    uint32_t current_frame_index_ = 0;
    uint32_t current_image_index_ = 0;

    bool is_frame_started_ = false;

    friend class SwapChainPass;
};
} // namespace RenderGraph
} // namespace GraphicsLab