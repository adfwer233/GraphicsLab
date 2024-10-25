#pragma once

#include "graphics_lab/render_graph/render_pass.hpp"

namespace GraphicsLab::RenderGraph {

/**
 * @brief special render pass used to render to swap chain
 *
 * `SpecialChainPass` blit a image to the swap chain rendering context
 */
struct SwapChainPass: public RenderPass {
    explicit SwapChainPass(VklDevice &device, const std::string& output_name) : RenderPass(device), output_name_(output_name) {}
    explicit SwapChainPass(VklDevice &device) : RenderPass(device) {}

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;

        reflection.add_input(output_name_, "The texture to present to swap chain");

        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        auto resource = render_context->resource_manager.get_resource(output_name_);
        auto color_texture = reinterpret_cast<ColorTextureResource*>(resource);
        output_texture_ = color_texture->getTexture();

        if (color_texture->get_resolved_texture()) {
            output_texture_ = color_texture->get_resolved_texture();
        }

        for (int i = 0; i < render_context->swap_chain_->imageCount(); i++) {
            device_.transitionImageLayout(render_context->swap_chain_->getImage(i),
                                          render_context->swap_chain_->getSwapChainImageFormat() , VK_IMAGE_LAYOUT_UNDEFINED,
                                          VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        VkImageBlit imageBlit = {};
        imageBlit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imageBlit.srcSubresource.layerCount = 1;
        imageBlit.srcOffsets[1] = { 1024, 1024, 1 };

        imageBlit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imageBlit.dstSubresource.layerCount = 1;
        imageBlit.dstOffsets[1] = { static_cast<int32_t>(render_context->get_width()), static_cast<int32_t>(render_context->get_height()), 1 };

        // device_.transitionImageLayout(commandBuffer, output_texture_->getTextureImageView(), VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

        /**
         * @todo: transition image layout
         */

        device_.transitionImageLayout(output_texture_->image_,
                                      VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                      VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, commandBuffer);

        device_.transitionImageLayout(render_context->swap_chain_->getImage(render_context->current_image_index_),
                                      render_context->swap_chain_->getSwapChainImageFormat() , VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                                      VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, commandBuffer);

        vkCmdBlitImage(
                commandBuffer,
                output_texture_->image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                render_context->swap_chain_->getImage(render_context->current_image_index_), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                1, &imageBlit,
                VK_FILTER_LINEAR // or VK_FILTER_NEAREST
        );

        device_.transitionImageLayout(output_texture_->image_,
                                      VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                      VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, commandBuffer);

        device_.transitionImageLayout(render_context->swap_chain_->getImage(render_context->current_image_index_),
                                      render_context->swap_chain_->getSwapChainImageFormat() , VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                      VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, commandBuffer);
    }

protected:
    VklSwapChain* get_current_swap_chain(RenderContext* context) {
        return context->swap_chain_.get();
    }

    void begin_swap_chain_render_pass(VkCommandBuffer commandBuffer, RenderContext* context) {
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = context->swap_chain_->getRenderPass();
        renderPassInfo.framebuffer = context->swap_chain_->getFrameBuffer(context->current_image_index_);

        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = context->swap_chain_->getSwapChainExtent();

        std::array<VkClearValue, 2> clearValues{};
        clearValues[0].color = {0.01f, 0.01f, 0.01f, 1.0f};
        clearValues[1].depthStencil = {1.0f, 0};
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(context->swap_chain_->getSwapChainExtent().width);
        viewport.height = static_cast<float>(context->swap_chain_->getSwapChainExtent().height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        VkRect2D scissor{{0, 0}, context->swap_chain_->getSwapChainExtent()};
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
    }

    VklTexture* output_texture_ = nullptr;
    std::string output_name_;
};

}