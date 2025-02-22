#pragma once

#include <functional>
#include <utility>

#include "vkl/system/render_system/simple_render_system.hpp"
#include "graphics_lab/render_graph/render_pass.hpp"

#include "simulation/fluid/grid.hpp"

#include <vkl/core/vkl_image.hpp>
#include <vkl/utils/imgui_utils.hpp>

namespace GraphicsLab::RenderGraph {

struct Grid2DRenderPass : public RenderPass {
    std::function<Simulation::Grid2D<float>&()> get_gird_to_show;
    int grid_width, grid_height;
    explicit Grid2DRenderPass(VklDevice &device, decltype(get_gird_to_show) get_grid_func) : RenderPass(device), get_gird_to_show(std::move(get_grid_func)) {
        const auto& grid = get_gird_to_show();
        grid_width = grid.width;
        grid_height = grid.height;
        spdlog::info("Grid2DRenderPass created");
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_input("grid_input", "the input grid texture")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(1)
            .visibility(RenderPassReflection::Field::Visibility::Internal)
            .layout(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(1024, 1024);
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        data_buffer = std::make_unique<vkl::Buffer>(device_, grid_width * grid_height * sizeof(float), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO);

        if (render_context->resource_manager.get_resource("grid_input")) {
            spdlog::info("Resource has been released");
        }
        if (auto input = dynamic_cast<ColorTextureResource*>(render_context->resource_manager.get_resource("grid_input"))) {
            spdlog::info("Grid2DRenderPass create imgui texture");
            render_context->imgui_resources.imguiImages["grid_result"] = vkl::ImguiUtils::getImguiTextureFromVklTexture(input->getTexture());
        }

    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto grid = get_gird_to_show();
        data_buffer->update(grid.data.data(), grid_width * grid_height * sizeof(float));

        auto commandBuffer = render_context->get_current_command_buffer();

        VkBufferImageCopy copyRegion = {};
        copyRegion.bufferOffset = 0; // Starting offset in the buffer
        copyRegion.bufferRowLength = grid_width; // Number of texels per row in the buffer
        copyRegion.bufferImageHeight = grid_height; // Height of the buffer

        // Image subresource, we want to copy to the color channel (red) of the image
        copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; // Color aspect (red channel in this case)
        copyRegion.imageSubresource.mipLevel = 0;
        copyRegion.imageSubresource.baseArrayLayer = 0;
        copyRegion.imageSubresource.layerCount = 1;

        // Image offset in 2D space and the extent of the image
        copyRegion.imageOffset = {0, 0, 0};
        copyRegion.imageExtent = VkExtent3D{static_cast<uint32_t>(grid_width), static_cast<uint32_t>(grid_height), 1};

        if (auto input = dynamic_cast<ColorTextureResource*>(render_context->resource_manager.get_resource("grid_input"))) {
            vkCmdCopyBufferToImage(commandBuffer, data_buffer->get_handle(), input->getTexture()->image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);
        }
    }

private:
    std::unique_ptr<vkl::Buffer> data_buffer = nullptr;
    std::unique_ptr<VklTexture> result_texture = nullptr;
};
}