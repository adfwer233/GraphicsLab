#pragma once

#include <functional>
#include <utility>

#include "graphics_lab/render_graph/render_pass.hpp"
#include "vkl/system/render_system/single_texture_render_system.hpp"

#include "simulation/fluid/grid.hpp"

#include <vkl/core/vkl_image.hpp>
#include <vkl/utils/imgui_utils.hpp>

namespace GraphicsLab::RenderGraph {

struct Grid2DRenderPass : public RenderPass {
    std::function<Simulation::Grid2D<float> &()> get_gird_to_show;
    int grid_width, grid_height;
    explicit Grid2DRenderPass(VklDevice &device, decltype(get_gird_to_show) get_grid_func)
        : RenderPass(device), get_gird_to_show(std::move(get_grid_func)) {
        const auto &grid = get_gird_to_show();
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
            .layout(VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(grid_width, grid_height);

        reflection.add_input("grid_output", "the input grid texture")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .visibility(RenderPassReflection::Field::Visibility::Output)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);

        reflection.add_input("grid_output_depth", "the input grid texture")
            .type(RenderPassReflection::Field::Type::TextureDepth)
            .sample_count(8)
            .visibility(RenderPassReflection::Field::Visibility::Output)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048);
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        simple_render_system = std::make_unique<SingleTextureRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/2d_texture_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/2d_texture_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT},
                {std::format("{}/2d_texture_shader.geom.spv", SHADER_DIR), VK_SHADER_STAGE_GEOMETRY_BIT}});
        data_buffer = std::make_unique<vkl::Buffer>(device_, grid_width * grid_height * sizeof(float),
                                                    VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO);

        if (render_context->resource_manager.get_resource("grid_input")) {
            spdlog::info("Resource has been released");
        }
        if (auto input =
                dynamic_cast<ColorTextureResource *>(render_context->resource_manager.get_resource("grid_input"))) {
            spdlog::info("Grid2DRenderPass create imgui texture");
            render_context->imgui_resources.imguiImages["grid_result2"] =
                vkl::ImguiUtils::getImguiTextureFromVklTexture(input->getTexture());
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto grid = get_gird_to_show();
        data_buffer->update(grid.data.data(), grid_width * grid_height * sizeof(float));

        auto commandBuffer = render_context->get_current_command_buffer();

        VkBufferImageCopy copyRegion = {};
        copyRegion.bufferOffset = 0;                // Starting offset in the buffer
        copyRegion.bufferRowLength = grid_width;    // Number of texels per row in the buffer
        copyRegion.bufferImageHeight = grid_height; // Height of the buffer

        // Image subresource, we want to copy to the color channel (red) of the image
        copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT; // Color aspect (red channel in this case)
        copyRegion.imageSubresource.mipLevel = 0;
        copyRegion.imageSubresource.baseArrayLayer = 0;
        copyRegion.imageSubresource.layerCount = 1;

        // Image offset in 2D space and the extent of the image
        copyRegion.imageOffset = {0, 0, 0};
        copyRegion.imageExtent = VkExtent3D{static_cast<uint32_t>(grid_width), static_cast<uint32_t>(grid_height), 1};

        if (auto input =
                dynamic_cast<ColorTextureResource *>(render_context->resource_manager.get_resource("grid_input"))) {

            VkImageMemoryBarrier read2dst = VklImageUtils::ReadOnlyToDstBarrier(input->getTexture()->image_);
            vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &read2dst);

            vkCmdCopyBufferToImage(commandBuffer, data_buffer->get_handle(), input->getTexture()->image_,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

            VkImageMemoryBarrier dst2read = VklImageUtils::transferDstToReadOnlyBarrier(input->getTexture()->image_);
            vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                 0, 0, nullptr, 0, nullptr, 1, &dst2read);
        }

        begin_render_pass(commandBuffer);
        simple_render_system->renderPipeline(commandBuffer, std::nullopt, &descriptorSet);
        end_render_pass(commandBuffer);
    }

  private:
    std::unique_ptr<SingleTextureRenderSystem> simple_render_system = nullptr;
    std::unique_ptr<vkl::Buffer> data_buffer = nullptr;
    std::unique_ptr<VklTexture> result_texture = nullptr;
};
} // namespace GraphicsLab::RenderGraph