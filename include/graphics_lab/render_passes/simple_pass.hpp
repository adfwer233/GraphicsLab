#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "graphics_lab/render_graph/render_pass.hpp"

namespace GraphicsLab::RenderGraph {
struct SimpleRenderPass : public RenderPass {
    explicit SimpleRenderPass(VklDevice &device) : RenderPass(device) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("simple_output", "The output texture of simple pass, which will be a triangle")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        simple_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "first_triangle_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "first_triangle_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_render_pass(commandBuffer);
        simple_render_system->bindPipeline(commandBuffer);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        end_render_pass(commandBuffer);
    }

  private:
    std::unique_ptr<SimpleRenderSystem<>> simple_render_system = nullptr;
};
} // namespace GraphicsLab::RenderGraph
