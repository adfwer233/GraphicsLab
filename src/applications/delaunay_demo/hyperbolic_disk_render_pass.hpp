#pragma once
#include "graphics_lab/render_graph/render_pass.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/param_line_render_system.hpp"

struct UIState;
struct RenderResources;
namespace GraphicsLab::RenderGraph {

/**
 * Hyperbolic GBO, save data for Mobius transformation
 */
struct HyperbolicGBO {

};

struct HyperbolicDiskRenderPass: public RenderPass {
    explicit HyperbolicDiskRenderPass(VklDevice &device, SceneTree::VklSceneTree &sceneTree)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 0.0f}), sceneTree_(sceneTree) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("hyperbolic_render_result", "Hyperbolic disk render result")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", "true");

        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        line_render_system = std::make_unique<ParamLineRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/param_curve_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/param_curve_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();
    }

    SceneTree::VklSceneTree &sceneTree_;
    // UIState &uiState_;

    std::unique_ptr<ParamLineRenderSystem<>> line_render_system = nullptr;
};

}