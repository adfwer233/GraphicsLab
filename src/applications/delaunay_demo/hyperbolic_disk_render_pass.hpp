#pragma once

#include "graphics_lab/render_graph/render_pass.hpp"
#include "render_system/poincare_disk_render_system.hpp"
#include "ui/uistate.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/param_line_render_system.hpp"

namespace GraphicsLab::RenderGraph {

struct HyperbolicDiskRenderPass : public RenderPass {
    explicit HyperbolicDiskRenderPass(VklDevice &device, SceneTree::VklSceneTree &sceneTree, ProjectUIState &ui_state)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 0.0f}), sceneTree_(sceneTree), ui_state_(ui_state) {
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
        line_render_system = std::make_unique<PoincareDiskRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/mobius_trans_shader.vert.spv", CUSTOM_SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/mobius_trans_shader.frag.spv", CUSTOM_SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();
        uint32_t frameIndex = render_context->get_current_frame_index();

        begin_render_pass(commandBuffer);

        auto mesh_buffer = SceneTree::VklNodeMeshBuffer<CurveMesh2D>::instance();
        for (auto [mesh_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<CurveMesh2D>()) {
            if (not mesh_nodes->visible)
                continue;

            auto node_mesh = mesh_buffer->getGeometryModel(device_, mesh_nodes);

            FrameInfo frameInfo{
                .frameIndex = static_cast<int>(frameIndex) % 2,
                .frameTime = 0,
                .commandBuffer = commandBuffer,
                .model = *node_mesh->mesh,
            };

            PoincareDiskRenderSystemPushConstantData poincareDiskRenderSystemPushConstantData{};
            poincareDiskRenderSystemPushConstantData.a = complex_to_glm(ui_state_.trans.a);
            poincareDiskRenderSystemPushConstantData.b = complex_to_glm(ui_state_.trans.b);
            poincareDiskRenderSystemPushConstantData.c = complex_to_glm(ui_state_.trans.c);
            poincareDiskRenderSystemPushConstantData.d = complex_to_glm(ui_state_.trans.d);
            PoincareDiskRenderSystemPushConstantList list;
            list.data[0] = poincareDiskRenderSystemPushConstantData;
            line_render_system->renderObject(frameInfo, list);
        }

        end_render_pass(commandBuffer);
    }

    SceneTree::VklSceneTree &sceneTree_;
    ProjectUIState &ui_state_;

    std::unique_ptr<PoincareDiskRenderSystem<>> line_render_system = nullptr;

  private:
    auto complex_to_glm(std::complex<float> z) -> glm::vec2 {
        return {z.real(), z.imag()};
    }
};

} // namespace GraphicsLab::RenderGraph