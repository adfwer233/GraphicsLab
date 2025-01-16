#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "../../controller/ui_states.hpp"
#include "geometry/constructor/box3d.hpp"
#include "graphics_lab/render_graph/render_pass.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/line_render_system.hpp"
#include "vkl/system/render_system/simple_wireframe_render_system.hpp"

namespace GraphicsLab::RenderGraph {
struct InternalSceneRenderPass : public RenderPass {
    explicit InternalSceneRenderPass(VklDevice &device, SceneTree::VklSceneTree &sceneTree, UIState &uiState)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 0.0f}), sceneTree_(sceneTree), uiState_(uiState) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("scene_render_result", "Built in 3d scene rendering")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);

        reflection.add_output("scene_render_depth", "depth texture of built in 3d scene rendering")
            .type(RenderPassReflection::Field::Type::TextureDepth)
            .sample_count(8)
            .extent(2048, 2048);

        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        simple_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/simple_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        raw_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        color_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/simple_color_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        line_render_system = std::make_unique<LineRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/line_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        wireframe_render_system = std::make_unique<SimpleWireFrameRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        boxNode.data = Box3DConstructor::create({0, 0, 0}, {5, 5, 5});
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();
        begin_render_pass(commandBuffer);
        uint32_t frame_index = render_context->get_current_frame_index();
        std::scoped_lock sceneTreeLock(sceneTree_.sceneTreeMutex);

        GlobalUbo ubo{};

        if (sceneTree_.active_camera == nullptr) {
            spdlog::error("Scene camera is null");
            return;
        }

        ubo.view = sceneTree_.active_camera->camera.get_view_transformation();
        ubo.proj = sceneTree_.active_camera->camera.get_proj_transformation();
        ubo.model = glm::mat4(1.0f);
        ubo.pointLight = PointLight(glm::vec4(sceneTree_.active_camera->camera.position, 1.0), {1.0, 1.0, 1.0, 0.0});
        ubo.cameraPos = sceneTree_.active_camera->camera.position;

        auto textureKey = simple_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto colorKey = color_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto rawKey = raw_render_system->descriptorSetLayout->descriptorSetLayoutKey;

        try {
            auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<Mesh3D>::instance();
            for (auto [mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<Mesh3D>()) {

                if (not mesh3d_nodes->visible)
                    continue;

                ubo.model = trans;

                auto node_mesh = mesh3d_buffer->getGeometryModel(device_, mesh3d_nodes);

                if (mesh3d_nodes->updated) {
                    node_mesh->recreateMeshes();
                    mesh3d_nodes->updated = false;
                }

                if (node_mesh->mesh->uniformBuffers.contains(textureKey)) {
                    node_mesh->mesh->uniformBuffers[textureKey][frame_index]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[textureKey][frame_index]->flush();
                }

                if (node_mesh->mesh->uniformBuffers.contains(colorKey)) {
                    node_mesh->mesh->uniformBuffers[colorKey][frame_index]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[colorKey][frame_index]->flush();
                }

                if (node_mesh->mesh->uniformBuffers.contains(rawKey)) {
                    node_mesh->mesh->uniformBuffers[rawKey][frame_index]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[rawKey][frame_index]->flush();
                }

                FrameInfo<std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                    .frameIndex = static_cast<int>(frame_index) % 2,
                    .frameTime = 0,
                    .commandBuffer = commandBuffer,
                    .camera = sceneTree_.active_camera->camera,
                    .model = *node_mesh->mesh,
                };

                if (uiState_.renderMode == UIState::RenderMode::raw) {
                    raw_render_system->renderObject(frameInfo);
                } else if (uiState_.renderMode == UIState::RenderMode::wireframe) {
                    wireframe_render_system->renderObject(frameInfo);
                } else if (uiState_.renderMode == UIState::RenderMode::material) {
                    if (node_mesh->mesh->textures_.size() >=
                        simple_render_system->descriptorSetLayout->descriptorSetLayoutKey.sampledImageBufferDescriptors
                            .size()) {
                        simple_render_system->renderObject(frameInfo);
                    } else {
                        color_render_system->renderObject(frameInfo);
                    }
                }
            }

            // draw box for the picked object

            if (sceneTree_.activeNode != nullptr) {
                auto line_render_system_key = line_render_system->descriptorSetLayout->descriptorSetLayoutKey;
                auto wire3d_buffer = SceneTree::VklNodeMeshBuffer<Wire3D>::instance();

                auto wire_mesh = wire3d_buffer->getGeometryModel(device_, &boxNode);

                if (not uiState_.boxMeshRecreated) {
                    boxNode.data = Box3DConstructor::create(uiState_.box.min_pos, uiState_.box.max_pos);
                    wire_mesh->recreateMeshes();
                    uiState_.boxMeshRecreated = true;
                }

                if (wire_mesh->mesh->uniformBuffers.contains(line_render_system_key)) {
                    wire_mesh->mesh->uniformBuffers[line_render_system_key][frame_index]->writeToBuffer(&ubo);
                    wire_mesh->mesh->uniformBuffers[line_render_system_key][frame_index]->flush();
                }

                FrameInfo<std::decay_t<decltype(*wire_mesh)>::render_type> frameInfo{
                    .frameIndex = static_cast<int>(frame_index) % 2,
                    .frameTime = 0,
                    .commandBuffer = commandBuffer,
                    .camera = sceneTree_.active_camera->camera,
                    .model = *wire_mesh->mesh,
                };

                line_render_system->renderObject(frameInfo);
            }
        } catch (std::exception &e) {
            spdlog::error("{}", e.what());
        }
        end_render_pass(commandBuffer);
    }

  private:
    SceneTree::VklSceneTree &sceneTree_;
    UIState &uiState_;
    SceneTree::GeometryNode<Wire3D> boxNode;

    std::unique_ptr<SimpleRenderSystem<>> simple_render_system = nullptr;
    std::unique_ptr<SimpleRenderSystem<>> color_render_system = nullptr;
    std::unique_ptr<SimpleRenderSystem<>> raw_render_system = nullptr;
    std::unique_ptr<LineRenderSystem<>> line_render_system = nullptr;
    std::unique_ptr<SimpleWireFrameRenderSystem<>> wireframe_render_system = nullptr;
};
} // namespace GraphicsLab::RenderGraph
