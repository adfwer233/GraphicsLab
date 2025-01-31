#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "../../controller/ui_states.hpp"
#include "geometry/constructor/box3d.hpp"
#include "graphics_lab/render_graph/render_pass.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/aabb_box_render_system.hpp"
#include "vkl/system/render_system/line_render_system.hpp"
#include "vkl/system/render_system/normal_render_system.hpp"
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
                {std::format("{}/simple_color_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        color_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

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

        normal_render_system = std::make_unique<NormalRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/normal_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT},
                {std::format("{}/normal_generation.geom.spv", SHADER_DIR), VK_SHADER_STAGE_GEOMETRY_BIT}});

        box_render_system = std::make_unique<Box3DRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/3d_aabb_box.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/3d_aabb_box.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT},
                {std::format("{}/3d_aabb_box.geom.spv", SHADER_DIR), VK_SHADER_STAGE_GEOMETRY_BIT}});

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
        auto lineKey = line_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto normalKey = normal_render_system->descriptorSetLayout->descriptorSetLayoutKey;

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

                if (node_mesh->mesh->uniformBuffers.contains(normalKey)) {
                    node_mesh->mesh->uniformBuffers[normalKey][frame_index]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[normalKey][frame_index]->flush();
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

                if (uiState_.showNormal) {
                    NormalRenderSystemPushConstantData normalRenderSystemPushConstantData{};
                    normalRenderSystemPushConstantData.normalStrength = 0.005;
                    normalRenderSystemPushConstantData.normalColor = {0.0f, 0.0f, 1.0f};
                    NormalRenderSystemPushConstantDataList list;
                    list.data[0] = normalRenderSystemPushConstantData;
                    normal_render_system->renderObject(frameInfo, list);
                }
            }

            auto curve_mesh3d_buffer = SceneTree::VklNodeMeshBuffer<CurveMesh3D>::instance();
            for (auto [curve_mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<CurveMesh3D>()) {
                if (not curve_mesh3d_nodes->visible)
                    continue;

                ubo.model = trans;

                auto curve_node_mesh = curve_mesh3d_buffer->getGeometryModel(device_, curve_mesh3d_nodes);
                if (curve_node_mesh->mesh->uniformBuffers.contains(lineKey)) {
                    curve_node_mesh->mesh->uniformBuffers[lineKey][frame_index]->writeToBuffer(&ubo);
                    curve_node_mesh->mesh->uniformBuffers[lineKey][frame_index]->flush();
                }

                FrameInfo<std::decay_t<decltype(*curve_node_mesh)>::render_type> frameInfo{
                    .frameIndex = static_cast<int>(frame_index) % 2,
                    .frameTime = 0,
                    .commandBuffer = commandBuffer,
                    .camera = sceneTree_.active_camera->camera,
                    .model = *curve_node_mesh->mesh,
                };

                line_render_system->renderObject(frameInfo);
            }

            // draw box for the picked object
            if (sceneTree_.activeNode != nullptr) {
                glm::mat4 mvp = sceneTree_.active_camera->camera.get_proj_transformation() *
                                sceneTree_.active_camera->camera.get_view_transformation();
                glm::vec4 min_pos(uiState_.box.min_pos, 0.0f);
                glm::vec4 max_pos(uiState_.box.max_pos, 0.0f);
                min_pos.y *= -1;
                max_pos.y *= -1;
                Box3DRenderSystemPushConstantData push_constant_data{mvp, min_pos, max_pos};
                VklPushConstantInfoList<Box3DRenderSystemPushConstantData> push_constant_data_list;
                push_constant_data_list.data[0] = push_constant_data;
                box_render_system->renderPipeline(commandBuffer, push_constant_data_list);
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
    std::unique_ptr<NormalRenderSystem<>> normal_render_system = nullptr;
    std::unique_ptr<Box3DRenderSystem> box_render_system = nullptr;
};
} // namespace GraphicsLab::RenderGraph
