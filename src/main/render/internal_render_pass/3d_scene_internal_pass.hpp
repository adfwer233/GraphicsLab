#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "../../controller/ui_states.hpp"
#include "geometry/constructor/box3d.hpp"
#include "graphics_lab/render_graph/render_pass.hpp"
#include "vkl/scene_tree/vkl_geometry_mesh.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/aabb_box_render_system.hpp"
#include "vkl/system/render_system/directional_field_render_system.hpp"
#include "vkl/system/render_system/line_render_system.hpp"
#include "vkl/system/render_system/normal_render_system.hpp"
#include "vkl/system/render_system/simple_wireframe_render_system.hpp"
#include "vkl/system/render_system/world_axis_render_system.hpp"
#include "vkl/utils/imgui_utils.hpp"

#include "vkl/system/compute_system/scene_tree_path_tracing_compute_model.hpp"
#include "vkl/system/ray_tracing_system/simple_ray_tracing_system.hpp"

#include <geometry/parametric/curve_type_list.hpp>
#include <geometry/parametric/surface_type_list.hpp>
#include <ui/render_resources.hpp>

#include "vkl/core/vkl_soft_rasterizer.hpp"

#include <utils/sampler.hpp>

namespace GraphicsLab::RenderGraph {
struct InternalSceneRenderPass : public RenderPass {
    explicit InternalSceneRenderPass(VklDevice &device, SceneTree::VklSceneTree &sceneTree, UIState &uiState,
                                     RenderResources &renderResources)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 0.0f}), sceneTree_(sceneTree), uiState_(uiState),
          renderResources_(renderResources) {
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
                {(DEFAULT_SHADER_PATH / "simple_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "simple_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        raw_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "simple_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "point_light_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        color_render_system = std::make_unique<SimpleRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "simple_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "simple_color_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        line_render_system = std::make_unique<LineRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "line_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "line_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        wireframe_render_system = std::make_unique<SimpleWireFrameRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "simple_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "simple_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        normal_render_system = std::make_unique<NormalRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "normal_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "line_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT},
                {(DEFAULT_SHADER_PATH / "normal_generation.geom.spv").string(), VK_SHADER_STAGE_GEOMETRY_BIT}});

        box_render_system = std::make_unique<Box3DRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "3d_aabb_box.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "3d_aabb_box.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT},
                {(DEFAULT_SHADER_PATH / "3d_aabb_box.geom.spv").string(), VK_SHADER_STAGE_GEOMETRY_BIT}});

        axis_render_system = std::make_unique<WorldAxisRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "3d_world_axis.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "3d_world_axis.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT},
                {(DEFAULT_SHADER_PATH / "3d_world_axis.geom.spv").string(), VK_SHADER_STAGE_GEOMETRY_BIT}});

        point_cloud_render_system = std::make_unique<PointCloud3DRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "point_cloud_3d_shader.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "point_cloud_3d_shader.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT}});

        directional_filed_render_system = std::make_unique<PointCloud3DRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {(DEFAULT_SHADER_PATH / "directional_field_3d.vert.spv").string(), VK_SHADER_STAGE_VERTEX_BIT},
                {(DEFAULT_SHADER_PATH / "directional_field_3d.frag.spv").string(), VK_SHADER_STAGE_FRAGMENT_BIT},
                {(DEFAULT_SHADER_PATH / "directional_field_3d.geom.spv").string(), VK_SHADER_STAGE_GEOMETRY_BIT}});

        software_rasterizer = std::make_unique<vkl::SoftwareRasterizer>(device_, sceneTree_.material_manager, 512, 512);

        /**
         * Initialize path tracing texture
         */
        try {
            path_tracing_compute_model = std::make_unique<vkl::PathTracingComputeModel>(device_, sceneTree_);
            path_tracing_compute_system =
                std::make_unique<vkl::PathTracingComputeSystem>(device_, *path_tracing_compute_model);
            path_tracing_texture = std::make_unique<VklTexture>(
                device_, 1024, 1024, 4,
                VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_FORMAT_R8G8B8A8_SRGB, VK_SAMPLE_COUNT_1_BIT);
        } catch (...) {
            spdlog::info("Failed to initialize path tracing compute system, disabling path tracing");
            path_tracing_compute_system = nullptr;
            path_tracing_compute_model = nullptr;
            path_tracing_texture = nullptr;
        }
        vkDeviceWaitIdle(device_.device());

        auto imguiContext = ImguiContext::getInstance(device_, render_context->get_glfw_window(),
                                                      render_context->get_swap_chain_render_pass());
        if (path_tracing_texture != nullptr) {
            renderResources_.imguiImages["path_tracing_result"] =
                vkl::ImguiUtils::getImguiTextureFromVklTexture(path_tracing_texture.get());
        }

        /**
         * Initializing soft rasterizer textures
         */

        try {
            software_rasterizer_texture = std::make_unique<VklTexture>(
                device_, 512, 512, 4,
                VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_FORMAT_R8G8B8A8_SRGB, VK_SAMPLE_COUNT_1_BIT);

            renderResources_.imguiImages["software_rasterizer"] =
                vkl::ImguiUtils::getImguiTextureFromVklTexture(software_rasterizer_texture.get());
        } catch (...) {
            spdlog::error("Failed to initialize software rasterizer");
            software_rasterizer_texture = nullptr;
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        if (uiState_.reset_gpu_bvh and path_tracing_compute_system != nullptr) {
            path_tracing_compute_model = std::make_unique<vkl::PathTracingComputeModel>(device_, sceneTree_);
            path_tracing_compute_system =
                std::make_unique<vkl::PathTracingComputeSystem>(device_, *path_tracing_compute_model);
            vkDeviceWaitIdle(device_.device());
            uiState_.reset_gpu_bvh = false;
        }

        auto commandBuffer = render_context->get_current_command_buffer();

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
        ubo.pointLight = PointLight(glm::vec4(sceneTree_.active_camera->camera.position, 1.0), {1.0, 1.0, 1.0, 1.0});
        ubo.cameraPos = sceneTree_.active_camera->camera.position;

        auto textureKey = simple_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto colorKey = color_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto rawKey = raw_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto lineKey = line_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto normalKey = normal_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto directionalfieldKey = directional_filed_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto pointcloudKey = point_cloud_render_system->descriptorSetLayout->descriptorSetLayoutKey;

        using RenderableTypeList =
            MetaProgramming::TypeListFunctions::Append<Geometry::ParamSurfaceTypeList, Mesh3D>::type;

        if (uiState_.renderMode == UIState::RenderMode::soft_rasterizer and software_rasterizer != nullptr) {
            auto target = render_context->resource_manager.get_resource("soft_rasterizer_result");
            software_rasterizer->clear();
            for (auto [mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<Mesh3D>()) {
                software_rasterizer->rasterize_mesh(mesh3d_nodes->data, ubo.proj * ubo.view * trans, ubo.cameraPos,
                                                    ubo.cameraPos, mesh3d_nodes->material_index.value_or(0));
            }
            software_rasterizer->copy_result_to_image(commandBuffer, software_rasterizer_texture->image_);
            begin_render_pass(commandBuffer);
        } else if (uiState_.renderMode == UIState::RenderMode::path_tracing and
                   path_tracing_compute_system != nullptr) {
            auto target = render_context->resource_manager.get_resource("scene_render_result");

            auto targetTexture = path_tracing_compute_model->getTargetTexture();
            auto accumulationTexture = path_tracing_compute_model->getAccumulationTexture();

            path_tracing_compute_system->computeModel_.ubo.cameraZoom = sceneTree_.active_camera->camera.zoom;
            path_tracing_compute_system->computeModel_.ubo.cameraPosition = sceneTree_.active_camera->camera.position;
            path_tracing_compute_system->computeModel_.ubo.cameraUp = sceneTree_.active_camera->camera.camera_up_axis;
            path_tracing_compute_system->computeModel_.ubo.cameraFront = sceneTree_.active_camera->camera.camera_front;
            path_tracing_compute_system->computeModel_.ubo.currentSample += 1;
            path_tracing_compute_system->computeModel_.ubo.rand1 = sceneTree_.active_camera->camera.ratio;
            path_tracing_compute_system->computeModel_.ubo.rand2 = Sampler::sampleUniform(0, 10000);
            path_tracing_compute_system->updateUniformBuffer(frame_index);

            if (uiState_.reset_camera == true) {
                uiState_.reset_camera = false;
                path_tracing_compute_model->ubo.currentSample = 0;
            }

            if (auto colorTexture = dynamic_cast<ColorTextureResource *>(target)) {
                path_tracing_compute_system->recordCommandBuffer(commandBuffer, targetTexture, accumulationTexture,
                                                                 path_tracing_texture->image_, frame_index);
            }
            begin_render_pass(commandBuffer);

        } else {
            begin_render_pass(commandBuffer);
            try {
                MetaProgramming::ForEachType(RenderableTypeList{}, [&]<typename T>() {
                    auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<T>::instance();
                    for (auto [mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<T>()) {

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

                        FrameInfo<typename std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                            .frameIndex = static_cast<int>(frame_index) % 2,
                            .frameTime = 0,
                            .commandBuffer = commandBuffer,
                            .model = *node_mesh->mesh,
                        };

                        if (uiState_.renderMode == UIState::RenderMode::raw) {
                            raw_render_system->renderObject(frameInfo);
                        } else if (uiState_.renderMode == UIState::RenderMode::wireframe) {
                            wireframe_render_system->renderObject(frameInfo);
                        } else if (uiState_.renderMode == UIState::RenderMode::color) {
                            simple_render_system->renderObject(frameInfo);
                        } else if (uiState_.renderMode == UIState::RenderMode::material) {
                            if (node_mesh->mesh->textures_.size() !=
                                color_render_system->descriptorSetLayout->descriptorSetLayoutKey
                                    .sampledImageBufferDescriptors.size()) {
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
                });

                MetaProgramming::ForEachType(Geometry::ParametricCurve3DTypeList{}, [&]<typename T>() {
                    auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<T>::instance();
                    for (auto [mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<T>()) {
                        if (not mesh3d_nodes->visible)
                            continue;
                        ubo.model = trans;
                        auto node_mesh = mesh3d_buffer->getGeometryModel(device_, mesh3d_nodes);
                        static_assert(std::same_as<typename std::decay_t<decltype(*node_mesh)>::render_type,
                                                   SceneTree::VklMesh<Vertex3D, LineIndex>>);
                        if (node_mesh->mesh->uniformBuffers.contains(lineKey)) {
                            node_mesh->mesh->uniformBuffers[lineKey][frame_index]->writeToBuffer(&ubo);
                            node_mesh->mesh->uniformBuffers[lineKey][frame_index]->flush();
                        }

                        FrameInfo<typename std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                            .frameIndex = static_cast<int>(frame_index) % 2,
                            .frameTime = 0,
                            .commandBuffer = commandBuffer,
                            .model = *node_mesh->mesh,
                        };

                        line_render_system->renderObject(frameInfo);
                    }
                });

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
                        .model = *curve_node_mesh->mesh,
                    };

                    line_render_system->renderObject(frameInfo);
                }

                /**
                 * render direction field
                 */
                auto directional_field_buffer = SceneTree::VklNodeMeshBuffer<DirectionalField3D>::instance();
                for (auto [directional_field_node, trans] :
                     sceneTree_.traverse_geometry_nodes_with_trans<DirectionalField3D>()) {
                    if (not directional_field_node->visible)
                        continue;
                    ubo.model = trans;

                    auto mesh = directional_field_buffer->getGeometryModel(device_, directional_field_node);

                    if (mesh->mesh->uniformBuffers.contains(directionalfieldKey)) {
                        mesh->mesh->uniformBuffers[directionalfieldKey][frame_index]->writeToBuffer(&ubo);
                        mesh->mesh->uniformBuffers[directionalfieldKey][frame_index]->flush();
                    }

                    FrameInfo<std::decay_t<decltype(*mesh)>::render_type> frameInfo{
                        .frameIndex = static_cast<int>(frame_index) % 2,
                        .frameTime = 0,
                        .commandBuffer = commandBuffer,
                        .model = *mesh->mesh,
                    };

                    directional_filed_render_system->renderObject(frameInfo);
                }

                /**
                 * render point cloud
                 */
                auto point_cloud_buffer = SceneTree::VklNodeMeshBuffer<PointCloud3D>::instance();
                for (auto [point_cloud_node, trans] : sceneTree_.traverse_geometry_nodes_with_trans<PointCloud3D>()) {
                    if (not point_cloud_node->visible)
                        continue;

                    ubo.model = trans;
                    auto mesh = point_cloud_buffer->getGeometryModel(device_, point_cloud_node);

                    if (mesh->mesh->uniformBuffers.contains(pointcloudKey)) {
                        mesh->mesh->uniformBuffers[pointcloudKey][frame_index]->writeToBuffer(&ubo);
                        mesh->mesh->uniformBuffers[pointcloudKey][frame_index]->flush();
                    }

                    FrameInfo<std::decay_t<decltype(*mesh)>::render_type> frameInfo{
                        .frameIndex = static_cast<int>(frame_index) % 2,
                        .frameTime = 0,
                        .commandBuffer = commandBuffer,
                        .model = *mesh->mesh,
                    };

                    point_cloud_render_system->renderObject(frameInfo);
                }

                // draw box for the picked object
                if (uiState_.showBox) {
                    if (sceneTree_.activeNode != nullptr) {
                        glm::mat4 mvp = sceneTree_.active_camera->camera.get_proj_transformation() *
                                        sceneTree_.active_camera->camera.get_view_transformation();
                        glm::vec4 min_pos(uiState_.box.min_pos, 0.0f);
                        glm::vec4 max_pos(uiState_.box.max_pos, 0.0f);
                        Box3DRenderSystemPushConstantData push_constant_data{mvp, min_pos, max_pos};
                        VklPushConstantInfoList<Box3DRenderSystemPushConstantData> push_constant_data_list;
                        push_constant_data_list.data[0] = push_constant_data;
                        box_render_system->renderPipeline(commandBuffer, push_constant_data_list);
                    }
                }

                if (uiState_.showAxis) {
                    glm::mat4 mvp = sceneTree_.active_camera->camera.get_proj_transformation() *
                                    sceneTree_.active_camera->camera.get_view_transformation();
                    WorldAxisRenderSystemPushConstantData push_constant_data{mvp};
                    VklPushConstantInfoList<WorldAxisRenderSystemPushConstantData> push_constant_data_list;
                    push_constant_data_list.data[0] = push_constant_data;
                    axis_render_system->renderPipeline(commandBuffer, push_constant_data_list);
                }
            } catch (std::exception &e) {
                spdlog::error("{}", e.what());
            }
        }
        end_render_pass(commandBuffer);
    }

  private:
    SceneTree::VklSceneTree &sceneTree_;
    UIState &uiState_;
    RenderResources &renderResources_;
    SceneTree::GeometryNode<Wire3D> boxNode;

    std::unique_ptr<SimpleRenderSystem<>> simple_render_system = nullptr;
    std::unique_ptr<SimpleRenderSystem<>> color_render_system = nullptr;
    std::unique_ptr<SimpleRenderSystem<>> raw_render_system = nullptr;
    std::unique_ptr<LineRenderSystem<>> line_render_system = nullptr;
    std::unique_ptr<SimpleWireFrameRenderSystem<>> wireframe_render_system = nullptr;
    std::unique_ptr<NormalRenderSystem<>> normal_render_system = nullptr;
    std::unique_ptr<Box3DRenderSystem> box_render_system = nullptr;
    std::unique_ptr<WorldAxisRenderSystem> axis_render_system = nullptr;
    std::unique_ptr<PointCloud3DRenderSystem> directional_filed_render_system = nullptr;
    std::unique_ptr<PointCloud3DRenderSystem> point_cloud_render_system = nullptr;

    std::unique_ptr<vkl::PathTracingComputeModel> path_tracing_compute_model = nullptr;
    std::unique_ptr<vkl::PathTracingComputeSystem> path_tracing_compute_system = nullptr;
    std::unique_ptr<VklTexture> path_tracing_texture = nullptr;

    std::unique_ptr<vkl::SoftwareRasterizer> software_rasterizer = nullptr;
    std::unique_ptr<VklTexture> software_rasterizer_texture = nullptr;
};
} // namespace GraphicsLab::RenderGraph
