#pragma once
#include <vkl/system/render_system/point_cloud_2d_render_system.hpp>
#include <vkl/system/render_system/param_line_render_system.hpp>
#include <vkl/system/render_system/pure_shader_render_system.hpp>

namespace GraphicsLab::RenderGraph {

struct Ubo2D {
    float zoom;
    float offset_x;
    float offset_y;
};

struct InternalScene2DRenderPass: public RenderPass {
    explicit InternalScene2DRenderPass(VklDevice &device, SceneTree::VklSceneTree &sceneTree, UIState &uiState,
                                     RenderResources &renderResources)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 0.0f}), sceneTree_(sceneTree), uiState_(uiState),
          renderResources_(renderResources) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("scene_render_result_2d", "Built in 2d render pass")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);

        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        tessellation2d_wireframe_render_system = std::make_unique<SimpleWireFrameRenderSystem<>>(device_, vkl_render_pass->renderPass, std::vector<VklShaderModuleInfo>{
                {std::format("{}/simple_shader_2d.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/simple_shader_2d.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}
        });

        point_cloud_2d_render_system = std::make_unique<PointCloud2DRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/point_cloud_2d_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_cloud_2d_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        line_render_system = std::make_unique<ParamLineRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/param_curve_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/param_curve_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        rectangle_line_render_system = std::make_unique<PureShaderRenderSystem>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/rectangle.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/rectangle.geom.spv", SHADER_DIR), VK_SHADER_STAGE_GEOMETRY_BIT},
                {std::format("{}/rectangle.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT},
            });
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();
        uint32_t frameIndex = render_context->get_current_frame_index();

        std::scoped_lock sceneTreeLock(sceneTree_.sceneTreeMutex);

        begin_render_pass(commandBuffer);

        GlobalUbo ubo{};

        Ubo2D ubo2D{};

        ubo2D.zoom = 0.5f;
        ubo2D.offset_x = 0.0f;
        ubo2D.offset_y = 0.0f;

        auto lineKey = line_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto meshKey = tessellation2d_wireframe_render_system->descriptorSetLayout->descriptorSetLayoutKey;

        /**
         * Render point cloud in 2d
         */
        auto pointcloudKey = point_cloud_2d_render_system->descriptorSetLayout->descriptorSetLayoutKey;
        auto point_cloud_buffer = SceneTree::VklNodeMeshBuffer<PointCloud2D>::instance();
        for (auto [point_cloud_node, trans]: sceneTree_.traverse_geometry_nodes_with_trans<PointCloud2D>()) {
            if (not point_cloud_node->visible)
                continue;

            ubo.model = trans;
            auto mesh = point_cloud_buffer->getGeometryModel(device_, point_cloud_node);

            if (mesh->mesh->uniformBuffers.contains(pointcloudKey)) {
                mesh->mesh->uniformBuffers[pointcloudKey][frameIndex]->writeToBuffer(&ubo);
                mesh->mesh->uniformBuffers[pointcloudKey][frameIndex]->flush();
            }

            FrameInfo<std::decay_t<decltype(*mesh)>::render_type> frameInfo{
                .frameIndex = static_cast<int>(frameIndex) % 2,
                .frameTime = 0,
                .commandBuffer = commandBuffer,
                .model = *mesh->mesh,
            };

            PointCloud2DRenderSystemPushConstantData pointCloud2DRenderSystemPushConstantData{};
            pointCloud2DRenderSystemPushConstantData.zoom = 0.5;
            PointCloud2DRenderSystemPushConstantList list;
            list.data[0] = pointCloud2DRenderSystemPushConstantData;
            point_cloud_2d_render_system->renderObject(frameInfo, list);
        }

        /**
         * Render Curves in 2D
         */
        MetaProgramming::ForEachType(Geometry::ParametricCurve2DTypeList{}, [&]<typename T>() {
            auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<T>::instance();
            for (auto [mesh3d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<T>()) {
                if (not mesh3d_nodes->visible)
                    continue;
                ubo.model = trans;
                auto node_mesh = mesh3d_buffer->getGeometryModel(device_, mesh3d_nodes);
                if (node_mesh->mesh->uniformBuffers.contains(lineKey)) {
                    node_mesh->mesh->uniformBuffers[lineKey][frameIndex]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[lineKey][frameIndex]->flush();
                }

                FrameInfo<typename std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                    .frameIndex = static_cast<int>(frameIndex) % 2,
                    .frameTime = 0,
                    .commandBuffer = commandBuffer,
                    .model = *node_mesh->mesh,
                };

                ParamLineRenderSystemPushConstantData paramLineRenderSystemPushConstantData{};
                paramLineRenderSystemPushConstantData.zoom = 0.5;
                ParamLineRenderSystemPushConstantList list;
                list.data[0] = paramLineRenderSystemPushConstantData;
                line_render_system->renderObject(frameInfo, list);
            }
        });

        /**
         * Render Mesh in 2D
         */
        auto mesh2d_buffer = SceneTree::VklNodeMeshBuffer<Mesh2D>::instance();
        for (auto [mesh2d_nodes, trans] : sceneTree_.traverse_geometry_nodes_with_trans<Mesh2D>()) {
            if (not mesh2d_nodes->visible)
                continue;
            auto node_mesh = mesh2d_buffer->getGeometryModel(device_, mesh2d_nodes);

            if (node_mesh->mesh->uniformBuffers.contains(meshKey)) {
                node_mesh->mesh->uniformBuffers[meshKey][frameIndex]->writeToBuffer(&ubo2D);
                node_mesh->mesh->uniformBuffers[meshKey][frameIndex]->flush();
            }

            static_assert(std::same_as<SceneTree::VklMesh<Vertex2D, TriangleIndex, VklBox2D>, std::decay_t<decltype(*node_mesh)>::render_type>);

            FrameInfo<typename std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                .frameIndex = static_cast<int>(frameIndex) % 2,
                .frameTime = 0,
                .commandBuffer = commandBuffer,
                .model = *node_mesh->mesh,
            };

            tessellation2d_wireframe_render_system->renderObject(frameInfo);
        }


        PureShaderRenderSystemPushConstantData push_constant_data{0.5, 0.0, 0.0};
        VklPushConstantInfoList<PureShaderRenderSystemPushConstantData> push_constant_data_list;
        push_constant_data_list.data[0] = push_constant_data;
        rectangle_line_render_system->renderPipeline(commandBuffer, push_constant_data_list);
        end_render_pass(commandBuffer);
    }

    SceneTree::VklSceneTree &sceneTree_;
    UIState &uiState_;

    std::unique_ptr<PointCloud2DRenderSystem<>> point_cloud_2d_render_system = nullptr;
    std::unique_ptr<ParamLineRenderSystem<>> line_render_system = nullptr;
    std::unique_ptr<PureShaderRenderSystem> rectangle_line_render_system = nullptr;
    std::unique_ptr<SimpleWireFrameRenderSystem<>> tessellation2d_wireframe_render_system = nullptr;

    RenderResources &renderResources_;
};

}
