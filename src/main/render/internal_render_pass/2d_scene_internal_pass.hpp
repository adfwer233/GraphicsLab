#pragma once
#include <vkl/system/render_system/point_cloud_2d_render_system.hpp>

namespace GraphicsLab::RenderGraph {

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
        point_cloud_2d_render_system = std::make_unique<PointCloud2DRenderSystem<>>(
            device_, vkl_render_pass->renderPass,
            std::vector<VklShaderModuleInfo>{
                {std::format("{}/point_cloud_2d_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_cloud_2d_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();
        uint32_t frameIndex = render_context->get_current_frame_index();

        std::scoped_lock sceneTreeLock(sceneTree_.sceneTreeMutex);

        begin_render_pass(commandBuffer);

        GlobalUbo ubo{};

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


        end_render_pass(commandBuffer);
    }

    SceneTree::VklSceneTree &sceneTree_;
    UIState &uiState_;

    std::unique_ptr<PointCloud2DRenderSystem<>> point_cloud_2d_render_system = nullptr;

    RenderResources &renderResources_;
};

}