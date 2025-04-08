#pragma once

#include <graphics_lab/render_graph/render_pass.hpp>
#include <vkl/scene_tree/vkl_geometry_mesh.hpp>
#include <vkl/scene_tree/vkl_mesh_types.hpp>

#include <vkl/system/render_system/point_cloud_2d_render_system.hpp>

using SimplePointCloudRenderSystem = SimpleRenderSystem<0, SimplePushConstantInfoList, PointCloud2DPipelineModifier>;

struct ParametricSpaceScene {
    struct SceneTreeGeometryTypeTrait {};

    std::vector<glm::dvec2> points;
};

namespace SceneTree {

template<> struct VklNodeMesh<ParametricSpaceScene> {
    using PointCloudRenderType = SceneTree::VklPointCloudMesh2D;
    std::unique_ptr<PointCloudRenderType> point_cloud_mesh = nullptr;

    void create_mesh() {
        PointCloudRenderType::Builder point_cloud_mesh_builder;
        spdlog::info("test, {}", node_->data.points.size());
        for (auto &point : node_->data.points) {
            point_cloud_mesh_builder.vertices.push_back({point, glm::vec3(1.0f, 0.0f, 0.0f)});
        }
        spdlog::info("create mesh {}", point_cloud_mesh_builder.vertices.size());
        point_cloud_mesh = std::make_unique<PointCloudRenderType>(device_, point_cloud_mesh_builder);
    }

    VklNodeMesh(VklDevice &device, SceneTree::GeometryNode<ParametricSpaceScene> *node)
    : device_(device), node_(node) {
        create_mesh();
    }

    VklDevice& device_;
    SceneTree::GeometryNode<ParametricSpaceScene> *node_ = nullptr;
};

}

namespace GraphicsLab::RenderGraph {

struct ParametricSpaceRenderPass: public RenderPass {

    explicit ParametricSpaceRenderPass(VklDevice &device, SceneTree::GeometryNode<ParametricSpaceScene> *t_scene)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 1.0f}), scene(t_scene) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("parametric_space_output", "parametric space render result")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);

        reflection.add_input("parametric_space_output_depth", "the input grid texture")
            .type(RenderPassReflection::Field::Type::TextureDepth)
            .sample_count(8)
            .visibility(RenderPassReflection::Field::Visibility::Output)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048);

        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        point_cloud_render_system = std::make_unique<SimplePointCloudRenderSystem>(device_, vkl_render_pass->renderPass, std::vector<VklShaderModuleInfo>{
                {std::format("{}/point_cloud_2d_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/point_cloud_2d_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}
        });
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_render_pass(commandBuffer);

        uint32_t frame_index = render_context->get_current_frame_index();
        auto bezier_mesh_buffer = SceneTree::VklNodeMeshBuffer<ParametricSpaceScene>::instance();

        auto scene_model = bezier_mesh_buffer->getGeometryModel(device_, scene);

        FrameInfo<std::decay_t<decltype(*scene_model)>::PointCloudRenderType> frameInfo{
            .frameIndex = static_cast<int>(frame_index) % 2,
            .frameTime = 0,
            .commandBuffer = commandBuffer,
            // .camera =
            .model = *scene_model->point_cloud_mesh,
        };

        point_cloud_render_system->renderObject(frameInfo);

        // auto simpleKey = render_system->descriptorSetLayout->descriptorSetLayoutKey;

        end_render_pass(commandBuffer);
    }

    SceneTree::GeometryNode<ParametricSpaceScene> *scene;

    std::unique_ptr<SimplePointCloudRenderSystem> point_cloud_render_system = nullptr;

};

}