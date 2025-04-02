#pragma once

#include "geometry/parametric/bezier_curve_2d.hpp"
#include "scene.hpp"

#include <vkl/scene_tree/vkl_geometry_mesh.hpp>
#include <vkl/system/render_system/line_render_system.hpp>

namespace GraphicsLab::BezierGenerator {

struct Scene2D {
    struct SceneTreeGeometryTypeTrait{};

    std::vector<Geometry::BezierCurve2D> curves;
};

}

namespace SceneTree {

template<> struct VklNodeMesh<GraphicsLab::BezierGenerator::Scene2D> {

    using CurveRenderType = VklCurveMesh2D;
    std::unique_ptr<CurveRenderType> curve_mesh = nullptr;
    void createMesh() {
        int num = 100;

        CurveRenderType::Builder curve_mesh_builder;
        for (auto& curve : node_->data.curves) {
            for (int i = 0; i < num; i++) {
                float t = i / (num - 1.0f);
                curve_mesh_builder.vertices.push_back({curve.evaluate(t), glm::vec3(1.0f, 0.0f, 0.0f)});
                if (i > 0) {
                    curve_mesh_builder.indices.push_back({
                        static_cast<uint32_t>(curve_mesh_builder.vertices.size() - 2),
                        static_cast<uint32_t>(curve_mesh_builder.vertices.size() - 1)
                    });
                }
            }
        }

        curve_mesh = std::make_unique<CurveRenderType>(device_, curve_mesh_builder);
    }

    VklNodeMesh(VklDevice& device, SceneTree::GeometryNode<GraphicsLab::BezierGenerator::Scene2D>* node): device_(device), node_(node) {
        createMesh();
    }

private:
    VklDevice& device_;
    SceneTree::GeometryNode<GraphicsLab::BezierGenerator::Scene2D>* node_;
};

}

namespace GraphicsLab::RenderGraph {
    struct BezierRenderPass: public RenderPass {
    explicit BezierRenderPass(VklDevice &device, SceneTree::GeometryNode<BezierGenerator::Scene2D> *t_scene)
        : RenderPass(device, {0.0f, 0.0f, 0.0f, 1.0f}), scene(t_scene) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        reflection.add_output("bezier_render", "output of bezier editor")
            .type(RenderPassReflection::Field::Type::Texture2D)
            .sample_count(8)
            .format(VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048)
            .set_annotation("imgui_show", true);
            // .set_annotation("do_not_release", true);

        reflection.add_input("bezier_render_depth", "the input grid texture")
            .type(RenderPassReflection::Field::Type::TextureDepth)
            .sample_count(8)
            .visibility(RenderPassReflection::Field::Visibility::Output)
            .format(VkFormat::VK_FORMAT_R8G8B8A8_SRGB)
            .extent(2048, 2048);
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {
        spdlog::info("{}/curve2d.vert", SHADER_DIR);
        render_system = std::make_unique<LineRenderSystem<>>(device_, vkl_render_pass->renderPass, std::vector<VklShaderModuleInfo>{
                {std::format("{}/curve2d.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                {std::format("{}/curve2d.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}
        });

        spdlog::info("{}/curve2d.vert compile success", SHADER_DIR);

    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_render_pass(commandBuffer);

        uint32_t frame_index = render_context->get_current_frame_index();
        auto bezier_mesh_buffer = SceneTree::VklNodeMeshBuffer<GraphicsLab::BezierGenerator::Scene2D>::instance();

        auto scene_model = bezier_mesh_buffer->getGeometryModel(device_, scene);

        FrameInfo<std::decay_t<decltype(*scene_model)>::CurveRenderType> frameInfo {
            .frameIndex = static_cast<int>(frame_index) % 2,
            .frameTime = 0,
            .commandBuffer = commandBuffer,
            // .camera =
            .model = *scene_model->curve_mesh,
        };

        render_system->renderObject(frameInfo);

        auto simpleKey = render_system->descriptorSetLayout->descriptorSetLayoutKey;

        end_render_pass(commandBuffer);
    }


private:
    std::unique_ptr<LineRenderSystem<>> render_system = nullptr;

    SceneTree::GeometryNode<GraphicsLab::BezierGenerator::Scene2D> *scene;
};
}