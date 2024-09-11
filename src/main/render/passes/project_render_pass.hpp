#pragma once

#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "render_pass_base.hpp"
#include "render_pass_register.hpp"

class ProjectRenderPass : public RenderPassDeclarationBase {
private:
    SimpleRenderSystem<> *simple_render_system;

public:
    ProjectRenderPass(SceneTree::VklSceneTree &sceneTree) : RenderPassDeclarationBase(sceneTree) {
    }

    virtual void descriptorStage(RenderGraphDescriptor &descriptor) final {
        auto subgraph_pass_desc = descriptor.pass<RenderGraphSubgraphPass>("project_subgraph");
        subgraph_pass_desc->renderGraphDescriptor = new RenderGraphDescriptor;

        auto render_texture = subgraph_pass_desc->renderGraphDescriptor->attachment<RenderGraphTextureAttachment>("project_render_result");
        render_texture->clear = true;
        render_texture->isSwapChain = false;
        render_texture->input_flag = true;
        render_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
        render_texture->width = 1024;
        render_texture->height = 1024;

        auto render_depth_texture = subgraph_pass_desc->renderGraphDescriptor->attachment<RenderGraphTextureAttachment>("project_render_depth");
        render_depth_texture->clear = true;
        render_depth_texture->type =
                RenderGraphAttachmentDescriptor<RenderGraphTextureAttachment>::AttachmentType::DepthAttachment;
        render_depth_texture->isSwapChain = false;
        render_depth_texture->input_flag = false;
        render_depth_texture->format = sceneTree_.device_.findSupportedFormat(
                {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT}, VK_IMAGE_TILING_OPTIMAL,
                VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
        render_depth_texture->width = 1024;
        render_depth_texture->height = 1024;

        auto project_render_pass = subgraph_pass_desc->renderGraphDescriptor->pass<RenderGraphRenderPass>("project_subgraph_pass");
        project_render_pass->outTextureAttachmentDescriptors.push_back(render_texture);
        project_render_pass->outTextureAttachmentDescriptors.push_back(render_depth_texture);
        project_render_pass->width = 1024;
        project_render_pass->height = 1024;

    };

    virtual void instanceStage(RenderGraph &renderGraph) final {
        auto simple_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("project_subgraph_pass");

        simple_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
                renderGraph.device_, "project_simple_render_system",
                {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
                 {std::format("{}/simple_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        simple_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
            GlobalUbo ubo{};

            ubo.view = sceneTree_.active_camera->camera.get_view_transformation();
            ubo.proj = sceneTree_.active_camera->camera.get_proj_transformation();
            ubo.model = glm::mat4(1.0f);
            ubo.pointLight = PointLight({0, 10, 0, 1.0}, {1.0, 1.0, 1.0, 0.0});
            ubo.cameraPos = sceneTree_.active_camera->camera.position;

            auto key = simple_render_system->descriptorSetLayout->descriptorSetLayoutKey;

            auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<Mesh3D>::instance();
            for (auto mesh3d_nodes : sceneTree_.traverse_geometry_nodes<Mesh3D>()) {
                auto node_mesh = mesh3d_buffer->getGeometryModel(renderGraph.device_, mesh3d_nodes);

                if (node_mesh->mesh->uniformBuffers.contains(key)) {
                    node_mesh->mesh->uniformBuffers[key][frame_index]->writeToBuffer(&ubo);
                    node_mesh->mesh->uniformBuffers[key][frame_index]->flush();
                }

                FrameInfo<std::decay_t<decltype(*node_mesh)>::render_type> frameInfo{
                        .frameIndex = static_cast<int>(frame_index) % 2,
                        .frameTime = 0,
                        .commandBuffer = commandBuffer,
                        .camera = sceneTree_.active_camera->camera,
                        .model = *node_mesh->mesh.get(),
                };

                simple_render_system->renderObject(frameInfo);
            }
        };
    };
};

// META_REGISTER_TYPE(RenderGraphPassRegisterTag, ProjectRenderPass)