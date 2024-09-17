#pragma once

#include "vkl/scene_tree/vkl_scene_tree.hpp"
#include "vkl/system/render_system/line_render_system.hpp"
#include "vkl/system/render_system/simple_wireframe_render_system.hpp"

#include "geometry/constructor/box3d.hpp"

#include "render_pass_base.hpp"
#include "render_pass_register.hpp"

class ScenePass : public RenderPassDeclarationBase {
  private:
    SimpleRenderSystem<> *simple_render_system;
    SimpleRenderSystem<> *color_render_system;
    SimpleRenderSystem<> *raw_render_system;
    LineRenderSystem<> *line_render_system;
    SimpleWireFrameRenderSystem<> *wireframe_render_system;

    SceneTree::GeometryNode<Wire3D> boxNode;
    UIState &uiState_;

    bool flag = false;

  public:
    ScenePass(SceneTree::VklSceneTree &sceneTree, UIState &uiState)
        : RenderPassDeclarationBase(sceneTree), uiState_(uiState) {
        boxNode.data = Box3DConstructor::create({0, 0, 0}, {5, 5, 5});
    }

    virtual void descriptorStage(RenderGraphDescriptor &descriptor) final {
        auto render_texture = descriptor.attachment<RenderGraphTextureAttachment>("render_result");
        render_texture->clear = true;
        render_texture->isSwapChain = false;
        render_texture->input_flag = true;
        render_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
        render_texture->width = 1024;
        render_texture->height = 1024;

        auto render_depth_texture = descriptor.attachment<RenderGraphTextureAttachment>("render_depth");
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

        auto simple_render_pass = descriptor.pass<RenderGraphRenderPass>("simple_render_pass");
        simple_render_pass->outTextureAttachmentDescriptors.push_back(render_texture);
        simple_render_pass->outTextureAttachmentDescriptors.push_back(render_depth_texture);
        simple_render_pass->width = 1024;
        simple_render_pass->height = 1024;
    };

    virtual void instanceStage(RenderGraph &renderGraph) final {
        auto simple_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("simple_render_pass");
        simple_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
            renderGraph.device_, "simple_render_system",
            {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/simple_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        raw_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
            renderGraph.device_, "raw_render_system",
            {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        color_render_system = simple_render_pass_obj->getRenderSystem<SimpleRenderSystem<>>(
            renderGraph.device_, "color_render_system",
            {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/simple_color_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        line_render_system = simple_render_pass_obj->getRenderSystem<LineRenderSystem<>>(
            renderGraph.device_, "line_render_system",
            {{std::format("{}/line_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/line_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        wireframe_render_system = simple_render_pass_obj->getRenderSystem<SimpleWireFrameRenderSystem<>>(
            renderGraph.device_, "wireframe_render_system",
            {{std::format("{}/simple_shader.vert.spv", SHADER_DIR), VK_SHADER_STAGE_VERTEX_BIT},
             {std::format("{}/point_light_shader.frag.spv", SHADER_DIR), VK_SHADER_STAGE_FRAGMENT_BIT}});

        simple_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
            GlobalUbo ubo{};

            ubo.view = sceneTree_.active_camera->camera.get_view_transformation();
            ubo.proj = sceneTree_.active_camera->camera.get_proj_transformation();
            ubo.model = glm::mat4(1.0f);
            ubo.pointLight =
                PointLight(glm::vec4(sceneTree_.active_camera->camera.position, 1.0), {1.0, 1.0, 1.0, 0.0});
            ubo.cameraPos = sceneTree_.active_camera->camera.position;

            auto textureKey = simple_render_system->descriptorSetLayout->descriptorSetLayoutKey;
            auto colorKey = color_render_system->descriptorSetLayout->descriptorSetLayoutKey;
            auto rawKey = raw_render_system->descriptorSetLayout->descriptorSetLayoutKey;

            auto mesh3d_buffer = SceneTree::VklNodeMeshBuffer<Mesh3D>::instance();
            for (auto mesh3d_nodes : sceneTree_.traverse_geometry_nodes<Mesh3D>()) {
                auto node_mesh = mesh3d_buffer->getGeometryModel(renderGraph.device_, mesh3d_nodes);

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

                auto wire_mesh = wire3d_buffer->getGeometryModel(renderGraph.device_, &boxNode);

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
        };
    };
};

META_REGISTER_TYPE(RenderGraphPassRegisterTag, ScenePass)