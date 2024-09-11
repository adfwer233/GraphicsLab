#pragma once

#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "render_pass_base.hpp"
#include "render_pass_register.hpp"

#include "ui/ui_manager.hpp"

class ImguiPass : public RenderPassDeclarationBase {
    std::vector<VkDescriptorSet> render_texture_imgui;
    UIManager &uiManager_;

  public:
    ImguiPass(SceneTree::VklSceneTree &sceneTree, UIManager &uiManager)
        : RenderPassDeclarationBase(sceneTree), uiManager_(uiManager) {
    }

    virtual void descriptorStage(RenderGraphDescriptor &descriptor) final {
        auto output_texture = descriptor.attachment<RenderGraphTextureAttachment>("output_image");
        output_texture->isSwapChain = true;
        output_texture->format = VK_FORMAT_R8G8B8A8_SRGB;
        output_texture->width = 1024;
        output_texture->height = 1024;

        auto render_texture = descriptor.getAttachment<RenderGraphTextureAttachment>("render_result");

        auto imgui_render_pass = descriptor.pass<RenderGraphRenderPass>("imgui_render_pass");
        imgui_render_pass->outTextureAttachmentDescriptors.push_back(output_texture);
        imgui_render_pass->inTextureAttachmentDescriptors.push_back(render_texture);
        imgui_render_pass->width = 1024;
        imgui_render_pass->height = 1024;
        imgui_render_pass->is_submit_pass = true;
    };

    virtual void instanceStage(RenderGraph &renderGraph) final {

        auto render_texture_object = renderGraph.getAttachment<RenderGraphTextureAttachment>("render_result");
        uiManager_.renderResources.sceneRenderTexture = render_texture_object->getImguiTextures();

        // auto project_texture_object = renderGraph.getAttachment<RenderGraphTextureAttachment>("project_render_result");
        // uiManager_.renderResources.projectRenderTexture = project_texture_object->getImguiTextures();

        auto imgui_render_pass_obj = renderGraph.getPass<RenderGraphRenderPass>("imgui_render_pass");

        imgui_render_pass_obj->recordFunction = [&](VkCommandBuffer commandBuffer, uint32_t frame_index) {
            ImGui::DockSpaceOverViewport();
            uiManager_.render();
            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
        };
    };
};

META_REGISTER_TYPE(RenderGraphPassRegisterTag, ImguiPass)