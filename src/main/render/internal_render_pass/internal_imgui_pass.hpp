#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "../../ui/ui_manager.hpp"
#include "graphics_lab/render_passes/swap_chain_pass.hpp"

#include "vkl/utils/imgui_utils.hpp"

namespace GraphicsLab::RenderGraph {
struct InternalImguiPass : public SwapChainPass {
    explicit InternalImguiPass(VklDevice &device, UIManager &uiManager) : SwapChainPass(device), uiManager_(uiManager) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {

        imguiContext = ImguiContext::getInstance(device_, render_context->get_glfw_window(),
                                                 render_context->get_swap_chain_render_pass());

        for (auto r : render_context->resource_manager.get_resource_with_annotation("imgui_show")) {
            if (auto colorTexture = dynamic_cast<ColorTextureResource *>(r)) {
                uiManager_.renderResources.imguiImages[colorTexture->get_name()] =
                    vkl::ImguiUtils::getImguiTextureFromVklTexture(colorTexture->get_resolved_texture());
            }
        }

        auto swapChain = get_current_swap_chain(render_context);
        for (size_t i = 0; i < swapChain->imageCount(); i++) {
            device_.transitionImageLayout(swapChain->getImage(i), swapChain->getSwapChainImageFormat(),
                                          VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_swap_chain_render_pass(commandBuffer, render_context);
        ImGui::DockSpaceOverViewport();
        uiManager_.render();
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
        end_render_pass(commandBuffer);
    }

  private:
    UIManager &uiManager_;
    ImguiContext *imguiContext = nullptr;
};
} // namespace GraphicsLab::RenderGraph
