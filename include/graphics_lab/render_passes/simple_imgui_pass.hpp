#pragma once

#include "vkl/imgui/imgui_context.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"

#include "graphics_lab/render_passes/swap_chain_pass.hpp"

namespace GraphicsLab::RenderGraph {
struct SimpleImGuiPass : public SwapChainPass {
    explicit SimpleImGuiPass(VklDevice &device) : SwapChainPass(device) {
    }

    RenderPassReflection render_pass_reflect() override {
        RenderPassReflection reflection;
        return reflection;
    }

    void post_compile(RenderContext *render_context) override {

        imguiContext = ImguiContext::getInstance(device_, render_context->get_glfw_window(),
                                                 render_context->get_swap_chain_render_pass());

        auto swapChain = get_current_swap_chain(render_context);
        for (int i = 0; i < swapChain->imageCount(); i++) {
            device_.transitionImageLayout(swapChain->getImage(i), swapChain->getSwapChainImageFormat(),
                                          VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_swap_chain_render_pass(commandBuffer, render_context);
        ImGui::DockSpaceOverViewport();
        ImGui::ShowDemoWindow();
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
        end_render_pass(commandBuffer);
    }

  private:
    ImguiContext *imguiContext = nullptr;
};
} // namespace GraphicsLab::RenderGraph
