#pragma once

#include "vkl/system/render_system/simple_render_system.hpp"

#include "../../ui/ui_manager.hpp"
#include "graphics_lab/render_passes/swap_chain_pass.hpp"

#include "utils/imgui_utils.hpp"

namespace GraphicsLab::RenderGraph {
    struct InternalImguiPass: public SwapChainPass {
        explicit InternalImguiPass(VklDevice &device, UIManager &uiManager) : SwapChainPass(device), uiManager_(uiManager) {

        }

        RenderPassReflection render_pass_reflect() override {
            RenderPassReflection reflection;
            reflection.add_output("simple_output", "The output texture of simple pass, which will be a triangle")
                    .type(RenderPassReflection::Field::Type::Texture2D)
                    .sample_count(8)
                    .format(VK_FORMAT_R8G8B8A8_SRGB)
                    .extent(1024, 1024);
            return reflection;
        }

        void post_compile(RenderContext *render_context) override {
            auto resource = render_context->resource_manager.get_resource("scene_render_result");
            auto color_resource = reinterpret_cast<ColorTextureResource*>(resource);

            imguiContext = std::make_unique<ImguiContext>(device_, render_context->get_glfw_window(), render_context->get_swap_chain_render_pass());
            ImGuiIO &io = ImGui::GetIO();
            io.Fonts->AddFontFromFileTTF("font/segoeui.ttf", 30);

            uiManager_.renderResources.sceneRenderTexture = ImguiUtils::getImguiTextureFromVklTexture(color_resource->get_resolved_texture());

            auto swapChain = get_current_swap_chain(render_context);
            for (int i = 0; i < swapChain->imageCount(); i++) {
                device_.transitionImageLayout(swapChain->getImage(i),
                                              swapChain->getSwapChainImageFormat() , VK_IMAGE_LAYOUT_UNDEFINED,
                                              VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
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
        std::unique_ptr<ImguiContext> imguiContext = nullptr;

    };
}
