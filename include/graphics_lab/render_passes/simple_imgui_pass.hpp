#pragma once

#include "vkl/imgui/imgui_context.hpp"
#include "vkl/system/render_system/simple_render_system.hpp"

#include "graphics_lab/render_passes/swap_chain_pass.hpp"

#include <vkl/utils/imgui_utils.hpp>

namespace GraphicsLab::RenderGraph {
struct SimpleImGuiPass : public SwapChainPass {
    std::function<void()> imgui_render_callback = nullptr;

    explicit SimpleImGuiPass(VklDevice &device, std::optional<std::function<void()>> imgui_callback = std::nullopt)
        : SwapChainPass(device) {
        if (imgui_callback.has_value()) {
            imgui_render_callback = imgui_callback.value();
        }
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

        for (auto r : render_context->resource_manager.get_resource_with_annotation("imgui_show")) {
            if (auto colorTexture = dynamic_cast<ColorTextureResource *>(r)) {
                render_context->imgui_resources.imguiImages[colorTexture->get_name()] =
                    vkl::ImguiUtils::getImguiTextureFromVklTexture(colorTexture->get_resolved_texture());
            }
        }
    }

    void execute(RenderContext *render_context, const RenderPassExecuteData &execute_data) override {
        auto commandBuffer = render_context->get_current_command_buffer();

        begin_swap_chain_render_pass(commandBuffer, render_context);
        ImGui::DockSpaceOverViewport();
        ImGui::Begin("Render Result");
        {
            if (ImGui::BeginTabBar("RenderTabs")) { // Start a tab bar
                for (auto &[name, img] : render_context->imgui_resources.imguiImages) {
                    if (ImGui::BeginTabItem(name.c_str())) { // Create a tab for each image
                        auto wsize = ImGui::GetContentRegionAvail();
                        // todo: set frame index
                        float min_size = std::min(wsize.x, wsize.y);
                        ImGui::Image(reinterpret_cast<ImTextureID>(img.front()), ImVec2(min_size, min_size));

                        ImGui::EndTabItem(); // End the tab for the current image
                    }
                    if (ImGui::IsItemVisible()) {
                    }
                }
            }
            ImGui::EndTabBar(); // End the tab bar
        }

        ImGui::End();

        if (imgui_render_callback) {
            imgui_render_callback();
        }
        ImGui::Render();
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
        end_render_pass(commandBuffer);
    }

  private:
    ImguiContext *imguiContext = nullptr;
};
} // namespace GraphicsLab::RenderGraph
