#include "scene_render_ui.hpp"

#include "../ui_manager.hpp"
#include "demo/utils/controller.hpp"

SceneRenderUI::SceneRenderUI(VklScene &scene, UIManager &uiManager) : scene_(scene), uiManager_(uiManager) {
}

void SceneRenderUI::renderImgui() {
    auto controller = KeyboardCameraController::instance();

    ImGui::Begin("Render Result");
    {
        ImGui::BeginChild("RenderResult");
        ImVec2 wsize = ImGui::GetContentRegionMax();

        if (uiManager_.renderMode == RenderMode::PathTracing) {
            wsize = {1024, 1024};
        }
        scene_.camera.ratio = wsize.x / wsize.y;

        // ImGuiIO& io = ImGui::GetIO();
        // io.WantCaptureMouse = true;

        if (resTex.empty()) {
            resTex.resize(uiManager_.offscreenImageViews->size());
            for (uint32_t i = 0; i < uiManager_.offscreenImageViews->size(); i++)
                resTex[i] =
                    ImGui_ImplVulkan_AddTexture(uiManager_.offscreenSampler, (*uiManager_.offscreenImageViews)[i],
                                                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        if (uiManager_.renderMode == RenderMode::PathTracing) {
            if (uiManager_.pathTracingResTex == VK_NULL_HANDLE) {
                uiManager_.pathTracingResTex = ImGui_ImplVulkan_AddTexture(
                    uiManager_.renderResultTexture->getTextureSampler(),
                    uiManager_.renderResultTexture->getTextureImageView(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            }
            ImGui::Image(reinterpret_cast<ImTextureID>(uiManager_.pathTracingResTex), wsize, ImVec2(0, 1), ImVec2(1, 0));
        } else {
            ImGui::Image(reinterpret_cast<ImTextureID>(resTex[uiManager_.frameIndex]), wsize);
        }

        if (ImGui::IsItemVisible()) {

            auto min_pos = ImGui::GetItemRectMin();
            auto max_pos = ImGui::GetItemRectMax();

            controller->scope_min = {min_pos.x, min_pos.y};
            controller->scope_max = {max_pos.x, max_pos.y};

            controller->currentWidgets = DemoWidgets::SceneRendering;
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

SceneRenderUI::~SceneRenderUI() = default;
