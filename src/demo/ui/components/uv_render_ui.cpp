#include "uv_render_ui.hpp"

#include "../ui_manager.hpp"
#include "demo/utils/controller.hpp"

UVRenderUI::UVRenderUI(VklScene &scene, UIManager &uiManager) : scene_(scene), uiManager_(uiManager) {
}

void UVRenderUI::renderImgui() {

    ImGui::Begin("UV Render");
    {
        ImGui::BeginChild("UV Render Result");
        ImVec2 wsize = {1024, 1024};

        scene_.camera.ratio = wsize.x / wsize.y;

        if (resTex.empty()) {
            resTex.resize(uiManager_.uvImageViews->size());
            for (uint32_t i = 0; i < uiManager_.uvImageViews->size(); i++)
                resTex[i] = ImGui_ImplVulkan_AddTexture(uiManager_.uvSampler, (*uiManager_.uvImageViews)[i],
                                                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        if (uiManager_.renderMode != RenderMode::PathTracing) {
            ImGui::Image(reinterpret_cast<ImTextureID>(resTex[uiManager_.frameIndex]), wsize);
        }

        ImGui::EndChild();
    }
    ImGui::End();
}

UVRenderUI::~UVRenderUI() {
}
