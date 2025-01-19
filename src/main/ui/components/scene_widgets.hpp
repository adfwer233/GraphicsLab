#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "ui/render_resources.hpp"

class SceneWidgetComponent : public UIComponent {
    UIState &uiState_;
    RenderResources &renderResources_;

  public:
    SceneWidgetComponent(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState,
                         RenderResources &renderResources)
        : UIComponent(context), uiState_(uiState), renderResources_(renderResources) {
    }

    void render() final {

        ImGui::Begin("Render Result");
        {
            if (ImGui::BeginTabBar("RenderTabs")) { // Start a tab bar
                for (auto &[name, img] : renderResources_.imguiImages) {
                    if (ImGui::BeginTabItem(name.c_str())) { // Create a tab for each image
                        auto wsize = ImGui::GetContentRegionAvail();
                        if (context_.sceneTree->active_camera) {
                            context_.sceneTree->active_camera->camera.ratio = 1.0;
                        }
                        // todo: set frame index
                        float min_size = std::min(wsize.x, wsize.y);
                        ImGui::Image(reinterpret_cast<ImTextureID>(img.front()), ImVec2(min_size, min_size));

                        auto min_pos = ImGui::GetItemRectMin();
                        auto max_pos = ImGui::GetItemRectMax();
                        // spdlog::info("{} {} {}", name, min_pos.x, max_pos.x);

                        uiState_.scope_min[name] = {min_pos.x, min_pos.y};
                        uiState_.scope_max[name] = {max_pos.x, max_pos.y};

                        ImGui::EndTabItem(); // End the tab for the current image
                    }
                    if (ImGui::IsItemVisible()) {
                    }
                }
            }
            ImGui::EndTabBar(); // End the tab bar
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, SceneWidgetComponent)