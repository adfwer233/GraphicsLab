#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "ui/render_resources.hpp"

class SceneWidgetComponent : public UIComponent {
    UIState &uiState_;
    RenderResources &renderResources_;

  public:
    SceneWidgetComponent(SceneTree::VklSceneTree &sceneTree, UIState &uiState, RenderResources &renderResources)
        : UIComponent(sceneTree), uiState_(uiState), renderResources_(renderResources) {
    }

    void render() final {

        ImGui::Begin("Render Result");
        {
            auto wsize = ImGui::GetContentRegionAvail();
            if (sceneTree_.active_camera) {
                sceneTree_.active_camera->camera.ratio = wsize.x / wsize.y;
            }
            // todo: set frame index
            ImGui::Image(renderResources_.sceneRenderTexture[0], wsize);
        }

        if (ImGui::IsItemVisible()) {

            auto min_pos = ImGui::GetItemRectMin();
            auto max_pos = ImGui::GetItemRectMax();

            uiState_.scope_min = {min_pos.x, min_pos.y};
            uiState_.scope_max = {max_pos.x, max_pos.y};
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, SceneWidgetComponent)