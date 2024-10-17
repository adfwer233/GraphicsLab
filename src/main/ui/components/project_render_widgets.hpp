#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "ui/render_resources.hpp"

class ProjectRenderWidgetComponent : public UIComponent {
    UIState &uiState_;
    RenderResources &renderResources_;

  public:
    ProjectRenderWidgetComponent(SceneTree::VklSceneTree &sceneTree, UIState &uiState, RenderResources &renderResources)
        : UIComponent(sceneTree), uiState_(uiState), renderResources_(renderResources) {
    }

    void render() final {

        ImGui::Begin("Project Render Result");
        {
            auto wsize = ImGui::GetContentRegionMax();
            if (sceneTree_.active_camera) {
                sceneTree_.active_camera->camera.ratio = wsize.x / wsize.y;
            }
            // todo: set frame index
            ImGui::Image(reinterpret_cast<ImTextureID>(renderResources_.projectRenderTexture[0]), wsize);
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

// META_REGISTER_TYPE(MainComponentRegisterTag, ProjectRenderWidgetComponent)