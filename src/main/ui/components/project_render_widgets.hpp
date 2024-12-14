#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "ui/render_resources.hpp"

class ProjectRenderWidgetComponent : public UIComponent {
    UIState &uiState_;
    RenderResources &renderResources_;

  public:
    ProjectRenderWidgetComponent(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState,
                                 RenderResources &renderResources)
        : UIComponent(context), uiState_(uiState), renderResources_(renderResources) {
    }

    void render() final {

        ImGui::Begin("Project Render Result");
        {
            auto wsize = ImGui::GetContentRegionMax();
            if (context_.sceneTree->active_camera) {
                context_.sceneTree->active_camera->camera.ratio = wsize.x / wsize.y;
            }
            // todo: set frame index
            ImGui::Image(reinterpret_cast<ImTextureID>(renderResources_.projectRenderTexture[0]), wsize);
        }

        ImGui::End();
    }
};

// META_REGISTER_TYPE(MainComponentRegisterTag, ProjectRenderWidgetComponent)