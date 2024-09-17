#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

class RenderModelPanel : public UIComponent {
    UIState &uiState_;

  public:
    RenderModelPanel(SceneTree::VklSceneTree &sceneTree, UIState &uiState) : UIComponent(sceneTree), uiState_(uiState) {}

    void render() final {
        ImGui::Begin("Render Options");

        ImGui::SeparatorText("Rendering Mode");

        using RenderModelType = std::underlying_type_t<UIState::RenderMode>;

        ImGui::RadioButton("Raw", (RenderModelType *)&uiState_.renderMode, RenderModelType(UIState::RenderMode::raw));
        ImGui::SameLine();
        ImGui::RadioButton("Wire Frame", (RenderModelType *)&uiState_.renderMode, RenderModelType(UIState::RenderMode::wireframe));
        ImGui::SameLine();
        ImGui::RadioButton("With Texture", (RenderModelType *)&uiState_.renderMode, RenderModelType(UIState::RenderMode::material));

        // ImGui::SameLine();
        // ImGui::RadioButton("Path Tracing", (int *)&uiManager_.renderMode, RenderMode::PathTracing);

        ImGui::SeparatorText("Shading Mode");

        using ShadeModelType = std::underlying_type_t<UIState::LightingMode>;


        ImGui::RadioButton("Color Shading", (int *)&uiState_.lightingMode, ShadeModelType(UIState::LightingMode::simple));
        ImGui::SameLine();
        ImGui::RadioButton("Solid Shading", (int *)&uiState_.lightingMode,ShadeModelType(UIState::LightingMode::solid));

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, RenderModelPanel)