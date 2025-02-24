#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

class RenderModelPanel : public UIComponent {
    UIState &uiState_;

  public:
    RenderModelPanel(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState)
        : UIComponent(context), uiState_(uiState) {
    }

    void render() final {
        ImGui::Begin("Render Options");

        ImGui::SeparatorText("Rendering Mode");

        using RenderModelType = std::underlying_type_t<UIState::RenderMode>;

        ImGui::RadioButton("Raw", reinterpret_cast<RenderModelType *>(&uiState_.renderMode),
                           static_cast<RenderModelType>(UIState::RenderMode::raw));
        ImGui::SameLine();
        ImGui::RadioButton("Wire Frame", reinterpret_cast<RenderModelType *>(&uiState_.renderMode),
                           static_cast<RenderModelType>(UIState::RenderMode::wireframe));
        ImGui::SameLine();
        ImGui::RadioButton("Color", reinterpret_cast<RenderModelType *>(&uiState_.renderMode),
                           static_cast<RenderModelType>(UIState::RenderMode::color));
        ImGui::SameLine();
        ImGui::RadioButton("Material", reinterpret_cast<RenderModelType *>(&uiState_.renderMode),
                           static_cast<RenderModelType>(UIState::RenderMode::material));

        // ImGui::SameLine();
        ImGui::RadioButton("Path Tracing", reinterpret_cast<RenderModelType *>(&uiState_.renderMode),
                           static_cast<RenderModelType>(UIState::RenderMode::path_tracing));

        ImGui::SeparatorText("Shading Mode");

        using ShadeModelType = std::underlying_type_t<UIState::LightingMode>;

        ImGui::RadioButton("Color Shading", reinterpret_cast<int *>(&uiState_.lightingMode),
                           static_cast<ShadeModelType>(UIState::LightingMode::simple));
        ImGui::SameLine();
        ImGui::RadioButton("Solid Shading", reinterpret_cast<int *>(&uiState_.lightingMode),
                           static_cast<ShadeModelType>(UIState::LightingMode::solid));

        ImGui::Checkbox("Show Normal", &uiState_.showNormal);
        ImGui::SameLine();
        ImGui::Checkbox("Show Axis", &uiState_.showAxis);

        if (ImGui::Button("Reset Path Tracing GPU BVH")) {
            uiState_.reset_gpu_bvh = true;
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, RenderModelPanel)