#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

class ImportModelWidget : public UIComponent {
  public:
    ImportModelWidget(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState)
        : UIComponent(context), uiState_(uiState) {
    }

    void render() final {
        ImGui::Begin("Import Model");
        render_recent_model_reflection.render_functions(&uiState_.recent_models);
        render_recent_model_reflection.render_static_reflected_properties(uiState_.recent_models);

        static int current_item = 0;

        if (not uiState_.recent_models.models.empty()) {
            if (ImGui::BeginCombo("Recent", uiState_.recent_models.models[current_item].name.c_str())) {
                for (int i = 0; i < uiState_.recent_models.models.size(); ++i) {
                    const bool is_selected = (current_item == i);
                    if (ImGui::Selectable(uiState_.recent_models.models[i].name.c_str(), is_selected)) {
                        current_item = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();

            if (ImGui::Button("Import")) {
                std::string file = uiState_.recent_models.models[current_item].path;
                spdlog::info("Choose Obj File: {}", file);
                auto result = std::async(std::launch::async, [&]() {
                    std::scoped_lock scoped_lock(context_.sceneTree->sceneTreeMutex);
                    context_.sceneTree->importFromPath(file);
                    context_.sceneTree->addCameraNode("Camera 1", Camera({0, 0, 10}, {0, 1, 0}));
                    spdlog::info("Load Obj File: {}", file);
                });
            }
        }
        ImGui::End();
    }

    vkl::ImGuiReflection render_recent_model_reflection;
    UIState &uiState_;
};

META_REGISTER_TYPE(MainComponentRegisterTag, ImportModelWidget)