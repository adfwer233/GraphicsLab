#pragma once

#include "component.hpp"
#include "controller/controller.hpp"
#include "vkl/imgui/imgui_reflection.hpp"

class MaterialManagerWidget : public UIComponent {
  public:
    MaterialManagerWidget(GraphicsLab::GraphicsLabInternalContext &context, Controller &controller) : UIComponent(context) {
    }

    void render() final {
        // auto manager_members = this->context_.sceneTree->material_manager.reflect();

        ImGui::Begin("Material Manager");
            imgui_reflection_render.render_functions(&this->context_.sceneTree->material_manager);

            imgui_reflection_render.render_static_reflected_properties(this->context_.sceneTree->material_manager);
        ImGui::End();
    }

private:
    vkl::ImGuiReflection imgui_reflection_render;
};

META_REGISTER_TYPE(MainComponentRegisterTag, MaterialManagerWidget)