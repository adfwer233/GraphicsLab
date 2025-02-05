#pragma once

#include "components/component.hpp"

#include "render_resources.hpp"

#include "components/constructor_widget.hpp"
#include "components/dummy_component.hpp"
#include "components/logger_widget.hpp"
#include "components/project_render_widgets.hpp"
#include "components/project_widgets.hpp"
#include "components/python_scripts_explorer.hpp"
#include "components/python_terminal.hpp"
#include "components/render_mode_panel.hpp"
#include "components/scene_tree.hpp"
#include "components/scene_widgets.hpp"

#include "graphics_lab/graphics_lab_context.hpp"

using ComponentTypeList = META_GET_REGISTERED_TYPES(MainComponentRegisterTag);

class UIManager {
    std::vector<UIComponent *> component_ptrs;
    UIState &ui_state_;

  public:
    RenderResources renderResources;

    explicit UIManager(Controller &controller, UIState &uiState, GraphicsLab::GraphicsLabInternalContext &context)
        : ui_state_(uiState) {
        auto injector = di::make_injector(
            di::bind<Controller>().to(controller), di::bind<RenderResources>().to(renderResources),
            di::bind<UIState>().to(uiState), di::bind<GraphicsLab::GraphicsLabInternalContext>().to(context));
        create_component_instances(injector, ComponentTypeList{});
    }

    void render() {
        for (auto com : component_ptrs) {
            com->render();
        }

        // render custom components

        if (ui_state_.project != nullptr) {
            for (auto com : ui_state_.project->getImguiComponents()) {
                com->render();
            }
        }
    }

  private:
    template <typename InjectorType, typename... ts>
    void create_component_instances(InjectorType &injector, MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(injector.template create<ts *>()), ...);
    }
};