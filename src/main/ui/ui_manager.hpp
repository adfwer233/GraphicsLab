#pragma once

#include "components/component.hpp"

#include "render_resources.hpp"

#include "components/dummy_component.hpp"
#include "components/project_render_widgets.hpp"
#include "components/project_widgets.hpp"
#include "components/render_mode_panel.hpp"
#include "components/scene_tree.hpp"
#include "components/scene_widgets.hpp"

using ComponentTypeList = META_GET_REGISTERED_TYPES(MainComponentRegisterTag);

class UIManager {
    std::vector<UIComponent *> component_ptrs;

  public:
    RenderResources renderResources;

    explicit UIManager(SceneTree::VklSceneTree &sceneTree, Controller &controller, UIState &uiState) {
        auto injector =
            di::make_injector(di::bind<SceneTree::VklSceneTree>().to(sceneTree), di::bind<Controller>().to(controller),
                              di::bind<RenderResources>().to(renderResources), di::bind<UIState>().to(uiState));
        create_component_instances(injector, ComponentTypeList{});
    }

    void render() {
        for (auto com : component_ptrs) {
            com->render();
        }
    }

  private:
    template <typename InjectorType, typename... ts>
    void create_component_instances(InjectorType &injector, MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(injector.template create<ts *>()), ...);
    }
};