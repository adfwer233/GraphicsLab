#pragma once

#include "components/component.hpp"

#include "components/scene_tree.hpp"
#include "components/dummy_component.hpp"

using ComponentTypeList = META_GET_REGISTERED_TYPES(MainComponentRegisterTag);

class UIManager {
    std::vector<UIComponent *> component_ptrs;
  public:
    explicit UIManager(SceneTree::VklSceneTree &sceneTree, Controller& controller) {
        auto injector = di::make_injector(
                di::bind<SceneTree::VklSceneTree>().to(sceneTree),
                di::bind<Controller>().to(controller)
            );
        create_component_instances(injector, ComponentTypeList{});
    }

    void render() {
        for (auto com : component_ptrs) {
            com->render();
        }
    }

  private:
    template <typename InjectorType, typename... ts> void create_component_instances(InjectorType &injector, MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(injector.template create<ts*>()), ...);
    }
};