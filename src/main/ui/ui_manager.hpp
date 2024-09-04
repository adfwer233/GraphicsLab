#pragma once

#include "components/component.hpp"

#include "components/scene_tree.hpp"

using ComponentTypeList = META_GET_REGISTERED_TYPES(MainComponentRegisterTag);

class UIManager {
    SceneTree::VklSceneTree &sceneTree_;

    std::vector<UIComponent *> component_ptrs;

  public:
    explicit UIManager(SceneTree::VklSceneTree &sceneTree) : sceneTree_(sceneTree) {
        create_component_instances(ComponentTypeList{});
    }

    void render() {
        for (auto com : component_ptrs) {
            com->render();
        }
    }

  private:
    template <typename... ts> void create_component_instances(MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(new ts(sceneTree_)), ...);
    }
};