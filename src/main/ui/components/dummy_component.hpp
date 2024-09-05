#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

class DummyComponent : public UIComponent {
public:
    DummyComponent(SceneTree::VklSceneTree &sceneTree, Controller& controller) : UIComponent(sceneTree) {
        spdlog::info("[Controller Address] {}", (void*)&controller);
    }

    void render() final {}
};

META_REGISTER_TYPE(MainComponentRegisterTag, DummyComponent)