#pragma once

#include "component.hpp"
#include "controller/controller.hpp"

class DummyComponent : public UIComponent {
  public:
    DummyComponent(GraphicsLab::GraphicsLabInternalContext &context, Controller &controller) : UIComponent(context) {
    }

    void render() final {
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, DummyComponent)