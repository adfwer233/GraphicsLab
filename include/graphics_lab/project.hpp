#pragma once

#include "language/reflection/reflectors.hpp"

#include "graphics_lab_context.hpp"

/**
 * Interface for Graphics Lab Application
 */
class IGraphicsLabProject : public Reflectable {
  public:
    virtual ~IGraphicsLabProject() = default;

    void updateContext(GraphicsLabContext &&t_context) {
        context = t_context;
    }

    virtual void tick() = 0;

    virtual ReflectDataType reflect() {
        return {{"tick", TypeErasedValue(&IGraphicsLabProject::tick, this)}};
    }

  protected:
    GraphicsLabContext context;
};