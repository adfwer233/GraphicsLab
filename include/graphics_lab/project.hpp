#pragma once

#include "language/reflection/reflectors.hpp"

#include "graphics_lab_context.hpp"
#include "graphics_lab_controller.hpp"

#include "pybind11/pybind11.h"

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
    virtual std::string name() = 0;
    virtual void afterLoad() = 0;
    virtual void bindPython(pybind11::module &m) {
    }

    virtual GraphicsLab::IGraphicsLabProjectController *getController() {
        return new GraphicsLab::EmptyGraphicsLabController();
    }

    virtual ReflectDataType reflect() {
        return {{"tick", TypeErasedValue(&IGraphicsLabProject::tick, this)}};
    }

  protected:
    GraphicsLabContext context;
};