#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "component_register.hpp"
#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/graphics_lab_imgui_component.hpp"

#include "boost/di.hpp"
namespace di = boost::di;

class UIComponent : public GraphicsLab::IGraphicsLabImguiComponent {
  protected:
    GraphicsLab::GraphicsLabInternalContext &context_;

    explicit UIComponent(GraphicsLab::GraphicsLabInternalContext &context) : context_(context) {};

  public:
    virtual ~UIComponent() = default;
};

template <typename T>
concept DerivableFromUIComponent = std::is_base_of_v<UIComponent, T>;