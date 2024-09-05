#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_vulkan.h"

#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "component_register.hpp"

#include "boost/di.hpp"
namespace di = boost::di;

class UIComponent {
  protected:
    SceneTree::VklSceneTree &sceneTree_;

    explicit UIComponent(SceneTree::VklSceneTree &sceneTree) : sceneTree_(sceneTree) {};

  public:
    virtual void render() = 0;
};