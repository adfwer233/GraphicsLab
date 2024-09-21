#pragma once

#include "vkl/render_graph/render_graph.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "utils/imgui_sink.hpp"

struct GraphicsLabContext {
    VklDevice *device;
    SceneTree::VklSceneTree *sceneTree;
    LogManager* logManager;
};
