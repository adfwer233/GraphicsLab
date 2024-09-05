#pragma once

#include "vkl/scene_tree/vkl_scene_tree.hpp"

class RenderPassDeclarationBase {
protected:
    SceneTree::VklSceneTree &sceneTree_;

public:
    explicit RenderPassDeclarationBase(SceneTree::VklSceneTree &sceneTree): sceneTree_(sceneTree) {}

    virtual void descriptorStage(RenderGraphDescriptor& descriptor) = 0;

    virtual void instanceStage(RenderGraph& renderGraph) = 0;
};