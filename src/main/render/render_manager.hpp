#pragma once

#include "passes/render_pass_base.hpp"
#include "passes/render_pass_register.hpp"

#include "passes/3d_scene_pass.hpp"
#include "passes/imgui_pass.hpp"

#include "controller/controller.hpp"

using PassTypeList = META_GET_REGISTERED_TYPES(RenderGraphPassRegisterTag);

class RenderPassManager {
    SceneTree::VklSceneTree &sceneTree_;
    std::vector<RenderPassDeclarationBase *> component_ptrs;

public:
    explicit RenderPassManager(SceneTree::VklSceneTree &sceneTree, Controller &controller) : sceneTree_(sceneTree) {
        create_component_instances(PassTypeList {});
    }

    void descriptorStage(RenderGraphDescriptor &descriptor) {
        for (auto com : component_ptrs) {
            com->descriptorStage(descriptor);
        }
    }

    void instanceStage(RenderGraph &renderGraph) {
        for (auto com: component_ptrs)
            com->instanceStage(renderGraph);
    };

private:
    template <typename... ts> void create_component_instances(MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(new ts(sceneTree_)), ...);
    }
};