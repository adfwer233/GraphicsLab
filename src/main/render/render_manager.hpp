#pragma once

#include "passes/render_pass_base.hpp"
#include "passes/render_pass_register.hpp"

#include "passes/3d_scene_pass.hpp"
#include "passes/imgui_pass.hpp"
#include "passes/project_render_pass.hpp"

#include "controller/controller.hpp"

#include "boost/di.hpp"

namespace di = boost::di;

using PassTypeList = META_GET_REGISTERED_TYPES(RenderGraphPassRegisterTag);

class RenderPassManager {
    std::vector<RenderPassDeclarationBase *> component_ptrs;

  public:
    explicit RenderPassManager(SceneTree::VklSceneTree &sceneTree, Controller &controller, UIState &state,
                               UIManager &uiManager) {
        auto injector = di::make_injector(di::bind<SceneTree::VklSceneTree>().to(sceneTree),
                                          di::bind<UIManager>().to(uiManager), di::bind<UIState>().to(state));

        create_component_instances(injector, PassTypeList{});

        spdlog::info("[Controller Address] {}", (void *)&controller);
    }

    void descriptorStage(RenderGraphDescriptor &descriptor) {
        for (auto com : component_ptrs) {
            com->descriptorStage(descriptor);
        }
    }

    void instanceStage(RenderGraph &renderGraph) {
        for (auto com : component_ptrs)
            com->instanceStage(renderGraph);
    };

  private:
    template <typename InjectorType, typename... ts>
    void create_component_instances(InjectorType &injector, MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(injector.template create<ts *>()), ...);
    }
};