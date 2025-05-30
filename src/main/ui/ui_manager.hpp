#pragma once

#include "components/component.hpp"

#include "render_resources.hpp"

#include "components/constructor_widget.hpp"
#include "components/dummy_component.hpp"
#include "components/logger_widget.hpp"
#include "components/project_render_widgets.hpp"
#include "components/project_widgets.hpp"
#include "components/python_terminal.hpp"
#include "components/render_mode_panel.hpp"
#include "components/scene_tree.hpp"
#include "components/scene_widgets.hpp"
#include "components/texture_manager.hpp"

#include "graphics_lab/graphics_lab_context.hpp"

using ComponentTypeList = META_GET_REGISTERED_TYPES(MainComponentRegisterTag);

using ScenePanelRenderOrder = MetaProgramming::TypeList<TextureManager, SceneWidgetComponent>;

namespace UIManagerImpl {
using RenderOrder = MetaProgramming::TypeList<TextureManager, SceneWidgetComponent>;
}

class UIManager {
    std::vector<UIComponent *> component_ptrs;
    std::map<int, int> component_map;
    UIState &ui_state_;

  public:
    RenderResources renderResources;

    explicit UIManager(Controller &controller, UIState &uiState, GraphicsLab::GraphicsLabInternalContext &context)
        : ui_state_(uiState) {
        auto injector = di::make_injector(
            di::bind<Controller>().to(controller), di::bind<RenderResources>().to(renderResources),
            di::bind<UIState>().to(uiState), di::bind<GraphicsLab::GraphicsLabInternalContext>().to(context));
        create_component_instances(injector, ComponentTypeList{});

        for (int i = 0; i < component_ptrs.size(); ++i) {
            component_map[i] = i;
        }
        using PreviousType = typename MetaProgramming::TypeListFunctions::KthOf<UIManagerImpl::RenderOrder, 0>::type;
        constexpr int TypeIndex1 =
            MetaProgramming::TypeListFunctions::IndexOf<ComponentTypeList, SceneWidgetComponent>::value;
        constexpr int PreviousTypeIndex2 =
            MetaProgramming::TypeListFunctions::IndexOf<ComponentTypeList, PreviousType>::value;
        MetaProgramming::ForEachTypeWithIndices(UIManagerImpl::RenderOrder{}, [&]<typename T, size_t i>() {
            if constexpr (i > 0) {
                using PreviousType =
                    typename MetaProgramming::TypeListFunctions::KthOf<UIManagerImpl::RenderOrder, i - 1>::type;
                constexpr int TypeIndex = MetaProgramming::TypeListFunctions::IndexOf<ComponentTypeList, T>::value;
                constexpr int PreviousTypeIndex =
                    MetaProgramming::TypeListFunctions::IndexOf<ComponentTypeList, PreviousType>::value;
                component_map[TypeIndex] = component_map[PreviousTypeIndex] + 1;
            }
        });

        std::map<UIComponent *, int> components_priority;

        for (int i = 0; i < component_ptrs.size(); ++i) {
            components_priority[component_ptrs[i]] = component_map[i];
        }

        std::ranges::sort(component_ptrs, [&components_priority](UIComponent *l, UIComponent *r) -> bool {
            return components_priority[l] < components_priority[r];
        });
    }

    void render() {
        for (auto com : component_ptrs) {
            com->render();
        }

        // render custom components

        if (ui_state_.project != nullptr) {
            for (auto com : ui_state_.project->getImguiComponents()) {
                com->render();
            }
        }
    }

  private:
    template <typename InjectorType, typename... ts>
    void create_component_instances(InjectorType &injector, MetaProgramming::TypeList<ts...>) {
        (component_ptrs.push_back(injector.template create<ts *>()), ...);
    }
};