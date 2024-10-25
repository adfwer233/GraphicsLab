#pragma once

#include "vkl/render_graph/render_graph.hpp"
#include "vkl/scene_tree/vkl_scene_tree.hpp"

#include "utils/imgui_sink.hpp"

#include "graphics_lab/render_graph/render_graph.hpp"
#include "graphics_lab/render_graph/render_graph_compiler.hpp"
#include "graphics_lab/render_graph/render_graph_instance.hpp"

struct GraphicsLabContext {
    VklDevice *device;
    SceneTree::VklSceneTree *sceneTree;
    LogManager *logManager;
};


namespace GraphicsLab {
struct GraphicsLabInternalContext {
    VklWindow window_;
    VklDevice device_;
    GraphicsLab::RenderGraph::RenderContext renderContext;


    std::unique_ptr<RenderGraph::RenderGraph> renderGraph;
    std::unique_ptr<RenderGraph::RenderGraphInstance> renderGraphInstance;
    std::unique_ptr<RenderGraph::RenderGraphInstance> newRenderGraphInstance;

    /**
     * @brief mutex lock protecting render graph (i.e. the descriptor)
     */
    std::mutex renderGraphMutex;

    /**
     * @brief: mutex lock protecting render graph instance;
     */
    std::mutex renderGraphInstanceMutex;

    GraphicsLabInternalContext(uint32_t width, uint32_t height): window_(width, height), device_(window_), renderContext(device_, window_, {width, height}) {
        renderGraph = std::make_unique<RenderGraph::RenderGraph>();
    }

    void compileRenderGraph() {
        std::scoped_lock instanceLock(renderGraphInstanceMutex);
        RenderGraph::RenderGraphCompiler compiler(*renderGraph, device_);
        newRenderGraphInstance = compiler.compile(&renderContext);
    }
};
}
