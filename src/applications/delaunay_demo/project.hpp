#pragma once

#include "graphics_lab/project.hpp"
#include "spdlog/spdlog.h"
#include "hyperbolic_disk_render_pass.hpp"

struct DelaunayDemoProject : IGraphicsLabProject {
    void tick() override {
        spdlog::info("tick in delaunay demo project");
    }

    std::string name() override {
        return "Visualization";
    }

    void afterLoad() override {
        spdlog::info("project loaded");

        hyperbolic_disk_render_pass = std::make_unique<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass>(*context.device, *context.sceneTree);
        {
            std::scoped_lock renderGraphLock(context.applicationContext->renderGraphMutex);

            context.applicationContext->renderGraph->add_pass(hyperbolic_disk_render_pass.get(), "hyperbolic_pass");
            context.applicationContext->renderGraph->add_edge("hyperbolic_pass", "internal_imgui_pass");

            std::scoped_lock renderGraphInstanceLock(context.applicationContext->renderGraphInstanceMutex);
            context.applicationContext->compileRenderGraph();
        }

        spdlog::info("after load finished");
    }

    std::unique_ptr<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass> hyperbolic_disk_render_pass = nullptr;
    ReflectDataType reflect() override;
};