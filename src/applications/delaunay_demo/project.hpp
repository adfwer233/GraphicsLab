#pragma once

#include "graphics_lab/project.hpp"
#include "hyperbolic_disk_render_pass.hpp"
#include "hyperbolic_tessellation.hpp"
#include "spdlog/spdlog.h"

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

    void visualize_hyperbolic_tessellation() {
        PointCloud2D pc;

        HyperbolicTessellation tessellation(7, 3);
        // tessellation.create_initial_polygon();
        tessellation.create_polygon_tessellation(2);

        for (auto poly: tessellation.polygons) {
            for (auto vert: poly.vertices) {
                pc.vertices.push_back({{vert.real(), vert.imag()}});
            }
            // pc.vertices.push_back({{poly.center.real(), poly.center.imag()}});
        }

        std::scoped_lock lock(context.sceneTree->sceneTreeMutex);
        context.sceneTree->addGeometryNode(std::move(pc), "Vert");
    }

    std::unique_ptr<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass> hyperbolic_disk_render_pass = nullptr;
    ReflectDataType reflect() override;
};