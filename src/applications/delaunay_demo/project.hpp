#pragma once

#include "geometry/computational_geometry/convex_hull.hpp"
#include "graphics_lab/project.hpp"
#include "hyperbolic_disk_render_pass.hpp"
#include "hyperbolic_tessellation.hpp"
#include "spdlog/spdlog.h"
#include "utils/sampler.hpp"

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
        tessellation.create_polygon_tessellation(3);

        for (auto poly: tessellation.polygons) {
            for (auto vert: poly.vertices) {
                pc.vertices.push_back({{vert.real(), vert.imag()}});
            }
            pc.vertices.push_back({{poly.center.real(), poly.center.imag()}, {1.0, 0.0, 0.0}});
        }

        auto curve_mesh = tessellation.create_curve_mesh_2d();

        std::scoped_lock lock(context.sceneTree->sceneTreeMutex);
        context.sceneTree->addGeometryNode(std::move(pc), "Vert");
        context.sceneTree->addGeometryNode(std::move(curve_mesh), "tessellation");
    }

    void visualize_spherical_voronoi() {
        std::vector<glm::vec3> vertices;
        int n = 0;

        for (int i = 0; i < 16; i++) {
            vertices.push_back(GraphicsLab::Sampler::sampleUnitSphere<3>());
        }

        GraphicsLab::Geometry::ConvexHull3D convex_hull;
        convex_hull.build(vertices);

        auto indices = convex_hull.make_indices();

        Mesh3D mesh;
        PointCloud3D pc;
        PointCloud3D pc2;

        for (auto v: vertices) {
            mesh.vertices.push_back({v, {1.0, 0.0, 0.0}, v});
            pc.vertices.push_back({v, {1.0, 0.0, 0.0}});
        }

        for (auto& f: convex_hull.faces) {
            pc2.vertices.push_back({glm::normalize(f->spherical_circumcenter()), {0.0, 1.0, 0.0}});
        }

        CurveMesh3D voronoi_mesh;
        CurveMesh3D delaunay_mesh;

        for (auto& f: convex_hull.faces) {
            auto e = f->edge;
            do {
                int n = 100;
                auto a = e->face->spherical_circumcenter();
                if (e->twin == nullptr) continue;
                auto b = e->twin->face->spherical_circumcenter();

                bool use_long_arc = e->face->is_visible({0, 0, 0});

                for (int i = 0; i <= n; i++) {
                    float param = 1.0f * i / n;
                    auto pos = glm::mix(a, b, param);

                    voronoi_mesh.vertices.push_back({glm::normalize(pos), {0.0, 0.0, 1.0}});
                    if (i > 0) {
                        voronoi_mesh.indices.emplace_back(voronoi_mesh.vertices.size() - 2, voronoi_mesh.vertices.size() - 1);
                    }

                    auto delaunay_pos = glm::mix(e->origin->position, e->next->origin->position, param);
                    delaunay_mesh.vertices.push_back({glm::normalize(delaunay_pos), {1.0, 0.0, 0.0}});
                    if (i > 0) {
                        delaunay_mesh.indices.emplace_back(delaunay_mesh.vertices.size() - 2, delaunay_mesh.vertices.size() - 1);
                    }
                }
                e = e->next;
            } while (e != f->edge);
        }

        // pc.vertices.back().color = {0.0, 1.0 ,0.0};

        mesh.indices = indices;

        context.sceneTree->addGeometryNode(std::move(mesh), "convex hull");
        context.sceneTree->addGeometryNode(std::move(voronoi_mesh), "voronoi");
        context.sceneTree->addGeometryNode(std::move(delaunay_mesh), "delaunay");
        context.sceneTree->addGeometryNode(std::move(pc), "test points");
        context.sceneTree->addGeometryNode(std::move(pc2), "pc 2");
    }

    std::unique_ptr<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass> hyperbolic_disk_render_pass = nullptr;
    ReflectDataType reflect() override;
};