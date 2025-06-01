#pragma once

#include "geometry/computational_geometry/convex_hull.hpp"
#include "graphics_lab/project.hpp"
#include "hyperbolic_disk_render_pass.hpp"
#include "hyperbolic_tessellation.hpp"
#include "spdlog/spdlog.h"
#include "ui/custom_controller.hpp"
#include "utils/sampler.hpp"

#include "ui/uistate.hpp"

struct DelaunayDemoProject : IGraphicsLabProject {
    void tick() override {
        spdlog::info("tick in delaunay demo project");
    }

    std::string name() override {
        return "Visualization";
    }

    GraphicsLab::IGraphicsLabProjectController *getController() override {
        return controller.get();
    }

    void afterLoad() override {
        spdlog::info("project loaded");

        hyperbolic_disk_render_pass = std::make_unique<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass>(
            *context.device, *context.sceneTree, ui_state);
        {
            std::scoped_lock renderGraphLock(context.applicationContext->renderGraphMutex);

            context.applicationContext->renderGraph->add_pass(hyperbolic_disk_render_pass.get(), "hyperbolic_pass");
            context.applicationContext->renderGraph->add_edge("hyperbolic_pass", "internal_imgui_pass");

            std::scoped_lock renderGraphInstanceLock(context.applicationContext->renderGraphInstanceMutex);
            context.applicationContext->compileRenderGraph();
        }

        ui_state.trans = MobiusConstructor::identity();
        controller = std::make_unique<CustomController>(ui_state);

        spdlog::info("after load finished");
    }

    void visualize_hyperbolic_tessellation() {
        PointCloud2D pc;

        HyperbolicTessellation tessellation(7, 3);
        // tessellation.create_initial_polygon();
        tessellation.create_polygon_tessellation(3);

        for (auto poly : tessellation.polygons) {
            for (auto vert : poly.vertices) {
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

        for (int i = 0; i < 1000; i++) {
            vertices.push_back(GraphicsLab::Sampler::sampleUnitSphere<3>());
        }

        GraphicsLab::Geometry::ConvexHull3D convex_hull;
        convex_hull.build(vertices);

        auto indices = convex_hull.make_indices();

        Mesh3D mesh;
        PointCloud3D pc;
        PointCloud3D pc2;

        for (auto v : vertices) {
            pc.vertices.push_back({v, {1.0, 0.0, 0.0}});
        }

        auto barycentric_point = [](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, int i, int j, int N) -> glm::vec3 {
            float u = float(i) / N;
            float v = float(j) / N;
            float w = 1.0f - u - v;
            return a * w + b * u + c * v;
        };

        auto tessellate_triangle_and_add_to_mesh = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3 &color, int N, Mesh3D& target_mesh) {
            std::vector<std::vector<uint32_t>> vertex_indices(N + 1);
            auto normal = glm::normalize(glm::cross(b - a, c - a));

            for (int i = 0; i <= N; ++i) {
                vertex_indices[i].resize(N - i + 1);
                for (int j = 0; j <= N - i; ++j) {
                    glm::vec3 p = barycentric_point(a, b, c, i, j, N);
                    uint32_t index = static_cast<uint32_t>(target_mesh.vertices.size());
                    target_mesh.vertices.push_back({p, color, normal});
                    vertex_indices[i][j] = index;
                }
            }

            // Create triangles
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < N - i; ++j) {
                    uint32_t v0 = vertex_indices[i][j];
                    uint32_t v1 = vertex_indices[i + 1][j];
                    uint32_t v2 = vertex_indices[i][j + 1];
                    target_mesh.indices.emplace_back(v0, v1, v2);

                    if (j < N - i - 1) {
                        uint32_t v3 = vertex_indices[i + 1][j];
                        uint32_t v4 = vertex_indices[i + 1][j + 1];
                        uint32_t v5 = vertex_indices[i][j + 1];
                        target_mesh.indices.emplace_back(v3, v4, v5);
                    }
                }
            }
        };

        for (auto& f : convex_hull.faces) {
            auto a = f->edge->origin->position;
            auto b = f->edge->next->origin->position;
            auto c = f->edge->next->next->origin->position;

            int N = 10;  // Subdivision level

            glm::vec3 color = GraphicsLab::Sampler::sampleUniformVec3();

            tessellate_triangle_and_add_to_mesh(a, b, c, color, N, mesh);
        }

        using VoronoiEdge = std::pair<GraphicsLab::Geometry::ConvexHull3D::Face*, GraphicsLab::Geometry::ConvexHull3D::Face*>;
        std::map<GraphicsLab::Geometry::ConvexHull3D::Vertex*, std::vector<VoronoiEdge>> voronoi_cells;

        for (auto& f: convex_hull.faces) {
            auto e1 = f->edge;
            auto e2 = f->edge->next;
            auto e3 = f->edge->next->next;

            for (auto e: {e1, e2, e3}) {
                voronoi_cells[e->origin].push_back({e->face, e->twin->face});
                voronoi_cells[e->twin->origin].push_back({e->twin->face, e->face});
            }
        }

        Mesh3D voronoi_spherical_mesh;

        for (auto& [f, edges]: voronoi_cells) {
            decltype(edges) sorted_edges;

            sorted_edges.push_back(edges.front());

            while (edges.size() > sorted_edges.size()) {
                for (auto e: edges) {
                    if (e.first == sorted_edges.back().second) {
                        sorted_edges.push_back(e);
                        break;
                    }
                }
            }

            glm::vec3 color = GraphicsLab::Sampler::sampleUniformVec3();

            for (int i = 2; i < sorted_edges.size(); ++i) {
                auto a = sorted_edges[0].first->spherical_circumcenter();
                auto b = sorted_edges[i - 1].first->spherical_circumcenter();
                auto c = sorted_edges[i].first->spherical_circumcenter();

                tessellate_triangle_and_add_to_mesh(a, b, c, color, 10, voronoi_spherical_mesh);
            }
        }

        for (auto &f : convex_hull.faces) {
            pc2.vertices.push_back({glm::normalize(f->spherical_circumcenter()), {0.0, 1.0, 0.0}});
        }

        CurveMesh3D voronoi_mesh;
        CurveMesh3D delaunay_mesh;

        for (auto &f : convex_hull.faces) {
            auto e = f->edge;
            do {
                int n = 100;
                auto a = e->face->spherical_circumcenter();
                if (e->twin == nullptr)
                    continue;
                auto b = e->twin->face->spherical_circumcenter();

                bool use_long_arc = e->face->is_visible({0, 0, 0});

                for (int i = 0; i <= n; i++) {
                    float param = 1.0f * i / n;
                    auto pos = glm::mix(a, b, param);

                    voronoi_mesh.vertices.push_back({glm::normalize(pos), {0.0, 0.0, 1.0}});
                    if (i > 0) {
                        voronoi_mesh.indices.emplace_back(voronoi_mesh.vertices.size() - 2,
                                                          voronoi_mesh.vertices.size() - 1);
                    }

                    auto delaunay_pos = glm::mix(e->origin->position, e->next->origin->position, param);
                    delaunay_mesh.vertices.push_back({glm::normalize(delaunay_pos), {1.0, 0.0, 0.0}});
                    if (i > 0) {
                        delaunay_mesh.indices.emplace_back(delaunay_mesh.vertices.size() - 2,
                                                           delaunay_mesh.vertices.size() - 1);
                    }
                }
                e = e->next;
            } while (e != f->edge);
        }

        // pc.vertices.back().color = {0.0, 1.0 ,0.0};

        Mesh3D spherical_mesh;
        spherical_mesh.vertices = mesh.vertices;
        spherical_mesh.indices = mesh.indices;

        for (auto& vert: spherical_mesh.vertices) {
            vert.position = glm::normalize(vert.position);
            vert.normal = vert.position;
        }

        for (auto& vert: voronoi_spherical_mesh.vertices) {
            vert.position = glm::normalize(vert.position);
            vert.normal = vert.position;
        }

        context.sceneTree->addGeometryNode(std::move(mesh), "convex hull");
        context.sceneTree->addGeometryNode(std::move(spherical_mesh), "delaunay result");
        context.sceneTree->addGeometryNode(std::move(voronoi_spherical_mesh), "voronoi result");
        context.sceneTree->addGeometryNode(std::move(voronoi_mesh), "voronoi");
        context.sceneTree->addGeometryNode(std::move(delaunay_mesh), "delaunay");
        context.sceneTree->addGeometryNode(std::move(pc), "test points");
        context.sceneTree->addGeometryNode(std::move(pc2), "pc 2");

        auto delaunay_result = context.sceneTree->get_geometry_node<Mesh3D>("delaunay result");
        delaunay_result->visible = false;
    }

    std::unique_ptr<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass> hyperbolic_disk_render_pass = nullptr;
    ProjectUIState ui_state;
    std::unique_ptr<CustomController> controller = nullptr;

    ReflectDataType reflect() override;
};