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

    struct Line {
        glm::vec3 point;
        glm::vec3 direction;
    };

    // Return nullopt if planes are parallel or coincident
    std::optional<Line> intersectPlanes(const glm::vec3& p1, const glm::vec3& n1,
                                        const glm::vec3& p2, const glm::vec3& n2) {
        glm::vec3 dir = glm::cross(n1, n2);

        if (glm::length(dir) < 1e-6f) {
            // Planes are parallel or coincident
            return std::nullopt;
        }

        // Solve for a point on the intersection line using:
        // (n1 x n2)·x = d1(n2) - d2(n1) x (n1 x n2)
        float d1 = glm::dot(n1, p1);
        float d2 = glm::dot(n2, p2);
        glm::vec3 n1xn2 = glm::cross(n1, n2);
        glm::vec3 n1xn2_squared = n1xn2 * glm::dot(n1xn2, n1xn2);

        glm::vec3 point =
            ((glm::cross(n1xn2, n2) * d1) + (glm::cross(n1, n1xn2) * d2)) / glm::dot(n1xn2, n1xn2);

        return Line{ point, glm::normalize(dir) };
    }

    std::optional<glm::vec3> intersectLinePlane(const glm::vec3& l0, const glm::vec3& d,
                                            const glm::vec3& p0, const glm::vec3& n) {
        float denom = glm::dot(n, d);
        if (std::abs(denom) < 1e-6f) {
            // Line is parallel to plane
            return std::nullopt;
        }

        float t = glm::dot(n, p0 - l0) / denom;
        return l0 + t * d;
    }

    bool segmentIntersectsUnitDisk(const glm::vec2& a, const glm::vec2& b) {
        // Check if either endpoint is inside the unit disk
        if (glm::length(a) <= 1.0f || glm::length(b) <= 1.0f)
            return true;

        // Direction of the segment
        glm::vec2 d = b - a;

        // Solve quadratic equation for intersection of ray with circle
        float A = glm::dot(d, d);
        float B = 2.0f * glm::dot(a, d);
        float C = glm::dot(a, a) - 1.0f;

        float discriminant = B * B - 4.0f * A * C;
        if (discriminant < 0.0f) {
            return false; // No intersection
        }

        // Find intersection points along the segment using parameter t in [0,1]
        float sqrt_disc = sqrt(discriminant);
        float t1 = (-B - sqrt_disc) / (2.0f * A);
        float t2 = (-B + sqrt_disc) / (2.0f * A);

        // Check if any intersection point lies on the segment
        return (t1 >= 0.0f && t1 <= 1.0f) || (t2 >= 0.0f && t2 <= 1.0f);
    }

    void visualize_hyperbolic_delaunay() {

        auto glm_to_complex = [](glm::vec2 p) -> std::complex<float> {
            return {p.x, p.y};
        };

        auto get_circumcenter = [](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
            glm::vec3 ba = b - a;
            glm::vec3 ca = c - a;
            glm::vec3 n = glm::cross(ba, ca);
            float n2 = glm::dot(n, n); // Squared length of normal

            // Avoid division by zero for degenerate triangle
            if (n2 < 1e-10f) {
                return a; // fallback: triangle is degenerate
            }

            float ba2 = glm::dot(ba, ba);
            float ca2 = glm::dot(ca, ca);

            glm::vec3 numerator = ba2 * glm::cross(ca, n) + ca2 * glm::cross(n, ba);
            glm::vec3 center = a + numerator / (2.0f * n2);
            return center;
        };

        auto pi_infinity_projection = [](const glm::vec3& p) {
            float factor = (p.z + 1) / 2;
            static glm::vec3 phi_inf{0, 0, -1};
            return phi_inf + factor * (p - phi_inf);
        };

        std::vector<glm::vec2> points;

        for (int i = 0; i < 32; i++) {
            auto pt = GraphicsLab::Sampler::sampleUniformVec2(-1, 1);
            while (glm::length(pt) > 1) {
                pt = GraphicsLab::Sampler::sampleUniformVec2(-1, 1);
            }
            points.push_back(pt);
        }

        std::vector<glm::vec3> lifted_points;

        for (auto p: points) {
            lifted_points.emplace_back(p.x, p.y, p.x * p.x + p.y * p.y);
        }

        GraphicsLab::Geometry::ConvexHull3D convex_hull;
        convex_hull.build(lifted_points);

        auto convex_hull_indices = convex_hull.make_indices();
        std::vector<TriangleIndex> euclidean_delaunay_indices;

        for (auto index: convex_hull_indices) {
            auto a = lifted_points[index.i];
            auto b = lifted_points[index.j];
            auto c = lifted_points[index.k];

            auto cross = glm::cross(b - a, c - a);
            if (cross.z < 0) {
                euclidean_delaunay_indices.push_back(index);
            }
        }

        /**
         * create euclidean dealunay triangulation mesh.
         */
        CurveMesh2D euclidean_mesh;
        for (auto p: points) {
            euclidean_mesh.vertices.push_back({p, {1.0, 0.0, 0.0}});
        }
        for (auto index: euclidean_delaunay_indices) {
            euclidean_mesh.indices.emplace_back(index.i, index.j);
            euclidean_mesh.indices.emplace_back(index.j, index.k);
            euclidean_mesh.indices.emplace_back(index.k, index.i);
        }
        context.sceneTree->addGeometryNode(std::move(euclidean_mesh), "Euclidean Dealunay");

        /**
         * filter hyperbolic 2-simplex
         */
        using Edge = std::pair<int, int>;
        auto sort_edge = [](Edge a) -> Edge {
            return {std::min(a.first, a.second), std::max(a.first, a.second)};
        };

        std::vector<TriangleIndex> hyperbolic_delaunay_triangle_indices;
        std::map<Edge, std::vector<int>> adjacent_points;
        std::set<Edge> marked_edges;

        auto record_adjacent_points = [&](int i, int j, int k) {
            adjacent_points[sort_edge({i, j})].push_back(k);
            adjacent_points[sort_edge({j, k})].push_back(i);
            adjacent_points[sort_edge({k, i})].push_back(j);
        };

        auto mark_edges = [&](int i, int j, int k) {
            marked_edges.insert(sort_edge({i, j}));
            marked_edges.insert(sort_edge({j, k}));
            marked_edges.insert(sort_edge({k, i}));
        };

        auto parabolic_normal = [&](const glm::vec3& a) {
            glm::vec3 tmp{-2 * a.x, -2 * a.y, a.z};
            return glm::normalize(tmp);
        };

        for (auto index: euclidean_delaunay_indices) {
            auto a = lifted_points[index.i];
            auto b = lifted_points[index.j];
            auto c = lifted_points[index.k];

            a.z = b.z = c.z = 0;
            // euclidean circumcenter coordinate

            auto circumcenter_vec3 = get_circumcenter(a, b, c);
            glm::vec2 circumcenter{circumcenter_vec3.x, circumcenter_vec3.y};

            // 2-simplex is_hyperbolic
            if (glm::distance(circumcenter_vec3, a) + glm::length(circumcenter) < 1.0) {
                hyperbolic_delaunay_triangle_indices.push_back(index);
                mark_edges(index.i, index.j, index.k);
            }

            record_adjacent_points(index.i, index.j, index.k);
        }

        /**
         * filter 1-simplex
         */

        std::vector<LineIndex> hyperbolic_delaunay_line_indices;

        for (auto& [e, adj]: adjacent_points) {
            if (marked_edges.contains(e)) continue;

            if (adj.size() != 2) continue;

            glm::vec3 phi_a_point = lifted_points[e.first];
            glm::vec3 phi_b_point = lifted_points[e.second];
            glm::vec3 phi_a_normal = parabolic_normal(phi_a_point);
            glm::vec3 phi_b_normal = parabolic_normal(phi_b_point);

            auto inter = intersectPlanes(phi_a_point, phi_a_normal, phi_b_point, phi_b_normal);
            if (not inter.has_value()) continue;

            auto intersect_phi_a_phi_b = inter.value();

            glm::vec3 adj1_point = lifted_points[adj.front()];
            glm::vec3 adj2_point = lifted_points[adj.back()];
            glm::vec3 adj1_normal = parabolic_normal(adj1_point);
            glm::vec3 adj2_normal = parabolic_normal(adj2_point);

            auto inter1 = intersectLinePlane(intersect_phi_a_phi_b.point, intersect_phi_a_phi_b.direction, adj1_point, adj1_normal);
            auto inter2 = intersectLinePlane(intersect_phi_a_phi_b.point, intersect_phi_a_phi_b.direction, adj2_point, adj2_normal);

            if ((not inter1.has_value()) or (not inter2.has_value())) {
                continue;
            }

            auto u_sigma_point1 = inter1.value();
            auto u_sigma_point2 = inter2.value();

            auto proj1 = pi_infinity_projection(u_sigma_point1);
            auto proj2 = pi_infinity_projection(u_sigma_point2);

            glm::vec2 proj1_2d{proj1.x, proj1.y};
            glm::vec2 proj2_2d{proj2.x, proj2.y};

            if (segmentIntersectsUnitDisk(proj1_2d, proj2_2d)) {
                hyperbolic_delaunay_line_indices.emplace_back(e.first, e.second);
            }
        }

        /**
         * create hyperbolic delaunay mesh
         */
        CurveMesh2D hyperbolic_delaunay_triangle_mesh;

        // add 2-simplex
        for (auto index: hyperbolic_delaunay_triangle_indices) {
            std::vector<std::pair<int, int>> edges = {{index.i, index.j}, {index.j, index.k}, {index.k, index.i}};
            for (auto [u, v]: edges) {
                GraphicsLab::Geometry::HyperbolicLineSegment geodesic(glm_to_complex(points[u]), glm_to_complex(points[v]));
                int n = 10;
                for (int i = 0; i <= n; i++) {
                    float param = 1.0 * i / n;
                    auto pos = geodesic.evaluate(param);
                    auto id = hyperbolic_delaunay_triangle_mesh.vertices.size();
                    hyperbolic_delaunay_triangle_mesh.vertices.push_back({{pos.real(), pos.imag()}, {0.0, 0.0, 1.0}});
                    if (i > 0) {
                        hyperbolic_delaunay_triangle_mesh.indices.emplace_back(id - 1, id);
                    }
                }
            }
        }

        // add 1-simplex

        for (auto index: hyperbolic_delaunay_line_indices) {
            GraphicsLab::Geometry::HyperbolicLineSegment geodesic(glm_to_complex(points[index.i]), glm_to_complex(points[index.j]));
            int n = 10;
            for (int i = 0; i <= n; i++) {
                float param = 1.0 * i / n;
                auto pos = geodesic.evaluate(param);
                auto id = hyperbolic_delaunay_triangle_mesh.vertices.size();
                hyperbolic_delaunay_triangle_mesh.vertices.push_back({{pos.real(), pos.imag()}, {0.0, 0.0, 1.0}});
                if (i > 0) {
                    hyperbolic_delaunay_triangle_mesh.indices.emplace_back(id - 1, id);
                }
            }
        }
        context.sceneTree->addGeometryNode(std::move(hyperbolic_delaunay_triangle_mesh), "Hyperbolic Delaunay Complex");

    }

    void visualize_hyperbolic_tessellation(int p, int q, int depth) {
        PointCloud2D pc;

        HyperbolicTessellation tessellation(p, q);
        // tessellation.create_initial_polygon();
        tessellation.create_polygon_tessellation(depth);

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

    void visualize_spherical_voronoi(int vert_num, int tessellation_num) {
        std::vector<glm::vec3> vertices;
        int n = 0;

        for (int i = 0; i < vert_num; i++) {
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

        auto barycentric_point = [](const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, int i, int j,
                                    int N) -> glm::vec3 {
            float u = float(i) / N;
            float v = float(j) / N;
            float w = 1.0f - u - v;
            return a * w + b * u + c * v;
        };

        auto tessellate_triangle_and_add_to_mesh = [&](const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c,
                                                       const glm::vec3 &color, int N, Mesh3D &target_mesh) {
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

        for (auto &f : convex_hull.faces) {
            auto a = f->edge->origin->position;
            auto b = f->edge->next->origin->position;
            auto c = f->edge->next->next->origin->position;

            glm::vec3 color = GraphicsLab::Sampler::sampleUniformVec3();

            tessellate_triangle_and_add_to_mesh(a, b, c, color, tessellation_num, mesh);
        }

        using VoronoiEdge =
            std::pair<GraphicsLab::Geometry::ConvexHull3D::Face *, GraphicsLab::Geometry::ConvexHull3D::Face *>;
        std::map<GraphicsLab::Geometry::ConvexHull3D::Vertex *, std::vector<VoronoiEdge>> voronoi_cells;

        for (auto &f : convex_hull.faces) {
            auto e1 = f->edge;
            auto e2 = f->edge->next;
            auto e3 = f->edge->next->next;

            for (auto e : {e1, e2, e3}) {
                voronoi_cells[e->origin].push_back({e->face, e->twin->face});
                voronoi_cells[e->twin->origin].push_back({e->twin->face, e->face});
            }
        }

        Mesh3D voronoi_spherical_mesh;

        for (auto &[f, edges] : voronoi_cells) {
            decltype(edges) sorted_edges;

            sorted_edges.push_back(edges.front());

            while (edges.size() > sorted_edges.size()) {
                for (auto e : edges) {
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

                tessellate_triangle_and_add_to_mesh(a, b, c, color, tessellation_num, voronoi_spherical_mesh);
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

        for (auto &vert : spherical_mesh.vertices) {
            vert.position = glm::normalize(vert.position);
            vert.normal = vert.position;
        }

        for (auto &vert : voronoi_spherical_mesh.vertices) {
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