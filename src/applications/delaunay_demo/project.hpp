#pragma once

#include "geometry/computational_geometry/convex_hull.hpp"
#include "glm/gtx/norm.inl"
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
    std::optional<Line> intersectPlanes(const glm::vec3 &p1, const glm::vec3 &n1, const glm::vec3 &p2,
                                        const glm::vec3 &n2) {
        glm::vec3 direction = glm::cross(n1, n2);

        if (glm::length(direction) < 1e-6f) {
            // Planes are parallel or coincident
            return std::nullopt;
        }

        // Compute a point on the intersection line
        float d1 = -glm::dot(n1, p1);
        float d2 = -glm::dot(n2, p2);
        glm::vec3 n1xn2 = direction;

        glm::mat3 A(n1, n2, n1xn2);
        glm::vec3 rhs = -glm::vec3(d1, d2, 0.0f);

        glm::vec3 point = glm::inverse(glm::transpose(A)) * rhs;

        return Line{point, glm::normalize(direction)};
    }

    std::optional<glm::vec3> intersectLinePlane(const glm::vec3 &l0, const glm::vec3 &d, const glm::vec3 &p0,
                                                const glm::vec3 &n) {
        float denom = glm::dot(n, d);
        if (std::abs(denom) < 1e-6f) {
            // Line is parallel to plane
            return std::nullopt;
        }

        float t = glm::dot(n, p0 - l0) / denom;
        return l0 + t * d;
    }

    bool segmentIntersectsUnitDisk(const glm::vec2 &a, const glm::vec2 &b) {
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

    // Return 0, 1, or 2 intersection points between two circles
    std::vector<glm::vec2> intersectCircles(const glm::vec2 &c0, float r0, const glm::vec2 &c1, float r1) {
        glm::vec2 d = c1 - c0;
        float distSq = glm::length2(d);
        float dist = glm::sqrt(distSq);

        // No solution: circles are separate or one is within the other
        if (dist > r0 + r1 || dist < std::abs(r0 - r1)) {
            return {};
        }

        // Normalize direction
        glm::vec2 dir = d / dist;

        // Point `p` is the point along the line between centers where the chord crosses
        float a = (r0 * r0 - r1 * r1 + distSq) / (2.0f * dist);
        glm::vec2 p = c0 + a * dir;

        // Check for tangent (one solution)
        float hSq = r0 * r0 - a * a;
        if (hSq < 1e-6f) {
            return {p}; // one intersection (tangent)
        }

        // Two intersections
        float h = glm::sqrt(hSq);
        glm::vec2 perp = glm::vec2(-dir.y, dir.x); // perpendicular vector

        glm::vec2 i1 = p + h * perp;
        glm::vec2 i2 = p - h * perp;

        return {i1, i2};
    }

    void visualize_hyperbolic_voronoi(int site_num) {

        auto glm_to_complex = [](glm::vec2 p) -> std::complex<float> { return {p.x, p.y}; };

        auto get_circumcenter = [](const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c) {
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

        auto get_circumcenter_2d = [&](const glm::vec2 &a, const glm::vec2 &b, const glm::vec2 &c) {
            glm::vec3 a1{a.x, a.y, 0};
            glm::vec3 a2{b.x, b.y, 0};
            glm::vec3 a3{c.x, c.y, 0};
            auto res = get_circumcenter(a1, a2, a3);
            return glm::vec2{res.x, res.y};
        };

        auto pi_infinity_projection = [](const glm::vec3 &p) {
            float factor = (p.z + 1) / 2;
            static glm::vec3 phi_inf{0, 0, -1};
            return phi_inf + (p - phi_inf) / factor;
        };

        auto get_hyperbolic_circumcenter = [&](const glm::vec2 &c, float r) {
            float h = glm::length(c);
            float a = h;
            float b = -(1 + h * h - r * r);
            float c0 = h;

            float root = (-b - std::sqrt(b * b - 4 * a * c0)) / (2 * a);

            float factor = (1 + root * root) / (-b);
            return factor * c;
        };

        /**
         * construct bisector between two points p and q
         */
        auto construct_hyperbolic_bisector = [&](const glm::vec2 &p,
                                                 const glm::vec2 &q) -> std::pair<glm::vec2, float> {
            float op = std::pow(glm::length(p), 2);
            float oq = std::pow(glm::length(q), 2);

            float alpha = (1 - oq) / (op - oq);
            auto center = alpha * p + (1 - alpha) * q;
            float squared_radius = std::pow(glm::length(center), 2) - 1;

            return {center, std::sqrt(squared_radius)};
        };

        std::vector<glm::vec2> points;

        for (int i = 0; i < site_num; i++) {
            auto pt = GraphicsLab::Sampler::sampleUniformVec2(-1, 1);
            while (glm::length(pt) > 1) {
                pt = GraphicsLab::Sampler::sampleUniformVec2(-1, 1);
            }
            points.push_back(pt);
        }

        std::vector<glm::vec3> lifted_points;

        for (auto p : points) {
            lifted_points.emplace_back(p.x, p.y, p.x * p.x + p.y * p.y);
        }

        GraphicsLab::Geometry::ConvexHull3D convex_hull;
        convex_hull.build(lifted_points);

        auto convex_hull_indices = convex_hull.make_indices();
        std::vector<TriangleIndex> euclidean_delaunay_indices;

        for (auto index : convex_hull_indices) {
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
        for (auto p : points) {
            euclidean_mesh.vertices.push_back({p, {1.0, 0.0, 0.0}});
        }
        for (auto index : euclidean_delaunay_indices) {
            euclidean_mesh.indices.emplace_back(index.i, index.j);
            euclidean_mesh.indices.emplace_back(index.j, index.k);
            euclidean_mesh.indices.emplace_back(index.k, index.i);
        }

        auto euclidean_mesh_node =
            context.sceneTree->addGeometryNodeAsync(std::move(euclidean_mesh), "Euclidean Delaunay");
        euclidean_mesh_node->visible = false;

        /**
         * filter hyperbolic 2-simplex
         */
        using Edge = std::pair<int, int>;
        auto sort_edge = [](Edge a) -> Edge { return {std::min(a.first, a.second), std::max(a.first, a.second)}; };

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

        auto parabolic_normal = [&](const glm::vec3 &a) {
            glm::vec3 tmp{-2 * a.x, -2 * a.y, 1};
            return glm::normalize(tmp);
        };

        auto is_hyperbolic_triangle = [&](glm::vec2 a, glm::vec2 b, glm::vec2 c) {
            glm::vec2 circumcenter = get_circumcenter_2d(a, b, c);
            return glm::distance(circumcenter, a) + glm::length(circumcenter) < 1.0;
        };

        for (auto index : euclidean_delaunay_indices) {
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

        for (auto &[e, adj] : adjacent_points) {
            if (marked_edges.contains(e))
                continue;

            if (adj.size() != 2) {
                spdlog::info("adj size {}", adj.size());
                continue;
            }

            glm::vec3 phi_a_point = lifted_points[e.first];
            glm::vec3 phi_b_point = lifted_points[e.second];
            glm::vec3 phi_a_normal = parabolic_normal(phi_a_point);
            glm::vec3 phi_b_normal = parabolic_normal(phi_b_point);

            auto inter = intersectPlanes(phi_a_point, phi_a_normal, phi_b_point, phi_b_normal);

            auto dot1 = glm::dot(phi_a_normal, inter->point - phi_a_point);
            auto dot2 = glm::dot(phi_b_normal, inter->point - phi_b_point);
            auto dot3 = glm::dot(phi_a_normal, inter->direction);
            auto dot4 = glm::dot(phi_b_normal, inter->direction);

            glm::vec2 possible_center{inter->point.x, inter->point.y};
            float d1 = glm::distance(possible_center, points[e.first]);
            float d2 = glm::distance(possible_center, points[e.second]);

            if (not inter.has_value())
                continue;

            auto intersect_phi_a_phi_b = inter.value();

            glm::vec3 adj1_point = lifted_points[adj.front()];
            glm::vec3 adj2_point = lifted_points[adj.back()];
            glm::vec3 adj1_normal = parabolic_normal(adj1_point);
            glm::vec3 adj2_normal = parabolic_normal(adj2_point);

            auto inter1 = intersectLinePlane(intersect_phi_a_phi_b.point, intersect_phi_a_phi_b.direction, adj1_point,
                                             adj1_normal);
            auto inter2 = intersectLinePlane(intersect_phi_a_phi_b.point, intersect_phi_a_phi_b.direction, adj2_point,
                                             adj2_normal);

            auto dot2_1 = glm::dot(adj1_normal, inter1.value() - adj1_point);
            auto cross1 = glm::cross(intersect_phi_a_phi_b.direction, inter1.value() - intersect_phi_a_phi_b.point);
            if ((not inter1.has_value()) or (not inter2.has_value())) {
                continue;
            }

            auto u_sigma_point1 = inter1.value();

            glm::vec2 u_sigma_point1_projected{u_sigma_point1.x, u_sigma_point1.y};
            auto D1 = glm::distance(u_sigma_point1_projected, points[e.first]);
            auto D2 = glm::distance(u_sigma_point1_projected, points[e.second]);
            auto D3 = glm::distance(u_sigma_point1_projected, points[adj.front()]);

            auto u_sigma_point2 = inter2.value();

            auto u_sigma_mid = (u_sigma_point1 + u_sigma_point2) / 2.0f;
            auto u1 = glm::dot(adj1_normal, u_sigma_mid - adj1_point);
            auto u2 = glm::dot(adj2_normal, u_sigma_mid - adj2_point);

            if (u1 < 0 or u2 < 0)
                continue;

            auto proj1 = pi_infinity_projection(u_sigma_point1);
            auto proj2 = pi_infinity_projection(u_sigma_point2);

            glm::vec2 proj1_2d{proj1.x, proj1.y};
            glm::vec2 proj2_2d{proj2.x, proj2.y};

            if (segmentIntersectsUnitDisk(proj1_2d, proj2_2d)) {
                hyperbolic_delaunay_line_indices.emplace_back(e.first, e.second);
                marked_edges.insert(e);
            } else {
                // marked_edges.insert(e);
                int x = 0;
            }
        }

        /**
         * create hyperbolic delaunay mesh
         */
        CurveMesh2D hyperbolic_delaunay_triangle_mesh;

        auto add_segment_to_disk = [&](const glm::vec2 &x, const glm::vec2 &y, CurveMesh2D &target_mesh,
                                       glm::vec3 color) {
            GraphicsLab::Geometry::HyperbolicLineSegment geodesic(glm_to_complex(x), glm_to_complex(y));
            int n = 20;
            for (int i = 0; i <= n; i++) {
                float param = 1.0 * i / n;
                auto pos = geodesic.evaluate(param);
                auto id = target_mesh.vertices.size();
                target_mesh.vertices.push_back({{pos.real(), pos.imag()}, color});
                if (i > 0) {
                    target_mesh.indices.emplace_back(id - 1, id);
                }
            }
        };

        // add 2-simplex
        for (auto index : hyperbolic_delaunay_triangle_indices) {
            std::vector<std::pair<int, int>> edges = {{index.i, index.j}, {index.j, index.k}, {index.k, index.i}};
            for (auto [u, v] : edges) {
                add_segment_to_disk(points[u], points[v], hyperbolic_delaunay_triangle_mesh, {0.0, 0.0, 1.0});
            }
        }

        // add 1-simplex

        for (auto index : hyperbolic_delaunay_line_indices) {
            add_segment_to_disk(points[index.i], points[index.j], hyperbolic_delaunay_triangle_mesh, {0.0, 0.0, 1.0});
        }
        context.sceneTree->addGeometryNodeAsync(std::move(hyperbolic_delaunay_triangle_mesh),
                                                "Hyperbolic Delaunay Complex");

        /**
         * construct voronoi diagram
         */
        CurveMesh2D hyperbolic_voronoi_mesh;
        glm::vec3 voronoi_color{0.0, 1.0, 1.0};

        auto add_bisector_free = [&](const glm::vec2 &p, const glm::vec2 &q) {
            auto [center, radius] = construct_hyperbolic_bisector(p, q);
            auto inter = intersectCircles(center, radius, glm::vec2(0), 0.9999f);

            auto start = glm::normalize(center) * (glm::length(center) - radius);

            if (inter.size() == 2) {
                add_segment_to_disk(start, inter.front(), hyperbolic_voronoi_mesh, voronoi_color);
                add_segment_to_disk(start, inter.back(), hyperbolic_voronoi_mesh, voronoi_color);
            }
        };

        auto add_bisector_one_endpoint = [&](const glm::vec2 &p, const glm::vec2 &q, const glm::vec2 &r,
                                             const glm::vec2 &hyperbolic_center) {
            auto [center, radius] = construct_hyperbolic_bisector(p, q);
            auto inter = intersectCircles(center, radius, glm::vec2(0), 0.9999f);

            auto d1 = q - p;
            auto d2 = r - p;
            auto cross = d1.x * d2.y - d1.y * d2.x;

            glm::vec2 segment_dir = d1;
            if (cross < 0)
                segment_dir *= -1;

            if (inter.size() == 2) {
                auto bisector_dir = inter.front() - hyperbolic_center;
                auto cross1 = segment_dir.x * bisector_dir.y - segment_dir.y * bisector_dir.x;

                auto bisector_dir2 = inter.back() - hyperbolic_center - segment_dir;
                auto cross2 = segment_dir.x * bisector_dir2.y - segment_dir.y * bisector_dir2.x;

                if (cross1 < cross2) {
                    add_segment_to_disk(hyperbolic_center, inter.front(), hyperbolic_voronoi_mesh, voronoi_color);
                } else {
                    add_segment_to_disk(hyperbolic_center, inter.back(), hyperbolic_voronoi_mesh, voronoi_color);
                }
            }
        };

        for (auto &[edge, adj] : adjacent_points) {
            if (not marked_edges.contains(edge))
                continue;
            if (adj.size() == 2) {
                auto euclidean_center1 =
                    get_circumcenter_2d(points[edge.first], points[edge.second], points[adj.front()]);
                auto r1 = glm::distance(euclidean_center1, points[edge.first]);
                auto hyperbolic_center1 = get_hyperbolic_circumcenter(euclidean_center1, r1);

                auto euclidean_center2 =
                    get_circumcenter_2d(points[edge.first], points[edge.second], points[adj.back()]);
                auto r2 = glm::distance(euclidean_center2, points[edge.first]);
                auto hyperbolic_center2 = get_hyperbolic_circumcenter(euclidean_center2, r2);

                bool is_hyperbolic1 =
                    is_hyperbolic_triangle(points[edge.first], points[edge.second], points[adj.front()]);
                bool is_hyperbolic2 =
                    is_hyperbolic_triangle(points[edge.first], points[edge.second], points[adj.back()]);

                if (not is_hyperbolic1 and not is_hyperbolic2) {
                    add_bisector_free(points[edge.first], points[edge.second]);
                    continue;
                }

                if (not is_hyperbolic1) {
                    add_bisector_one_endpoint(points[edge.first], points[edge.second], points[adj.back()],
                                              hyperbolic_center2);
                    continue;
                }

                if (not is_hyperbolic2) {
                    add_bisector_one_endpoint(points[edge.first], points[edge.second], points[adj.front()],
                                              hyperbolic_center1);
                    continue;
                }

                if (glm::length(hyperbolic_center1) > 1 or glm::length(hyperbolic_center2) > 1) {
                    spdlog::info("Hyperbolic Voronoi compute incorrect length");
                }

                add_segment_to_disk(hyperbolic_center1, hyperbolic_center2, hyperbolic_voronoi_mesh, voronoi_color);
            } else if (adj.size() == 1) {
                bool is_hyperbolic1 =
                    is_hyperbolic_triangle(points[edge.first], points[edge.second], points[adj.front()]);

                if (is_hyperbolic1) {
                    auto euclidean_center1 =
                        get_circumcenter_2d(points[edge.first], points[edge.second], points[adj.front()]);
                    auto r1 = glm::distance(euclidean_center1, points[edge.first]);
                    auto hyperbolic_center1 = get_hyperbolic_circumcenter(euclidean_center1, r1);

                    add_bisector_one_endpoint(points[edge.first], points[edge.second], points[adj.front()],
                                              hyperbolic_center1);
                } else {
                    add_bisector_free(points[edge.first], points[edge.second]);
                }
            }
        }

        context.sceneTree->addGeometryNodeAsync(std::move(hyperbolic_voronoi_mesh), "Hyperbolic Voronoi Diagram");

        PointCloud2D pc;
        for (auto p : points) {
            pc.vertices.push_back({{p.x, p.y}, {1.0, 0.0, 0.0}, {1.0, 0.0, 1.0}});
        }
        context.sceneTree->addGeometryNodeAsync(std::move(pc), "Site Points");

        /**
         * Visualize the disk boundary
         */

        CurveMesh2D circle;

        int circle_segment_num = 100;
        for (int i = 0; i <= circle_segment_num; i++) {
            float param = 2 * std::numbers::pi / circle_segment_num * i;

            float x = std::cos(param);
            float y = std::sin(param);

            int id = circle.vertices.size();
            circle.vertices.push_back({{x, y}, {0.0, 0.0, 0.0}});
            if (i > 0) {
                circle.indices.emplace_back(id - 1, id);
            }
        }
        context.sceneTree->addGeometryNodeAsync(std::move(circle), "Circle");
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
        context.sceneTree->addGeometryNodeAsync(std::move(pc), "Vert");
        context.sceneTree->addGeometryNodeAsync(std::move(curve_mesh), "tessellation");
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

        context.sceneTree->addGeometryNodeAsync(std::move(mesh), "convex hull");
        context.sceneTree->addGeometryNodeAsync(std::move(spherical_mesh), "delaunay result");
        context.sceneTree->addGeometryNodeAsync(std::move(voronoi_spherical_mesh), "voronoi result");
        context.sceneTree->addGeometryNodeAsync(std::move(voronoi_mesh), "voronoi");
        context.sceneTree->addGeometryNodeAsync(std::move(delaunay_mesh), "delaunay");
        context.sceneTree->addGeometryNodeAsync(std::move(pc), "test points");
        context.sceneTree->addGeometryNodeAsync(std::move(pc2), "pc 2");

        auto delaunay_result = context.sceneTree->get_geometry_node<Mesh3D>("delaunay result");
        delaunay_result->visible = false;
    }

    std::unique_ptr<GraphicsLab::RenderGraph::HyperbolicDiskRenderPass> hyperbolic_disk_render_pass = nullptr;
    ProjectUIState ui_state;
    std::unique_ptr<CustomController> controller = nullptr;

    ReflectDataType reflect() override;
};