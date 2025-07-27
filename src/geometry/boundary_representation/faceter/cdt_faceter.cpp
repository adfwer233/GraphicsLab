#include "geometry/boundary_representation/faceter/cdt_faceter.hpp"
#include "CDT.h"
#include "geometry/boundary_representation/topology/topology_utils.hpp"
#include "geometry/boundary_representation/topology/trimming_utils.hpp"

Mesh3D GraphicsLab::Geometry::BRep::CDTFaceter::naive_facet(Face *face, int n, int m) {
    using PointType = glm::vec<2, double>;

    std::vector<PointType> points;
    std::vector<std::pair<size_t, size_t>> edges;

    auto is_in_domain = [](const PointType &point) -> bool {
        double eps = 1e-8;
        return point.x >= -eps && point.y >= -eps && point.x <= 1 + eps && point.y <= 1 + eps;
    };

    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= m; j++) {
            double param = static_cast<double>(i) / static_cast<double>(n);
            double param2 = static_cast<double>(j) / static_cast<double>(m);

            points.emplace_back(param, param2);
        }
    }

    int curve_sample_num = 25;

    int u_repeat = 0;
    if (face->geometry()->param_geometry()->u_periodic)
        u_repeat = 1;

    for (int u = -u_repeat; u <= u_repeat; u++) {
        PointType offset{u, 0};
        for (auto loop : TopologyUtils::get_all_loops(face)) {
            for (auto coedge : TopologyUtils::get_all_coedges(loop)) {
                auto param_curve = coedge->geometry()->param_geometry();

                for (int i = 0; i <= curve_sample_num; i++) {
                    double param = static_cast<double>(i) / static_cast<double>(curve_sample_num);
                    auto par_pos = param_curve->evaluate(param) + offset;

                    par_pos = glm::clamp(par_pos, 0.0, 1.0);
                    points.push_back(par_pos);
                    if (i > 0) {
                        edges.emplace_back(points.size() - 2, points.size() - 1);
                    }
                }
            }
        }
    }

    auto areClose = [](const PointType &a, const PointType &b, double epsilon) {
        return std::fabs(a.x - b.x) < epsilon && std::fabs(a.y - b.y) < epsilon;
    };

    double epsilon = 1e-8;
    std::vector<PointType> uniquePoints;
    std::unordered_map<size_t, size_t> indexMap;

    // Sort points by x and y for efficient deduplication
    std::vector<size_t> sortedIndices(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        sortedIndices[i] = i;

    std::ranges::sort(sortedIndices, [&](size_t i, size_t j) {
        return (points[i].x < points[j].x) || (points[i].x == points[j].x && points[i].y < points[j].y);
    });

    // Deduplicate points and create the index map
    for (size_t i = 0; i < sortedIndices.size(); ++i) {
        size_t idx = sortedIndices[i];
        if (uniquePoints.empty() || !areClose(uniquePoints.back(), points[idx], epsilon)) {
            uniquePoints.push_back(points[idx]);
            indexMap[idx] = uniquePoints.size() - 1; // Map old index to new index
        } else {
            indexMap[idx] = indexMap[sortedIndices[i - 1]]; // Map to already existing unique point
        }
    }

    // Update the points vector with unique points
    points = std::move(uniquePoints);

    // Update the edges to use the new indices
    for (auto &edge : edges) {
        edge.first = indexMap[edge.first];
        edge.second = indexMap[edge.second];
    }

    spdlog::debug("CDT begin, points num {}, edge num {}", points.size(), edges.size());
    // CDT::Triangulation<double> cdt;
    CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::TryResolve, 1e-8);
    cdt.insertVertices(
        points.begin(), points.end(), [](const auto &p) { return p.x; }, [](const auto &p) { return p.y; });
    cdt.insertEdges(
        edges.begin(), edges.end(), [](const auto &e) { return e.first; }, [](const auto &e) { return e.second; });
    spdlog::debug("CDT start culling");
    cdt.eraseSuperTriangle();
    spdlog::debug("CDT end");

    spdlog::debug("CDT input vertices {}, find vertices {}", points.size(), cdt.vertices.size());

    if (points.size() < cdt.vertices.size()) {
        points.clear();
        for (auto vert : cdt.vertices) {
            points.emplace_back(vert.x, vert.y);
        }
    }

    std::vector<TriangleIndex> triangles;
    spdlog::debug("Winding Number Culling begin");
    for (auto &tri : cdt.triangles) {
        auto barycenter = (points[tri.vertices[0]] + points[tri.vertices[1]] + points[tri.vertices[2]]) / 3.0;
        if (is_in_domain(barycenter) and is_in_domain(points[tri.vertices[0]]) and
            is_in_domain(points[tri.vertices[1]]) and is_in_domain(points[tri.vertices[2]])) {
            if (ContainmentQuery::contained(face, barycenter) == ContainmentQuery::ContainmentResult::Inside) {
                triangles.emplace_back(tri.vertices[0], tri.vertices[1], tri.vertices[2]);
            }
        }
    }
    spdlog::debug("Winding Number Culling end, {} triangle, remain {}", cdt.triangles.size(), triangles.size());

    Mesh3D mesh;
    mesh.indices = triangles;

    for (auto &p : cdt.vertices) {
        Vertex3D vertex;
        vertex.position = face->geometry()->param_geometry()->evaluate({p.x, p.y});
        vertex.normal = face->geometry()->param_geometry()->normal({p.x, p.y});

        if (not face->is_forward()) {
            vertex.normal = -vertex.normal;
        }

        mesh.vertices.push_back(vertex);
    }

    return mesh;
}