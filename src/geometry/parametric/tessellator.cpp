#include "geometry/parametric/tessellator.hpp"

#include <geometry/parametric_topology/brep_face.hpp>
#include <geometry/parametric_topology/brep_loop.hpp>
#include <geometry/parametric_topology/brep_coedge.hpp>

#include "CDT.h"

namespace GraphicsLab::Geometry {

void Tessellator::tessellate(ParamCurve3D &curve, const int n) {
    curve.mesh = std::make_unique<CurveMesh3D>();

    for (int i = 0; i <= n; i++) {
        double param = static_cast<double>(i) / static_cast<double>(n);

        CurveMesh3D::vertex_type vertex;
        vertex.position = curve.evaluate(param);
        vertex.color = {0.0, 0.0, 1.0};
        curve.mesh->vertices.push_back(vertex);
    }

    for (int i = 0; i < n; i++) {
        curve.mesh->indices.push_back({static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1)});
    }
}
void Tessellator::tessellate(ParamCurve2D &curve, const int n) {
    curve.mesh = std::make_unique<CurveMesh2D>();

    for (int i = 0; i <= n; i++) {
        double param = static_cast<double>(i) / static_cast<double>(n);

        CurveMesh2D::vertex_type vertex;
        vertex.position = curve.evaluate(param);
        vertex.color = {0.0, 0.0, 1.0};
        curve.mesh->vertices.push_back(vertex);
    }

    for (int i = 0; i < n; i++) {
        curve.mesh->indices.push_back({static_cast<uint32_t>(i), static_cast<uint32_t>(i + 1)});
    }
}

void Tessellator::tessellate(BRepFace *face) {
    int n = 64;
    int m = 64;

    using PointType = glm::vec<2, double>;

    std::vector<PointType> points;
    std::vector<std::pair<size_t, size_t>> edges;

    auto is_in_domain = [](const PointType& point) -> bool {
        double eps = 1e-8;
        return point.x >= -eps && point.y >= -eps && point.x <= 1 + eps && point.y <= 1 + eps;
    };

    for (int i = 0 ; i <= n; i++) {
        for (int j = 0; j <= m; j++) {
            double param = static_cast<double>(i) / static_cast<double>(n);
            double param2 = static_cast<double>(j) / static_cast<double>(m);

            points.emplace_back(param, param2);
        }
    }

    int curve_sample_num = 128;

    int u_repeat = 0;
    if (face->surface->u_periodic) u_repeat = 1;

    for (int u = -u_repeat; u <= u_repeat; u++) {
        PointType offset{u, 0};
        for (auto loop: face->boundary) {
            for (auto coedge: loop->coedges) {
                auto param_curve = coedge->geometry;

                for (int i = 0; i <= curve_sample_num; i++) {
                    double param = static_cast<double>(i) / static_cast<double>(curve_sample_num);
                    points.emplace_back(param_curve->evaluate(param) + offset);

                    if (i > 0) {
                        edges.emplace_back(points.size() - 2, points.size() - 1);
                    }
                }
            }
        }
    }

    spdlog::info("CDT begin, points num {}, edge num {}", points.size(), edges.size());
    // CDT::Triangulation<double> cdt;
    CDT::Triangulation<double> cdt(CDT::VertexInsertionOrder::Auto, CDT::IntersectingConstraintEdges::DontCheck, 1e-8);
    cdt.insertVertices(points.begin(), points.end(), [](const auto& p){ return p.x; }, [](const auto& p){ return p.y; });
    cdt.insertEdges(edges.begin(), edges.end(), [](const auto& e){ return e.first; }, [](const auto& e){ return e.second; });
    spdlog::info("CDT start culling");
    cdt.eraseSuperTriangle();
    spdlog::info("CDT end");

    std::vector<TriangleIndex> triangles;
    spdlog::info("Winding Number Culling begin");
    for (auto& tri: cdt.triangles) {
        auto barycenter = (points[tri.vertices[0]] + points[tri.vertices[1]] + points[tri.vertices[2]]) / 3.0;
        if (is_in_domain(barycenter) and is_in_domain(points[tri.vertices[0]]) and is_in_domain(points[tri.vertices[1]]) and is_in_domain(points[tri.vertices[2]]) ) {
            triangles.emplace_back(tri.vertices[0], tri.vertices[1], tri.vertices[2]);
        }
    }
    spdlog::info("Winding Number Culling end");

    face->mesh = std::make_unique<Mesh3D>();
    face->mesh->indices = triangles;
    face->mesh2d = std::make_unique<Mesh2D>();
    face->mesh2d->indices = triangles;

    for (auto &p: points) {
        Vertex3D vertex;
        vertex.position = face->surface->evaluate(p);
        vertex.normal = face->surface->normal(p);

        Vertex2D vertex2d;
        vertex2d.position = p;

        face->mesh->vertices.push_back(vertex);
        face->mesh2d->vertices.push_back(vertex2d);
    }
}

} // namespace GraphicsLab::Geometry