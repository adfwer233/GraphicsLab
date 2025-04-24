#include "geometry/parametric/tessellator.hpp"

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

} // namespace GraphicsLab::Geometry