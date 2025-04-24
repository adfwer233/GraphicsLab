#pragma once

#include <memory>

#include "spdlog/spdlog.h"

#include "geometry/mesh/mesh.hpp"
#include "geometry/vertex/vertex.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Tessellator {
    static void tessellate(ParamSurface &surface, const int n = 32, const int m = 32) {
        surface.mesh = std::make_unique<Mesh3D>();

        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= m; j++) {
                double param_u = static_cast<double>(i) / static_cast<double>(n);
                double param_v = static_cast<double>(j) / static_cast<double>(m);

                Mesh3D::vertex_type vertex;
                vertex.position = surface.evaluate({param_u, param_v});
                vertex.color = {0, 1, 0};
                vertex.normal = surface.normal({param_u, param_v});
                surface.mesh->vertices.push_back(vertex);
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                uint32_t idx1 = i * (n + 1) + j;
                uint32_t idx2 = i * (n + 1) + j + 1;
                uint32_t idx3 = (i + 1) * (n + 1) + j;
                uint32_t idx4 = (i + 1) * (n + 1) + j + 1;

                surface.mesh->indices.push_back({idx1, idx2, idx4});
                surface.mesh->indices.push_back({idx1, idx4, idx3});
            }
        }

        spdlog::info("Tessellator tessellate() finish");
    }
};

} // namespace GraphicsLab::Geometry