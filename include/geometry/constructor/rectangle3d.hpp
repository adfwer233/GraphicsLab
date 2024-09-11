#pragma once

#include "geometry/mesh/mesh.hpp"

struct RectangleConstructor {
    static Mesh3D create(glm::vec3 pos, glm::vec3 dir1, glm::vec3 dir2, uint32_t n = 100, uint32_t m = 100) {
        Mesh3D mesh;

        dir1 /= n;
        dir2 /= m;

        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= m; j++) {
                Mesh3D::vertex_type vertex;
                vertex.position = pos + dir1 * float(i) + dir2 * float(j);
                vertex.color = {0, 1, 0};
                mesh.vertices.push_back(vertex);
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                mesh.indices.push_back({i * n + j, i * n + j + 1, (i + 1) * n + j + 1});
                mesh.indices.push_back({i * n + j, (i + 1) * n + j + 1, (i + 1) * n + j});
            }
        }

        return mesh;
    }
};