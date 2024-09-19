#pragma once

#include "geometry/mesh/mesh.hpp"

struct RectangleConstructor {
    static Mesh3D create(glm::vec3 pos, glm::vec3 dir1, glm::vec3 dir2, uint32_t n = 100, uint32_t m = 100) {
        Mesh3D mesh;

        dir1 /= n;
        dir2 /= m;

        auto norm = glm::normalize(glm::cross(dir1, dir2));

        for (int i = 0; i <= n; i++) {
            for (int j = 0; j <= m; j++) {
                Mesh3D::vertex_type vertex;
                vertex.position = pos + dir1 * float(i) + dir2 * float(j);
                vertex.color = {0, 1, 0};
                vertex.normal = norm;
                mesh.vertices.push_back(vertex);
            }
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                auto idx1 = i * (n + 1) + j;
                auto idx2 = i * (n + 1) + j + 1;
                auto idx3 = (i + 1) * (n + 1) + j;
                auto idx4 = (i + 1) * (n + 1) + j + 1;

                mesh.indices.push_back({idx1, idx2, idx4});
                mesh.indices.push_back({idx1, idx4, idx3});
            }
        }

        return mesh;
    }
};