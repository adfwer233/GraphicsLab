#pragma once

#include "geometry/mesh/mesh.hpp"

namespace GraphicsLab::Geometry {

struct SimpleDirectionalFieldConstructor {
    static DirectionalField3D create() {
        glm::vec3 base = glm::vec3(0, 0, 0);
        glm::vec3 x_axis = glm::vec3(1, 0, 0);
        glm::vec3 y_axis = glm::vec3(0, 1, 0);
        glm::vec3 z_axis = glm::vec3(0, 0, 1);

        int m = 64;
        int n = 64;
        int l = 64;

        DirectionalField3D result;

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < l; k++) {
                    DirectionalFieldVertex3D vertex;
                    float x = static_cast<float>(i) / static_cast<float>(m - 1);
                    float y = static_cast<float>(j) / static_cast<float>(n - 1);
                    float z = static_cast<float>(k) / static_cast<float>(l - 1);
                    vertex.position = base + x * x_axis + y * y_axis + z * z_axis;
                    vertex.direction = glm::vec3(1, 0, 0);
                    vertex.color = glm::vec3(0, 0, 1);
                    vertex.values = glm::vec3(0, 0, 0);
                    result.vertices.push_back(vertex);
                }
            }
        }

        return result;
    }
};

}