#pragma once

#include "geometry/mesh/mesh.hpp"

struct Box3DConstructor {
    static Wire3D create(glm::vec3 min_pos, glm::vec3 max_pos) {
        Wire3D mesh;

        glm::vec3 dir_x = {max_pos.x - min_pos.x, 0, 0};
        glm::vec3 dir_y = {0, max_pos.y - min_pos.y, 0};
        glm::vec3 dir_z = {0, 0, max_pos.z - min_pos.z};

        mesh.vertices.push_back(Vertex3DRaw{min_pos});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_x});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_y});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_x + dir_y});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_z});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_x + dir_z});
        mesh.vertices.push_back(Vertex3DRaw{min_pos + dir_y + dir_z});
        mesh.vertices.push_back(Vertex3DRaw{max_pos});

        mesh.indices.push_back({0, 1});
        mesh.indices.push_back({1, 3});
        mesh.indices.push_back({3, 2});
        mesh.indices.push_back({2, 0});

        mesh.indices.push_back({4, 5});
        mesh.indices.push_back({5, 7});
        mesh.indices.push_back({7, 6});
        mesh.indices.push_back({6, 4});

        mesh.indices.push_back({0, 4});
        mesh.indices.push_back({1, 5});
        mesh.indices.push_back({2, 6});
        mesh.indices.push_back({3, 7});

        return mesh;
    }
};