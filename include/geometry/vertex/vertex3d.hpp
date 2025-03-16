#pragma once

#include "glm/glm.hpp"

struct Vertex3D {
    glm::vec3 position{};
    glm::vec3 color{};
    glm::vec3 normal{};
    glm::vec2 uv{};

    REFLECT(Property{"position", &Vertex3D::position}, Property{"color", &Vertex3D::color},
            Property{"normal", &Vertex3D::normal}, Property{"uv", &Vertex3D::uv})
};

struct Vertex3DRaw {
    glm::vec3 position{};

    REFLECT(Property{"position", &Vertex3D::position})
};

struct DirectionalFieldVertex3D {
    glm::vec3 position{};
    glm::vec3 color{};
    glm::vec3 direction{};
    glm::vec3 values{};

    REFLECT(Property{"position", &DirectionalFieldVertex3D::position},
            Property{"color", &DirectionalFieldVertex3D::color},
            Property{"direction", &DirectionalFieldVertex3D::direction},
            Property{"values", &DirectionalFieldVertex3D::values})
};

struct TriangleIndex {
    uint32_t i, j, k;
    static constexpr size_t vertexCount = 3;
};
