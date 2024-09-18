#pragma once

#include "glm/glm.hpp"
#include "language/reflection/static_reflector.hpp"

struct Vertex2D {
    alignas(8) glm::vec2 position{};
    glm::vec3 color{};
    glm::vec3 normal{};
    glm::vec2 uv{};

    REFLECT(
            Property{"position", &Vertex2D::position},
            Property{"color", &Vertex2D::color},
            Property{"normal", &Vertex2D::normal},
            Property{"uv", &Vertex2D::uv}
            )
};

struct Vertex2DRaw {
    glm::vec2 position{};

    REFLECT(
            Property{"position", &Vertex2DRaw::position}
            )
};

struct LineIndex {
    uint32_t i, j;
    static constexpr size_t vertexCount = 2;
};
