#pragma once

#include "glm/glm.hpp"

struct Vertex2D {
    alignas(8) glm::vec2 position{};
    glm::vec3 color{};
    glm::vec3 normal{};
    glm::vec2 uv{};
};

struct Vertex2DRaw {
    glm::vec2 position{};
};

struct LineIndex {
    uint32_t i, j;
    static constexpr size_t vertexCount = 2;
};
