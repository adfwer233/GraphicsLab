#pragma once

#include "glm/glm.hpp"

namespace VklBVHGPUModel {

struct Triangle {
    alignas(16) glm::vec3 v0;
    alignas(16) glm::vec3 v1;
    alignas(16) glm::vec3 v2;
    alignas(4) uint32_t materialIndex;
};

struct BVHNode {
    alignas(16) glm::vec3 min;
    alignas(16) glm::vec3 max;
    alignas(4) int leftNodeIndex = -1;
    alignas(4) int rightNodeIndex = -1;
    alignas(4) int objectIndex = -1;
};

struct Light {
    alignas(4) uint32_t triangleIndex;
    alignas(4) float area;
};
} // namespace VklBVHGPUModel