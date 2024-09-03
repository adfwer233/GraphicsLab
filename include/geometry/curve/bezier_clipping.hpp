#pragma once

#include <optional>
#include <vector>

#include "glm/glm.hpp"

/**
 * @brief class to perform bezier clipping
 *
 * given a 2D bezier curve given by the control points and a test point, return the number of intersection points
 * between the curve and ray (u, v) -> (+inf, v)
 */
struct BezierClipping {
  public:
    static uint32_t bezier_clipping(glm::vec2 test_point, const std::vector<glm::vec2> &points);
};