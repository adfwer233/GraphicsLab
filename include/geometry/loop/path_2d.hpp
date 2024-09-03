#pragma once

#include <vector>

#include "geometry/curve/bezier_curve_2d.hpp"

class Path2D {
  public:
    std::vector<std::unique_ptr<BezierCurve2D>> curves;

    float derivative_bound = -1.0;

    Path2D(std::vector<std::vector<std::array<float, 2>>> &&path_data);

    float winding_number(glm::vec2 test_point);

  private:
    float winding_number_internal(glm::vec2 test_point, size_t start_index, size_t end_index);
};