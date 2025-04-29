#pragma once

#include <glm/glm.hpp>

namespace GraphicsLab::Geometry {

struct CurveCurveIntersectionResult2D {
    using PointType = glm::dvec2;

    std::vector<PointType> inter_points;
    std::vector<double> curve1_param;
    std::vector<double> curve2_param;
};

} // namespace GraphicsLab::Geometry