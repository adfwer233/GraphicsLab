#pragma once

#include "glm/glm.hpp"
#include "parametric_curve.hpp"

namespace GraphicsLab::Geometry {

template<size_t dim>
struct StraightLineBase: ParamCurveBase<dim> {
    using PointType = glm::vec<dim, double>;

    PointType start_point, end_point;

    PointType evaluate(double t) const override {
        return glm::mix(start_point, end_point, t);
    }
};

}