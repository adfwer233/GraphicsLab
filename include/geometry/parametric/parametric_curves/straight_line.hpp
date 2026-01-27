#pragma once

#include "glm/glm.hpp"
#include "parametric_curve.hpp"

namespace GraphicsLab::Geometry {

template <size_t dim> struct StraightLineBase : ParamCurveBase<dim> {
    using PointType = std::conditional_t<dim == 2, glm::dvec2, glm::dvec3>;

    PointType start_point, end_point;

    PointType evaluate(double t) const override {
        return glm::mix(start_point, end_point, t);
    }

    PointType derivative(double) const override {
        return end_point - start_point;
    }

    PointType second_derivative(double) const override {
        return PointType(0);
    }

    virtual bool is_closed() const {
        return false;
    }

    virtual std::pair<PointType, double> projection(PointType test_point, std::optional<double> param_guess) const {
        auto start_to_test = test_point - start_point;
        auto start_to_end = end_point - start_point;
        double param = glm::dot(start_to_test, start_to_end) / glm::length(start_to_end);
        param = glm::clamp(param / glm::length(start_to_end), 0.0, 1.0);
        return {evaluate(param), param};
    }

    PointType normal(double t) const
        requires(dim == 2)
    {
        auto deriv = derivative(t);
        return {-deriv.y, deriv.x};
    }
    StraightLineBase() = default;

    StraightLineBase(const PointType &start, const PointType &end) {
        start_point = start;
        end_point = end;

        this->box.max = glm::max(end_point, start_point);
        this->box.min = glm::min(end_point, start_point);
    }
};

using StraightLine2D = StraightLineBase<2>;
using StraightLine3D = StraightLineBase<3>;

} // namespace GraphicsLab::Geometry