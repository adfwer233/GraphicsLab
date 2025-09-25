#pragma once
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "parametric_curve.hpp"

namespace GraphicsLab::Geometry {

/**
 * @brief Curve degenerating to a simple point.
 * @tparam dim
 */
template <size_t dim> struct DegeneratedCurve : ParamCurveBase<dim> {
    using PointType = glm::vec<dim, double>;
    PointType point_;

    PointType evaluate(double t) const override {
        return point_;
    }

    PointType derivative(double) const override {
        return PointType(0);
    }

    PointType second_derivative(double) const override {
        return PointType(0);
    }

    PointType normal(double t) const
        requires(dim == 2)
    {
        return PointType(0);
    }
    DegeneratedCurve() = default;

    explicit DegeneratedCurve(const PointType &point) {
        point_ = point;
    }

    std::pair<PointType, double> projection(PointType test_point, std::optional<double> param_guess) const override {
        return {point_, 0};
    }
};

using DegeneratedCurve3D = DegeneratedCurve<3>;
using DegeneratedCurve2D = DegeneratedCurve<2>;

} // namespace GraphicsLab::Geometry