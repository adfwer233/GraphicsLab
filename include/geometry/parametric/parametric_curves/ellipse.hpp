#pragma once

#include "glm/glm.hpp"
#include "parametric_curve.hpp"

namespace GraphicsLab::Geometry {

template<size_t dim> struct Ellipse: ParamCurveBase<dim> {
    using PointType = glm::vec<dim, double>;

    explicit Ellipse(PointType center, PointType major_radius, PointType minor_radius)
        : center(center), major_radius(major_radius), minor_radius(minor_radius) {}

    [[nodiscard]] PointType evaluate(double t) const override {
        const double theta = 2 * std::numbers::pi * t;
        return center + major_radius * std::cos(theta) + minor_radius * std::sin(theta);
    }

    [[nodiscard]] PointType derivative(double t) const override {
        const double theta = 2 * std::numbers::pi * t;
        return 2 * std::numbers::pi * (-major_radius * std::sin(theta) + minor_radius * std::cos(theta));
    }

    [[nodiscard]] PointType second_derivative(double t) const override {
        const double theta = 2 * std::numbers::pi * t;
        return std::pow(2 * std::numbers::pi, 2) * (-major_radius * std::cos(theta) - minor_radius * std::sin(theta));
    }

    [[nodiscard]] PointType normal(double t) const {
        return glm::normalize(center - evaluate(t));
    }

    PointType center;
    PointType major_radius, minor_radius;
};

using Ellipse2D = Ellipse<2>;
using Ellipse3D = Ellipse<3>;

}