#pragma once

#include "glm/glm.hpp"
#include "parametric_curve.hpp"

#include <numeric/polynomial/polynomial_solver.hpp>
#include <numeric/polynomial/real_polynomial.hpp>

namespace GraphicsLab::Geometry {

template <size_t dim> struct Ellipse final : ParamCurveBase<dim> {
    using PointType = glm::vec<dim, double>;

    explicit Ellipse(PointType center, PointType major_radius, PointType minor_radius)
        : center(center), major_radius(major_radius), minor_radius(minor_radius) {
        if (glm::length(major_radius) < glm::length(minor_radius)) {
            spdlog::warn("[Ellipse constructor]: the length of major axis is smaller than the minor axis");
        }
    }

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

    std::pair<PointType, double> projection(PointType test_point, std::optional<double> param_guess) const override {
        PointType xaxis = glm::normalize(major_radius);
        PointType yaxis = glm::normalize(minor_radius);
        double a = glm::length(major_radius);
        double b = glm::length(minor_radius);

        double x = glm::dot(xaxis, test_point - center);
        double y = glm::dot(yaxis, test_point - center);

        double A = b * y;
        double B = 2 * (a * x - (b * b - a * a));
        double C = 0;
        double D = 2 * (a * x + (b * b - a * a));
        double E = -b * y;

        RealPolynomial poly{E, D, C, B, A};
        auto roots = Numeric::QuarticPolynomialSolver::solve(poly);

        double best_dist2 = std::numeric_limits<double>::infinity();
        double best_theta = 0.0;

        auto eval_theta = [&](double theta) {
            // normalize to [0, 2π)
            theta = std::fmod(theta, 2.0 * std::numbers::pi);
            if (theta < 0.0)
                theta += 2.0 * std::numbers::pi;

            double cx = a * std::cos(theta);
            double cy = b * std::sin(theta);

            double dx = cx - x;
            double dy = cy - y;
            double d2 = dx * dx + dy * dy;

            if (d2 < best_dist2) {
                best_dist2 = d2;
                best_theta = theta;
            }
        };

        // evaluate quartic roots
        for (const auto &r : roots) {
            double theta = 2.0 * std::atan(r);
            eval_theta(theta);
        }

        // fallback / safety extrema
        eval_theta(0.0);
        eval_theta(std::numbers::pi / 2);
        eval_theta(std::numbers::pi);
        eval_theta(3.0 * std::numbers::pi / 2);

        PointType proj = center + (a * std::cos(best_theta)) * xaxis + (b * std::sin(best_theta)) * yaxis;

        return {proj, best_theta / (2 * std::numbers::pi)};
    }

    PointType center;
    PointType major_radius, minor_radius;
};

using Ellipse2D = Ellipse<2>;
using Ellipse3D = Ellipse<3>;

} // namespace GraphicsLab::Geometry