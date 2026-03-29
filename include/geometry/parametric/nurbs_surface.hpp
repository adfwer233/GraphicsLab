#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "glm/glm.hpp"
#include "geometry/parametric/configuration.hpp"
#include "parametric_space.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {

struct NURBSSurface : public ParamSurface {
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    using PointType = glm::vec<3, T>;
    using ParamType = glm::vec<2, T>;

    std::vector<std::vector<PointType>> control_points;
    std::vector<T> knot_vector_u;
    std::vector<T> knot_vector_v;
    std::vector<std::vector<T>> weights;

    int order_u = 0;
    int degree_u = 0;
    int order_v = 0;
    int degree_v = 0;

    NURBSSurface(std::vector<std::vector<PointType>> ctrl_pts, std::vector<T> knots_u, std::vector<T> knots_v,
                 std::vector<std::vector<T>> wts, const int degree_u, const int degree_v)
        : control_points(std::move(ctrl_pts)), knot_vector_u(std::move(knots_u)), knot_vector_v(std::move(knots_v)),
          weights(std::move(wts)), degree_u(degree_u), degree_v(degree_v) {
        order_u = degree_u + 1;
        order_v = degree_v + 1;
        assert(is_valid());
    }

    [[nodiscard]] PointType evaluate(ParamType param) const override {
        const auto [numerator, denominator] = evaluate_weighted(param);
        return numerator / denominator;
    }

    [[nodiscard]] PointType normal(ParamType param) const override {
        auto [du, dv] = derivative(param);
        const PointType n = glm::cross(du, dv);
        const T length = glm::length(n);
        if (length <= kTolerance) {
            return PointType(0.0);
        }
        return n / length;
    }

    [[nodiscard]] std::pair<VectorType, VectorType> derivative(const ParamType param) const override {
        const auto [u, v] = map_to_knot_domain(param);

        PointType a(0.0);
        PointType a_u(0.0);
        PointType a_v(0.0);
        T w = 0.0;
        T w_u = 0.0;
        T w_v = 0.0;

        for (size_t i = 0; i < control_points.size(); ++i) {
            const T nu = basis_function(i, degree_u, u, knot_vector_u);
            const T dnu = basis_function_derivative(i, degree_u, u, knot_vector_u);

            for (size_t j = 0; j < control_points[i].size(); ++j) {
                const T nv = basis_function(j, degree_v, v, knot_vector_v);
                const T dnv = basis_function_derivative(j, degree_v, v, knot_vector_v);
                const T wij = weights[i][j];

                const T base = wij * nu * nv;
                const T base_u = wij * dnu * nv;
                const T base_v = wij * nu * dnv;

                a += base * control_points[i][j];
                a_u += base_u * control_points[i][j];
                a_v += base_v * control_points[i][j];

                w += base;
                w_u += base_u;
                w_v += base_v;
            }
        }

        if (std::abs(w) <= kTolerance) {
            throw std::runtime_error("NURBS surface derivative encountered near-zero weight denominator.");
        }

        const PointType s_u = (a_u * w - a * w_u) / (w * w);
        const PointType s_v = (a_v * w - a * w_v) / (w * w);
        return {s_u, s_v};
    }

    std::pair<PointType, ParamType> project(const PointType point) const override {
        // @todo: implement project function
        return {point, {0.0, 0.0}};
    }

    bool test_point(const PointType point) const override {
        // @todo: implement test point function
        return false;
    }

  private:
    static constexpr T kTolerance = ParametricConfiguration::system_tolerance;

    [[nodiscard]] std::pair<PointType, T> evaluate_weighted(const ParamType param) const {
        const auto [u, v] = map_to_knot_domain(param);

        PointType numerator(0.0);
        T denominator = 0.0;

        for (size_t i = 0; i < control_points.size(); ++i) {
            const T nu = basis_function(i, degree_u, u, knot_vector_u);
            for (size_t j = 0; j < control_points[i].size(); ++j) {
                const T nv = basis_function(j, degree_v, v, knot_vector_v);
                const T combined_weight = nu * nv * weights[i][j];

                numerator += combined_weight * control_points[i][j];
                denominator += combined_weight;
            }
        }

        if (std::abs(denominator) <= kTolerance) {
            throw std::runtime_error("NURBS surface evaluation encountered near-zero weight denominator.");
        }

        return {numerator, denominator};
    }

    [[nodiscard]] std::pair<T, T> map_to_knot_domain(ParamType param) const {
        param = move_param_to_std_domain(param);
        const T u_start = knot_vector_u[static_cast<size_t>(degree_u)];
        const T u_end = knot_vector_u[knot_vector_u.size() - static_cast<size_t>(degree_u) - 1];
        const T v_start = knot_vector_v[static_cast<size_t>(degree_v)];
        const T v_end = knot_vector_v[knot_vector_v.size() - static_cast<size_t>(degree_v) - 1];

        T u = u_start + std::clamp(param.x, T(0.0), T(1.0)) * (u_end - u_start);
        T v = v_start + std::clamp(param.y, T(0.0), T(1.0)) * (v_end - v_start);

        if (std::abs(u - u_end) <= kTolerance) {
            u = std::nextafter(u_end, u_start);
        }
        if (std::abs(v - v_end) <= kTolerance) {
            v = std::nextafter(v_end, v_start);
        }

        return {u, v};
    }

    // Derivative of the basis function
    T basis_function_derivative(size_t i, int d, T t, const std::vector<T> &knot_vector) const {
        if (d == 0) {
            return 0.0;
        }

        const T left_denom = knot_vector[i + static_cast<size_t>(d)] - knot_vector[i];
        const T right_denom = knot_vector[i + static_cast<size_t>(d) + 1] - knot_vector[i + 1];

        T left = 0.0;
        T right = 0.0;
        if (std::abs(left_denom) > kTolerance) {
            left = static_cast<T>(d) * basis_function(i, d - 1, t, knot_vector) / left_denom;
        }
        if (std::abs(right_denom) > kTolerance) {
            right = static_cast<T>(d) * basis_function(i + 1, d - 1, t, knot_vector) / right_denom;
        }

        return left - right;
    }

    T basis_function(size_t i, int d, T t, const std::vector<T> &knot_vector) const {
        if (d == 0) {
            return (t >= knot_vector[i] && t < knot_vector[i + 1]) ? 1.0 : 0.0;
        }

        const T left_denom = knot_vector[i + static_cast<size_t>(d)] - knot_vector[i];
        const T right_denom = knot_vector[i + static_cast<size_t>(d) + 1] - knot_vector[i + 1];

        T left_term = 0.0;
        T right_term = 0.0;
        if (std::abs(left_denom) > kTolerance) {
            left_term = (t - knot_vector[i]) / left_denom * basis_function(i, d - 1, t, knot_vector);
        }
        if (std::abs(right_denom) > kTolerance) {
            right_term =
                (knot_vector[i + static_cast<size_t>(d) + 1] - t) / right_denom * basis_function(i + 1, d - 1, t, knot_vector);
        }
        return left_term + right_term;
    }

    bool is_valid() const {
        if (degree_u < 0 || degree_v < 0) {
            return false;
        }
        const size_t num_control_points_u = control_points.size();
        if (num_control_points_u == 0 || weights.size() != num_control_points_u) {
            return false;
        }

        const size_t num_control_points_v = control_points[0].size();
        if (num_control_points_v == 0 || num_control_points_u <= static_cast<size_t>(degree_u) ||
            num_control_points_v <= static_cast<size_t>(degree_v)) {
            return false;
        }

        for (size_t i = 0; i < num_control_points_u; ++i) {
            if (control_points[i].size() != num_control_points_v || weights[i].size() != num_control_points_v) {
                return false;
            }
            for (const T w : weights[i]) {
                if (w <= 0.0) {
                    return false;
                }
            }
        }

        const size_t expected_knots_u = num_control_points_u + static_cast<size_t>(degree_u) + 1;
        const size_t expected_knots_v = num_control_points_v + static_cast<size_t>(degree_v) + 1;
        if (knot_vector_u.size() != expected_knots_u || knot_vector_v.size() != expected_knots_v) {
            return false;
        }
        if (!std::is_sorted(knot_vector_u.begin(), knot_vector_u.end()) ||
            !std::is_sorted(knot_vector_v.begin(), knot_vector_v.end())) {
            return false;
        }

        return true;
    }
};

} // namespace GraphicsLab::Geometry