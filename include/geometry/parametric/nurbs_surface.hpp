#pragma once

#include "glm/glm.hpp"
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
        assert(is_valid());
    }

    PointType evaluate(ParamType param) const override {
        T u = param.x * knot_vector_u.back();
        T v = param.y * knot_vector_v.back();
        assert(is_valid());
        assert(u >= knot_vector_u[degree_u - 1] && u <= knot_vector_u[knot_vector_u.size() - degree_u]);
        assert(v >= knot_vector_v[degree_v - 1] && v <= knot_vector_v[knot_vector_v.size() - degree_v]);

        T denominator = 0;
        PointType numerator(0);

        for (size_t i = 0; i < control_points.size(); ++i) {
            for (size_t j = 0; j < control_points[i].size(); ++j) {
                T basis_u = basis_function(i, degree_u, u, knot_vector_u);
                T basis_v = basis_function(j, degree_v, v, knot_vector_v);
                T weight = weights[i][j];

                T combined_weight = basis_u * basis_v * weight;
                numerator += combined_weight * control_points[i][j];
                denominator += combined_weight;
            }
        }

        return numerator / denominator;
    }

    PointType normal(ParamType param) const override {
        T u = param.x * knot_vector_u.back();
        T v = param.y * knot_vector_v.back();

        // Compute partial derivatives
        PointType du = partial_derivative_u(u, v);
        PointType dv = partial_derivative_v(u, v);

        // Compute the normal as the cross product of the partial derivatives
        PointType n = glm::cross(du, dv);

        // Normalize the resulting normal vector
        return glm::normalize(n);
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
    PointType partial_derivative_u(T u, T v) const {
        assert(is_valid());
        T denominator = 0;
        PointType numerator(0);
        T denominator_derivative = 0;
        PointType numerator_derivative(0);

        for (size_t i = 0; i < control_points.size(); ++i) {
            for (size_t j = 0; j < control_points[i].size(); ++j) {
                T basis_u = basis_function(i, degree_u, u, knot_vector_u);
                T basis_v = basis_function(j, degree_v, v, knot_vector_v);
                T basis_u_derivative = basis_function_derivative(i, degree_u, u, knot_vector_u);
                T weight = weights[i][j];

                T combined_weight = basis_u * basis_v * weight;
                T combined_weight_derivative = basis_u_derivative * basis_v * weight;

                numerator += combined_weight * control_points[i][j];
                denominator += combined_weight;

                numerator_derivative += combined_weight_derivative * control_points[i][j];
                denominator_derivative += combined_weight_derivative;
            }
        }

        return (numerator_derivative * denominator - numerator * denominator_derivative) / (denominator * denominator);
    }

    PointType partial_derivative_v(T u, T v) const {
        assert(is_valid());
        T denominator = 0;
        PointType numerator(0);
        T denominator_derivative = 0;
        PointType numerator_derivative(0);

        for (size_t i = 0; i < control_points.size(); ++i) {
            for (size_t j = 0; j < control_points[i].size(); ++j) {
                T basis_u = basis_function(i, degree_u, u, knot_vector_u);
                T basis_v = basis_function(j, degree_v, v, knot_vector_v);
                T basis_v_derivative = basis_function_derivative(j, degree_v, v, knot_vector_v);
                T weight = weights[i][j];

                T combined_weight = basis_u * basis_v * weight;
                T combined_weight_derivative = basis_u * basis_v_derivative * weight;

                numerator += combined_weight * control_points[i][j];
                denominator += combined_weight;

                numerator_derivative += combined_weight_derivative * control_points[i][j];
                denominator_derivative += combined_weight_derivative;
            }
        }

        return (numerator_derivative * denominator - numerator * denominator_derivative) / (denominator * denominator);
    }

    // Derivative of the basis function
    T basis_function_derivative(size_t i, int d, T t, const std::vector<T> &knot_vector) const {
        if (d == 0) {
            return 0;
        }

        T left = (t - knot_vector[i]);
        T right = (knot_vector[i + d] - knot_vector[i]);
        T left_term = (right == 0) ? 0 : basis_function(i, d - 1, t, knot_vector) / right;

        left = (knot_vector[i + d + 1] - t);
        right = (knot_vector[i + d + 1] - knot_vector[i + 1]);
        T right_term = (right == 0) ? 0 : basis_function(i + 1, d - 1, t, knot_vector) / right;

        return d * (left_term - right_term);
    }

    T basis_function(size_t i, int d, T t, const std::vector<T> &knot_vector) const {
        if (d == 0) {
            return (t >= knot_vector[i] && t <= knot_vector[i + 1]) ? 1.0 : 0.0;
        }

        T left = (t - knot_vector[i]);
        T right = (knot_vector[i + d] - knot_vector[i]);
        T left_term = (std::abs(right) <= 1e-6) ? 0 : left / right * basis_function(i, d - 1, t, knot_vector);

        left = (knot_vector[i + d + 1] - t);
        right = (knot_vector[i + d + 1] - knot_vector[i + 1]);
        T right_term = (std::abs(right) <= 1e-6) ? 0 : left / right * basis_function(i + 1, d - 1, t, knot_vector);
        // spdlog::info("N {} {} = {}", i, d, left_term + right_term);

        return left_term + right_term;
    }

    bool is_valid() const {
        size_t num_control_points_u = control_points.size();
        size_t num_weights_u = weights.size();

        if (num_control_points_u == 0 || num_weights_u == 0 || control_points[0].size() != weights[0].size()) {
            return false;
        }

        size_t num_control_points_v = control_points[0].size();
        size_t num_weights_v = weights[0].size();
        size_t num_knots_u = knot_vector_u.size();
        size_t num_knots_v = knot_vector_v.size();

        return num_control_points_u > degree_u && num_control_points_v > degree_v &&
               num_weights_u == num_control_points_u && num_weights_v == num_control_points_v &&
               num_knots_u == num_control_points_u + degree_u + 1 && num_knots_v == num_control_points_v + degree_v + 1;
    }
};

} // namespace GraphicsLab::Geometry