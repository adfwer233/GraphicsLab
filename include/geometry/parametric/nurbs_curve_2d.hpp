#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace GraphicsLab::Geometry {

struct NURBSCurve {
    using PointType = glm::vec<2, double>;
    struct SceneTreeGeometryTypeTrait {};

    std::vector<PointType> control_points;
    std::vector<size_t> knot_vector;
    std::vector<double> weights;

    int degree = 0;

    NURBSCurve(std::vector<PointType> ctrl_pts,
               std::vector<size_t> knots,
               std::vector<double> wts,
               int degree)
        : control_points(std::move(ctrl_pts)),
          knot_vector(std::move(knots)),
          weights(std::move(wts)),
          degree(degree) {
        assert(is_valid());
    }

    PointType evaluate(double param) const {
        assert(is_valid());
        assert(param >= knot_vector[degree] && param <= knot_vector[knot_vector.size() - degree - 1]);

        // Compute the denominator and weighted control points
        double denominator = 0;
        PointType numerator(0);

        for (size_t i = 0; i < control_points.size(); ++i) {
            double basis = basis_function(i, degree, param);
            double weight = basis * weights[i];

            numerator += weight * control_points[i];
            denominator += weight;
        }

        return numerator / denominator;
    }

    PointType derivative(double param) const {
        assert(is_valid());
        assert(param >= knot_vector[degree] && param <= knot_vector[knot_vector.size() - degree - 1]);

        double denominator = 0;
        PointType numerator(0);

        double denominator_derivative = 0;
        PointType numerator_derivative(0);

        for (size_t i = 0; i < control_points.size(); ++i) {
            double basis = basis_function(i, degree, param);
            double basis_derivative = basis_derivative_function(i, degree, param);

            double weight = weights[i];

            numerator += weight * basis * control_points[i];
            denominator += weight * basis;

            numerator_derivative += weight * basis_derivative * control_points[i];
            denominator_derivative += weight * basis_derivative;
        }

        double denom_squared = denominator * denominator;
        return (numerator_derivative * denominator - numerator * denominator_derivative) / denom_squared;
    }

    PointType start_point() const {
        return evaluate(knot_vector[degree]);
    }

    PointType end_point() const {
        return evaluate(knot_vector[knot_vector.size() - degree - 1]);
    }

private:
    double basis_function(size_t i, int d, double t) const {
        if (d == 0) {
            return (t >= knot_vector[i] && t < knot_vector[i + 1]) ? 1.0 : 0.0;
        }

        double left = (t - knot_vector[i]);
        double right = (knot_vector[i + d] - knot_vector[i]);
        double left_term = (right == 0) ? 0 : left / right * basis_function(i, d - 1, t);

        left = (knot_vector[i + d + 1] - t);
        right = (knot_vector[i + d + 1] - knot_vector[i + 1]);
        double right_term = (right == 0) ? 0 : left / right * basis_function(i + 1, d - 1, t);

        return left_term + right_term;
    }

    double basis_derivative_function(size_t i, int d, double t) const {
        if (d == 0) {
            return 0.0;
        }

        double left = (t - knot_vector[i]);
        double right = (knot_vector[i + d] - knot_vector[i]);
        double left_term = (right == 0) ? 0 : d / right * basis_function(i, d - 1, t);

        left = (knot_vector[i + d + 1] - t);
        right = (knot_vector[i + d + 1] - knot_vector[i + 1]);
        double right_term = (right == 0) ? 0 : d / right * basis_function(i + 1, d - 1, t);

        return left_term - right_term;
    }

    bool is_valid() const {
        size_t num_control_points = control_points.size();
        size_t num_weights = weights.size();
        size_t num_knots = knot_vector.size();

        return num_control_points > degree &&
               num_weights == num_control_points &&
               num_knots == num_control_points + degree + 1;
    }

    static void fit(std::vector<PointType> &points) {

    }
};

}