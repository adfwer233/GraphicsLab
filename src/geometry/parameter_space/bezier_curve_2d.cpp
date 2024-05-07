#include "geometry/parameter_space/bezier_curve_2d.hpp"

#include <set>

#define M_PI 3.14159265358979323846 // pi
#include "root_finder/root_finder.hpp"

#include <numbers>

std::tuple<float, float> BezierCurve2D::projection(glm::vec2 test_point) {
    auto target_poly =
        (test_point.x - polynomial1) * polynomial1_deriv + (test_point.y - polynomial2) * polynomial2_deriv;
    auto n = target_poly.degree();

    Eigen::VectorXd coeffs;
    coeffs.resize(n + 1);

    for (int i = 0; i <= n; i++) {
        coeffs(i) = target_poly.data()[n - i];
    }

    std::set<double> roots;

    if (n == 1) {
        double sol = -coeffs[1] / coeffs[0];
        if (sol > 0 and sol < 1)
            roots.insert(sol);
    } else if (n == 3) {
        auto tmp_roots = RootFinderPriv::solveCub(coeffs[3], coeffs[2], coeffs[1], coeffs[0]);
        for (auto r : tmp_roots) {
            if (r > 0 and r < 1)
                roots.insert(r);
        }
    } else {
        roots = RootFinder::solvePolynomial(coeffs, 0.0, 1.0, 0.01);
    }

    float dist = 10;
    float param = -1;

    double dist_to_start = glm::length(glm::vec2{control_points_[0][1], control_points_[0][1]} - test_point);
    double dist_to_end = glm::length(glm::vec2{control_points_.back()[0], control_points_.back()[1]} - test_point);

    if (dist_to_start < dist_to_end) {
        param = 0;
        dist = dist_to_start;
    } else {
        param = 1;
        dist = dist_to_end;
    }

    for (auto r : roots) {
        auto eval = evaluate(r);
        auto new_dist = glm::length(eval - test_point);

        if (new_dist < dist) {
            dist = new_dist;
            param = r;
        }
    }

    if (param < 0)
        throw std::runtime_error("wrong root");

    return std::make_tuple(dist, param);
}

glm::vec2 BezierCurve2D::evaluate_polynomial(float param) const {
    return {polynomial1.evaluate(param), polynomial2.evaluate(param)};
}

float BezierCurve2D::winding_number(glm::vec2 test_point) {
    // computing derivative bound

    return winding_number_internal(test_point, control_point_vec2.front(), control_point_vec2.back(), 0.0f, 1.0f,
                                   derivative_bound);
}

float BezierCurve2D::winding_number_internal(glm::vec2 test_point, glm::vec2 start_pos, glm::vec2 end_pos, float start,
                                             float end, float derivative_bound) {
    auto d1 = glm::length(start_pos - test_point);
    auto d2 = glm::length(end_pos - test_point);

    if (d1 < 1e-8 or d2 < 1e-8)
        return 0;

    if (d1 + d2 > derivative_bound * (end - start) or (end - start) < 1e-8) {
        auto v1 = glm::normalize(start_pos - test_point);
        auto v2 = glm::normalize(end_pos - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        if (std::isnan(std::acos(inner))) {
            return 0;
        }

        return outer > 0 ? std::acos(inner) : -std::acos(inner);
    }

    auto mid_param = (start + end) / 2;

    auto mid_pos = evaluate(mid_param);

    return winding_number_internal(test_point, start_pos, mid_pos, start, mid_param, derivative_bound) +
           winding_number_internal(test_point, mid_pos, end_pos, mid_param, end, derivative_bound);
}

float BezierCurve2D::winding_number_u_periodic_internal(glm::vec2 test_point, glm::vec2 start_pos, glm::vec2 end_pos,
                                                        float start, float end, float derivative_bound) {
    auto d1 = glm::length(start_pos - test_point);
    auto d2 = glm::length(end_pos - test_point);

    if (d1 < 1e-3 or d2 < 1e-3)
        return 0;

    if (d1 + d2 > derivative_bound * (end - start) or (end - start) < 1e-3) {
        auto v1 = start_pos - test_point;
        auto v2 = end_pos - test_point;

        int N = 100;

        glm::vec2 e = {1.0f, 0.0f};

        float res = 0;

        double series_main_part = 0;

        for (int i = -N; i <= N; i++) {
            if (i != 0) {
                series_main_part += (1.0f / (i * i));
            }
            auto w1 = glm::normalize(v1 + e * (1.0f * i));
            auto w2 = glm::normalize(v2 + e * (1.0f * i));
            auto outer_v = w1.x * w2.y - w1.y * w2.x;
            auto inner_v = glm::dot(w1, w2);

            if (std::isnan(std::acos(inner_v))) {
                continue;
            }

            res += outer_v > 0 ? std::acos(inner_v) : -std::acos(inner_v);
        }

        // truncated estimator

        double pi_square = std::numbers::pi * std::numbers::pi;

        auto v1_norm = glm::normalize(v1);
        auto v2_norm = glm::normalize(v2);
        auto outer = v1_norm.x * v2_norm.y - v1_norm.y * v2_norm.x;
        auto reminder_coeff = (pi_square / 3 - series_main_part);
        auto reminder = reminder_coeff * std::asin(outer);
        // res += reminder;

        return res;
    }

    auto mid_param = (start + end) / 2;
    auto mid_pos = evaluate_polynomial(mid_param);

    return winding_number_u_periodic_internal(test_point, start_pos, mid_pos, start, mid_param, derivative_bound) +
           winding_number_u_periodic_internal(test_point, mid_pos, end_pos, mid_param, end, derivative_bound);
}

void BezierCurve2D::initialize() {
    // computing the polynomial coefficients

    auto n = control_points_.size() - 1;

    std::vector<float> polynomial_coeff1, polynomial_coeff2;

    for (int j = 0; j <= n; j++) {
        if (j == 0) {
            polynomial_coeff1.push_back(control_points_[0][0]);
            polynomial_coeff2.push_back(control_points_[0][1]);
            continue;
        }
        float res1{1.0f};
        float res2{1.0f};

        float tmp1{0.0f};
        float tmp2{0.0f};
        for (int i = 0; i <= j; i++) {
            auto denom = boost::math::factorial<float>(i) * boost::math::factorial<float>(j - i);
            tmp1 += control_points_[i][0] * std::pow(-1, i + j) / denom;
            tmp2 += control_points_[i][1] * std::pow(-1, i + j) / denom;
        }

        auto factor = boost::math::factorial<float>(n) / boost::math::factorial<float>(n - j);

        res1 = factor * tmp1;
        res2 = factor * tmp2;

        polynomial_coeff1.push_back(res1);
        polynomial_coeff2.push_back(res2);
    }

    polynomial1 = boost::math::tools::polynomial<float>(polynomial_coeff1.cbegin(), polynomial_coeff1.cend());
    polynomial2 = boost::math::tools::polynomial<float>(polynomial_coeff2.cbegin(), polynomial_coeff2.cend());

    polynomial1_deriv = polynomial1.prime();
    polynomial2_deriv = polynomial2.prime();

    control_point_vec2.clear();
    for (auto &control_point : control_points_) {
        control_point_vec2.emplace_back(control_point[0], control_point[1]);
    }

    for (int i = 1; i <= n; i++) {
        derivative_bound =
                std::max(derivative_bound, n * glm::length(control_point_vec2[i] - control_point_vec2[i - 1]));
    }
}

void BezierCurve2D::add_control_point(std::array<float, 2> pt) {
    control_points_.push_back(pt);
    initialize();
}

void BezierCurve2D::update_control_point(size_t index, std::array<float, 2> offset) {
    control_points_[index][0] += offset[0];
    control_points_[index][1] += offset[1];
    initialize();
}
