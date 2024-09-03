#include "geometry/curve/bezier_curve_2d.hpp"

#include <set>

#include "Eigen/Eigen"

#define M_PI 3.14159265358979323846 // pi
#include "root_finder/root_finder.hpp"

#include <numbers>

#include "geometry/curve/bezier_clipping.hpp"
#include "geometry/curve/bezier_root_finder.hpp"

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

    bool quadrant1 = false, quadrant2 = false, quadrant3 = false, quadrant4 = false;

    for (auto &point : control_point_vec2) {
        if (point.x > test_point.x and point.y > test_point.y)
            quadrant1 = true;
        if (point.x < test_point.x and point.y > test_point.y)
            quadrant2 = true;
        if (point.x < test_point.x and point.y < test_point.y)
            quadrant3 = true;
        if (point.x > test_point.x and point.y < test_point.y)
            quadrant4 = true;
    }

    uint32_t quadrant_num = 0;
    if (quadrant1)
        quadrant_num++;
    if (quadrant2)
        quadrant_num++;
    if (quadrant3)
        quadrant_num++;
    if (quadrant4)
        quadrant_num++;

    bool out_bound_flag = false;

    // if (quadrant_num == 1) {
    //     out_bound_flag = true;
    // }
    //
    // if (quadrant1 and quadrant2 and (not quadrant3) and (not quadrant4)) out_bound_flag = true;
    // if (quadrant1 and quadrant4 and (not quadrant2) and (not quadrant3)) out_bound_flag = true;
    // if (quadrant2 and quadrant3 and (not quadrant1) and (not quadrant4)) out_bound_flag = true;
    // if (quadrant3 and quadrant4 and (not quadrant1) and (not quadrant2)) out_bound_flag = true;
    //

    if (out_bound_flag) {
        auto v1 = glm::normalize(control_point_vec2.front() - test_point);
        auto v2 = glm::normalize(control_point_vec2.back() - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        auto acos_value = std::acos(inner);

        if (std::isnan(acos_value)) {
            return 0;
        }

        return outer > 0 ? acos_value : -acos_value;
    } else {
        return winding_number_internal(test_point, control_point_vec2.front(), control_point_vec2.back(), 0.0f, 1.0f,
                                       derivative_bound);
    }
}

float BezierCurve2D::winding_number_internal(glm::vec2 test_point, glm::vec2 start_pos, glm::vec2 end_pos, float start,
                                             float end, float derivative_bound) {
    auto d1 = glm::length(start_pos - test_point);
    auto d2 = glm::length(end_pos - test_point);

    if (d1 < 1e-6 or d2 < 1e-6)
        return 0;

    if (d1 + d2 > derivative_bound * (end - start) or (end - start) < 1e-7) {
        auto v1 = glm::normalize(start_pos - test_point);
        auto v2 = glm::normalize(end_pos - test_point);
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        auto acos_value = std::acos(inner);

        if (std::isnan(acos_value)) {
            return 0;
        }

        return outer > 0 ? acos_value : -acos_value;
    }

    auto mid_param = (start + end) / 2;

    auto mid_pos = evaluate_linear(mid_param);

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
            auto denom = boost::math::factorial<double>(i) * boost::math::factorial<double>(j - i);
            tmp1 += control_points_[i][0] * std::pow(-1, i + j) / denom;
            tmp2 += control_points_[i][1] * std::pow(-1, i + j) / denom;
        }

        auto factor = boost::math::factorial<double>(n) / boost::math::factorial<double>(n - j);

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

    derivative_bound = -1.0;
    for (int i = 1; i <= n; i++) {
        derivative_bound =
            std::max(derivative_bound, n * glm::length(control_point_vec2[i] - control_point_vec2[i - 1]));
    }

    if (control_points_.size() >= 3) {
        compute_extreme_points_new();
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

glm::vec2 BezierCurve2D::evaluate_linear(float param) const {
    float h = 1.0;
    glm::vec2 result = control_point_vec2[0];
    float t = param;
    float u = 1 - t;
    uint32_t n = control_points_.size() - 1;
    uint32_t n1 = n + 1;
    if (param <= 0.5) {
        u = t / u;
        for (int k = 1; k <= n; k++) {
            h = h * u * (n1 - k);
            h = h / (k + h);
            float h1 = 1 - h;
            result = result * h1 + control_point_vec2[k] * h;
        }
    } else {
        u = u / t;
        for (int k = 1; k <= n; k++) {
            h = h * (n1 - k);
            h = h / (k * u + h);
            float h1 = 1 - h;
            result = result * h1 + control_point_vec2[k] * h;
        }
    }
    return result;
}

std::vector<glm::vec2> BezierCurve2D::compute_extreme_points() {
    return compute_extreme_points_new();

    int n = control_points_.size() - 1;

    std::vector<glm::vec2> result;

    auto solvePolynomialZeros = [&](decltype(this->polynomial1) poly) {
        Eigen::VectorXd coeffs;
        coeffs.resize(n);

        for (int i = 0; i < n; i++) {
            coeffs(i) = poly.data()[n - i - 1];
        }

        auto roots = RootFinder::solvePolynomial(coeffs, 0.0, 1.0, 1e-6);

        for (auto r : roots) {
            result.push_back(evaluate_linear(r));
        }
    };

    solvePolynomialZeros(polynomial1_deriv);
    solvePolynomialZeros(polynomial2_deriv);

    return result;
}

std::vector<glm::vec2> BezierCurve2D::compute_extreme_points_new() {
    std::vector<glm::vec2> deriv;
    size_t n = control_points_.size() - 1;
    for (int i = 1; i < control_point_vec2.size(); i++) {
        deriv.push_back((control_point_vec2[i] - control_point_vec2[i - 1]) * (1.0f * n));
    }

    std::vector<double> x_poly, y_poly;

    std::ranges::for_each(deriv, [&](glm::vec2 v) {
        x_poly.push_back(v.x);
        y_poly.push_back(v.y);
    });

    BezierRootFinder x_finder(std::move(x_poly));
    BezierRootFinder y_finder(std::move(y_poly));

    auto x_res = x_finder.get_roots();
    auto y_res = y_finder.get_roots();

    // std::ranges::copy(x_res, std::ostream_iterator<double>(std::cout, ", "));
    // std::ranges::copy(y_res, std::ostream_iterator<double>(std::cout, ", "));
    // std::cout << std::endl;

    std::vector<glm::vec2> result;

    auto evaluate_fn = [&](double x) { return evaluate_linear(x); };

    std::ranges::copy(x_res | std::views::transform(evaluate_fn), std::back_inserter(result));
    std::ranges::copy(y_res | std::views::transform(evaluate_fn), std::back_inserter(result));

    auto evaluate_fn_pair = [&](double x) { return std::make_pair(x, evaluate(x)); };

    extreme_points.clear();

    extreme_points.emplace_back(0, control_point_vec2.front());
    extreme_points.emplace_back(1, control_point_vec2.back());
    std::ranges::copy(x_res | std::views::transform(evaluate_fn_pair), std::back_inserter(extreme_points));
    std::ranges::copy(y_res | std::views::transform(evaluate_fn_pair), std::back_inserter(extreme_points));

    std::ranges::sort(extreme_points, [](extreme_info_type x, extreme_info_type y) { return x.first < y.first; });

    return result;
}

float BezierCurve2D::winding_number_monotonic(glm::vec2 test_point) {
    size_t n = extreme_points.size() - 1;

    float winding_number = 0.0f;

    for (int i = 1; i <= n; i++) {
        auto [param1, pos1] = extreme_points[i - 1];
        auto [param2, pos2] = extreme_points[i];
        winding_number += winding_number_monotonic_internal(test_point, pos1, pos2, param1, param2);
    }

    return winding_number;
}

float BezierCurve2D::winding_number_monotonic_internal(glm::vec2 test_point, glm::vec2 start_pos, glm::vec2 end_pos,
                                                       float start, float end) {

    bool out_x = (test_point.x < start_pos.x and test_point.x < end_pos.x) or
                 (test_point.x > start_pos.x and test_point.x > end_pos.x);
    bool out_y = (test_point.y < start_pos.y and test_point.y < end_pos.y) or
                 (test_point.y > start_pos.y and test_point.y > end_pos.y);

    auto v1 = glm::normalize(start_pos - test_point);
    auto v2 = glm::normalize(end_pos - test_point);

    if (glm::length(start_pos - test_point) < 1e-6 or glm::length(end_pos - start_pos) < 1e-6) {
        return 0;
    }

    if (out_x or out_y) {
        auto outer = v1.x * v2.y - v1.y * v2.x;
        auto inner = glm::dot(v1, v2);

        if (std::isnan(std::acos(inner))) {
            return 0;
        }

        return outer > 0 ? std::acos(inner) : -std::acos(inner);
    } else {
        auto mid_param = (start + end) / 2;

        auto mid_pos = evaluate_linear(mid_param);

        return winding_number_monotonic_internal(test_point, start_pos, mid_pos, start, mid_param) +
               winding_number_monotonic_internal(test_point, mid_pos, end_pos, mid_param, end);
    }
}

float BezierCurve2D::winding_number_u_periodic(glm::vec2 test_point) {
    // the closed case

    auto start_pos = control_point_vec2.front();
    auto end_pos = control_point_vec2.back();

    if (glm::length(end_pos - start_pos) < 1e-5) {
        double winding_number = winding_number_internal(test_point, start_pos, end_pos, 0.0, 1.0, derivative_bound);

        for (int i = 1; i < 3; i++) {
            winding_number += winding_number_internal(test_point + glm::vec2{1.0f * i, 0.0f}, start_pos, end_pos, 0.0,
                                                      1.0, derivative_bound);
            winding_number += winding_number_internal(test_point - glm::vec2{1.0f * i, 0.0f}, start_pos, end_pos, 0.0,
                                                      1.0, derivative_bound);
        }

        return winding_number;
    }

    // the non-closed case
    double winding_number_main = winding_number_internal(test_point, start_pos, end_pos, 0.0, 1.0, derivative_bound);

    double winding_number_v = 0;
    if (start_pos.x < end_pos.x) {
        winding_number_v = start_pos.y > test_point.y ? -std::numbers::pi : std::numbers::pi;
    } else {
        winding_number_v = start_pos.y > test_point.y ? std::numbers::pi : -std::numbers::pi;
    }

    double winding_number_complement = 0;

    auto v1 = glm::normalize(start_pos - test_point);
    auto v2 = glm::normalize(end_pos - test_point);

    auto outer = v1.x * v2.y - v1.y * v2.x;
    auto inner = glm::dot(v1, v2);

    if (std::isnan(std::acos(inner))) {
        winding_number_complement = winding_number_v = 0;
    } else {
        winding_number_complement = outer > 0 ? std::acos(inner) : -std::acos(inner);
    }

    return winding_number_main + winding_number_v - winding_number_complement;
}

float BezierCurve2D::winding_number_bi_periodic(glm::vec2 test_point) {
    auto start_pos = control_point_vec2.front();
    auto end_pos = control_point_vec2.back();

    int N = 2;

    bool closed = glm::length(start_pos - end_pos) < 1e-5;

    double winding_number = 0.0;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            auto trans_point = test_point + glm::vec2{1.0f * i, 1.0f * j};

            double winding_number_main =
                winding_number_internal(trans_point, start_pos, end_pos, 0.0, 1.0, derivative_bound);
            double winding_number_v = 0;
            double winding_number_complement = 0;

            if (closed) {
                winding_number += winding_number_main;
            } else {
                auto v1 = glm::normalize(start_pos - trans_point);
                auto v2 = glm::normalize(end_pos - trans_point);
                auto v3 = glm::normalize(end_pos - start_pos);

                if (v1.x * v3.y - v1.y * v3.x > 0) {
                    winding_number_v = std::numbers::pi;
                } else {
                    winding_number_v = -std::numbers::pi;
                }

                auto outer = v1.x * v2.y - v1.y * v2.x;
                auto inner = glm::dot(v1, v2);

                if (std::isnan(std::acos(inner))) {
                    winding_number_complement = winding_number_v = 0;
                } else {
                    winding_number_complement = outer > 0 ? std::acos(inner) : -std::acos(inner);
                }

                winding_number += winding_number_main + winding_number_v - winding_number_complement;
            }
        }
    }

    return winding_number;
}

uint32_t BezierCurve2D::bezier_clipping(glm::vec2 test_point) const {
    return BezierClipping::bezier_clipping(test_point, control_point_vec2);
}
