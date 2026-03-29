#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "geometry/parametric/configuration.hpp"
#include "parametric_curve.hpp"

namespace GraphicsLab::Geometry {

template <size_t dim> struct NURBSCurveBase : ParamCurveBase<dim> {
    struct SceneTreeGeometryTypeTrait {};

    using PointType = glm::vec<dim, double>;

    std::vector<PointType> control_points_;
    std::vector<double> knots_;
    std::vector<double> weights_;
    int degree_ = 0;

    NURBSCurveBase(std::vector<PointType> control_points, std::vector<double> knots, std::vector<double> weights,
                   int degree)
        : control_points_(std::move(control_points)), knots_(std::move(knots)), weights_(std::move(weights)),
          degree_(degree) {
        assert(is_valid());
        build_cache();
        for (const auto &point : control_points_) {
            this->box.max = glm::max(this->box.max, point);
            this->box.min = glm::min(this->box.min, point);
        }
    }

    [[nodiscard]] PointType evaluate(double param) const override {
        const auto [numerator, denominator] = evaluate_rational_terms(param);
        return numerator / denominator;
    }

    [[nodiscard]] PointType derivative(double param) const override {
        if (degree_ == 0) {
            return PointType(0.0);
        }

        const auto [a, w] = evaluate_rational_terms(param);
        const auto [a1, w1] = evaluate_rational_terms_first_derivative(param);
        return (a1 * w - a * w1) / (w * w);
    }

    [[nodiscard]] PointType second_derivative(double param) const override {
        if (degree_ <= 1) {
            return PointType(0.0);
        }

        const auto [a, w] = evaluate_rational_terms(param);
        const auto [a1, w1] = evaluate_rational_terms_first_derivative(param);
        const auto [a2, w2] = evaluate_rational_terms_second_derivative(param);

        const PointType first_term = (a2 * w - a * w2) / (w * w);
        const PointType second_term = 2.0 * w1 * (a1 * w - a * w1) / (w * w * w);
        return first_term - second_term;
    }

  private:
    std::vector<PointType> weighted_control_points_;
    std::vector<PointType> first_weighted_control_points_;
    std::vector<double> first_weight_control_points_;
    std::vector<double> first_knots_;

    std::vector<PointType> second_weighted_control_points_;
    std::vector<double> second_weight_control_points_;
    std::vector<double> second_knots_;

    static constexpr double kTolerance = ParametricConfiguration::system_tolerance;

    [[nodiscard]] std::pair<PointType, double> evaluate_rational_terms(double param) const {
        PointType a = bspline_evaluate(clamp_to_domain(param), weighted_control_points_, knots_, degree_);
        double w = bspline_evaluate(clamp_to_domain(param), weights_, knots_, degree_);
        if (std::abs(w) <= kTolerance) {
            throw std::runtime_error("NURBS evaluation encountered near-zero weight denominator.");
        }
        return {a, w};
    }

    [[nodiscard]] std::pair<PointType, double> evaluate_rational_terms_first_derivative(double param) const {
        const double u = clamp_to_domain(param);
        PointType a1 = bspline_evaluate(u, first_weighted_control_points_, first_knots_, degree_ - 1);
        double w1 = bspline_evaluate(u, first_weight_control_points_, first_knots_, degree_ - 1);
        return {a1, w1};
    }

    [[nodiscard]] std::pair<PointType, double> evaluate_rational_terms_second_derivative(double param) const {
        const double u = clamp_to_domain(param);
        PointType a2 = bspline_evaluate(u, second_weighted_control_points_, second_knots_, degree_ - 2);
        double w2 = bspline_evaluate(u, second_weight_control_points_, second_knots_, degree_ - 2);
        return {a2, w2};
    }

    [[nodiscard]] double clamp_to_domain(double param) const {
        const double start = knots_[static_cast<size_t>(degree_)];
        const double end = knots_[knots_.size() - static_cast<size_t>(degree_) - 1];

        double u = std::clamp(param, start, end);
        if (std::abs(u - end) <= kTolerance) {
            u = std::nextafter(end, start);
        }
        return u;
    }

    void build_cache() {
        weighted_control_points_.reserve(control_points_.size());
        for (size_t i = 0; i < control_points_.size(); ++i) {
            weighted_control_points_.push_back(control_points_[i] * weights_[i]);
        }

        if (degree_ == 0) {
            return;
        }

        first_weighted_control_points_ = derivative_control_points(weighted_control_points_, knots_, degree_);
        first_weight_control_points_ = derivative_control_points(weights_, knots_, degree_);
        first_knots_ = trimmed_knots(knots_);

        if (degree_ <= 1) {
            return;
        }

        second_weighted_control_points_ =
            derivative_control_points(first_weighted_control_points_, first_knots_, degree_ - 1);
        second_weight_control_points_ = derivative_control_points(first_weight_control_points_, first_knots_, degree_ - 1);
        second_knots_ = trimmed_knots(first_knots_);
    }

    [[nodiscard]] bool is_valid() const {
        if (degree_ < 0 || control_points_.empty()) {
            return false;
        }
        if (weights_.size() != control_points_.size()) {
            return false;
        }
        if (knots_.size() != control_points_.size() + static_cast<size_t>(degree_) + 1) {
            return false;
        }
        if (control_points_.size() <= static_cast<size_t>(degree_)) {
            return false;
        }
        if (!std::is_sorted(knots_.begin(), knots_.end())) {
            return false;
        }

        return std::all_of(weights_.begin(), weights_.end(), [](double weight) { return weight > 0.0; });
    }

    template <typename ValueType>
    static std::vector<ValueType> derivative_control_points(const std::vector<ValueType> &control_points,
                                                            const std::vector<double> &knots, int degree) {
        std::vector<ValueType> result;
        if (degree <= 0 || control_points.size() < 2) {
            return result;
        }

        result.reserve(control_points.size() - 1);
        const int n = static_cast<int>(control_points.size()) - 1;
        for (int i = 0; i < n; ++i) {
            const double denom = knots[static_cast<size_t>(i + degree + 1)] - knots[static_cast<size_t>(i + 1)];
            if (std::abs(denom) <= kTolerance) {
                result.emplace_back(ValueType(0.0));
            } else {
                result.emplace_back(static_cast<double>(degree) * (control_points[static_cast<size_t>(i + 1)] -
                                                                    control_points[static_cast<size_t>(i)]) /
                                    denom);
            }
        }
        return result;
    }

    static std::vector<double> trimmed_knots(const std::vector<double> &knots) {
        if (knots.size() < 2) {
            return {};
        }
        return std::vector<double>(knots.begin() + 1, knots.end() - 1);
    }

    template <typename ValueType>
    static ValueType bspline_evaluate(double u, const std::vector<ValueType> &control_points, const std::vector<double> &knots,
                                      int degree) {
        if (control_points.empty()) {
            throw std::runtime_error("B-spline evaluation received empty control points.");
        }
        if (degree < 0) {
            return ValueType(0.0);
        }

        const int n = static_cast<int>(control_points.size()) - 1;
        int span = n;
        for (int i = degree; i <= n; ++i) {
            if (u >= knots[static_cast<size_t>(i)] && u < knots[static_cast<size_t>(i + 1)]) {
                span = i;
                break;
            }
        }

        std::vector<ValueType> d(static_cast<size_t>(degree + 1));
        for (int j = 0; j <= degree; ++j) {
            d[static_cast<size_t>(j)] = control_points[static_cast<size_t>(span - degree + j)];
        }

        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                const int i = span - degree + j;
                const double denom = knots[static_cast<size_t>(i + degree - r + 1)] - knots[static_cast<size_t>(i)];
                const double alpha = std::abs(denom) <= kTolerance ? 0.0 : (u - knots[static_cast<size_t>(i)]) / denom;
                d[static_cast<size_t>(j)] =
                    (1.0 - alpha) * d[static_cast<size_t>(j - 1)] + alpha * d[static_cast<size_t>(j)];
            }
        }

        return d[static_cast<size_t>(degree)];
    }
};

} // namespace GraphicsLab::Geometry



