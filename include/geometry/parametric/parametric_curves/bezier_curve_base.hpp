#pragma once

#include "glm/glm.hpp"
#include "language/reflection/static_reflector.hpp"
#include "parametric_curve.hpp"
#include <vector>

namespace GraphicsLab::Geometry {

template <size_t dim> struct BezierCurveBase : ParamCurveBase<dim> {
    struct SceneTreeGeometryTypeTrait {};

    using MeshType = std::conditional_t<dim == 3, CurveMesh3D, CurveMesh2D>;
    using PointType = glm::vec<dim, double>;
    std::vector<PointType> control_points_;
    std::vector<PointType> derivative_points_;
    std::vector<double> weights_;
    double derivative_bound = -1.0;

    REFLECT(Property{"control_points", &BezierCurveBase::control_points_})

    BezierCurveBase(const BezierCurveBase<dim> &other) noexcept {
        control_points_ = other.control_points_;
        derivative_bound = other.derivative_bound;
        weights_ = other.weights_;
        this->mesh = nullptr;

        this->min_x = other.min_x;
        this->max_x = other.max_x;
        this->min_y = other.min_y;
        this->max_y = other.max_y;
    }

    BezierCurveBase(BezierCurveBase<dim> &&other) noexcept {
        control_points_ = std::move(other.control_points_);
        derivative_bound = other.derivative_bound;
        weights_ = std::move(other.weights_);
        this->mesh = std::move(other.mesh);

        this->min_x = other.min_x;
        this->max_x = other.max_x;
        this->min_y = other.min_y;
        this->max_y = other.max_y;
    }

    explicit BezierCurveBase(decltype(control_points_) &&control_points) : control_points_(control_points) {
        int n = control_points_.size() - 1;
        for (int i = 1; i < control_points_.size(); i++) {
            derivative_points_.push_back((control_points_[i] - control_points_[i - 1]) * static_cast<double>(n));
        }
        weights_.resize(control_points_.size());
        std::ranges::fill(weights_, 1.0);
        update_bounds();
    }

    explicit BezierCurveBase(const std::vector<PointType> &control_points, const std::vector<double>& weights): control_points_(control_points), weights_(weights) {
        int n = control_points_.size() - 1;
        for (int i = 1; i < control_points_.size(); i++) {
            derivative_points_.push_back((control_points_[i] - control_points_[i - 1]) * static_cast<double>(n));
        }
        update_bounds();
    }

    /**
     * evaluate the Bézier curve with de casteljau algorithm
     * @param param
     * @return
     */
    PointType evaluate(double param) const override {
        return evaluate_linear(param, control_points_);
    }

    PointType derivative(double param) const override {
        return evaluate_linear(param, derivative_points_);
    }

    double min_x = 100, min_y = 100, max_x = -100, max_y = -100;

    void update_bounds() {
        int n = control_points_.size() - 1;

        // compute the derivative bound
        for (int i = 1; i <= n; i++) {
            derivative_bound = std::max(derivative_bound, n * glm::length(control_points_[i] - control_points_[i - 1]));
        }

        auto [min_it, max_it] = std::ranges::minmax_element(weights_);
        derivative_bound = std::pow(*max_it / *min_it, 2);

        for (auto &pt : control_points_) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        this->box.max = {max_x, max_y};
        this->box.min = {min_x, min_y};
    }

    /**
     * @brief elevate the degree of current bezier curve
     */
    void degree_elevation() {
        std::vector<PointType> new_control_points;
        std::vector<double> new_weights;

        int n = control_points_.size() - 1;

        new_control_points.push_back(control_points_.front());
        new_weights.push_back(weights_.front());

        for (int i = 1; i <= n; ++i) {
            double alpha = static_cast<double>(i) / (n + 1);

            double w1 = weights_[i - 1];
            double w2 = weights_[i];
            double new_weight = alpha * w1 + (1 - alpha) * w2;

            PointType new_point =
                (control_points_[i - 1] * (alpha * w1) + control_points_[i] * ((1 - alpha) * w2)) / new_weight;

            new_control_points.push_back(new_point);
            new_weights.push_back(new_weight);
        }

        new_control_points.push_back(control_points_.back());
        new_weights.push_back(weights_.back());

        control_points_ = std::move(new_control_points);
        weights_ = std::move(new_weights);
        update_bounds();
    }


    // Subdivide at t into two curves
    std::pair<BezierCurveBase, BezierCurveBase> subdivide(double t) const {
        int n = control_points_.size();

        // Build triangle in homogeneous space (PointType * weight, weight)
        std::vector<std::vector<PointType>> point_triangle;
        std::vector<std::vector<double>> weight_triangle;

        std::vector<PointType> weighted_pts;
        for (int i = 0; i < n; ++i)
            weighted_pts.push_back(control_points_[i] * weights_[i]);

        point_triangle.push_back(weighted_pts);
        weight_triangle.push_back(weights_);

        for (int k = 1; k < n; ++k) {
            std::vector<PointType> next_row_pts;
            std::vector<double> next_row_weights;

            for (int i = 0; i < n - k; ++i) {
                double w1 = weight_triangle.back()[i];
                double w2 = weight_triangle.back()[i + 1];
                PointType p1 = point_triangle.back()[i];
                PointType p2 = point_triangle.back()[i + 1];

                double new_w = (1 - t) * w1 + t * w2;
                PointType new_p = ((1 - t) * p1 + t * p2);

                next_row_pts.push_back(new_p);
                next_row_weights.push_back(new_w);
            }

            point_triangle.push_back(next_row_pts);
            weight_triangle.push_back(next_row_weights);
        }

        std::vector<PointType> left_ctrls, right_ctrls;
        std::vector<double> left_weights, right_weights;

        for (int i = 0; i < n; ++i) {
            PointType lh = point_triangle[i][0];
            double lw = weight_triangle[i][0];
            left_ctrls.push_back(lh / lw);
            left_weights.push_back(lw);

            PointType rh = point_triangle[n - i - 1][i];
            double rw = weight_triangle[n - i - 1][i];
            right_ctrls.push_back(rh / rw);
            right_weights.push_back(rw);
        }

        return {
            BezierCurveBase(std::move(left_ctrls), std::move(left_weights)),
            BezierCurveBase(std::move(right_ctrls), std::move(right_weights))
        };
    }


    // Bounding box
    std::pair<PointType, PointType> boundingBox() const {
        PointType minPt = control_points_[0];
        PointType maxPt = control_points_[0];
        for (const auto &pt : control_points_) {
            minPt = glm::min(minPt, pt);
            maxPt = glm::max(maxPt, pt);
        }
        return {minPt, maxPt};
    }

    BezierCurveBase operator+(const PointType &offset) const {
        BezierCurveBase new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt += offset;
        }
        return new_curve;
    }

    BezierCurveBase operator-(const PointType &offset) const {
        BezierCurveBase new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt -= offset;
        }
        return new_curve;
    }

    [[nodiscard]] std::vector<glm::dvec2> sample(const int resolution) const {
        std::vector<glm::dvec2> pts;
        for (int i = 0; i <= resolution; ++i) {
            const double t = static_cast<double>(i) / resolution;
            pts.push_back(evaluate_linear(t));
        }
        return pts;
    }

    PointType start_position() const {
        return control_points_.front();
    }

    PointType end_position() const {
        return control_points_.back();
    }

    [[nodiscard]] size_t degree() const {
        return control_points_.size() - 1;
    }

    void set_weight(size_t index, double weight) {
        if (weight < 0) {
            throw std::invalid_argument("weight must be non-negative");
        }
        weights_[index] = weight;
    }
  private:
    /**
     * evaluate the Bézier curve with linear method [Woźny and Chudy 2020]
     * @param param
     * @return
     */
    PointType evaluate_linear(double param, const std::vector<PointType> &ctrl_pts) const {
        double h = 1.0;
        PointType result = ctrl_pts[0];
        double t = param;
        double u = 1 - t;
        uint32_t n = ctrl_pts.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (uint32_t k = 1; k <= n; k++) {
                h = h * u * (n1 - k) * weights_[k];
                h = h / (k * weights_[k - 1] + h);
                double h1 = 1 - h;
                result = result * h1 + ctrl_pts[k] * h;
            }
        } else {
            u = u / t;
            for (uint32_t k = 1; k <= n; k++) {
                h = h * (n1 - k) * weights_[k];
                h = h / (k * u * weights_[k - 1] + h);
                double h1 = 1 - h;
                result = result * h1 + ctrl_pts[k] * h;
            }
        }
        return result;
    }
};

} // namespace GraphicsLab::Geometry