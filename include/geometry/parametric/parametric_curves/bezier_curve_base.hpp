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

    double derivative_bound = -1.0;

    REFLECT(Property{"control_points", &BezierCurveBase::control_points_})

    BezierCurveBase(const BezierCurveBase<dim> &other) noexcept {
        control_points_ = other.control_points_;
        derivative_bound = other.derivative_bound;
        this->mesh = nullptr;

        this->min_x = other.min_x;
        this->max_x = other.max_x;
        this->min_y = other.min_y;
        this->max_y = other.max_y;
    }

    BezierCurveBase(BezierCurveBase<dim> &&other) noexcept {
        control_points_ = std::move(other.control_points_);
        derivative_bound = other.derivative_bound;
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
        new_control_points.push_back(control_points_.front());
        for (int i = 0; i < control_points_.size() - 1; i++) {
            double lambda = static_cast<double>(i) / control_points_.size();
            new_control_points.push_back(control_points_[i] * lambda + control_points_[i + 1] * (1 - lambda));
        }

        new_control_points.push_back(control_points_.back());
        control_points_ = new_control_points;
        update_bounds();
    }

    // Subdivide at t into two curves
    std::pair<BezierCurveBase, BezierCurveBase> subdivide(double t) const {
        std::vector<std::vector<PointType>> triangle;
        triangle.push_back(control_points_);
        int n = control_points_.size();
        for (int k = 1; k < n; ++k) {
            std::vector<PointType> row;
            for (int i = 0; i < n - k; ++i) {
                row.push_back(glm::mix(triangle.back()[i], triangle.back()[i + 1], t));
            }
            triangle.push_back(row);
        }

        std::vector<PointType> left, right;
        for (int i = 0; i < n; ++i) {
            left.push_back(triangle[i][0]);
            right.push_back(triangle[n - i - 1][i]);
        }

        return {BezierCurveBase(std::move(left)), BezierCurveBase(std::move(right))};
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

    std::vector<glm::dvec2> sample(const int resolution) const {
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

    size_t degree() const {
        return control_points_.size() - 1;
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
                h = h * u * (n1 - k);
                h = h / (k + h);
                double h1 = 1 - h;
                result = result * h1 + ctrl_pts[k] * h;
            }
        } else {
            u = u / t;
            for (uint32_t k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                double h1 = 1 - h;
                result = result * h1 + ctrl_pts[k] * h;
            }
        }
        return result;
    }
};

} // namespace GraphicsLab::Geometry