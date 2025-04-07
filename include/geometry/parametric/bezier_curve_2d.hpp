#pragma once

#include "glm/glm.hpp"
#include <vector>

namespace GraphicsLab::Geometry {

struct BezierCurve2D {
    struct SceneTreeGeometryTypeTrait {};

    using PointType = glm::vec<2, double>;
    std::vector<PointType> control_points_;
    double derivative_bound = -1.0;

    // for debug
    glm::vec3 color;

    /**
     * evaluate the Bézier curve with de casteljau algorithm
     * @param param
     * @return
     */
    PointType evaluate(double param) const {
        return evaluate_linear(param);
    }

    /**
     * evaluate the Bézier curve with linear method [Woźny and Chudy 2020]
     * @param param
     * @return
     */
    PointType evaluate_linear(double param) const {
        double h = 1.0;
        PointType result = control_points_[0];
        double t = param;
        double u = 1 - t;
        uint32_t n = control_points_.size() - 1;
        uint32_t n1 = n + 1;
        if (param <= 0.5) {
            u = t / u;
            for (int k = 1; k <= n; k++) {
                h = h * u * (n1 - k);
                h = h / (k + h);
                double h1 = 1 - h;
                result = result * h1 + control_points_[k] * h;
            }
        } else {
            u = u / t;
            for (int k = 1; k <= n; k++) {
                h = h * (n1 - k);
                h = h / (k * u + h);
                double h1 = 1 - h;
                result = result * h1 + control_points_[k] * h;
            }
        }
        return result;
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
    }

    explicit BezierCurve2D(decltype(control_points_) &&control_points) : control_points_(control_points) {
        update_bounds();

        color = glm::vec<3, double>(1.0, 0.0, 0.0);
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
    std::pair<BezierCurve2D, BezierCurve2D> subdivide(double t) const {
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

        return { BezierCurve2D(std::move(left)), BezierCurve2D(std::move(right)) };
    }

    // Bounding box
    std::pair<PointType, PointType> boundingBox() const {
        PointType minPt = control_points_[0];
        PointType maxPt = control_points_[0];
        for (const auto& pt : control_points_) {
            minPt = glm::min(minPt, pt);
            maxPt = glm::max(maxPt, pt);
        }
        return {minPt, maxPt};
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
};

} // namespace GraphicsLab::Geometry