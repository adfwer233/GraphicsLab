#pragma once

#include "bezier_curve_2d.hpp"
#include "parametric_curve.hpp"

#include <Eigen/Eigen>

namespace GraphicsLab::Geometry {

struct BSplineCurve2D : ParamCurve2D {
    struct SceneTreeGeometryTypeTrait {};

    using PointType = glm::vec<2, double>;
    std::vector<PointType> control_points_;
    std::vector<double> knots_;
    int degree_;

    // for debug
    glm::vec3 color;

    BSplineCurve2D(std::vector<PointType> control_points, std::vector<double> knots, int degree)
        : control_points_(std::move(control_points)), knots_(std::move(knots)), degree_(degree) {
    }

    PointType evaluate(double u) const override {
        using namespace glm;

        const int n = static_cast<int>(control_points_.size()) - 1;
        const int k = degree_;
        const int m = static_cast<int>(knots_.size()) - 1;

        // Clamp u to [0,1]
        u = std::clamp(u, 0.0, 1.0);

        spdlog::info("knots size: {}", knots_.size());
        // Find knot span index s such that knots_[s] <= u < knots_[s+1]
        int s = -1;
        if (u == 1.0) {
            s = n; // Special case at the end
        } else {
            for (int i = k; i <= n; ++i) {
                if (u >= knots_[i] && u < knots_[i + 1]) {
                    s = i;
                    break;
                }
            }
        }
        spdlog::info("test");
        if (s == -1)
            return control_points_.back(); // fallback

        // Initialize d[j] = control point at i = s - k + j
        std::vector<PointType> d(k + 1);
        for (int j = 0; j <= k; ++j) {
            d[j] = control_points_[s - k + j];
        }

        spdlog::info("living");
        // De Boor recursion
        for (int r = 1; r <= k; ++r) {
            for (int j = k; j >= r; --j) {
                int i = s - k + j;
                double alpha = (u - knots_[i]) / (knots_[i + k - r + 1] - knots_[i]);
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
            }
        }

        return d[k];
    }

    std::vector<BezierCurve2D> convert_to_bezier() const {
        std::vector<BezierCurve2D> result;
        int n = static_cast<int>(control_points_.size()) - 1;
        int m = static_cast<int>(knots_.size()) - 1;
        int k = degree_;

        // Assumes the curve is clamped and can be represented as a set of Bezier segments
        int num_segments = m - 2 * k;
        for (int seg = 0; seg < num_segments; ++seg) {
            std::vector<PointType> bezier_pts;
            for (int i = 0; i <= k; ++i) {
                int ctrl_index = seg + i;
                bezier_pts.push_back(control_points_[ctrl_index]);
            }
            result.emplace_back(std::move(bezier_pts));
        }
        return result;
    }

    std::pair<PointType, PointType> boundingBox() const {
        PointType minPt = control_points_[0];
        PointType maxPt = control_points_[0];
        for (const auto &pt : control_points_) {
            minPt = glm::min(minPt, pt);
            maxPt = glm::max(maxPt, pt);
        }
        return {minPt, maxPt};
    }

    BSplineCurve2D operator+(const PointType &offset) const {
        BSplineCurve2D new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt += offset;
        }
        return new_curve;
    }

    BSplineCurve2D operator-(const PointType &offset) const {
        BSplineCurve2D new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt -= offset;
        }
        return new_curve;
    }

    static BSplineCurve2D fit(const std::vector<PointType> &points, int degree, int num_ctrl_points) {
        using namespace Eigen;
        const int num_points = static_cast<int>(points.size());
        const int k = degree;

        assert(num_ctrl_points >= k + 1);
        assert(num_points > num_ctrl_points); // Overdetermined

        // 1. Chord-length parameterization in [0, 1]
        std::vector<double> u(num_points);
        u[0] = 0.0;
        for (int i = 1; i < num_points; ++i)
            u[i] = u[i - 1] + glm::length(points[i] - points[i - 1]);
        double total_length = u.back();
        for (double &val : u)
            val /= total_length;

        // 2. Normalized knot vector in [0, 1]
        int m = num_ctrl_points + k + 1;
        std::vector<double> knots(m);

        // Clamped: first and last k+1 knots = 0 and 1
        for (int i = 0; i <= k; ++i)
            knots[i] = 0.0;
        for (int i = 1; i <= num_ctrl_points - k - 1; ++i)
            knots[k + i] = double(i) / (num_ctrl_points - k);
        for (int i = m - k - 1; i < m; ++i)
            knots[i] = 1.0;

        // 3. Build basis matrix N (num_points × num_ctrl_points)
        MatrixXd N(num_points, num_ctrl_points);
        for (int i = 0; i < num_points; ++i) {
            for (int j = 0; j < num_ctrl_points; ++j) {
                N(i, j) = basis_function(j, k, u[i], knots);
            }
        }

        // 4. Build target data matrix D (num_points × 2)
        MatrixXd D(num_points, 2);
        for (int i = 0; i < num_points; ++i) {
            D(i, 0) = points[i].x;
            D(i, 1) = points[i].y;
        }

        // 5. Solve least squares: N * P = D
        MatrixXd P = N.colPivHouseholderQr().solve(D);

        // 6. Construct control points
        std::vector<PointType> control_points;
        for (int i = 0; i < num_ctrl_points; ++i) {
            control_points.emplace_back(P(i, 0), P(i, 1));
        }

        return BSplineCurve2D(std::move(control_points), std::move(knots), k); // knots saved separately below
    }

    static double basis_function(int i, int k, double u, const std::vector<double> &knots) {
        if (k == 0) {
            // Handle the special case at u = 1
            if (u == 1.0 && i == knots.size() - 2)
                return 1.0;
            return (u >= knots[i] && u < knots[i + 1]) ? 1.0 : 0.0;
        }
        double denom1 = knots[i + k] - knots[i];
        double denom2 = knots[i + k + 1] - knots[i + 1];
        double term1 = 0.0, term2 = 0.0;

        if (denom1 > 1e-8)
            term1 = (u - knots[i]) / denom1 * basis_function(i, k - 1, u, knots);
        if (denom2 > 1e-8)
            term2 = (knots[i + k + 1] - u) / denom2 * basis_function(i + 1, k - 1, u, knots);
        return term1 + term2;
    }
};
} // namespace GraphicsLab::Geometry