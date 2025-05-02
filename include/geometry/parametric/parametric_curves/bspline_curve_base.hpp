#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <glm/glm.hpp>
#include <spdlog/spdlog.h>

#include "bezier_curve_base.hpp"

namespace GraphicsLab::Geometry {

template <size_t dim> struct BSplineCurveBase : ParamCurveBase<dim> {
    using PointType = glm::vec<dim, double>;
    using MeshType = std::conditional_t<dim == 3, CurveMesh3D, CurveMesh2D>;

    std::vector<PointType> control_points_;
    std::vector<double> knots_;

    std::vector<PointType> derivative_control_points_;
    std::vector<double> derivative_knots_;

    int degree_;

    BSplineCurveBase(std::vector<PointType> control_points, std::vector<double> knots, int degree)
        : control_points_(std::move(control_points)), knots_(std::move(knots)), degree_(degree) {

        /**
         * Create derivative data for derivative evaluation
         */
        const int n = static_cast<int>(control_points_.size()) - 1;

        for (int i = 0; i < n; ++i) {
            double denom = knots_[i + degree_ + 1] - knots_[i + 1];
            if (denom == 0.0)
                derivative_control_points_.push_back(PointType(0.0));
            else
                derivative_control_points_.push_back(static_cast<double>(degree_) *
                                                     (control_points_[i + 1] - control_points_[i]) / denom);
        }

        // Remove first and last knot to match the reduced degree
        derivative_knots_ = std::vector(knots_.begin() + 1, knots_.end() - 1);

        for (auto pts : control_points_) {
            this->box.max = glm::max(this->box.max, pts);
            this->box.min = glm::min(this->box.min, pts);
        }
    }

    BSplineCurveBase(const BSplineCurveBase<dim> &other) {
        degree_ = other.degree_;
        control_points_ = other.control_points_;
        knots_ = other.knots_;
        this->box = other.box;
        // this->mesh = std::make_unique<MeshType>(*other.mesh);
    }

    BSplineCurveBase(BSplineCurveBase<dim> &&other) noexcept {
        control_points_ = std::move(other.control_points_);
        knots_ = std::move(other.knots_);
        degree_ = other.degree_;
        this->box = other.box;
        this->mesh = std::move(other.mesh);
    }

    PointType evaluate(double u) const override {
        return bspline_evaluate(u, control_points_, knots_, degree_);
    }

    PointType derivative(double param) const override {
        return bspline_evaluate(param, derivative_control_points_, derivative_knots_, degree_ - 1);
    }

    std::vector<BezierCurveBase<dim>> convert_to_bezier() const {
        std::vector<BezierCurveBase<dim>> result;

        const int d = degree_;
        const int num_control_points = static_cast<int>(control_points_.size());

        // Each Bézier segment has (d + 1) control points
        const int num_segments = (num_control_points - 1) / d;

        for (int i = 0; i < num_segments; ++i) {
            std::vector<PointType> bezier_ctrl_pts;
            for (int j = 0; j <= d; ++j) {
                bezier_ctrl_pts.push_back(control_points_[i * d + j]);
            }
            result.emplace_back(std::move(bezier_ctrl_pts));
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

    BSplineCurveBase operator+(const PointType &offset) const {
        BSplineCurveBase new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt += offset;
        }
        return new_curve;
    }

    BSplineCurveBase operator-(const PointType &offset) const {
        BSplineCurveBase new_curve = *this;
        for (auto &pt : new_curve.control_points_) {
            pt -= offset;
        }
        return new_curve;
    }

    bool is_in_bezier_form() const {
        const int d = degree_;
        const int m = static_cast<int>(knots_.size()) - 1;

        // Check start multiplicity
        double start = knots_.front();
        int start_mult = count_multiplicity(start);
        if (start_mult != d + 1)
            return false;

        // Check end multiplicity
        double end = knots_.back();
        int end_mult = count_multiplicity(end);
        if (end_mult != d + 1)
            return false;

        // Check internal knot multiplicities
        for (int i = start_mult; i < m + 1 - end_mult;) {
            double u = knots_[i];
            int mult = 1;
            while (i + mult < m + 1 - end_mult && std::abs(u - knots_[i + mult]) < 1e-8)
                ++mult;
            if (mult != d)
                return false;
            i += mult;
        }
        return true;
    }

    void insert_all_knots_to_bezier_form() {
        const int d = degree_;
        const int m = static_cast<int>(knots_.size()) - 1;
        if (is_in_bezier_form())
            return;

        // Gather unique internal knots and their multiplicities
        double start = knots_.front();
        double end = knots_.back();
        std::map<double, int> multiplicity;

        for (double u : knots_) {
            if (u != start && u != end)
                multiplicity[u]++;
        }

        // Insert knots until each internal knot has multiplicity == degree
        for (const auto &[u, mult] : multiplicity) {
            int to_insert = d - mult;
            for (int i = 0; i < to_insert; ++i)
                insert_knot(u);
        }
    }

    void insert_knot(double u) {
        const int d = degree_;
        int n = static_cast<int>(control_points_.size()) - 1;
        int m = static_cast<int>(knots_.size()) - 1;

        // Find the span index s where u is to be inserted: knots_[s] <= u < knots_[s+1]
        int s = -1;
        for (int i = 0; i < m; ++i) {
            if (knots_[i] - 1e-8 <= u && u < knots_[i + 1]) {
                s = i;
                break;
            }
        }
        // Special case: if u == knots_[m], set s = m - 1
        if (s == -1 && same_knot(u, knots_[m])) {
            s = m - 1;
        }
        if (s == -1) {
            throw std::runtime_error("Invalid knot value to insert.");
        }

        // Count how many times u already exists in the knot vector
        int u_multiplicity = count_multiplicity(u);
        if (u_multiplicity > d) {
            throw std::runtime_error("Knot multiplicity exceeds degree; cannot insert.");
        }

        // Compute new control points
        std::vector<PointType> new_ctrl_pts;
        for (int i = 0; i <= s - d + 1; ++i) {
            new_ctrl_pts.push_back(control_points_[i]);
        }

        for (int i = s - d + 2; i <= s; ++i) {
            double alpha = (u - knots_[i]) / (knots_[i + d] - knots_[i]);
            PointType pt = (1.0 - alpha) * control_points_[i - 1] + alpha * control_points_[i];
            new_ctrl_pts.push_back(pt);
        }

        for (int i = s + 1; i <= n + 1; ++i) {
            new_ctrl_pts.push_back(control_points_[i - 1]);
        }

        // spdlog::info("n {}, {}", n, new_ctrl_pts.size());

        // Insert the knot into the knot vector
        knots_.insert(knots_.begin() + s + 1, u);
        control_points_ = std::move(new_ctrl_pts);

        // spdlog::info("knot {}, ctrl_pts: {}, size diff {}", knots_.size(), control_points_.size(), knots_.size() -
        // control_points_.size());
    }

    static BSplineCurveBase fit(const std::vector<PointType> &points, int degree, int num_ctrl_points) {
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
        MatrixXd D(num_points, dim);
        for (int i = 0; i < num_points; ++i) {
            for (int j = 0; j < dim; j++) {
                D(i, j) = points[i][j];
            }
        }

        // 5. Solve least squares: N * P = D
        MatrixXd P = N.colPivHouseholderQr().solve(D);

        // 6. Construct control points
        std::vector<PointType> control_points;
        for (int i = 0; i < num_ctrl_points; ++i) {
            if constexpr (dim == 2) {
                control_points.emplace_back(P(i, 0), P(i, 1));
            } else {
                control_points.emplace_back(P(i, 0), P(i, 1), P(i, 2));
            }
        }

        return BSplineCurveBase(std::move(control_points), std::move(knots), k); // knots saved separately below
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

  private:
    bool same_knot(double u, double v) {
        return std::abs(u - v) < 1e-8;
    }

    int count_multiplicity(double u, double epsilon = 1e-8) const {
        return std::count_if(knots_.begin(), knots_.end(),
                             [u, epsilon](double v) { return std::abs(v - u) < epsilon; });
    }

    static PointType bspline_evaluate(double u, const std::vector<PointType> &ctrl_pts, std::vector<double> knots,
                                      int degree) {
        using namespace glm;

        const int n = static_cast<int>(ctrl_pts.size()) - 1;
        const int k = degree;

        // Clamp u to [0,1]
        u = std::clamp(u, 0.0, 1.0);

        // Find knot span index s such that knots_[s] <= u < knots_[s+1]
        int s = -1;
        if (u == 1.0) {
            s = n; // Special case at the end
        } else {
            for (int i = k; i <= n; ++i) {
                if (u >= knots[i] && u < knots[i + 1]) {
                    s = i;
                    break;
                }
            }
        }
        if (s == -1)
            return ctrl_pts.back(); // fallback

        // Initialize d[j] = control point at i = s - k + j
        std::vector<PointType> d(k + 1);
        for (int j = 0; j <= k; ++j) {
            d[j] = ctrl_pts[s - k + j];
        }

        // De Boor recursion
        for (int r = 1; r <= k; ++r) {
            for (int j = k; j >= r; --j) {
                int i = s - k + j;
                double alpha = (u - knots[i]) / (knots[i + k - r + 1] - knots[i]);
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
            }
        }

        return d[k];
    }
};

} // namespace GraphicsLab::Geometry