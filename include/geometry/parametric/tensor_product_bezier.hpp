#pragma once

#include "glm/ext/scalar_constants.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {

struct TensorProductBezier : ParamSurface {
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    using PointType = glm::dvec3;
    using VectorType = glm::dvec3;
    using ParamType = glm::dvec2;

    std::vector<std::vector<PointType>> control_points;

    // Default constructor
    explicit TensorProductBezier() = delete;

    // Constructor with control points
    explicit TensorProductBezier(const std::vector<std::vector<PointType>> &cps) : control_points(cps) {
        degree_u = static_cast<int>(control_points.size()) - 1;
        degree_v = !control_points.empty() ? static_cast<int>(control_points[0].size()) - 1 : 0;
    }

    // Constructor with move semantics
    explicit TensorProductBezier(std::vector<std::vector<PointType>> &&cps) : control_points(std::move(cps)) {
        degree_u = static_cast<int>(control_points.size()) - 1;
        degree_v = !control_points.empty() ? static_cast<int>(control_points[0].size()) - 1 : 0;
    }

    // Copy constructor
    TensorProductBezier(const TensorProductBezier &other)
        : control_points(other.control_points), degree_u(other.degree_u), degree_v(other.degree_v) {
    }

    // Move constructor
    TensorProductBezier(TensorProductBezier &&other) noexcept
        : control_points(std::move(other.control_points)), degree_u(other.degree_u), degree_v(other.degree_v) {
        mesh = std::move(other.mesh);
    }

    // Copy assignment
    TensorProductBezier &operator=(const TensorProductBezier &other) {
        if (this != &other) {
            control_points = other.control_points;
            degree_u = other.degree_u;
            degree_v = other.degree_v;
        }
        return *this;
    }

    // Move assignment
    TensorProductBezier &operator=(TensorProductBezier &&other) noexcept {
        if (this != &other) {
            control_points = std::move(other.control_points);
            degree_u = other.degree_u;
            degree_v = other.degree_v;
            mesh = std::move(other.mesh);
        }
        return *this;
    }

    int degree_u, degree_v;
    [[nodiscard]] PointType evaluate(const ParamType param) const override {
        std::vector<PointType> intermediate;
        for (const auto &row : control_points) {
            intermediate.push_back(de_casteljau_1d(row, param.y));
        }
        return de_casteljau_1d(intermediate, param.x);
    }

    [[nodiscard]] PointType normal(const ParamType param) const override {
        VectorType du = derivative_u(param);
        VectorType dv = derivative_v(param);

        return glm::normalize(glm::cross(du, dv));
    }

    [[nodiscard]] VectorType derivative_u(const ParamType param) const {
        std::vector<PointType> intermediate;
        for (const auto &row : control_points) {
            intermediate.push_back(de_casteljau_1d(row, param.y));
        }
        return de_casteljau_derivative(intermediate, param.x);
    }

    [[nodiscard]] VectorType derivative_v(const ParamType param) const {
        std::vector<PointType> col(degree_v + 1);
        std::vector<PointType> row(degree_u + 1);

        for (size_t i = 0; i <= degree_v; ++i) {
            for (size_t j = 0; j <= degree_u; ++j) {
                row[j] = control_points[j][i];
            }
            col[i] = de_casteljau_1d(row, param.x);
        }
        return de_casteljau_derivative(col, param.y);
    }

    std::pair<VectorType, VectorType> derivative(const ParamType param) const {
        return std::make_pair(derivative_u(param), derivative_v(param));
    }

    std::pair<PointType, ParamType> project(const PointType point) const override {
        // parameters
        constexpr int max_iters = 1000;
        constexpr double step = 0.01;
        constexpr double epsilon = 1e-4;

        ParamType param(0.5, 0.5); // Initial guess

        for (int i = 0; i < max_iters; ++i) {
            PointType s = evaluate(param);
            VectorType du = derivative_u(param);
            VectorType dv = derivative_v(param);
            VectorType diff = s - point;

            glm::dvec2 grad(glm::dot(diff, du), glm::dot(diff, dv));
            param -= step * grad;

            // Clamp to [0,1]^2
            param = glm::clamp(param, glm::dvec2(0.0), glm::dvec2(1.0));

            if (glm::length(grad) < epsilon)
                break;
        }

        return {evaluate(param), param};
    }

    bool test_point(const PointType point) const override {
        auto [pos, param] = project(point);
        return glm::length(pos - point) < 1e-3;
    }

  private:
    // De Casteljau evaluation along 1D Bézier curve
    static PointType de_casteljau_1d(std::vector<PointType> points, double t) {
        const int n = static_cast<int>(points.size());
        for (int k = 1; k < n; ++k) {
            for (int i = 0; i < n - k; ++i) {
                points[i] = (1 - t) * points[i] + t * points[i + 1];
            }
        }
        return points[0];
    }

    // First derivative using De Casteljau differences
    static VectorType de_casteljau_derivative(const std::vector<PointType> &points, double t) {
        const int n = static_cast<int>(points.size()) - 1;
        std::vector<PointType> diff(n);
        for (int i = 0; i < n; ++i) {
            diff[i] = static_cast<double>(n) * (points[i + 1] - points[i]);
        }
        return de_casteljau_1d(diff, t);
    }
};

} // namespace GraphicsLab::Geometry