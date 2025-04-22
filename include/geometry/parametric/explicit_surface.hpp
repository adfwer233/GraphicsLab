#pragma once

#include "glm/glm.hpp"
#include "parametric_surface.hpp"
#include "geometry/autodiff/autodiff.hpp"

namespace GraphicsLab::Geometry {

struct ExplicitSurface: ParamSurface {
    struct SurfaceTrait{};
    struct SceneTreeGeometryTypeTrait{};

    using PointType = glm::dvec3;
    using VectorType = glm::dvec3;
    using ParamType = glm::dvec2;

    using ExpressionType = std::function<autodiff_vec3(autodiff_vec2)>;
    ExpressionType f;

    explicit ExplicitSurface(const ExpressionType &f) : f(f) {
        v_periodic = v_periodic_check();
        u_periodic = u_periodic_check();
    }

    ExplicitSurface(ExplicitSurface &&other) noexcept {
        mesh = std::move(other.mesh);
    }

    [[nodiscard]] PointType evaluate(const ParamType t_param) const override {
        auto param = move_param_to_std_domain(t_param);
        autodiff_vec2 p(param.x, param.y);
        auto pos = f(p);
        return {pos.x(), pos.y(), pos.z()};
    }

    [[nodiscard]] PointType normal(const ParamType t_param) const override {
        auto param = move_param_to_std_domain(t_param);

        VectorType du = derivative_u(param);
        VectorType dv = derivative_v(param);

        return glm::normalize(glm::cross(du, dv));
    }

    [[nodiscard]] VectorType derivative_u(const ParamType t_param) const {
        auto param = move_param_to_std_domain(t_param);

        autodiff_vec2 p(param.x, param.y);
        auto pos = f(p);

        // using namespace autodiff;

        auto [dx] = autodiff::derivatives(pos.x(), autodiff::wrt(p.x()));
        auto [dy] = autodiff::derivatives(pos.y(), autodiff::wrt(p.x()));
        auto [dz] = autodiff::derivatives(pos.z(), autodiff::wrt(p.x()));

        return {dx, dy, dz};
    }

    [[nodiscard]] VectorType derivative_v(const ParamType t_param) const {
        auto param = move_param_to_std_domain(t_param);

        autodiff_vec2 p(param.x, param.y);
        auto pos = f(p);

        // using namespace autodiff;

        auto [dx] = autodiff::derivatives(pos.x(), autodiff::wrt(p.y()));
        auto [dy] = autodiff::derivatives(pos.y(), autodiff::wrt(p.y()));
        auto [dz] = autodiff::derivatives(pos.z(), autodiff::wrt(p.y()));

        return {dx, dy, dz};
    }

    std::pair<VectorType, VectorType> derivative(const ParamType t_param) const {
        auto param = move_param_to_std_domain(t_param);

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
            // param = glm::clamp(param, glm::dvec2(0.0), glm::dvec2(1.0));

            if (glm::length(grad) < epsilon)
                break;
        }

        return {evaluate(param), param};
    }

    bool test_point(const PointType point) const override {
        auto [pos, param] = project(point);
        return glm::length(pos - point) < 1e-3;
    }
};

}