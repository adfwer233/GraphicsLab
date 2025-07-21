#pragma once

#include "glm/glm.hpp"

#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Plane : public ParamSurface {
    PointType base_point;
    VectorType u_direction;
    VectorType v_direction;
    VectorType normal_;

    explicit Plane() = delete;
    explicit Plane(PointType base_point, VectorType u_direction, VectorType v_direction)
        : base_point(base_point), u_direction(u_direction), v_direction(v_direction) {
        u_direction = glm::normalize(u_direction);
        v_direction = glm::normalize(v_direction);
        normal_ = glm::cross(u_direction, v_direction);
    }

    [[nodiscard]] PointType evaluate(const ParamType param) const override {
        return base_point + u_direction * param.x + v_direction * param.y;
    }

    [[nodiscard]] PointType normal(const ParamType param) const override {
        return normal_;
    }

    [[nodiscard]] std::pair<PointType, ParamType> project(const PointType point) const override {
        VectorType diff = point - base_point;

        // Build local coordinate system projection
        double u = glm::dot(diff, u_direction);
        double v = glm::dot(diff, v_direction);

        PointType proj_point = base_point + u_direction * u + v_direction * v;
        return {proj_point, ParamType(u, v)};
    }

    [[nodiscard]] bool test_point(const PointType point) const override {
        VectorType diff = point - base_point;
        double distance = glm::dot(diff, normal_);
        return std::abs(distance) < 1e-6; // Accept small floating point errors
    }
};
} // namespace GraphicsLab::Geometry