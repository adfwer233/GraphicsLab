#pragma once

#include "glm/glm.hpp"

#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Plane : public ParamSurface {
    PointType base_point_;
    VectorType u_direction_;
    VectorType v_direction_;
    VectorType normal_;

    explicit Plane() = delete;
    explicit Plane(PointType base_point, VectorType u_direction, VectorType v_direction)
        : base_point_(base_point), u_direction_(u_direction), v_direction_(v_direction) {
        // u_direction_ = glm::normalize(u_direction);
        // v_direction_ = glm::normalize(v_direction);
        normal_ = glm::normalize(glm::cross(u_direction, v_direction));
    }

    [[nodiscard]] PointType evaluate(const ParamType param) const override {
        return base_point_ + u_direction_ * param.x + v_direction_ * param.y;
    }

    std::pair<VectorType, VectorType> derivative(const ParamType) const override {
        return {u_direction_, v_direction_};
    }

    [[nodiscard]] PointType normal(const ParamType param) const override {
        return normal_;
    }

    [[nodiscard]] std::pair<PointType, ParamType> project(const PointType point) const override {
        VectorType diff = point - base_point_;

        // Build local coordinate system projection
        double u = glm::dot(diff, glm::normalize(u_direction_));
        double v = glm::dot(diff, glm::normalize(v_direction_));

        PointType proj_point = base_point_ + glm::normalize(u_direction_) * u + glm::normalize(v_direction_) * v;
        return {proj_point, ParamType(u / glm::length(u_direction_), v / glm::length(v_direction_))};
    }

    [[nodiscard]] bool test_point(const PointType point) const override {
        VectorType diff = point - base_point_;
        double distance = glm::dot(diff, normal_);
        return std::abs(distance) < 1e-6; // Accept small floating point errors
    }
};
} // namespace GraphicsLab::Geometry