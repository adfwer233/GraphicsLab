#pragma once

#include "glm/glm.hpp"

#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Plane : public ParamSurface {
    PointType base_point;
    VectorType u_direction;
    VectorType v_direction;
    VectorType normal;

    explicit Plane() = delete;
    explicit Plane(PointType base_point, VectorType u_direction, VectorType v_direction)
        : base_point(base_point), u_direction(u_direction), v_direction(v_direction) {
        u_direction = glm::normalize(u_direction);
        v_direction = glm::normalize(v_direction);
        normal = glm::cross(u_direction, v_direction);
    }

    PointType evaluate(const ParamType param) const override {
        return base_point + u_direction * param.x + v_direction * param.y;
    }
};
} // namespace GraphicsLab::Geometry