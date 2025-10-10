#pragma once

#include "glm/glm.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Cone : public ParamSurface {
    explicit Cone() = delete;
    explicit Cone(const PointType& base_point, const VectorType& direction, const VectorType& radius, double length, double radius_ratio = 1.0)
            : base_point_(base_point), direction_(direction), radius_(radius), length_(length) {
        minor_radius_ = glm::cross(direction, radius) * radius_ratio;
    }

    Cone(Cone &&rhs) noexcept {
        u_periodic = true;

        base_point_ = rhs.base_point_;
        direction_ = rhs.direction_;
        radius_ = rhs.radius_;
        length_ = rhs.length_;
        minor_radius_ = rhs.minor_radius_;

        mesh = std::move(rhs.mesh);
    }

    PointType evaluate(const ParamType param) const override {
        const double theta = 2 * std::numbers::pi * param.x;
        auto base_dir = radius_ * std::cos(theta) + minor_radius_ * std::sin(theta);
        return base_point_ + base_dir * (1 - param.y) + direction_ * length_ * param.y;
    }

    std::pair<PointType, PointType> derivative(const ParamType param) const override {
        const double theta = 2 * std::numbers::pi * param.x;
        VectorType dx = 2 * std::numbers::pi * (1 - param.y) * (-radius_ * std::sin(theta) + minor_radius_ * std::cos(theta));
        VectorType dy = - (radius_ * std::cos(theta) + minor_radius_ * std::sin(theta)) + direction_ * length_;
        return {dx, dy};
    }

    bool is_singular(ParamType param) const override {
        if (std::abs(param.y - 1.0) < 1e-6) {
            return true;
        }

        return false;
    }

    PointType normal(const ParamType param) const override {
        auto [dx, dy] = derivative(param);
        return glm::normalize(glm::cross(dx, dy));
    }

    // @todo: rewrite projection function for efficiency

  private:
    PointType base_point_;
    VectorType direction_;
    VectorType radius_;
    VectorType minor_radius_;

    double length_;
};
} // namespace GraphicsLab::Geometry