#pragma once

#include "glm/glm.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Cylinder : public ParamSurface {
    explicit Cylinder() = delete;
    explicit Cylinder(const PointType &base_point, const VectorType &direction, const VectorType &radius, double length,
                      double radius_ratio = 1.0)
        : base_point_(base_point), direction_(direction), radius_(radius), length_(length) {
        minor_radius_ = glm::cross(direction, radius) * radius_ratio;
    }

    Cylinder(Cylinder &&rhs) noexcept {
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
        return base_point_ + base_dir + direction_ * length_ * param.y;
    }

    std::pair<PointType, PointType> derivative(const ParamType param) const override {
        const double theta = 2 * std::numbers::pi * param.x;
        VectorType dx = 2 * std::numbers::pi * (-radius_ * std::sin(theta) + minor_radius_ * std::cos(theta));
        VectorType dy = -(radius_ * std::cos(theta) + minor_radius_ * std::sin(theta)) + direction_ * length_;
        return {dx, dy};
    }

    bool is_singular(ParamType) const override {
        return false;
    }

    PointType normal(const ParamType param) const override {
        auto [dx, dy] = derivative(param);
        return glm::normalize(glm::cross(dx, dy));
    }

    bool test_point(const PointType point) const override {
        auto [proj, proj_param] = project(point);

        return glm::distance(proj, point) < 1e-6;
    }

    std::pair<PointType, ParamType> project(const PointType point) const override {
        double normal_dir_len = glm::dot(point - base_point_, direction_);

        // project to the base plane
        auto base_plane_proj = point - normal_dir_len * direction_;

        // @todo: handle non-circle case
        if (std::abs(glm::length(radius_) - glm::length(minor_radius_)) < 1e-6) {
            auto x = glm::dot(base_plane_proj - base_point_, radius_);
            auto y = glm::dot(base_plane_proj - base_point_, minor_radius_);

            double theta = std::atan2(y, x);
            double v = std::clamp(normal_dir_len, 0.0, 1.0);
            return {base_point_ + glm::length(radius_) * glm::normalize(base_plane_proj - base_point_) + direction_ * v,
                    {theta, v}};
        } else {
            throw cpptrace::logic_error("Not implemented");
        }
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