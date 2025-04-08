#pragma once

#include "glm/glm.hpp"
#include <numbers>

#include "parametric_space.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {

struct Torus : public ParamSurface {
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    using VecType = PointType;

    PointType center;
    VecType base_normal, direction1, direction2{};

    double major_radius;
    double minor_radius;
    ParametricSpace parametric_space;

    explicit Torus(const PointType &center, const double major_radius, const double minor_radius,
                   const VecType &base_normal, const VecType &direction1)
        : center(center), base_normal(base_normal), direction1(direction1), major_radius(major_radius),
          minor_radius(minor_radius) {
        this->base_normal = glm::normalize(base_normal);
        this->direction1 = glm::normalize(direction1);
        direction2 = glm::normalize(glm::cross(base_normal, direction1));
    }

    Torus(Torus &&rhs) noexcept
        : center(rhs.center), base_normal(rhs.base_normal), direction1(rhs.direction1), direction2(rhs.direction2),
          major_radius(rhs.major_radius), minor_radius(rhs.minor_radius) {
        mesh = std::move(rhs.mesh);
    }

    [[nodiscard]] PointType get_center() const {
        return center;
    }
    [[nodiscard]] double get_major_radius() const {
        return major_radius;
    }
    [[nodiscard]] double get_minor_radius() const {
        return minor_radius;
    }

    PointType evaluate(const ParamType param_to_evaluate) const override {
        ParamType param = move_param_to_domain(param_to_evaluate);

        double x = (major_radius + minor_radius * std::cos(2 * std::numbers::pi * param.y)) *
                   std::cos(2 * std::numbers::pi * param.x);
        double y = (major_radius + minor_radius * std::cos(2 * std::numbers::pi * param.y)) *
                   std::sin(2 * std::numbers::pi * param.x);
        double z = minor_radius * std::sin(2 * std::numbers::pi * param.y);

        return center + direction1 * x + direction2 * y + base_normal * z;
    }

    PointType normal(const ParamType param_to_evaluate) const override {
        ParamType param = move_param_to_domain(param_to_evaluate);

        PointType position = evaluate(param);
        double major_circle_x = major_radius * std::cos(2 * std::numbers::pi * param.x);
        double major_circle_y = major_radius * std::sin(2 * std::numbers::pi * param.x);
        auto circle_position = center + direction1 * major_circle_x + direction2 * major_circle_y;
        return glm::normalize(position - circle_position);
    }

    std::pair<VecType, VecType> derivative(const ParamType param) const {
        double u = 2 * std::numbers::pi * param.x; // major angle
        double v = 2 * std::numbers::pi * param.y; // minor angle

        // Orthonormal basis assumed
        VecType n = glm::normalize(base_normal);
        VecType d1 = glm::normalize(direction1);
        VecType d2 = glm::normalize(direction2);

        double R = major_radius;
        double r = minor_radius;

        // Common terms
        double cos_u = std::cos(u), sin_u = std::sin(u);
        double cos_v = std::cos(v), sin_v = std::sin(v);

        // Partial derivative w.r.t u (around the major circle)
        VecType dP_du = -(R + r * cos_v) * sin_u * d1 + (R + r * cos_v) * cos_u * d2;

        // Partial derivative w.r.t v (around the minor circle)
        VecType dP_dv = -r * sin_v * (cos_u * d1 + sin_u * d2) + r * cos_v * n;

        return {2 * std::numbers::pi * dP_du, 2 * std::numbers::pi * dP_dv};
    }

    std::pair<PointType, ParamType> project(const PointType point) const override {
        // Transform the point into the local coordinate system of the torus
        VecType rel = point - center;

        // Project onto the plane of the torus
        double height = glm::dot(rel, base_normal);
        VecType projected = rel - height * base_normal;

        // Get angle u along the major circle
        double x = glm::dot(projected, direction1);
        double y = glm::dot(projected, direction2);
        double u = std::atan2(y, x);

        // Center of the minor circle in world space
        PointType circle_center = center + major_radius * (std::cos(u) * direction1 + std::sin(u) * direction2);

        // Vector from the minor circle center to the point
        VecType to_point = point - circle_center;

        // Remove any component along base_normal
        VecType radial = to_point - glm::dot(to_point, base_normal) * base_normal;

        // Get angle v around the minor circle
        VecType ortho1 = glm::normalize(base_normal);
        VecType ortho2 = glm::normalize(circle_center - center); // normal to both

        double s = glm::dot(to_point, ortho2);
        double t = glm::dot(to_point, ortho1);
        double v = std::atan2(t, s);

        // Compute closest point on torus
        PointType surface_point = circle_center + minor_radius * (cos(v) * ortho2 + sin(v) * ortho1);

        return {surface_point, {u / (2 * std::numbers::pi), v / (2 * std::numbers::pi)}};
    }

    bool test_point(const PointType point) const override {
        VecType rel = point - center;

        // Distance from the point to the torus axis
        VecType d = rel - dot(rel, base_normal) * base_normal; // projection on torus plane
        double dist_major = glm::length(d);
        double dist_minor = glm::length(rel - d); // height above/below torus plane

        double r = sqrt((dist_major - major_radius) * (dist_major - major_radius) + dist_minor * dist_minor);
        return std::abs(r - minor_radius) < 1e-6;
    }

  private:
    static ParamType move_param_to_domain(const ParamType &param) {
        ParamType result = param;
        while (result.x > 1)
            result.x--;
        while (result.x < 0)
            result.x++;
        while (result.y > 1)
            result.y--;
        while (result.y < 0)
            result.y++;
        return result;
    }
};

} // namespace GraphicsLab::Geometry