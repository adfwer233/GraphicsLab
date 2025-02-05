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

    PointType evaluate(const ParamType param_to_evaluate) override {
        ParamType param = move_param_to_domain(param_to_evaluate);

        double x = (major_radius + minor_radius * std::cos(2 * std::numbers::pi * param.y)) *
                   std::cos(2 * std::numbers::pi * param.x);
        double y = (major_radius + minor_radius * std::cos(2 * std::numbers::pi * param.y)) *
                   std::sin(2 * std::numbers::pi * param.x);
        double z = minor_radius * std::sin(2 * std::numbers::pi * param.y);

        return center + direction1 * x + direction2 * y + base_normal * z;
    }

    PointType normal(const ParamType &param_to_evaluate) {
        ParamType param = move_param_to_domain(param_to_evaluate);

        PointType position = evaluate(param);
        double major_circle_x = major_radius * std::cos(2 * std::numbers::pi * param.x);
        double major_circle_y = major_radius * std::sin(2 * std::numbers::pi * param.x);
        auto circle_position = center + direction1 * major_circle_x + direction2 * major_circle_y;
        return glm::normalize(position - circle_position);
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