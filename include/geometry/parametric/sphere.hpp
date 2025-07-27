#pragma once

#include "glm/glm.hpp"
#include "parametric_space.hpp"
#include "parametric_surface.hpp"
#include <numbers>

#include "configuration.hpp"

namespace GraphicsLab::Geometry {
struct Sphere : public ParamSurface {
    PointType center{};
    double radius{};

    explicit Sphere() = delete;
    explicit Sphere(const PointType &center, const double radius) : center(center), radius(radius) {
        u_periodic = true;
        v_periodic = false;
    }
    Sphere(Sphere &&rhs) noexcept {
        u_periodic = true;
        v_periodic = false;
        center = rhs.center;
        radius = rhs.radius;
        mesh = std::move(rhs.mesh);
    }

    [[nodiscard]] PointType evaluate(const ParamType param) const override {
        const double theta = 2 * std::numbers::pi * param.x; // Azimuthal angle (0 to 2π)
        const double phi = std::numbers::pi * param.y;       // Polar angle (0 to π)

        return center +
               radius * PointType(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi));
    }

    [[nodiscard]] std::pair<PointType, PointType> derivative(const ParamType param) const override {
        const double theta = 2 * std::numbers::pi * param.x; // Azimuthal angle (0 to 2π)
        const double phi = std::numbers::pi * param.y;       // Polar angle (0 to π)

        VectorType dx = 2 * std::numbers::pi *
                        VectorType(-std::sin(phi) * std::sin(theta), std::sin(phi) * std::cos(theta), std::cos(phi));
        VectorType dy = std::numbers::pi *
                        VectorType(std::cos(phi) * std::cos(theta), std::cos(phi) * std::sin(theta), -std::sin(phi));

        return {dx, dy};
    }

    [[nodiscard]] bool is_singular(ParamType param) const override {
        if (std::abs(param.y) < 1e-6 or std::abs(param.y - 1.0) < 1e-6) {
            return true;
        }
        return false;
    }

    PointType normal(const ParamType param) const override {
        auto pos = evaluate(param);
        return glm::normalize(pos - center);
    }

    bool test_point(const PointType point) const override {
        return std::abs(glm::distance(point, center) - radius) < ParametricConfiguration::system_tolerance;
    }

    std::pair<PointType, ParamType> project(const PointType point) const override {
        auto projection = glm::normalize(point - center) * radius + center;

        auto p = glm::normalize(point - center);

        double phi = std::acos(p.z);
        double theta = 0;
        if (std::abs(std::sin(phi)) > 1e-10) {
            double sin_theta = p.y / std::sin(phi);
            theta = std::asin(sin_theta);
        } else {
            theta = 0;
        }

        return {projection, {theta / (2 * std::numbers::pi), phi / std::numbers::pi}};
    }
};
} // namespace GraphicsLab::Geometry
