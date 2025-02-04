#pragma once

#include "glm/glm.hpp"
#include "parametric_space.hpp"
#include "parametric_surface.hpp"

#include <numbers>

namespace GraphicsLab::Geometry {
    struct Sphere: public ParamSurface {
        PointType center;
        double radius;

        explicit Sphere() = default;
        explicit Sphere(const PointType &center, const double radius): center(center), radius(radius) {}
        explicit Sphere(Sphere&& rhs) {
            center = rhs.center;
            radius = rhs.radius;
            mesh = std::move(rhs.mesh);
        }
        PointType evaluate(const ParamType param) override {
            const double theta = 2 * std::numbers::pi * param.x; // Azimuthal angle (0 to 2π)
            const double phi = std::numbers::pi * param.y;   // Polar angle (0 to π)

            return center + radius * PointType(
                std::sin(phi) * std::cos(theta),
                std::sin(phi) * std::sin(theta),
                std::cos(phi)
            );
        }
    };
}
