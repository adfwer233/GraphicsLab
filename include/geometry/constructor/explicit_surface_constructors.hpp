#pragma once

#include "geometry/parametric/explicit_surface.hpp"

namespace GraphicsLab::Geometry {
struct ExplicitSurfaceConstructor {
    static GraphicsLab::Geometry::ExplicitSurface createHyperboloid() {
        auto f = [](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
            GraphicsLab::Geometry::autodiff_vec3 result;

            double a = 2;
            double b = 1;

            auto v = (param.y() - 0.5) * 2;
            result.x() = a * cosh(v) *  cos(2 * std::numbers::pi * param.x());
            result.y() = -b * sinh(v);
            result.z() = a * cosh(v) * sin(2 * std::numbers::pi * param.x());

            return result;
        };

        return GraphicsLab::Geometry::ExplicitSurface(f);
    }
};
}