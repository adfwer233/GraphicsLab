#include "explicit_surface_examples.hpp"

GraphicsLab::Geometry::ExplicitSurface ExplicitSurfaceExample::createDeformedTorus() {
    auto f = [](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
        GraphicsLab::Geometry::autodiff_vec3 result;

        double R = 2;
        double r = 0.7;

        auto u = 2 * std::numbers::pi * param.x();
        auto v = 2 * std::numbers::pi * (param.y() - 0.5) * 2;
        result.x() = (R + r * (1 + 0.3 * cos(4 * u)) * cos(v)) * cos(u);
        result.y() = -r* (1 + 0.3 * cos(4 * u)) * sin(v);
        result.z() = (R + r* (1 + 0.3 * cos(4 * u)) * cos(v)) * sin(u);

        return result;
    };

    return GraphicsLab::Geometry::ExplicitSurface(f);
}