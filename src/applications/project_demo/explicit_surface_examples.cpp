#include "explicit_surface_examples.hpp"

#include <numbers>

GraphicsLab::Geometry::ExplicitSurface ExplicitSurfaceExample::createDeformedTorus(glm::vec3 offset) {
    auto f = [offset](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
        GraphicsLab::Geometry::autodiff_vec3 result;

        double R = 2;
        double r = 0.7;

        auto u = 2 * std::numbers::pi * param.x();
        auto v = 2 * std::numbers::pi * (param.y() - 0.5);
        result.x() = (R + r * (1 + 0.3 * cos(4 * u)) * cos(v)) * cos(u) + offset.x;
        result.y() = -r * (1 + 0.3 * cos(4 * u)) * sin(v) + offset.y;
        result.z() = (R + r * (1 + 0.3 * cos(4 * u)) * cos(v)) * sin(u) + offset.z;

        return result;
    };

    return GraphicsLab::Geometry::ExplicitSurface(f);
}
GraphicsLab::Geometry::ExplicitSurface ExplicitSurfaceExample::createDeformedTorus2(glm::vec3 offset,
                                                                                    glm::vec2 param_offset) {
    auto f = [offset,
              param_offset](GraphicsLab::Geometry::autodiff_vec2 param) -> GraphicsLab::Geometry::autodiff_vec3 {
        GraphicsLab::Geometry::autodiff_vec3 result;

        double R = 2;
        double r = 0.7;

        auto u = 2 * std::numbers::pi * param.x();
        auto v = 2 * std::numbers::pi * (param.y() - 0.5) + param_offset.y;
        result.x() = (R + r * (1 + 0.5 * cos(u + param_offset.x)) * cos(v)) * cos(u) + offset.x;
        result.y() = -r * (1 + 0.5 * cos(u + param_offset.x)) * sin(v) + offset.y;
        result.z() = (R + r * (1 + 0.5 * cos(u + param_offset.x)) * cos(v)) * sin(u) + offset.z;

        return result;
    };

    return GraphicsLab::Geometry::ExplicitSurface(f);
}