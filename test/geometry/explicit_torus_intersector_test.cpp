#include "gtest/gtest.h"

#include "geometry/constructor/explicit_surface_constructors.hpp"
#include "geometry/parametric_intersector/surface_surface_intersector.hpp"

#include <geometry/parametric/explicit_surface.hpp>
#include <geometry/parametric/torus.hpp>
#include <numbers>

TEST(ExplicitTorusIntersection, Intersect1) {
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

    GraphicsLab::Geometry::ExplicitSurface surf(f);
    GraphicsLab::Geometry::Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});

    auto test = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect(torus, surf);

    // for (auto info: test) {
    //     spdlog::info("{} {}", info.param1.x, info.param2.y);
    // }

    spdlog::info(test.size());

    auto result = GraphicsLab::Geometry::SurfaceSurfaceIntersector::intersect_all(torus, surf);

    EXPECT_EQ(result.traces.size(), 2);
}