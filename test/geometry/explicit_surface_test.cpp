#include "gtest/gtest.h"

#include "spdlog/spdlog.h"

#include "geometry/parametric/explicit_surface.hpp"

#include "geometry/autodiff/autodiff.hpp"

#include <numbers>

TEST(TestAutodiff, ReverseSin) {
    autodiff::var x(1.0);
    auto y = sin(x);
}

TEST(TestExplicitSurface, PeriodicityCheck) {
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

    EXPECT_TRUE(surf.u_periodic);
    EXPECT_FALSE(surf.v_periodic);
}