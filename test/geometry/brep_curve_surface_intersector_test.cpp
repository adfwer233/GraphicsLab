#include "gtest/gtest.h"

#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/intersector/curve_surface_intersection/general_curve_surface_intersection.hpp"
#include "geometry/boundary_representation/validator/rule_based_validator.hpp"
#include "geometry/parametric_topology/brep_face.hpp"

TEST(BRepCurveSurfaceIntersectorTest, LineExplicitIntersection) {
    using namespace GraphicsLab::Geometry::BRep;
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
    Face* face = FaceConstructors::explicit_surface(f);
    auto surface = face->geometry()->param_geometry();

    GraphicsLab::Geometry::StraightLine3D line({-3, 0, -3}, {3, 0, 3});

    auto result = GeneralCurveSurfaceIntersection::solve(&line, surface);

    EXPECT_EQ(result.size(), 2);

    for (auto res: result) {
        EXPECT_TRUE(glm::distance(res.inter_position, surface->evaluate(res.surface_parameter)) < Tolerance::default_tolerance);
        EXPECT_TRUE(glm::distance(res.inter_position, line.evaluate(res.curve_parameter)) < Tolerance::default_tolerance);
        EXPECT_TRUE(glm::distance(surface->evaluate(res.surface_parameter), line.evaluate(res.curve_parameter)) < Tolerance::default_tolerance);
    }
}