#include "gtest/gtest.h"

#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/validator/rule_based_validator.hpp"
#include "geometry/parametric_topology/brep_face.hpp"

TEST(BRepFaceTest, PlaneConstructTest) {
    using namespace GraphicsLab::Geometry::BRep;
    Face* face = FaceConstructors::plane({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});

    EXPECT_TRUE(RuleBasedValidator::validate(face));
}

TEST(BRepFaceTest, ExplicitSurfaceConstructTest) {
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

    auto faces = TopologyUtils::get_all_edges(face);

    EXPECT_EQ(faces.size(), 2);
}