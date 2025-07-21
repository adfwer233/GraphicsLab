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