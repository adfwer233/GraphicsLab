#include "gtest/gtest.h"

#include "geometry/boundary_representation/constructors/constructors.hpp"
#include "geometry/boundary_representation/validator/rule_based_validator.hpp"

TEST(BRepConstructorTests, CubeConstructorTest) {
    using namespace GraphicsLab::Geometry::BRep;

    auto cube = BodyConstructors::cube({0.0, 0.0, 0.0},{1.0, 1.0, 1.0});

    // EXPECT_TRUE(RuleBasedValidator::validate(cube));

}