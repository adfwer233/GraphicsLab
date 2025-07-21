#include "gtest/gtest.h"

#include "geometry/boundary_representation/constructors/constructors.hpp"

TEST(BRepConstructorTests, CubeConstructorTest) {
    using namespace GraphicsLab::Geometry::BRep;

    auto temp = BodyConstructors::cube({0.0, 0.0, 0.0},{1.0, 1.0, 1.0});
}