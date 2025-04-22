#include "gtest/gtest.h"

#include "geometry/parametric/tensor_product_bezier.hpp"
#include "geometry/constructor/tensor_product_bezier_example.hpp"


TEST(TensorProductBezierTest, TestConstructor) {
    using Point = GraphicsLab::Geometry::TensorProductBezier::PointType;

    std::vector<std::vector<Point>> control_points = {{Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
                                                      {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)}};

    GraphicsLab::Geometry::TensorProductBezier surface(control_points);

    EXPECT_EQ(surface.degree_u, 1);
    EXPECT_EQ(surface.degree_v, 1);
}