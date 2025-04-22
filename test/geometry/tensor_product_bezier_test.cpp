#include "gtest/gtest.h"

#include "geometry/constructor/tensor_product_bezier_example.hpp"
#include "geometry/parametric/tensor_product_bezier.hpp"

#include "utils/sampler.hpp"

#include "spdlog/spdlog.h"

TEST(TensorProductBezierTest, TestConstructor) {
    using Point = GraphicsLab::Geometry::TensorProductBezier::PointType;

    std::vector<std::vector<Point>> control_points = {{Point(0.0, 0.0, 0.0), Point(0.0, 1.0, 0.0)},
                                                      {Point(1.0, 0.0, 0.0), Point(1.0, 1.0, 0.0)}};

    GraphicsLab::Geometry::TensorProductBezier surface(control_points);

    EXPECT_EQ(surface.degree_u, 1);
    EXPECT_EQ(surface.degree_v, 1);
}

TEST(TensorProductBezierTest, TestDerivative) {
    auto surf = GraphicsLab::Geometry::TensorProductBezierExample1::create();
    auto du = surf.derivative_u({0.5, 0.5});
    auto dv = surf.derivative_v({0.5, 0.5});

    spdlog::info("du {} {} {}", du.x, du.y, du.z);
    spdlog::info("dv {} {} {}", dv.x, dv.y, dv.z);
}

TEST(TensorProductBezierTest, TestNormal) {
    auto surf = GraphicsLab::Geometry::TensorProductBezierExample1::create();

    auto n = surf.normal({0.5, 0.5});

    spdlog::info("{} {} {}", n.x, n.y, n.z);

    EXPECT_FLOAT_EQ(n.x, 0.0);
    EXPECT_FLOAT_EQ(n.y, 0.0);
    EXPECT_FLOAT_EQ(n.z, 1.0);
}

TEST(TensorProductBezierTest, TestNormal2) {
    auto surf = GraphicsLab::Geometry::TensorProductBezierExample2::create();

    auto n = surf.normal({0.5, 0.5});

    spdlog::info("{} {} {}", n.x, n.y, n.z);
}

TEST(TensorProductBezierTest, TestProjection) {
    auto surf = GraphicsLab::Geometry::TensorProductBezierExample2::create();

    for (int i = 0; i < 10; i++) {
        glm::dvec3 pos = GraphicsLab::Sampler::sampleUnitSphere<3>();
        auto [proj, param] = surf.project(pos);

        auto normal = surf.normal(param);

        auto inner_prod = glm::dot(glm::normalize(normal), glm::normalize(pos - proj));

        spdlog::info("{}, {} {}", inner_prod, param.x, param.y);
    }
}