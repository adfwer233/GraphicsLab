#include "gtest/gtest.h"

#include "geometry/parametric/bezier_curve_2d.hpp"
#include "utils/sampler.hpp"

TEST(RationalCurvesTest, RationalBezierCircle) {
    using namespace GraphicsLab::Geometry;

    BezierCurve2D circle_arc({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
    circle_arc.set_weight(1, 1 / std::sqrt(2));

    for (int i = 0; i < 100; i++) {
        auto param = GraphicsLab::Sampler::sampleUniform();
        auto pos = circle_arc.evaluate(param);
        EXPECT_TRUE(std::abs(glm::length(pos) - 1.0) < 1e-6);
    }
}

TEST(RationalCurvesTest, RationalBezierConstructor) {
    using namespace GraphicsLab::Geometry;
    BezierCurve2D circle_arc({{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, {1.0, 1.0, 1.0});
}