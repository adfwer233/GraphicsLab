#include "gtest/gtest.h"

#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/parametric_curves/ellipse.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"

#include <numbers>

namespace {

void expect_vec2_near(const glm::dvec2 &lhs, const glm::dvec2 &rhs, double eps) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
}

void expect_vec3_near(const glm::dvec3 &lhs, const glm::dvec3 &rhs, double eps) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
    EXPECT_NEAR(lhs.z, rhs.z, eps);
}

TEST(ParametricCurvePrimitivesTest, StraightLine2DBasicOperations) {
    using GraphicsLab::Geometry::StraightLine2D;

    StraightLine2D line({1.0, 2.0}, {5.0, 6.0});

    expect_vec2_near(line.evaluate(0.0), {1.0, 2.0}, 1e-12);
    expect_vec2_near(line.evaluate(1.0), {5.0, 6.0}, 1e-12);
    expect_vec2_near(line.evaluate(0.25), {2.0, 3.0}, 1e-12);

    expect_vec2_near(line.derivative(0.5), {4.0, 4.0}, 1e-12);
    expect_vec2_near(line.second_derivative(0.5), {0.0, 0.0}, 1e-12);

    const auto [projection, t] = line.projection({10.0, 3.0}, std::nullopt);
    EXPECT_NEAR(t, 1.0, 1e-12);
    expect_vec2_near(projection, {5.0, 6.0}, 1e-12);

    expect_vec2_near(line.normal(0.5), {-4.0, 4.0}, 1e-12);
    EXPECT_FALSE(line.is_closed());
}

TEST(ParametricCurvePrimitivesTest, Ellipse2DBasicOperations) {
    using GraphicsLab::Geometry::Ellipse2D;

    Ellipse2D ellipse({1.0, -1.0}, {3.0, 0.0}, {0.0, 2.0});

    expect_vec2_near(ellipse.evaluate(0.0), {4.0, -1.0}, 1e-12);
    expect_vec2_near(ellipse.evaluate(0.25), {1.0, 1.0}, 1e-12);
    expect_vec2_near(ellipse.evaluate(0.5), {-2.0, -1.0}, 1e-12);

    const glm::dvec2 derivative = ellipse.derivative(0.0);
    expect_vec2_near(derivative, {0.0, 4.0 * std::numbers::pi}, 1e-12);

    const glm::dvec2 second = ellipse.second_derivative(0.0);
    expect_vec2_near(second, {-12.0 * std::numbers::pi * std::numbers::pi, 0.0}, 1e-12);

    EXPECT_TRUE(ellipse.is_closed());

    const auto [projection, param] = ellipse.projection({10.0, -1.0}, std::nullopt);
    EXPECT_NEAR(param, 0.0, 1e-6);
    expect_vec2_near(projection, {4.0, -1.0}, 1e-6);
}

TEST(ParametricCurvePrimitivesTest, DegeneratedCurve3DIsConstant) {
    using GraphicsLab::Geometry::DegeneratedCurve3D;

    DegeneratedCurve3D curve({1.0, 2.0, 3.0});

    expect_vec3_near(curve.evaluate(0.0), {1.0, 2.0, 3.0}, 1e-12);
    expect_vec3_near(curve.evaluate(1.0), {1.0, 2.0, 3.0}, 1e-12);
    expect_vec3_near(curve.derivative(0.5), {0.0, 0.0, 0.0}, 1e-12);
    expect_vec3_near(curve.second_derivative(0.5), {0.0, 0.0, 0.0}, 1e-12);

    const auto [projection, param] = curve.projection({100.0, -1.0, 8.0}, std::nullopt);
    expect_vec3_near(projection, {1.0, 2.0, 3.0}, 1e-12);
    EXPECT_NEAR(param, 0.0, 1e-12);

    EXPECT_TRUE(curve.is_closed());
}

TEST(ParametricCurvePrimitivesTest, ParametricCurveSamplingAndDiscretization) {
    using GraphicsLab::Geometry::Ellipse2D;

    Ellipse2D ellipse({0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0});

    const auto samples = ellipse.sample(5);
    ASSERT_EQ(samples.size(), 5U);
    EXPECT_NEAR(samples.front().second, 0.0, 1e-12);
    EXPECT_NEAR(samples.back().second, 1.0, 1e-12);

    const auto first_discrete = ellipse.discretize();
    const auto second_discrete = ellipse.discretize();

    ASSERT_EQ(first_discrete.size(), 100U);
    ASSERT_EQ(second_discrete.size(), 100U);

    expect_vec2_near(first_discrete.front(), ellipse.evaluate(0.0), 1e-12);
    expect_vec2_near(first_discrete.back(), ellipse.evaluate(99.0 / 100.0), 1e-12);
    expect_vec2_near(second_discrete[42], first_discrete[42], 1e-12);
}

} // namespace

