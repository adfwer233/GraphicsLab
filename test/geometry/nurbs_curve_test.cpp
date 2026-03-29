#include "gtest/gtest.h"

#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/nurbs_curve_2d.hpp"
#include "geometry/parametric/nurbs_curve_3d.hpp"

namespace {

using GraphicsLab::Geometry::BSplineCurve2D;
using GraphicsLab::Geometry::NURBSCurve2D;
using GraphicsLab::Geometry::NURBSCurve3D;

TEST(NURBSCurveTest, UnitWeightsMatchBSpline) {
    std::vector<glm::dvec2> control_points{{0.0, 0.0}, {0.25, 0.9}, {0.8, 0.5}, {1.0, 0.0}};
    std::vector<double> knots{0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    std::vector<double> weights(control_points.size(), 1.0);

    BSplineCurve2D bspline(control_points, knots, 2);
    NURBSCurve2D nurbs(control_points, knots, weights, 2);

    for (double t : {0.0, 0.1, 0.25, 0.5, 0.75, 1.0}) {
        const glm::dvec2 p_bspline = bspline.evaluate(t);
        const glm::dvec2 p_nurbs = nurbs.evaluate(t);
        EXPECT_NEAR(p_bspline.x, p_nurbs.x, 1e-9);
        EXPECT_NEAR(p_bspline.y, p_nurbs.y, 1e-9);
    }
}

TEST(NURBSCurveTest, ClampedEndpointsInterpolate) {
    std::vector<glm::dvec2> control_points{{0.0, 0.0}, {0.4, 1.0}, {0.8, -0.5}, {1.0, 0.0}};
    std::vector<double> knots{0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0};
    std::vector<double> weights{1.0, 0.5, 2.0, 1.0};

    NURBSCurve2D nurbs(control_points, knots, weights, 2);

    const glm::dvec2 start = nurbs.evaluate(0.0);
    const glm::dvec2 end = nurbs.evaluate(1.0);

    EXPECT_NEAR(start.x, control_points.front().x, 1e-10);
    EXPECT_NEAR(start.y, control_points.front().y, 1e-10);
    EXPECT_NEAR(end.x, control_points.back().x, 1e-10);
    EXPECT_NEAR(end.y, control_points.back().y, 1e-10);
}

TEST(NURBSCurveTest, DerivativeMatchesFiniteDifference) {
    std::vector<glm::dvec2> control_points{{0.0, 0.0}, {0.3, 1.0}, {0.7, -0.1}, {1.0, 0.2}};
    std::vector<double> knots{0.0, 0.0, 0.0, 0.45, 1.0, 1.0, 1.0};
    std::vector<double> weights{1.0, 2.0, 0.8, 1.0};

    NURBSCurve2D nurbs(control_points, knots, weights, 2);

    constexpr double t = 0.37;
    constexpr double h = 1e-6;

    const glm::dvec2 p_minus = nurbs.evaluate(t - h);
    const glm::dvec2 p_plus = nurbs.evaluate(t + h);
    const glm::dvec2 derivative_fd = (p_plus - p_minus) / (2.0 * h);
    const glm::dvec2 derivative = nurbs.derivative(t);

    EXPECT_NEAR(derivative.x, derivative_fd.x, 5e-5);
    EXPECT_NEAR(derivative.y, derivative_fd.y, 5e-5);
}

TEST(NURBSCurveTest, WeightChangesCurveShape) {
    std::vector<glm::dvec2> control_points{{1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    std::vector<double> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};

    NURBSCurve2D uniform_weights(control_points, knots, {1.0, 1.0, 1.0}, 2);
    NURBSCurve2D biased_weights(control_points, knots, {1.0, 10.0, 1.0}, 2);

    const glm::dvec2 p_uniform = uniform_weights.evaluate(0.5);
    const glm::dvec2 p_biased = biased_weights.evaluate(0.5);

    EXPECT_GT(p_biased.x, p_uniform.x);
    EXPECT_GT(p_biased.y, p_uniform.y);
}

TEST(NURBSCurveTest, NURBS3DAliasIsUsable) {
    std::vector<glm::dvec3> control_points{{0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, {1.0, 0.0, 0.0}};
    std::vector<double> knots{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    std::vector<double> weights{1.0, 1.0, 1.0};

    NURBSCurve3D curve(control_points, knots, weights, 2);
    const glm::dvec3 p = curve.evaluate(0.5);

    EXPECT_NEAR(p.x, 0.5, 1e-9);
    EXPECT_NEAR(p.y, 0.25, 1e-9);
    EXPECT_NEAR(p.z, 0.25, 1e-9);
}

} // namespace

