#include "gtest/gtest.h"

#include "geometry/parametric/nurbs_surface.hpp"

namespace {

using GraphicsLab::Geometry::NURBSSurface;

NURBSSurface create_bilinear_surface(const std::vector<std::vector<double>> &weights) {
    std::vector<std::vector<glm::dvec3>> control_points = {
        {glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0)},
        {glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(1.0, 1.0, 0.0)},
    };

    return NURBSSurface(control_points, {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, weights, 1, 1);
}

TEST(NURBSSurfaceTest, UnitWeightsProduceBilinearPatch) {
    NURBSSurface surface = create_bilinear_surface({{1.0, 1.0}, {1.0, 1.0}});

    for (const glm::dvec2 param : {glm::dvec2(0.1, 0.2), glm::dvec2(0.5, 0.5), glm::dvec2(0.8, 0.25)}) {
        const glm::dvec3 p = surface.evaluate(param);
        EXPECT_NEAR(p.x, param.x, 1e-9);
        EXPECT_NEAR(p.y, param.y, 1e-9);
        EXPECT_NEAR(p.z, 0.0, 1e-9);
    }
}

TEST(NURBSSurfaceTest, ClampedEndpointsInterpolateCorners) {
    NURBSSurface surface = create_bilinear_surface({{1.0, 2.0}, {3.0, 4.0}});

    const glm::dvec3 p00 = surface.evaluate({0.0, 0.0});
    const glm::dvec3 p11 = surface.evaluate({1.0, 1.0});

    EXPECT_NEAR(p00.x, 0.0, 1e-10);
    EXPECT_NEAR(p00.y, 0.0, 1e-10);
    EXPECT_NEAR(p00.z, 0.0, 1e-10);

    EXPECT_NEAR(p11.x, 1.0, 1e-10);
    EXPECT_NEAR(p11.y, 1.0, 1e-10);
    EXPECT_NEAR(p11.z, 0.0, 1e-10);
}

TEST(NURBSSurfaceTest, DerivativeAndNormalOnPlane) {
    NURBSSurface surface = create_bilinear_surface({{1.0, 1.0}, {1.0, 1.0}});

    const auto [du, dv] = surface.derivative({0.37, 0.69});
    const glm::dvec3 n = surface.normal({0.37, 0.69});

    EXPECT_NEAR(du.x, 1.0, 1e-9);
    EXPECT_NEAR(du.y, 0.0, 1e-9);
    EXPECT_NEAR(du.z, 0.0, 1e-9);

    EXPECT_NEAR(dv.x, 0.0, 1e-9);
    EXPECT_NEAR(dv.y, 1.0, 1e-9);
    EXPECT_NEAR(dv.z, 0.0, 1e-9);

    EXPECT_NEAR(n.x, 0.0, 1e-9);
    EXPECT_NEAR(n.y, 0.0, 1e-9);
    EXPECT_NEAR(n.z, 1.0, 1e-9);
}

TEST(NURBSSurfaceTest, WeightsInfluenceRationalShape) {
    std::vector<std::vector<glm::dvec3>> control_points = {
        {glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0)},
        {glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(1.0, 1.0, 1.0)},
    };

    NURBSSurface uniform(control_points, {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, {{1.0, 1.0}, {1.0, 1.0}}, 1,
                         1);
    NURBSSurface biased(control_points, {0.0, 0.0, 1.0, 1.0}, {0.0, 0.0, 1.0, 1.0}, {{1.0, 1.0}, {1.0, 8.0}}, 1, 1);

    const glm::dvec3 p_uniform = uniform.evaluate({0.75, 0.75});
    const glm::dvec3 p_biased = biased.evaluate({0.75, 0.75});

    EXPECT_GT(p_biased.z, p_uniform.z);
}

} // namespace

