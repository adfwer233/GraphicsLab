#include "gtest/gtest.h"

#include "cpptrace/cpptrace.hpp"
#include "geometry/parametric/cylinder.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/tessellator.hpp"
#include "geometry/parametric/torus.hpp"

#include <numbers>

namespace {

void expect_vec3_near(const glm::dvec3 &lhs, const glm::dvec3 &rhs, double eps) {
    EXPECT_NEAR(lhs.x, rhs.x, eps);
    EXPECT_NEAR(lhs.y, rhs.y, eps);
    EXPECT_NEAR(lhs.z, rhs.z, eps);
}

TEST(ParametricSurfacePrimitivesTest, PlaneBasicOperations) {
    using GraphicsLab::Geometry::Plane;

    Plane plane({1.0, 2.0, 3.0}, {2.0, 0.0, 0.0}, {0.0, 3.0, 0.0});

    const glm::dvec3 p = plane.evaluate({0.25, 0.5});
    expect_vec3_near(p, {1.5, 3.5, 3.0}, 1e-12);

    const auto [du, dv] = plane.derivative({0.1, 0.9});
    expect_vec3_near(du, {2.0, 0.0, 0.0}, 1e-12);
    expect_vec3_near(dv, {0.0, 3.0, 0.0}, 1e-12);

    const glm::dvec3 n = plane.normal({0.3, 0.4});
    expect_vec3_near(n, {0.0, 0.0, 1.0}, 1e-12);

    const auto [projection, param] = plane.project({1.5, 3.5, 10.0});
    expect_vec3_near(projection, {1.5, 3.5, 3.0}, 1e-12);
    EXPECT_NEAR(param.x, 0.25, 1e-12);
    EXPECT_NEAR(param.y, 0.5, 1e-12);

    EXPECT_TRUE(plane.test_point({4.0, 8.0, 3.0}));
    EXPECT_FALSE(plane.test_point({4.0, 8.0, 3.2}));
}

TEST(ParametricSurfacePrimitivesTest, SphereBasicOperations) {
    using GraphicsLab::Geometry::Sphere;

    Sphere sphere({1.0, -2.0, 0.5}, 2.0);

    const glm::dvec3 p0 = sphere.evaluate({0.0, 0.0});
    expect_vec3_near(p0, {1.0, -2.0, 2.5}, 1e-12);

    const glm::dvec3 p_equator = sphere.evaluate({0.25, 0.5});
    expect_vec3_near(p_equator, {1.0, 0.0, 0.5}, 1e-12);

    const glm::dvec3 n = sphere.normal({0.25, 0.5});
    expect_vec3_near(n, {0.0, 1.0, 0.0}, 1e-12);

    EXPECT_TRUE(sphere.is_singular({0.2, 0.0}));
    EXPECT_TRUE(sphere.is_singular({0.7, 1.0}));
    EXPECT_FALSE(sphere.is_singular({0.7, 0.5}));

    EXPECT_TRUE(sphere.u_periodic);
    EXPECT_FALSE(sphere.v_periodic);
    EXPECT_TRUE(sphere.u_periodic_check());
    EXPECT_FALSE(sphere.v_periodic_check());

    const glm::dvec3 outside = sphere.center() + glm::normalize(glm::dvec3(1.0, 2.0, 3.0)) * 5.0;
    const auto [projection, param] = sphere.project(outside);

    EXPECT_NEAR(glm::distance(projection, sphere.center()), sphere.radius(), 1e-9);
    EXPECT_NEAR(glm::distance(projection, sphere.evaluate(param)), 0.0, 1e-8);
    EXPECT_TRUE(sphere.test_point(projection));
}

TEST(ParametricSurfacePrimitivesTest, TorusBasicOperations) {
    using GraphicsLab::Geometry::Torus;

    Torus torus({0.0, 0.0, 0.0}, 3.0, 1.0, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const glm::dvec3 p = torus.evaluate({0.0, 0.0});
    expect_vec3_near(p, {4.0, 0.0, 0.0}, 1e-12);

    const auto [du, dv] = torus.derivative({0.2, 0.3});
    const glm::dvec3 n = torus.normal({0.2, 0.3});
    EXPECT_NEAR(glm::dot(glm::normalize(du), n), 0.0, 1e-9);
    EXPECT_NEAR(glm::dot(glm::normalize(dv), n), 0.0, 1e-9);

    EXPECT_TRUE(torus.u_periodic);
    EXPECT_TRUE(torus.v_periodic);
    EXPECT_TRUE(torus.u_periodic_check());
    EXPECT_TRUE(torus.v_periodic_check());

    const glm::dvec3 on_surface = torus.evaluate({0.37, 0.82});
    const auto [projection, proj_param] = torus.project(on_surface);
    EXPECT_NEAR(glm::distance(projection, on_surface), 0.0, 1e-6);
    EXPECT_TRUE(torus.test_point(projection));
    EXPECT_NEAR(glm::distance(projection, torus.evaluate(proj_param)), 0.0, 1e-6);
}

TEST(ParametricSurfacePrimitivesTest, CylinderEvaluateNormalAndPointTest) {
    using GraphicsLab::Geometry::Cylinder;

    Cylinder cylinder({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, 2.0);

    const glm::dvec3 bottom = cylinder.evaluate({0.0, 0.0});
    const glm::dvec3 top = cylinder.evaluate({0.0, 1.0});
    expect_vec3_near(bottom, {1.0, 0.0, 0.0}, 1e-12);
    expect_vec3_near(top, {1.0, 0.0, 2.0}, 1e-12);

    const glm::dvec3 n = cylinder.normal({0.0, 0.5});
    EXPECT_NEAR(glm::length(n), 1.0, 1e-9);

    EXPECT_TRUE(cylinder.test_point(cylinder.evaluate({0.25, 0.2})));
    EXPECT_FALSE(cylinder.test_point({4.0, 4.0, 4.0}));
}

TEST(ParametricSurfacePrimitivesTest, BaseSamplingAndDomainWrap) {
    using GraphicsLab::Geometry::Sphere;

    Sphere sphere({0.0, 0.0, 0.0}, 1.0);

    const auto samples = sphere.sample(4, 3);
    ASSERT_EQ(samples.size(), 12U);

    expect_vec3_near(samples.front().first, sphere.evaluate({0.0, 0.0}), 1e-12);
    expect_vec3_near(samples.back().first, sphere.evaluate({1.0, 1.0}), 1e-12);

    const auto wrapped = sphere.move_param_to_std_domain({2.3, -1.1});
    EXPECT_NEAR(wrapped.x, 0.3, 1e-12);
    EXPECT_NEAR(wrapped.y, -1.1, 1e-12);
}

TEST(ParametricSurfacePrimitivesTest, TessellatorCreatesExpectedMeshSize) {
    using GraphicsLab::Geometry::Plane;
    using GraphicsLab::Geometry::Tessellator;

    Plane plane({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    Tessellator::tessellate(plane, 4, 4);

    ASSERT_NE(plane.mesh, nullptr);
    EXPECT_EQ(plane.mesh->vertices.size(), 25U);
    EXPECT_EQ(plane.mesh->indices.size(), 32U);

    for (const auto &tri : plane.mesh->indices) {
        EXPECT_LT(tri.i, plane.mesh->vertices.size());
        EXPECT_LT(tri.j, plane.mesh->vertices.size());
        EXPECT_LT(tri.k, plane.mesh->vertices.size());
    }
}

} // namespace

