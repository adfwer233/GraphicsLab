#include "intersector_test_common.hpp"

#include "geometry/boundary_representation/intersector/surface_surface_intersection/general_surface_surface_intersection.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/torus.hpp"

using namespace GraphicsLab::Geometry;
using namespace GraphicsLab::Geometry::BRep;
using namespace GraphicsLab::Geometry::BRep::IntersectorTest;

TEST(IntersectorFrameworkSSI, PlanePlane) {
    Plane plane_xy({-4.0, -4.0, 0.0}, {8.0, 0.0, 0.0}, {0.0, 8.0, 0.0});
    Plane plane_yz({0.0, -4.0, -4.0}, {0.0, 8.0, 0.0}, {0.0, 0.0, 8.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&plane_xy, &plane_yz);
    ASSERT_GE(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&plane_xy, &plane_yz, res, kDefaultTol);
    }
}

TEST(IntersectorFrameworkSSI, PlaneSphere) {
    Plane plane_xy({-4.0, -4.0, 0.0}, {8.0, 0.0, 0.0}, {0.0, 8.0, 0.0});
    Sphere sphere({0.0, 0.0, 0.0}, 1.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&plane_xy, &sphere);
    ASSERT_GE(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&plane_xy, &sphere, res, kLooseTol);
    }
}

TEST(IntersectorFrameworkSSI, PlaneTorus) {
    Plane plane_xy({-4.0, -4.0, 0.0}, {8.0, 0.0, 0.0}, {0.0, 8.0, 0.0});
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&plane_xy, &torus);
    ASSERT_GE(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&plane_xy, &torus, res, 5e-2);
    }
}


