#include "intersector_test_common.hpp"

#include "geometry/boundary_representation/intersector/curve_surface_intersection/general_curve_surface_intersection.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/torus.hpp"

using namespace GraphicsLab::Geometry;
using namespace GraphicsLab::Geometry::BRep;
using namespace GraphicsLab::Geometry::BRep::IntersectorTest;

TEST(IntersectorFrameworkCSI, LinePlane) {
    StraightLine3D line_plane({0.25, 0.25, -1.0}, {0.25, 0.25, 1.0});
    Plane plane({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});

    const auto results = GeneralCurveSurfaceIntersection::solve(&line_plane, &plane);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_csi_residual_ok(&line_plane, &plane, res, kTightTol);
    }
}

TEST(IntersectorFrameworkCSI, LineSphere) {
    StraightLine3D line_sphere({-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    Sphere sphere({0.0, 0.0, 0.0}, 1.0);

    const auto results = GeneralCurveSurfaceIntersection::solve(&line_sphere, &sphere);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_csi_residual_ok(&line_sphere, &sphere, res, kDefaultTol);
    }
}

TEST(IntersectorFrameworkCSI, LineTorus) {
    StraightLine3D line_torus({-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralCurveSurfaceIntersection::solve(&line_torus, &torus);
    ASSERT_EQ(results.size(), 4);
    for (const auto &res : results) {
        expect_csi_residual_ok(&line_torus, &torus, res, kLooseTol);
    }
}



