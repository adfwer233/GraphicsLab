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

TEST(IntersectorFrameworkSSI, SphereSphereNoIntersection) {
    Sphere sphere1({0.0, 0.0, 0.0}, 1.0);
    Sphere sphere2({3.5, 0.0, 0.0}, 1.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere1, &sphere2);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, SphereSphereSingleBranch) {
    Sphere sphere1({0.0, 0.0, 0.0}, 2.0);
    Sphere sphere2({1.0, 0.0, 0.0}, 2.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere1, &sphere2);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere1, &sphere2, res, kLooseTol);
    }
}

TEST(IntersectorFrameworkSSI, SphereSphereTangentialSingleBranch) {
    Sphere sphere1({0.0, 0.0, 0.0}, 1.0);
    Sphere sphere2({2.0, 0.0, 0.0}, 1.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere1, &sphere2);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere1, &sphere2, res, kDefaultTol);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusNoIntersection) {
    Sphere sphere({0.0, 0.0, 0.0}, 4.0);
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, SphereTorusTwoBranches) {
    Sphere sphere({0.0, 0.0, 0.0}, 2.0);
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere, &torus, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusTangentialSingleBranch) {
    Sphere sphere({0.0, 0.0, 0.0}, 2.5);
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere, &torus, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetNoIntersection) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Sphere sphere({0.0, 0.0, 1.2}, 1.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetNoIntersectionSwappedOrder) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Sphere sphere({0.0, 0.0, 1.2}, 1.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus, &sphere);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetTangentialSingleBranch) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    constexpr double dz = 1.2;
    const double tangential_radius = std::sqrt(2.0 * 2.0 + dz * dz) - 0.5;
    Sphere sphere({0.0, 0.0, dz}, tangential_radius);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere, &torus, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetTangentialSingleBranchSwappedOrder) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    constexpr double dz = 1.2;
    const double tangential_radius = std::sqrt(2.0 * 2.0 + dz * dz) - 0.5;
    Sphere sphere({0.0, 0.0, dz}, tangential_radius);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus, &sphere);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus, &sphere, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetTwoBranches) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Sphere sphere({0.0, 0.0, 1.2}, 2.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&sphere, &torus);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&sphere, &torus, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, SphereTorusCoaxialOffsetTwoBranchesSwappedOrder) {
    Torus torus({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Sphere sphere({0.0, 0.0, 1.2}, 2.0);

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus, &sphere);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus, &sphere, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusNoIntersection) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.0}, 0.5, 0.2, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, TorusTorusTangentialSingleBranch) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.0}, 1.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusTwoBranches) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.0}, 1.5, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusTwoBranchesSwappedOrder) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.0}, 1.5, 0.5, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});

    const auto results12 = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    const auto results21 = GeneralSurfaceSurfaceIntersection::solve(&torus2, &torus1);

    ASSERT_EQ(results12.size(), 2);
    ASSERT_EQ(results21.size(), 2);

    for (const auto &res : results12) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
    for (const auto &res : results21) {
        expect_ssi_curve_residual_ok(&torus2, &torus1, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusAntiparallelNormalSingleBranch) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.0}, 1.0, 0.5, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusCoaxialOffsetNoIntersection) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 1.2}, 2.0, 0.5, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    EXPECT_TRUE(results.empty());
}

TEST(IntersectorFrameworkSSI, TorusTorusCoaxialOffsetTangentialSingleBranch) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 1.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    ASSERT_EQ(results.size(), 1);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
}

TEST(IntersectorFrameworkSSI, TorusTorusCoaxialOffsetTwoBranches) {
    Torus torus1({0.0, 0.0, 0.0}, 2.0, 0.5, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    Torus torus2({0.0, 0.0, 0.4}, 2.0, 0.5, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});

    const auto results = GeneralSurfaceSurfaceIntersection::solve(&torus1, &torus2);
    ASSERT_EQ(results.size(), 2);
    for (const auto &res : results) {
        expect_ssi_curve_residual_ok(&torus1, &torus2, res, 5e-2);
    }
}


