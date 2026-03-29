#include "intersector_test_common.hpp"

#include "cpptrace/cpptrace.hpp"
#include "geometry/boundary_representation/intersector/pcurve_pcurve_intersection/general_pcurve_pcurve_intersection.hpp"
#include "geometry/parametric/bezier_curve_2d.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/nurbs_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"

using namespace GraphicsLab::Geometry;
using namespace GraphicsLab::Geometry::BRep;
using namespace GraphicsLab::Geometry::BRep::IntersectorTest;

TEST(IntersectorFrameworkPPI, LineLine) {
    StraightLine2D line1({0.0, 0.0}, {1.0, 1.0});
    StraightLine2D line2({0.0, 1.0}, {1.0, 0.0});

    const auto results = GeneralPCurvePCurveIntersection::solve(&line1, &line2);
    ASSERT_GE(results.size(), 1);
    for (const auto &res : results) {
        expect_ppi_residual_ok(&line1, &line2, res, BRepPoint2(0.0), kLooseTol);
    }
}

TEST(IntersectorFrameworkPPI, BezierBSpline) {
    BezierCurve2D bezier_diag1({BRepPoint2{0.0, 0.0}, BRepPoint2{1.0, 1.0}});
    BSplineCurve2D bspline_diag2({BRepPoint2{0.0, 1.0}, BRepPoint2{1.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, 1);

    const auto results = GeneralPCurvePCurveIntersection::solve(&bezier_diag1, &bspline_diag2);
    ASSERT_GE(results.size(), 1);
    for (const auto &res : results) {
        expect_ppi_residual_ok(&bezier_diag1, &bspline_diag2, res, BRepPoint2(0.0), kLooseTol);
    }
}

TEST(IntersectorFrameworkPPI, UnsupportedNURBS2DThrows) {
    NURBSCurve2D nurbs1({BRepPoint2{0.0, 0.0}, BRepPoint2{1.0, 1.0}}, {0.0, 0.0, 1.0, 1.0}, {1.0, 1.0}, 1);
    NURBSCurve2D nurbs2({BRepPoint2{0.0, 1.0}, BRepPoint2{1.0, 0.0}}, {0.0, 0.0, 1.0, 1.0}, {1.0, 1.0}, 1);

    EXPECT_THROW(
        {
            const auto res = GeneralPCurvePCurveIntersection::solve(&nurbs1, &nurbs2);
            (void)res;
        },
        cpptrace::logic_error);
}


