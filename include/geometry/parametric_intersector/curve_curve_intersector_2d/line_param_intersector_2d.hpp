#pragma once

#include "curve_curve_inter_result.hpp"

#include <geometry/parametric/bspline_curve_2d.hpp>
#include <geometry/parametric/parametric_curves/straight_line.hpp>

namespace GraphicsLab::Geometry {

struct LineParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(StraightLine2D &line, ParamCurve2D &curve) {
        return {};
    }
};

struct LineBSplineParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(const StraightLine2D &line, const BSplineCurve2D &curve) {
        CurveCurveIntersectionResult2D result;
        auto bezier_curves = curve.convert_to_bezier();

        return result;
    }
};

struct LineBezierParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(const StraightLine2D &line, const BezierCurve2D &curve) {
        BezierCurve2D l({line.start_point, line.end_point});
        return BezierBezierIntersector2D::intersect(l, curve);
    }
};

} // namespace GraphicsLab::Geometry