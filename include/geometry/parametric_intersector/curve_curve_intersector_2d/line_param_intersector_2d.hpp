#pragma once

#include "curve_curve_inter_result.hpp"
#include <geometry/parametric/parametric_curves/straight_line.hpp>

namespace GraphicsLab::Geometry {

struct LineParamIntersector2D {
    static CurveCurveIntersectionResult2D intersect(StraightLine2D &line, ParamCurve2D &curve) {
        return {};
    }
};

}