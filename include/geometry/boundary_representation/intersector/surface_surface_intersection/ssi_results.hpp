#pragma once
#include "geometry/parametric/bspline_curve_3d.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief: Result info for surface/surface intersection
 */
struct SSIResult {
    ParamCurve3D *inter_curve;
    ParamCurve2D *pcurve1;
    ParamCurve2D *pcurve2;
};

struct IntersectionTraceInfo {
    BRepPoint2 param1;
    BRepPoint2 param2;
    BRepPoint3 position;
};

} // namespace GraphicsLab::Geometry::BRep