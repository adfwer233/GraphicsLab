#pragma once

#include "language/meta_programming/type_list.hpp"

#include "bezier_curve_2d.hpp"
#include "bezier_curve_3d.hpp"
#include "bspline_curve_2d.hpp"
#include "bspline_curve_3d.hpp"

namespace GraphicsLab::Geometry {

using ParametricCurve2DTypeList = MetaProgramming::TypeList<BezierCurve2D, BSplineCurve2D>;
using ParametricCurve3DTypeList = MetaProgramming::TypeList<BezierCurve3D, BSplineCurve3D>;
}