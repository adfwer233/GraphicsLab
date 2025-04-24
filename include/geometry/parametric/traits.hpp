#pragma once

#include "sphere.hpp"
#include "surface_type_list.hpp"
#include "curve_type_list.hpp"
#include "torus.hpp"

#include "language/meta_programming/type_list.hpp"

namespace GraphicsLab::Geometry {

template <typename T>
concept IsParametricSurface = MetaProgramming::TypeListFunctions::IsAnyOf<ParamSurfaceTypeList, T>::value;

template <typename T>
concept IsParametricCurve3D = MetaProgramming::TypeListFunctions::IsAnyOf<ParametricCurve3DTypeList, T>::value;

template <typename T>
concept IsParametricCurve2D = MetaProgramming::TypeListFunctions::IsAnyOf<ParametricCurve2DTypeList, T>::value;
} // namespace GraphicsLab::Geometry