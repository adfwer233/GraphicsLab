#pragma once

#include "sphere.hpp"
#include "surface_type_list.hpp"
#include "torus.hpp"

#include "language/meta_programming/type_list.hpp"

namespace GraphicsLab::Geometry {


template <typename T>
concept IsParametricSurface = MetaProgramming::TypeListFunctions::IsAnyOf<ParamSurfaceTypeList, T>::value;

} // namespace GraphicsLab::Geometry