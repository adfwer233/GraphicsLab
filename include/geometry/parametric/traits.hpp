#pragma once

#include "sphere.hpp"
#include "torus.hpp"

#include "language/meta_programming/type_list.hpp"

namespace GraphicsLab::Geometry {

using ParametricSurfaceTypeList = MetaProgramming::TypeList<Sphere, Torus>;

template<typename T> concept IsParametricSurface = MetaProgramming::TypeListFunctions::IsAnyOf<ParametricSurfaceTypeList, T>::value;

}