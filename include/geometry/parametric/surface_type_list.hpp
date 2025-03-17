#pragma once

#include "language/meta_programming/type_list.hpp"

#include "sphere.hpp"
#include "plane.hpp"
#include "torus.hpp"
#include "cone.hpp"

#include "nurbs_surface.hpp"

namespace GraphicsLab::Geometry {
    using ParamSurfaceTypeList = MetaProgramming::TypeList<Sphere, Torus, NURBSSurface>;
}