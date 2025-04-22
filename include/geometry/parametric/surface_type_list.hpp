#pragma once

#include "language/meta_programming/type_list.hpp"

#include "cone.hpp"
#include "plane.hpp"
#include "sphere.hpp"
#include "torus.hpp"

#include "nurbs_surface.hpp"
#include "tensor_product_bezier.hpp"

namespace GraphicsLab::Geometry {
using ParamSurfaceTypeList = MetaProgramming::TypeList<Sphere, Torus, TensorProductBezier>;
}