#pragma once

#include "geometry/boundary_representation/base/vec_def.hpp"

namespace GraphicsLab::Geometry::BRep {

struct PPIResult {
    double param1;
    double param2;
    BRepPoint2 inter_position;
};

} // namespace GraphicsLab::Geometry::BRep