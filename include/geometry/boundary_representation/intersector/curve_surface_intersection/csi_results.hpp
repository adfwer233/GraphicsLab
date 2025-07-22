#pragma once
#include "geometry/boundary_representation/base/vec_def.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Result info for curve/surface intersection
 */
struct CSIResult {
    BRepPoint3 inter_position;
    double curve_parameter;
    BRepPoint2 surface_parameter;
};

} // namespace GraphicsLab::Geometry::BRep