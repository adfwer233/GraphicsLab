#pragma once

#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

namespace GraphicsLab::Geometry {

struct BRepFace;
struct BRepCoedge;

struct BRepLoop {
    std::vector<BRepCoedge *> coedges;
    std::vector<BRepFace *> faces;
};

} // namespace GraphicsLab::Geometry