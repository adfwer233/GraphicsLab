#pragma once

#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

struct BRepFace;
struct BRepCoedge;
namespace GraphicsLab::Geometry {

struct BRepLoop {
    std::vector<BRepCoedge*> coedges;
    std::vector<BRepFace*> faces;
};

}