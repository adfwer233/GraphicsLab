#pragma once

#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

struct ParamCurve3D;

namespace GraphicsLab::Geometry {

struct BRepCoedge;

struct BRepEdge {
    enum class Orientation {
        FORWARD,
        BACKWARD,
    };

    std::vector<BRepCoedge*> coedges;
    Orientation orientation;
    ParamCurve3D* geometry;
};

}