#pragma once

#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

namespace GraphicsLab::Geometry {

struct BRepFace;
struct BRepEdge;
struct ParamCurve2D;

struct BRepCoedge {
    enum class Orientation {
        FORWARD,
        BACKWARD,
    };


    Orientation orientation;
    BRepEdge* edge;
    BRepFace* face;
    ParamCurve2D* geometry;
};

}