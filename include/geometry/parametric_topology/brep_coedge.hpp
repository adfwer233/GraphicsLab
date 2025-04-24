#pragma once

#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

struct BRepFace;
struct BRepEdge;
struct ParamCurve2D;

namespace GraphicsLab::Geometry {

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