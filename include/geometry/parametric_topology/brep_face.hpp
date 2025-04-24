#pragma once

#include "geometry/parametric/parametric_surface.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

namespace GraphicsLab::Geometry {

struct BRepLoop;
struct BRepFace {
    using ParamType = ParamSurface::ParamType;
    using PointType = ParamSurface::PointType;

    ParamSurface* surface;

    std::vector<BRepLoop*> boundary;

    bool containment(ParamType param) const {
        return true;
    }

    Mesh3D tessllate(int n, int m) {
        return Mesh3D();
    }
};
}