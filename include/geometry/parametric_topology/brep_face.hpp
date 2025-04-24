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

    std::unique_ptr<Mesh3D> mesh = nullptr;
    std::unique_ptr<Mesh2D> mesh2d = nullptr;
};
}