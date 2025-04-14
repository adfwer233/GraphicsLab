#pragma once

#include <geometry/parametric/bspline_curve_2d.hpp>
#include <geometry/parametric/nurbs_surface.hpp>
#include <geometry/parametric/plane.hpp>
#include <geometry/parametric/sphere.hpp>
#include <vector>

#include "geometry/parametric/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

namespace GraphicsLab::Geometry::BoundaryRepresentation {
struct ParamRange {
    double l, r;
};

struct Face;
struct Edge;

struct Vertex {
    glm::dvec3 position;
};

struct Solid {
    std::vector<Face> faces;
};

struct Coedge {
    Face *face;
    ParamCurve2D *pcurve;
};

struct Loop {
    std::vector<Edge> edges;
};

struct Face {
    ParamSurface *surface;
    std::vector<Loop> loops;
};

struct Edge {
    Coedge *coedge;
    ParamCurve3D *curve;
    Vertex *start_position;
    Vertex *end_position;
};

struct BoundaryRepresentationAllocator {
    // save all geometric data
    using ParamSurfaceTypeList = MetaProgramming::TypeList<Plane, Sphere, Torus, NURBSSurface>;
    using ParamCurve3DTypeList = MetaProgramming::TypeList<>;
    using ParamCurve2DTypeList = MetaProgramming::TypeList<BezierCurve2D, BSplineCurve2D>;

    using ParamCurve2DVecsTypeList = ParamCurve2DTypeList::monad<std::vector>::to<std::tuple>;
    using ParamSurfaceVecsTypeList = ParamSurfaceTypeList::monad<std::vector>::to<std::tuple>;

    ParamCurve2DVecsTypeList param_curves_2d;
    ParamSurfaceVecsTypeList param_surfaces;

    template <typename T>
        requires TypeListFunctions::IsAnyOf<ParamSurfaceTypeList, T>::value
    consteval std::vector<T> &get_surface_list() {
        constexpr size_t idx = MetaProgramming::TypeListFunctions::IndexOf<ParamSurfaceTypeList, T>::value;
        return std::get<idx>(param_surfaces);
    }

    template <typename T>
        requires MetaProgramming::TypeListFunctions::IsAnyOf<ParamCurve3DTypeList, T>::value
    consteval std::vector<T> &get_curve_list() {
        constexpr size_t idx = MetaProgramming::TypeListFunctions::IndexOf<ParamCurve3DTypeList, T>::value;
        return std::get<idx>(param_curves_2d);
    }
};

} // namespace GraphicsLab::Geometry::BoundaryRepresentation