#pragma once
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

#include "geometry/boundary_representation/brep_definition.hpp"

#include <memory>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct BRepAllocator {

    static BRepAllocator *instance() {
        static BRepAllocator instance;
        return &instance;
    }

    template <typename T, typename... Args>
        requires std::is_base_of_v<ParamCurve2D, T>
    T *alloc_param_pcurve(Args &&...args) {
        auto pcurve = std::make_unique<T>(std::forward<Args>(args)...);
        param_pcurves.push_back(std::move(pcurve));
        return reinterpret_cast<T *>(param_pcurves.back().get());
    }

    template <typename T, typename... Args>
        requires std::is_base_of_v<ParamCurve3D, T>
    T *alloc_param_curve(Args &&...args) {
        auto curve = std::make_unique<T>(std::forward<Args>(args)...);
        param_curves.push_back(std::move(curve));
        return reinterpret_cast<T *>(param_curves.back().get());
    }

    template <typename T, typename... Args>
        requires std::is_base_of_v<ParamSurface, T>
    T *alloc_param_surface(Args &&...args) {
        auto surface = std::make_unique<T>(std::forward<Args>(args)...);
        param_surfaces.push_back(std::move(surface));
        return reinterpret_cast<T *>(param_surfaces.back().get());
    }

    Curve *alloc_curve() {
        auto curve = std::make_unique<Curve>();
        curves.push_back(std::move(curve));
        return curves.back().get();
    }

    PCurve *alloc_pcurve() {
        auto pcurve = std::make_unique<PCurve>();
        pcurves.push_back(std::move(pcurve));
        return pcurves.back().get();
    }

    Surface *alloc_surface() {
        auto surface = std::make_unique<Surface>();
        surfaces.push_back(std::move(surface));
        return surfaces.back().get();
    }

    Face *alloc_face() {
        auto face = std::make_unique<Face>();
        faces.push_back(std::move(face));
        return faces.back().get();
    }

    Edge *alloc_edge() {
        auto edge = std::make_unique<Edge>();
        edges.push_back(std::move(edge));
        return edges.back().get();
    }

    Point *alloc_point() {
        auto point = std::make_unique<Point>();
        points.push_back(std::move(point));
        return points.back().get();
    }

    Vertex *alloc_vertex() {
        auto vertex = std::make_unique<Vertex>();
        vertices.push_back(std::move(vertex));
        return vertices.back().get();
    }

    Coedge *alloc_coedge() {
        auto coedge = std::make_unique<Coedge>();
        coedges.push_back(std::move(coedge));
        return coedges.back().get();
    }

    Loop *alloc_loop() {
        auto loop = std::make_unique<Loop>();
        loops.push_back(std::move(loop));
        return loops.back().get();
    }

    Shell *alloc_shell() {
        auto shell = std::make_unique<Shell>();
        shells.push_back(std::move(shell));
        return shells.back().get();
    }

    Body *alloc_body() {
        auto body = std::make_unique<Body>();
        bodies.push_back(std::move(body));
        return bodies.back().get();
    }

  private:
    BRepAllocator() = default;

    std::vector<std::unique_ptr<ParamCurve2D>> param_pcurves;
    std::vector<std::unique_ptr<ParamCurve3D>> param_curves;
    std::vector<std::unique_ptr<ParamSurface>> param_surfaces;

    // topology data
    std::vector<std::unique_ptr<Vertex>> vertices;
    std::vector<std::unique_ptr<Edge>> edges;
    std::vector<std::unique_ptr<Coedge>> coedges;
    std::vector<std::unique_ptr<Face>> faces;
    std::vector<std::unique_ptr<Loop>> loops;
    std::vector<std::unique_ptr<Shell>> shells;
    std::vector<std::unique_ptr<Body>> bodies;

    // geometry data
    std::vector<std::unique_ptr<Point>> points;
    std::vector<std::unique_ptr<Surface>> surfaces;
    std::vector<std::unique_ptr<Curve>> curves;
    std::vector<std::unique_ptr<PCurve>> pcurves;
};

} // namespace GraphicsLab::Geometry::BRep