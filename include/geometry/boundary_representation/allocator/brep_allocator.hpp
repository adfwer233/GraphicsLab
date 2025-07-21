#pragma once
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

#include "geometry/boundary_representation/brep_definition.hpp"

#include <memory>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct BRepAllocator {

    static BRepAllocator & instance() {
        static BRepAllocator instance;
        return instance;
    }

    template<typename T, typename... Args> requires std::is_base_of_v<ParamCurve2D, T>
    T* alloc_param_pcurve(Args... args) {
        auto pcurve = std::make_unique<T>(args...);
        param_pcurves.push_back(std::move(pcurve));
        return param_pcurves.back().get();
    }

    template<typename T, typename... Args> requires std::is_base_of_v<ParamCurve3D, T>
    T* alloc_param_curve(Args... args) {
        auto curve = std::make_unique<T>(args...);
        param_curves.push_back(std::move(curve));
        return param_curves.back().get();
    }

    template<typename T, typename... Args> requires std::is_base_of_v<ParamSurface, T>
    T* alloc_param_surface(Args... args) {
        auto surface = std::make_unique<T>(args...);
        param_surfaces.push_back(std::move(surface));
        return param_surfaces.back().get();
    }

    Surface* alloc_surface() {
        auto surface = std::make_unique<Surface>();
        surfaces.push_back(std::move(surface));
        return surfaces.back().get();
    }

    Face* alloc_face() {
        auto face = std::make_unique<Face>();
        faces.push_back(std::move(face));
        return faces.back().get();
    }

    std::vector<std::unique_ptr<ParamCurve2D>> param_pcurves;
    std::vector<std::unique_ptr<ParamCurve3D>> param_curves;
    std::vector<std::unique_ptr<ParamSurface>> param_surfaces;

    std::vector<std::unique_ptr<Face>> faces;
    std::vector<std::unique_ptr<Surface>> surfaces;
};

}