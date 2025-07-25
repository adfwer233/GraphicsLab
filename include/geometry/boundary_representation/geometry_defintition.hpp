#pragma once
#include "base/vec_def.hpp"
#include "geometry/boundary_representation/base/param_range.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"
#include "glm/vec3.hpp"

namespace GraphicsLab::Geometry::BRep {

struct Point {
    [[nodiscard]] BRepPoint3 position() const {
        return position_;
    };
    void set_position(const BRepPoint3 &position) {
        position_ = position;
    }

  private:
    BRepPoint3 position_ = {};
};

struct Curve {
    [[nodiscard]] ParamCurve3D *param_geometry() const {
        return geometry_;
    }

    void set_param_geometry(ParamCurve3D *param_geometry) {
        geometry_ = param_geometry;
    }

  private:
    ParamCurve3D *geometry_ = nullptr;
};

struct PCurve {
    [[nodiscard]] ParamCurve2D *param_geometry() const {
        return geometry_;
    }

    void set_param_geometry(ParamCurve2D *param_geometry) {
        geometry_ = param_geometry;
    }

    [[nodiscard]] bool is_forward() const {
        return forward_;
    }
    void set_forward(bool is_forward) {
        forward_ = is_forward;
    }

  private:
    bool forward_ = true;
    ParamCurve2D *geometry_ = nullptr;
};

struct Surface {
    void set_param_geometry(ParamSurface *param_surface) {
        geometry_ = param_surface;
    }
    [[nodiscard]] ParamSurface *param_geometry() const {
        return geometry_;
    }

  private:
    ParamSurface *geometry_ = nullptr;
};

} // namespace GraphicsLab::Geometry::BRep