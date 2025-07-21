#pragma once
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"
#include "glm/vec3.hpp"

namespace GraphicsLab::Geometry::BRep {

    struct Point {
        glm::vec3 position;
    };

    struct Curve {
        ParamCurve3D* param_geometry();
    private:
        ParamCurve3D* geometry_ = nullptr;
    };

    struct PCurve {
        ParamCurve2D* param_geometry() {
            return geometry_;
        }

    private:
        ParamCurve2D* geometry_ = nullptr;
    };

    struct Surface {
        void set_param_geometry(ParamSurface* param_surface) { geometry_ = param_surface; }
        [[nodiscard]] ParamSurface* param_geometry() const { return geometry_; }

    private:
        ParamSurface* geometry_ = nullptr;
    };

}