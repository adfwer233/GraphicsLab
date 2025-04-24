#pragma once

#include "brep_coedge.hpp"
#include "brep_loop.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

namespace GraphicsLab::Geometry {

struct BRepLoop;
struct BRepFace {
    using ParamType = ParamSurface::ParamType;
    using PointType = ParamSurface::PointType;

    ParamSurface *surface;

    std::vector<BRepLoop *> boundary;

    bool trim_flag = true;

    bool contain(ParamType param) {
        double wn = 0;

        int u_repeat = 0;
        if (surface->u_periodic)
            u_repeat = 8;
        for (auto loop : boundary) {
            for (auto coedge : loop->coedges) {
                double pcurve_wn = 0;

                auto pcurve = coedge->geometry->discretize();

                for (int u = -u_repeat; u <= u_repeat; u++) {
                    ParamType offset{u, 0};
                    for (int i = 1; i < pcurve.size(); i++) {
                        auto v1 = pcurve[i - 1] - param - offset;
                        auto v2 = pcurve[i] - param - offset;
                        auto cross = v1.x * v2.y - v1.y * v2.x;
                        auto dot = glm::dot(glm::normalize(v1), glm::normalize(v2));
                        auto acos_value = std::acos(dot);

                        if (not std::isnan(acos_value)) {
                            pcurve_wn += cross > 0 ? acos_value : -acos_value;
                        }
                    }
                }

                if (coedge->orientation == BRepCoedge::Orientation::BACKWARD)
                    pcurve_wn *= -1;
                wn += pcurve_wn;
            }
        }

        if (trim_flag)
            return wn > 3.14;
        else
            return wn > -3.14;
    }

    std::unique_ptr<Mesh3D> mesh = nullptr;
    std::unique_ptr<Mesh2D> mesh2d = nullptr;
};
} // namespace GraphicsLab::Geometry