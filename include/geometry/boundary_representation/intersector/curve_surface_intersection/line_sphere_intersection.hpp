#pragma once
#include "csi_results.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/sphere.hpp"

namespace GraphicsLab::Geometry {
struct Plane;
}
namespace GraphicsLab::Geometry::BRep {
struct CSIResult;

/**
 * @brief Intersect line with sphere
 *
 * @note
 *  let line vec{p} + t vec{d}, sphere(vec{o}, r), we have
 *
 *  at^2 + bt + c = 0, where
 *
 *  a = d * d
 *  b = 2(d * p - o * d)
 *  c = (o - p) * (o - p) - r^2
 *
 *  "*" is dot product
 */
struct LineSphereIntersection {

    static std::vector<CSIResult> solve(const StraightLine3D *line, const Sphere *sphere, bool check_line_range = true) {
        BRepPoint3 p = line->start_point;
        BRepPoint3 d = line->end_point - line->start_point;
        BRepPoint3 o = sphere->center;
        double r = sphere->radius;

        double a = glm::dot(d, d);
        double b = 2 * glm::dot(d, p) - 2 * glm::dot(o, d);
        double c = glm::dot(o - p, o - p) - r * r;

        double det = b * b - 4 * a * c;

        auto get_csi = [line, sphere](double t) -> CSIResult {
            return CSIResult {
                .inter_position = line->evaluate(t),
                .curve_parameter = t,
                .surface_parameter = sphere->project(line->evaluate(t)).second
            };
        };

        if (det > 1e-10) {
            auto pt = line->evaluate((-b - std::sqrt(det)) / (2 * a));
            auto dis1 = glm::distance(pt, o);

            int x = 0;
        }

        std::vector<CSIResult> csi_results;
        if (std::abs(det) < 1e-10) {
            CSIResult csi = get_csi(- b / (2 * a));
            csi_results = {csi};
        } else {
            if (det < 0) {
                csi_results =  {};
            } else {
                double t1 = (-b - std::sqrt(det)) / (2 * a);
                double t2 = (-b + std::sqrt(det)) / (2 * a);

                csi_results =  {get_csi(t1), get_csi(t2)};
            }
        }

        if (check_line_range) {
            std::vector<CSIResult> csi_results2;
            for (auto &res: csi_results) {
                if (res.curve_parameter > 0 and res.curve_parameter < 1) {
                    csi_results2.push_back(res);
                }
            }
            return csi_results2;
        } else {
            return csi_results;
        }
    }

};
}