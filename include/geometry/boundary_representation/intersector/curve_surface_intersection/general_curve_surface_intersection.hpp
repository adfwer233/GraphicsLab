#pragma once

#include "csi_results.hpp"
#include "line_plane_intersection.hpp"
#include "line_sphere_intersection.hpp"

#include <Eigen/Eigen>

#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

namespace GraphicsLab::Geometry::BRep {
struct CSIResult;
/**
 * @brief A general refinement algorithm to find the intersection between param curve and param surface
 *
 * @param
 *  S(u,v) in R^3 and C(t) in R^3.
 *
 * @note
 *  Define F(u, v, t) = S(u, v) - C(t). Intersection iff F(u, v, t) = 0.
 *
 *  Outline:
 *  - Find potential intersection guess by grid sampling
 *  - Refine possible guess via newton
 */
struct GeneralCurveSurfaceIntersection {

    static std::vector<CSIResult> solve(const ParamCurve3D *curve, const ParamSurface *surface) {

        if (auto line = dynamic_cast<const StraightLine3D *>(curve)) {
            if (auto plane = dynamic_cast<const Plane *>(surface)) {
                return LinePlaneIntersection::solve(line, plane);
            }

            if (auto sphere = dynamic_cast<const Sphere *>(surface)) {
                return LineSphereIntersection::solve(line, sphere);
            }
        }

        return intersect(curve, surface);
    }

  private:
    static auto check_surf_out_boundary(const BRepPoint2 &param, const ParamSurface *surf) -> bool {
        if (not surf->u_periodic) {
            if (param.x < 0 or param.x > 1)
                return true;
        }
        if (not surf->v_periodic) {
            if (param.y < 0 or param.y > 1)
                return true;
        }
        return false;
    };

    static std::pair<BRepPoint2, double> refine_with_newton(const ParamSurface *surf, const ParamCurve3D *cur,
                                                            BRepPoint2 surf_param, double curve_param) {
        constexpr int max_newton_iter = 30;
        constexpr double tol = Tolerance::default_tolerance;

        for (int i = 0; i < max_newton_iter; i++) {
            BRepPoint3 p1 = surf->evaluate(surf_param);
            BRepPoint3 p2 = cur->evaluate(curve_param);

            BRepVector3 F = p1 - p2;

            if (glm::length(F) < tol)
                break;

            auto [du, dv] = surf->derivative(surf_param);
            auto dt = cur->derivative(curve_param);

            Eigen::MatrixXd J(3, 3);
            J.col(0) = Eigen::Vector3d(du.x, du.y, du.z);
            J.col(1) = Eigen::Vector3d(dv.x, dv.y, dv.z);
            J.col(2) = -Eigen::Vector3d(dt.x, dt.y, dt.z);

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector3d b(F.x, F.y, F.z);
            Eigen::Vector3d delta = svd.solve(b);

            surf_param.x -= delta[0];
            surf_param.y -= delta[1];
            curve_param -= delta[2];

            if (check_surf_out_boundary(surf_param, surf)) {
                break;
            }
        }

        return std::make_pair(surf_param, curve_param);
    }

    static std::vector<std::pair<BRepPoint2, double>> find_all_initial_guess(const ParamSurface *surface,
                                                                             const ParamCurve3D *curve) {
        double distance_threshold = 1e-1;
        std::vector<std::pair<BRepPoint2, double>> initial_guess;

        auto curve_sample = curve->sample(50);

        for (auto [pos, param] : curve_sample) {
            auto [proj, proj_param] = surface->project(pos);

            if (glm::distance(pos, proj) > distance_threshold) {
                continue;
            }

            if (check_surf_out_boundary(proj_param, surface)) {
                continue;
            }

            auto [refine_surface, refine_curve] = refine_with_newton(surface, curve, proj_param, param);

            double dist_start = glm::distance(pos, proj);
            double dist_refined = glm::distance(surface->evaluate(refine_surface), curve->evaluate(refine_curve));

            // spdlog::info("[Curve intersector] dist {} -> {}", dist_start, dist_refined);

            if (glm::distance(surface->evaluate(refine_surface), curve->evaluate(refine_curve)) < 1e-3) {
                initial_guess.emplace_back(refine_surface, refine_curve);
            }
        }

        return initial_guess;
    }

    static std::vector<CSIResult> intersect(const ParamCurve3D *curve, const ParamSurface *surface) {
        auto initial = find_all_initial_guess(surface, curve);

        std::vector<CSIResult> result;

        for (int i = 0; i < initial.size(); i++) {

            auto [surf_param, curve_param] = initial[i];
            auto surf_pos = surface->evaluate(surf_param);
            auto cur_pos = curve->evaluate(curve_param);

            auto distance = glm::distance(surf_pos, cur_pos);

            if (distance < Tolerance::default_tolerance) {
                bool already_exists = false;
                for (auto res : result) {
                    if (glm::distance(res.inter_position, surf_pos) < Tolerance::default_tolerance * 10) {
                        already_exists = true;
                    }
                }

                if (not already_exists) {
                    double dist = glm::distance(surf_pos, curve->evaluate(curve_param));
                    if (dist > 1e-3) {
                        throw cpptrace::runtime_error("curve surface intersection failed");
                    }

                    result.emplace_back(surf_pos, curve_param, surf_param);
                }
            }
        }

        return result;
    }
};
} // namespace GraphicsLab::Geometry::BRep
