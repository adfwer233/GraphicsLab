#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_surface.hpp"
#include "geometry/spatial_datastructure/kd_tree.hpp"
#include "ssi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief: A general marching algorithm to find the intersection between parametric surfaces
 *
 * @param
 *  S_1(u, v) in R^3 and S_2(s, t) in R^3 are two ParamSurface.
 *
 *  Define F(u, v, s, t) = S_1(u, v) - S_2(s, t). Intersection iff F(u, v, s, t) = 0.
 *
 * @note:
 *
 *  Outline:
 *  - Find initial intersection point guess.
 *  - Refine possible guess with newton refinement on F.
 *  - Trace the curve with condition: dF = ... du + ... dv + ... ds + ... dt = 0
 */
struct GeneralSurfaceSurfaceIntersection {

    static std::vector<SSIResult> solve(const ParamSurface *surf1, const ParamSurface *surf2) {
        return intersect_all(surf1, surf2);
    }

  private:
    /**
     * @brief: Newton refinement.
     * @param surf1
     * @param surf2
     * @param param1 param1 guess (u, v)
     * @param param2 param2 guess (s, t)
     * @return (refined param 1 (u', v'), refined param 2 (s', t'))
     *
     * @note:
     *  We want F(u,v,s,t) = 0. And we know that
     *
     *  dF = S1_u du + S1_v dv - S2_s ds - S2_t dt.
     *  Let J = [S1_u, S1_v, S2_s, S2_t]. We use SVD to solve
     *  delta = argmin (J delta + F)
     *  Then update with (u,v,s,t) <- (u,v,s,t) + delta
     */
    static std::pair<BRepPoint2, BRepPoint2> refine_with_newton(const ParamSurface *surf1, const ParamSurface *surf2,
                                                                BRepPoint2 param1, BRepPoint2 param2) {
        constexpr int max_newton_iter = 10;
        constexpr double tol = 1e-8;

        for (int i = 0; i < max_newton_iter; ++i) {
            BRepPoint3 p1 = surf1->evaluate(param1);
            BRepPoint3 p2 = surf2->evaluate(param2);
            BRepVector3 F = p1 - p2;

            if (glm::length(F) < tol)
                break;

            auto [du1, dv1] = surf1->derivative(param1);
            auto [du2, dv2] = surf2->derivative(param2);

            Eigen::MatrixXd J(3, 4);
            J.col(0) = Eigen::Vector3d(du1.x, du1.y, du1.z);
            J.col(1) = Eigen::Vector3d(dv1.x, dv1.y, dv1.z);
            J.col(2) = -Eigen::Vector3d(du2.x, du2.y, du2.z);
            J.col(3) = -Eigen::Vector3d(dv2.x, dv2.y, dv2.z);

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector3d b(F.x, F.y, F.z);
            Eigen::Vector4d delta = svd.solve(b);

            param1 -= BRepPoint2(delta[0], delta[1]);
            param2 -= BRepPoint2(delta[2], delta[3]);
        }

        return {param1, param2};
    }

    /**
     * Compute the tangent direction in parameter space by crossing the normals
     * @param surf1
     * @param surf2
     * @param param1
     * @param param2
     * @return tangent direction (du,dv,ds,dt) in parameter space (u,v,s,t).
     *
     * @note
     * Find the nullspace of dF = S1_u du + S1_v dv - S2_s ds - S2_t dt = 0
     */
    static std::pair<BRepPoint2, BRepPoint2> compute_tangent_direction(const ParamSurface *surf1,
                                                                       const ParamSurface *surf2, BRepPoint2 param1,
                                                                       BRepPoint2 param2) {
        BRepVector3 n1 = surf1->normal(param1);
        BRepVector3 n2 = surf2->normal(param2);
        BRepVector3 tangent = glm::normalize(glm::cross(n1, n2));

        auto [du1, dv1] = surf1->derivative(param1);
        auto [du2, dv2] = surf2->derivative(param2);

        Eigen::Matrix2d A1;
        A1 << glm::dot(du1, tangent), glm::dot(dv1, tangent), 0, 0;
        Eigen::Vector2d b1;
        b1 << glm::dot(tangent, tangent), 0;

        Eigen::Vector2d d_param1 = A1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b1);

        Eigen::Matrix2d A2;
        A2 << glm::dot(du2, tangent), glm::dot(dv2, tangent), 0, 0;
        Eigen::Vector2d b2;
        b2 << glm::dot(tangent, tangent), 0;

        Eigen::Vector2d d_param2 = A2.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b2);

        return {BRepPoint2(d_param1[0], d_param1[1]), BRepPoint2(d_param2[0], d_param2[1])};
    }

    /**
     * @brief: find the possible initial guess
     * @param surf1
     * @param surf2
     * @return
     *
     * Grid sample from param surfaces and find the closet pair.
     */
    static std::pair<BRepPoint2, BRepPoint2> find_initial_guess(const ParamSurface *surf1, const ParamSurface *surf2) {
        // Implement the logic to find the initial guess for the intersection

        auto surf1_sample = surf1->sample(10, 10);

        double min_distance = std::numeric_limits<double>::max();
        BRepPoint2 param1, param2;

        for (auto [pos, param] : surf1_sample) {
            auto [proj, proj_param] = surf2->project(pos);
            if (glm::distance(proj, pos) < min_distance) {
                min_distance = glm::distance(proj, pos);
                param1 = param;
                param2 = proj_param;
            }
        }

        spdlog::debug("initial guess min distance: {}", min_distance);
        spdlog::debug("initial guess before refine: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);

        return refine_with_newton(surf1, surf2, param1, param2);
    }

    /**
     * @brief: find all possible initial guesses.
     * @param surf1
     * @param surf2
     * @return
     *
     * @note Grid sample in two surfaces and report all pairs in a given threshold.
     */
    static std::vector<std::pair<BRepPoint2, BRepPoint2>> find_all_possible_initial_guess(const ParamSurface *surf1,
                                                                                          const ParamSurface *surf2) {
        auto surf1_sample = surf1->sample(20, 20);

        std::vector<std::pair<BRepPoint2, BRepPoint2>> result;
        double distance_threshold = 1e-1;
        // ParamType param1, param2;

        for (auto [pos, param] : surf1_sample) {
            auto [proj, proj_param] = surf2->project(pos);
            auto [refine1, refine2] = refine_with_newton(surf1, surf2, param, proj_param);

            double distance_before_refine = glm::distance(surf1->evaluate(param), surf2->evaluate(proj_param));
            double distance_after_refine = glm::distance(surf1->evaluate(refine1), surf2->evaluate(refine2));
            spdlog::debug("pos {} {} {}", pos.x, pos.y, pos.z);
            spdlog::debug("Distance before refine: {}, distance after refine {}", distance_before_refine,
                          distance_after_refine);
            if (glm::distance(surf1->evaluate(refine1), surf2->evaluate(refine2)) < distance_threshold) {
                result.push_back({refine1, refine2});
            }
        }

        return result;
    }

    /**
     * @brief Trace pcurve in the parameter space (u,v,s,t)
     * @param surf1
     * @param surf2
     * @param begin_param1
     * @param begin_param2
     * @param sign trace forward or back
     * @return
     *
     * @note Hyperparameters
     *
     *  max_iter: trace iteration length.
     *  length_iter: if the current F is greater than this value, refine with newton refinement.
     *  step_length: step length in the parameter space.
     *
     * @todo Handle param boundary.
     */
    static std::vector<IntersectionTraceInfo> trace_pcurve_single_direction(const ParamSurface *surf1,
                                                                            const ParamSurface *surf2,
                                                                            BRepPoint2 begin_param1,
                                                                            BRepPoint2 begin_param2, int sign) {
        std::vector<IntersectionTraceInfo> intersections;

        spdlog::debug("initial guess: ({}, {}), ({}, {})", begin_param1.x, begin_param1.y, begin_param2.x,
                      begin_param2.y);
        spdlog::debug("initial distance: {}",
                      glm::distance(surf1->evaluate(begin_param1), surf2->evaluate(begin_param2)));

        constexpr int max_iter = 2000;
        constexpr double length_iter = 0.0001;
        constexpr double step_length = 0.02;

        begin_param1 = surf1->move_param_to_std_domain(begin_param1);
        begin_param2 = surf2->move_param_to_std_domain(begin_param2);

        BRepPoint2 param1 = begin_param1;
        BRepPoint2 param2 = begin_param2;

        auto check_out_boundary = [](const BRepPoint2 &param, const ParamSurface *surf) -> bool {
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

        for (int step = 0; step < max_iter; step++) {
            // spdlog::info("current param: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);
            auto F = surf1->evaluate(param1) - surf2->evaluate(param2);

            if (glm::length(F) > length_iter) {
                auto [refine_param1, refine_param2] = refine_with_newton(surf1, surf2, param1, param2);
                param1 = refine_param1;
                param2 = refine_param2;
            }

            if (check_out_boundary(param1, surf1) or check_out_boundary(param2, surf2)) {
                break;
            }

            intersections.emplace_back(param1, param2, surf1->evaluate(param1));

            auto [param1_tangent, param2_tangent] = compute_tangent_direction(surf1, surf2, param1, param2);

            param1 = param1 + step_length * param1_tangent * static_cast<double>(sign);
            param2 = param2 + step_length * param2_tangent * static_cast<double>(sign);

            if (glm::distance(surf1->move_param_to_std_domain(param1), begin_param1) < 1e-3 or
                glm::distance(surf2->move_param_to_std_domain(param2), begin_param2) < 1e-3) {
                auto offset1 = begin_param1 - surf1->move_param_to_std_domain(param1);
                auto offset2 = begin_param2 - surf2->move_param_to_std_domain(param2);
                intersections.emplace_back(param1 + offset1, param2 + offset2, surf1->evaluate(begin_param1));
                spdlog::debug("start end distance {}",
                              glm::distance(surf1->move_param_to_std_domain(param1 + offset1), begin_param1));
                break;
            }
        }

        return intersections;
    }

    static std::vector<IntersectionTraceInfo> trace_pcurve(const ParamSurface *surf1, const ParamSurface *surf2,
                                                           BRepPoint2 begin_param1, BRepPoint2 begin_param2) {
        auto forward_trace = trace_pcurve_single_direction(surf1, surf2, begin_param1, begin_param2, 1);
        auto backward_trace = trace_pcurve_single_direction(surf1, surf2, begin_param1, begin_param2, -1);

        std::ranges::reverse(backward_trace);
        if (not backward_trace.empty())
            backward_trace.pop_back();

        std::ranges::copy(forward_trace, std::back_inserter(backward_trace));
        return backward_trace;
    }

    static std::vector<SSIResult> intersect_all(const ParamSurface *surf1, const ParamSurface *surf2) {
        auto inital = find_all_possible_initial_guess(surf1, surf2);

        spdlog::info("initial size {}", inital.size());

        std::vector<SSIResult> result;

        std::vector<KDTree::KDTree<3, KDTree::PointPrimitive<3>>> curve_kd_trees;

        std::vector<std::vector<IntersectionTraceInfo>> trace_results;

        for (auto [begin_param1, begin_param2] : inital) {
            auto [refine_param1, refine_param2] = refine_with_newton(surf1, surf2, begin_param1, begin_param2);
            bool traced = false;

            float min_dis_pos = 1;

            for (auto &t : curve_kd_trees) {
                auto pos1 = surf1->evaluate(refine_param1);
                auto dist = t.nearestNeighbor(t.root, pos1).distance_to(pos1);
                auto dist_pos = glm::distance(pos1, surf2->evaluate(refine_param2));
                // spdlog::info("dist {}, dist pos {}", dist, dist_pos);

                min_dis_pos = std::min(min_dis_pos, dist);
                if (dist < 5 * 1e-2 and dist_pos < 1e-3) {
                    traced = true;
                    break;
                }
            }

            if (traced)
                continue;
            spdlog::info("min dis {}", min_dis_pos);

            auto trace = trace_pcurve(surf1, surf2, refine_param1, refine_param2);
            if (trace.size() > 0) {
                trace_results.push_back(trace);

                std::vector<KDTree::PointPrimitive<3>> points;
                for (auto info : trace) {
                    points.emplace_back(surf1->evaluate(info.param1));
                }
                curve_kd_trees.emplace_back(points);
            }
        }

        spdlog::info("components num {}", trace_results.size());

        auto allocator = BRepAllocator::instance();

        for (auto &trace : trace_results) {
            SSIResult ssi_result;
            spdlog::info("size {}", trace.size());
            std::vector<BRepPoint3> points;
            std::vector<BRepPoint2> params1, params2;
            for (auto [param1, param2, pos] : trace) {
                points.push_back(pos);
                params1.push_back(param1);
                params2.push_back(param2);
            }

            int control_points_count = std::min(static_cast<size_t>(50), std::max(static_cast<size_t>(10), points.size() / 2));
            // fit the 3d curve with BSpline curve
            auto &&curve = BSplineCurve3D::fit(points, 5, control_points_count);
            BSplineCurve3D *curve_alloc = allocator->alloc_param_curve<BSplineCurve3D>(std::move(curve));
            ssi_result.inter_curve = curve_alloc;

            // fit the pcurves with BSpline curves
            auto &&pcurve1 = BSplineCurve2D::fit(params1, 3, control_points_count);
            pcurve1.control_points_.front() = params1.front();
            pcurve1.control_points_.back() = params1.back();
            BSplineCurve2D *pcurve1_alloc = allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(pcurve1));
            ssi_result.pcurve1 = pcurve1_alloc;

            auto &&pcurve2 = BSplineCurve2D::fit(params2, 3, control_points_count);
            pcurve2.control_points_.front() = params2.front();
            pcurve2.control_points_.back() = params2.back();
            auto pcurve2_alloc = allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(pcurve2));
            ssi_result.pcurve2 = pcurve2_alloc;

            result.push_back(ssi_result);
        }

        return result;
    }
};

} // namespace GraphicsLab::Geometry::BRep