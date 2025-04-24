#pragma once

#include "Eigen/Eigen"
#include "glm/glm.hpp"
#include "spdlog/spdlog.h"

#include "geometry/parametric/parametric_surface.hpp"

#include <geometry/parametric/bspline_curve_2d.hpp>
#include <geometry/parametric/bspline_curve_3d.hpp>
#include <geometry/spatial_datastructure/kd_tree.hpp>

namespace GraphicsLab::Geometry {

struct SurfaceSurfaceIntersector {
    using PointType = glm::dvec3;
    using VecType = glm::dvec3;
    using ParamType = glm::dvec2;

    // Newton refinement to solve F(u,v,s,t) = S1(u,v) - S2(s,t) = 0
    static std::pair<ParamType, ParamType> refine_with_newton(const ParamSurface &surf1, const ParamSurface &surf2,
                                                              ParamType param1, ParamType param2) {
        constexpr int max_newton_iter = 10;
        constexpr double tol = 1e-8;

        for (int i = 0; i < max_newton_iter; ++i) {
            PointType p1 = surf1.evaluate(param1);
            PointType p2 = surf2.evaluate(param2);
            VecType F = p1 - p2;

            if (glm::length(F) < tol)
                break;

            auto [du1, dv1] = surf1.derivative(param1);
            auto [du2, dv2] = surf2.derivative(param2);

            Eigen::MatrixXd J(3, 4);
            J.col(0) = Eigen::Vector3d(du1.x, du1.y, du1.z);
            J.col(1) = Eigen::Vector3d(dv1.x, dv1.y, dv1.z);
            J.col(2) = -Eigen::Vector3d(du2.x, du2.y, du2.z);
            J.col(3) = -Eigen::Vector3d(dv2.x, dv2.y, dv2.z);

            Eigen::JacobiSVD svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector3d b(F.x, F.y, F.z);
            Eigen::Vector4d delta = svd.solve(b);

            param1 -= ParamType(delta[0], delta[1]);
            param2 -= ParamType(delta[2], delta[3]);
        }

        return {param1, param2};
    }

    // Compute the tangent direction in parameter space by crossing the normals
    static std::pair<ParamType, ParamType> compute_tangent_direction(const ParamSurface &surf1,
                                                                     const ParamSurface &surf2, ParamType param1,
                                                                     ParamType param2) {
        VecType n1 = surf1.normal(param1);
        VecType n2 = surf2.normal(param2);
        VecType tangent = glm::normalize(glm::cross(n1, n2));

        auto [du1, dv1] = surf1.derivative(param1);
        auto [du2, dv2] = surf2.derivative(param2);

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

        return {ParamType(d_param1[0], d_param1[1]), ParamType(d_param2[0], d_param2[1])};
    }

    static std::pair<ParamType, ParamType> find_initial_guess(const ParamSurface &surf1, const ParamSurface &surf2) {
        // Implement the logic to find the initial guess for the intersection

        auto surf1_sample = surf1.sample(10, 10);

        double min_distance = std::numeric_limits<double>::max();
        ParamType param1, param2;

        for (auto [pos, param] : surf1_sample) {
            auto [proj, proj_param] = surf2.project(pos);
            if (glm::distance(proj, pos) < min_distance) {
                min_distance = glm::distance(proj, pos);
                param1 = param;
                param2 = proj_param;
            }
        }

        spdlog::info("initial guess min distance: {}", min_distance);
        spdlog::info("initial guess before refine: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);

        return refine_with_newton(surf1, surf2, param1, param2);
    }

    static std::vector<std::pair<ParamType, ParamType>> find_all_possible_initial_guess(const ParamSurface &surf1,
                                                                                        const ParamSurface &surf2) {
        auto surf1_sample = surf1.sample(20, 20);

        std::vector<std::pair<ParamType, ParamType>> result;
        double distance_threshold = 1e-1;
        // ParamType param1, param2;

        for (auto [pos, param] : surf1_sample) {
            auto [proj, proj_param] = surf2.project(pos);
            auto [refine1, refine2] = refine_with_newton(surf1, surf2, param, proj_param);
            if (glm::distance(surf1.evaluate(refine1), surf2.evaluate(refine2)) < distance_threshold) {
                result.push_back({refine1, refine2});
            }
        }

        return result;
    }

    struct IntersectionInfo {
        ParamType param1;
        ParamType param2;
        PointType position;
    };

    static std::vector<IntersectionInfo> trace_pcurve(const ParamSurface &surf1, const ParamSurface &surf2,
                                                      ParamType begin_param1, ParamType begin_param2) {
        std::vector<IntersectionInfo> intersections;

        spdlog::info("initial guess: ({}, {}), ({}, {})", begin_param1.x, begin_param1.y, begin_param2.x,
                     begin_param2.y);
        spdlog::info("initial distance: {}", glm::distance(surf1.evaluate(begin_param1), surf2.evaluate(begin_param2)));

        constexpr int max_iter = 2000;
        constexpr double length_iter = 0.0001;
        constexpr double step_length = 0.02;

        begin_param1 = surf1.move_param_to_std_domain(begin_param1);
        begin_param2 = surf2.move_param_to_std_domain(begin_param2);

        ParamType param1 = begin_param1;
        ParamType param2 = begin_param2;

        for (int step = 0; step < max_iter; step++) {
            // spdlog::info("current param: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);
            auto F = surf1.evaluate(param1) - surf2.evaluate(param2);

            if (glm::length(F) > length_iter) {
                auto [refine_param1, refine_param2] = refine_with_newton(surf1, surf2, param1, param2);
                param1 = refine_param1;
                param2 = refine_param2;
            }

            intersections.emplace_back(param1, param2, surf1.evaluate(param1));

            auto [param1_tangent, param2_tangent] = compute_tangent_direction(surf1, surf2, param1, param2);

            // spdlog::info(" {} {} ", glm::length(param1_tangent), glm::length(param2_tangent));
            param1 = param1 + step_length * param1_tangent;
            param2 = param2 + step_length * param2_tangent;

            if (glm::distance(surf1.move_param_to_std_domain(param1), begin_param1) < 1e-3 or
                glm::distance(surf2.move_param_to_std_domain(param2), begin_param2) < 1e-3) {
                auto offset1 = begin_param1 - surf1.move_param_to_std_domain(param1);
                auto offset2 = begin_param2 - surf2.move_param_to_std_domain(param2);
                intersections.emplace_back(param1 + offset1, param2 + offset2, surf1.evaluate(begin_param1));
                break;
            }
        }

        return intersections;
    }

    struct IntersectionResult {
        std::vector<std::vector<IntersectionInfo>> traces;

        std::vector<BSplineCurve3D> curve_list;
        std::vector<BSplineCurve2D> pcurve_list1;
        std::vector<BSplineCurve2D> pcurve_list2;
    };

    static std::vector<IntersectionInfo> intersect(const ParamSurface &surf1, const ParamSurface &surf2) {
        auto [begin_param1, begin_param2] = find_initial_guess(surf1, surf2);
        return trace_pcurve(surf1, surf2, begin_param1, begin_param2);
    }

    static IntersectionResult intersect_all(const ParamSurface &surf1, const ParamSurface &surf2) {
        auto inital = find_all_possible_initial_guess(surf1, surf2);

        spdlog::info("initial size {}", inital.size());

        IntersectionResult result;

        std::vector<KDTree::KDTree<3, KDTree::PointPrimitive<3>>> curve_kd_trees;

        for (auto [begin_param1, begin_param2] : inital) {
            auto [refine_param1, refine_param2] = refine_with_newton(surf1, surf2, begin_param1, begin_param2);
            bool traced = false;

            float min_dis_pos = 1;

            for (auto &t : curve_kd_trees) {
                auto pos1 = surf1.evaluate(refine_param1);
                auto dist = t.nearestNeighbor(t.root, pos1).distance_to(pos1);
                auto dist_pos = glm::distance(pos1, surf2.evaluate(refine_param2));
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
                result.traces.push_back(trace);

                std::vector<KDTree::PointPrimitive<3>> points;
                for (auto info : trace) {
                    points.emplace_back(surf1.evaluate(info.param1));
                }
                curve_kd_trees.emplace_back(points);
            }
        }

        spdlog::info("components num {}", result.traces.size());

        for (auto &trace : result.traces) {
            spdlog::info("size {}", trace.size());
            std::vector<PointType> points;
            std::vector<ParamType> params1, params2;
            for (auto [param1, param2, pos] : trace) {
                points.push_back(pos);
                params1.push_back(param1);
                params2.push_back(param2);
            }

            // fit the 3d curve with BSpline curve
            auto &&curve = BSplineCurve3D::fit(points, 5, 50);
            result.curve_list.push_back(std::move(curve));

            // fit the pcurves with BSpline curves
            auto &&pcurve1 = BSplineCurve2D::fit(params1, 3, 50);
            result.pcurve_list1.push_back(std::move(pcurve1));

            auto &&pcurve2 = BSplineCurve2D::fit(params2, 3, 50);
            result.pcurve_list2.push_back(std::move(pcurve2));
        }

        return result;
    }
};

} // namespace GraphicsLab::Geometry