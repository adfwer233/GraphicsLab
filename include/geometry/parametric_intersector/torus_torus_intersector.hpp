#pragma once

#include <glm/glm.hpp>
#include <Eigen/Eigen>

#include "geometry/parametric/torus.hpp"

namespace GraphicsLab::Geometry {

struct TorusTorusIntersector {

    using PointType = glm::dvec3;
    using VecType = glm::dvec3;
    using ParamType = glm::dvec2;

    // Newton refinement to solve F(u,v,s,t) = S1(u,v) - S2(s,t) = 0
    static std::pair<ParamType, ParamType> refine_with_newton(
        const Torus& torus1, const Torus& torus2,
        ParamType param1, ParamType param2
    ) {
        constexpr int max_newton_iter = 10;
        constexpr double tol = 1e-8;

        for (int i = 0; i < max_newton_iter; ++i) {
            PointType p1 = torus1.evaluate(param1);
            PointType p2 = torus2.evaluate(param2);
            VecType F = p1 - p2;

            if (glm::length(F) < tol) break;

            auto [du1, dv1] = torus1.derivative(param1);
            auto [du2, dv2] = torus2.derivative(param2);

            Eigen::Matrix<double, 3, 4> J;
            J.col(0) = Eigen::Vector3d(du1.x, du1.y, du1.z);
            J.col(1) = Eigen::Vector3d(dv1.x, dv1.y, dv1.z);
            J.col(2) = -Eigen::Vector3d(du2.x, du2.y, du2.z);
            J.col(3) = -Eigen::Vector3d(dv2.x, dv2.y, dv2.z);

            Eigen::JacobiSVD<Eigen::Matrix<double, 3, 4>> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::Vector3d b(F.x, F.y, F.z);
            Eigen::Vector4d delta = svd.solve(b);

            // std::cout << "Singular values: " << svd.singularValues().transpose() << std::endl;
            // std::cout << svd.matrixU() << std::endl;
            // std::cout << svd.matrixV() << std::endl;
            param1 -= ParamType(delta[0], delta[1]);
            param2 -= ParamType(delta[2], delta[3]);

            // spdlog::info("F: {} {} {}", F.x, F.y, F.z);
            // spdlog::info("du1: {}, {}, {}", du1.x, du1.y, du1.z);
            // spdlog::info("dv1: {}, {}, {}", dv1.x, dv1.y, dv1.z);
            // spdlog::info("du2: {}, {}, {}", du2.x, du2.y, du2.z);
            // spdlog::info("dv2: {}, {}, {}", dv2.x, dv2.y, dv2.z);
            // spdlog::info("J: {} {} {} {}", J(0, 0), J(0, 1), J(0, 2), J(0, 3));
            // spdlog::info("delta: {} {} {} {}", delta[0], delta[1], delta[2], delta[3]);
            // spdlog::info("refine newton iter: {}, param1: ({}, {}), param2: ({}, {})", i, param1.x, param1.y, param2.x, param2.y);
        }

        return {param1, param2};
    }

    // Compute the tangent direction in parameter space by crossing the normals
    static std::pair<ParamType, ParamType> compute_tangent_direction(
        const Torus& torus1, const Torus& torus2,
        ParamType param1, ParamType param2
    ) {
        VecType n1 = torus1.normal(param1);
        VecType n2 = torus2.normal(param2);
        VecType tangent = glm::normalize(glm::cross(n1, n2));

        auto [du1, dv1] = torus1.derivative(param1);
        auto [du2, dv2] = torus2.derivative(param2);

        Eigen::Matrix2d A1;
        A1 << glm::dot(du1, tangent), glm::dot(dv1, tangent),
              0, 0;
        Eigen::Vector2d b1;
        b1 << glm::dot(tangent, tangent), 0;

        Eigen::Vector2d d_param1 = A1.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b1);

        Eigen::Matrix2d A2;
        A2 << glm::dot(du2, tangent), glm::dot(dv2, tangent),
              0, 0;
        Eigen::Vector2d b2;
        b2 << glm::dot(tangent, tangent), 0;

        Eigen::Vector2d d_param2 = A2.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b2);

        return {ParamType(d_param1[0], d_param1[1]), ParamType(d_param2[0], d_param2[1])};
    }

    static std::pair<ParamType, ParamType> find_initial_guess(const Torus &torus1, const Torus &torus2) {
        // Implement the logic to find the initial guess for the intersection

        auto torus1_sample = torus1.sample(10, 10);

        double min_distance = std::numeric_limits<double>::max();
        ParamType param1, param2;

        for (auto [pos, param]: torus1_sample) {
            auto [proj, proj_param] = torus2.project(pos);
            if (glm::distance(proj, pos) < min_distance) {
                min_distance = glm::distance(proj, pos);
                param1 = param;
                param2 = proj_param;
            }
        }

        spdlog::info("initial guess min distance: {}", min_distance);
        spdlog::info("initial guess before refine: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);

        return refine_with_newton(torus1, torus2, param1, param2);
    }

    struct IntersectionInfo {
        ParamType param1;
        ParamType param2;
        PointType position;
    };

    static std::vector<IntersectionInfo> intersect(const Torus &torus1, const Torus &torus2) {
        std::vector<IntersectionInfo> intersections;

        auto [begin_param1, begin_param2] = find_initial_guess(torus1, torus2);

        // spdlog::info("initial guess: ({}, {}), ({}, {})", begin_param1.x, begin_param1.y, begin_param2.x, begin_param2.y);
        // spdlog::info("initial distance: {}", glm::distance(torus1.evaluate(begin_param1), torus2.evaluate(begin_param2)));
        constexpr int max_iter = 1000;
        constexpr double length_iter = 0.0001;
        constexpr double step_length = 0.01;

        ParamType param1 = begin_param1;
        ParamType param2 = begin_param2;

        for (int step = 0; step < max_iter; step++) {
            // spdlog::info("current param: ({}, {}), ({}, {})", param1.x, param1.y, param2.x, param2.y);

            auto F = torus1.evaluate(param1) - torus2.evaluate(param2);

            if (glm::length(F) > length_iter) {
                auto [refine_param1, refine_param2] = refine_with_newton(torus1, torus2, param1, param2);
                param1 = refine_param1;
                param2 = refine_param2;
            }

            intersections.emplace_back(param1, param2, torus1.evaluate(param1));

            auto [param1_tangent, param2_tangent] = compute_tangent_direction(torus1, torus2, param1, param2);

            param1 = param1 + step_length * param1_tangent;
            param2 = param2 + step_length * param2_tangent;

            if (glm::distance(param1, begin_param1) < 1e-3 and glm::distance(param2, begin_param2) < 1e-3) {
                break;
            }
        }

        return intersections;
    }

};

}