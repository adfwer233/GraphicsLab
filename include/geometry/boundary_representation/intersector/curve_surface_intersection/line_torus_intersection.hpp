#pragma once
#include "../../../../../cmake-build-relwithdebinfo/_deps/spdlog-src/include/spdlog/spdlog.h"
#include "csi_results.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/torus.hpp"
#include "numeric/polynomial/polynomial_solver.hpp"
#include "numeric/polynomial/real_polynomial.hpp"

namespace GraphicsLab::Geometry::BRep {

struct LineTorusIntersection {

    static std::vector<CSIResult> solve(const StraightLine3D *line, const Torus *torus, bool check_line_range = true) {
        std::vector<CSIResult> csi_results;

        double R = torus->major_radius;
        double r = torus->minor_radius;

        glm::dmat3x3 trans;

        trans[0][0] = torus->direction1.x;
        trans[1][0] = torus->direction1.y;
        trans[2][0] = torus->direction1.z;

        trans[0][1] = torus->direction2.x;
        trans[1][1] = torus->direction2.y;
        trans[2][1] = torus->direction2.z;

        trans[0][2] = torus->base_normal.x;
        trans[1][2] = torus->base_normal.y;
        trans[2][2] = torus->base_normal.z;

        BRepPoint3 p = (line->start_point - torus->center) * glm::transpose(trans);
        BRepVector3 d = glm::normalize(line->end_point - line->start_point) * glm::transpose(trans);

        double seg_len = glm::length(line->end_point - line->start_point);

        RealPolynomial s({ glm::dot(p, p) + R * R - r * r, glm::dot(d, p) * 2, glm::dot(d, d) });
        RealPolynomial b({ p.x * p.x + p.y * p.y, 2 * (p.x * d.x + p.y * d.y), d.x * d.x + d.y * d.y });

        RealPolynomial f = s * s - b * 4 * R * R;

        std::vector<double> line_params;

        auto roots = Numeric::PolynomialSolver::find_real_roots_with_multiplicity(f);

        for (auto [root, m]: roots) {
            if (check_line_range) {
                if (root > 0) {
                    line_params.push_back(root);
                }
            } else {
                line_params.push_back(root);
            }
        }

        for (auto param: line_params) {
            CSIResult csi_result{};

            csi_result.curve_parameter = param / seg_len;
            csi_result.inter_position = line->evaluate(csi_result.curve_parameter);

            auto [proj, surf_param] = torus->project(csi_result.inter_position);
            csi_result.surface_parameter = surf_param;

            csi_results.push_back(csi_result);
        }

        return csi_results;
    }

};

}
