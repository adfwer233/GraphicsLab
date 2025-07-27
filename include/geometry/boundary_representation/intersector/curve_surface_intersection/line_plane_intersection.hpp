#pragma once

#include "csi_results.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/plane.hpp"

namespace GraphicsLab::Geometry::BRep {

struct LinePlaneIntersection {

    /**
     * intersect line with plane
     * @param line
     * @param plane
     * @return
     *
     * @note
     *
     * plane: base o, dir x and y, param u, v
     * line base p, dir d, param t
     *
     * p + td = ux + vy + o
     * ---> [x, y, -d] [u, v, t]^T = p - o
     * ---> [u, v, t]^T = [x, y, -d]^{-1} (p - o)
     */
    static std::vector<CSIResult> solve(const StraightLine3D *line, const Plane *plane, bool check_line_range = true,
                                        bool check_plane_range = true) {
        glm::dmat3 M = glm::dmat3();
        M[0][0] = plane->u_direction_.x;
        M[1][0] = plane->u_direction_.y;
        M[2][0] = plane->u_direction_.z;
        M[0][1] = plane->v_direction_.x;
        M[1][1] = plane->v_direction_.y;
        M[2][1] = plane->v_direction_.z;

        auto d = line->end_point - line->start_point;
        M[0][2] = -d.x;
        M[1][2] = -d.y;
        M[2][2] = -d.z;

        double det = glm::determinant(M);

        if (std::abs(det) > 1e-4) {
            auto M_inv = glm::inverse(M);
            auto b = line->start_point - plane->base_point_;
            auto res = b * M_inv;

            double u = res[0], v = res[1], t = res[2];

            if (check_line_range) {
                if (t < 0 or t > 1)
                    return {};
            }

            if (not check_plane_range or (u > 0 and u < 1 and v > 0 and v < 1)) {
                CSIResult csi{};
                csi.surface_parameter = {u, v};
                csi.curve_parameter = t;
                csi.inter_position = line->evaluate(t);

                return {csi};
            }
        }

        return {};
    }
};

} // namespace GraphicsLab::Geometry::BRep