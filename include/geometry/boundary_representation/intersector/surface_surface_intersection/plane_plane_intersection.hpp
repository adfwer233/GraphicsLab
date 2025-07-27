#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/param_range.hpp"
#include "geometry/boundary_representation/intersector/curve_surface_intersection/line_plane_intersection.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/plane.hpp"
#include "ssi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

struct PlanePlaneIntersection {

    static std::vector<SSIResult> solve(const Plane *plane1, const Plane *plane2) {
        std::vector<SSIResult> result;

        auto dir1 = plane1->u_direction_ + plane1->v_direction_;
        auto dir2 = plane2->u_direction_ + plane2->v_direction_ * 2.0;

        dir1 = glm::normalize(dir1);
        dir2 = glm::normalize(dir2);

        auto n1 = plane1->normal_;
        auto n2 = plane2->normal_;

        auto inter_dir = glm::cross(n1, n2);

        if (glm::length(inter_dir) < 1e-10) {
            return {};
        }
        inter_dir = glm::normalize(inter_dir);

        StraightLine3D line1(plane1->base_point_, plane1->base_point_ + dir1 * 100.0);
        StraightLine3D line2(plane2->base_point_, plane2->base_point_ + dir2 * 100.0);

        auto inter1 = LinePlaneIntersection::solve(&line1, plane2, false, false);
        auto inter2 = LinePlaneIntersection::solve(&line2, plane2, false, false);

        BRepPoint3 base;

        if (not inter1.empty()) {
            base = inter1.front().inter_position;
        } else if (not inter2.empty()) {
            base = inter2.front().inter_position;
        } else {
            throw cpptrace::logic_error("No intersection found");
        }

        std::optional<ParamRange> r1_opt = intersect_with_plane_boundary(base, inter_dir, plane1);
        std::optional<ParamRange> r2_opt = intersect_with_plane_boundary(base, inter_dir, plane2);

        if (r1_opt.has_value() and r2_opt.has_value()) {
            auto r1 = r1_opt.value();
            auto r2 = r2_opt.value();

            auto l = std::max(r1.start(), r2.start());
            auto r = std::min(r1.end(), r2.end());

            BRepPoint3 inter_start = base + l * inter_dir;
            BRepPoint3 inter_end = base + r * inter_dir;

            BRepPoint2 inter_start_uv1 = plane1->project(inter_start).second;
            BRepPoint2 inter_start_uv2 = plane2->project(inter_start).second;
            BRepPoint2 inter_end_uv1 = plane1->project(inter_end).second;
            BRepPoint2 inter_end_uv2 = plane2->project(inter_end).second;

            auto allocator = BRepAllocator::instance();

            auto curve = allocator->alloc_param_curve<StraightLine3D>(inter_start, inter_end);
            auto pcurve1 = allocator->alloc_param_pcurve<StraightLine2D>(inter_start_uv1, inter_end_uv1);
            auto pcurve2 = allocator->alloc_param_pcurve<StraightLine2D>(inter_start_uv2, inter_end_uv2);

            SSIResult ssi;
            ssi.inter_curve = curve;
            ssi.pcurve1 = pcurve1;
            ssi.pcurve2 = pcurve2;

            return {ssi};
        }
        return {};
    }

  private:
    static std::optional<ParamRange> intersect_with_plane_boundary(BRepPoint3 p, BRepPoint3 d, const Plane *plane) {
        std::vector<double> params;

        BRepPoint2 p_uv = plane->project(p).second;
        BRepVector2 dir_uv = plane->project(p + d).second - p_uv;

        // intersect with u = 0 or 1, v \in [0, 1]
        if (std::abs(dir_uv.x) > 1e-10) {
            // u = 0
            {
                double t = -p_uv.x / dir_uv.x;
                auto par_pos = p_uv + t * dir_uv;
                if (par_pos.y > 0 and par_pos.y < 1) {
                    params.push_back(t);
                }
            }

            // u = 1
            {
                double t = (1 - p_uv.x) / dir_uv.x;
                auto par_pos = p_uv + t * dir_uv;
                if (par_pos.y > 0 and par_pos.y < 1) {
                    params.push_back(t);
                }
            }
        }

        if (std::abs(dir_uv.y) > 1e-10) {
            // v = 0
            {
                double t = -p_uv.y / dir_uv.y;
                auto par_pos = p_uv + t * dir_uv;
                if (par_pos.x > 0 and par_pos.x < 1) {
                    params.push_back(t);
                }
            }

            // v = 1
            {
                double t = (1 - p_uv.y) / dir_uv.y;
                auto par_pos = p_uv + t * dir_uv;
                if (par_pos.x > 0 and par_pos.x < 1) {
                    params.push_back(t);
                }
            }
        }

        std::ranges::sort(params);

        if (params.size() != 2) {
            return std::nullopt;
        }
        return ParamRange{params.front(), params.back()};
    }
};

} // namespace GraphicsLab::Geometry::BRep