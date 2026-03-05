#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/bspline_curve_3d.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/torus.hpp"
#include "ssi_results.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

// compute the intersection between plane and torus, together with pcurves
struct PlaneTorusIntersection {
    static std::vector<SSIResult> solve(const Plane *plane, const Torus *torus) {
        if (plane == nullptr || torus == nullptr) {
            throw cpptrace::runtime_error("[PlaneTorusIntersection::solve] plane or torus is null");
        }

        auto allocator = BRepAllocator::instance();

        BRepVector3 n_plane = plane->normal_;

        // coefficients in torus local frame: a = n_plane . dir1, b = n_plane . dir2, c = n_plane . base_normal
        double a = glm::dot(n_plane, torus->direction1);
        double b = glm::dot(n_plane, torus->direction2);
        double c = glm::dot(n_plane, torus->base_normal);

        // constant term d' = n_plane . (center - base_point)
        double dprime = glm::dot(n_plane, torus->center - plane->base_point_);

        double R = torus->major_radius;
        double r = torus->minor_radius;

        const int v_samples = 720;
        const int u_samples_special = 256;
        const double pi = 3.14159265358979323846;
        const double two_pi = 2.0 * pi;
        const double eps = 1e-12;
        const double rhs_tol = 1e-9;
        const double merge_u_tol = 1e-8;
        const double split_jump_tol = 0.45 * pi;

        struct BranchData {
            std::vector<BRepPoint3> points;
            std::vector<BRepPoint2> params_torus;
            std::vector<BRepPoint2> params_plane;
        };

        std::vector<BranchData> branches;

        auto normalize_u = [&](double u) -> double {
            while (u < 0)
                u += two_pi;
            while (u >= two_pi)
                u -= two_pi;
            return u;
        };

        auto wrap_angle_delta = [&](double a0, double a1) -> double {
            double d = a1 - a0;
            while (d <= -pi)
                d += two_pi;
            while (d > pi)
                d -= two_pi;
            return d;
        };

        auto push_solution = [&](BranchData &branch, double u, double v) {
            BRepPoint2 param_torus{u / two_pi, v / two_pi};
            BRepPoint3 pos = torus->evaluate(param_torus);
            branch.points.push_back(pos);
            branch.params_torus.push_back(param_torus);

            // plane pcurve: project pos onto plane to get plane param
            auto [proj, proj_uv] = plane->project(pos);
            (void)proj;
            branch.params_plane.push_back(proj_uv);
        };

        double ab_norm = std::sqrt(a * a + b * b);

        // Special axis-parallel case: equation does not depend on u and yields full-u loops.
        if (ab_norm < eps) {
            if (std::abs(c) < eps || std::abs(r) < eps) {
                return {};
            }

            double k = -dprime / (c * r);
            if (k < -1.0 - rhs_tol || k > 1.0 + rhs_tol) {
                return {};
            }

            k = std::clamp(k, -1.0, 1.0);
            double v0 = std::asin(k);
            std::array<double, 2> candidate_v = {normalize_u(v0), normalize_u(pi - v0)};

            std::array<double, 2> unique_v = {candidate_v[0], candidate_v[0]};
            int unique_count = 1;
            if (std::abs(wrap_angle_delta(candidate_v[0], candidate_v[1])) > merge_u_tol) {
                unique_v[1] = candidate_v[1];
                unique_count = 2;
            }

            for (int i = 0; i < unique_count; ++i) {
                BranchData branch;
                double v = unique_v[i];
                for (int uu = 0; uu <= u_samples_special; ++uu) {
                    double u = (static_cast<double>(uu) / u_samples_special) * two_pi;
                    push_solution(branch, normalize_u(u), v);
                }
                if (branch.points.size() >= 6) {
                    branches.push_back(std::move(branch));
                }
            }
        } else {
            // Track two analytic branches u_+(v), u_-(v) and split when continuity breaks.
            std::array<bool, 2> active = {false, false};
            std::array<BranchData, 2> current;
            std::array<double, 2> prev_u = {0.0, 0.0};

            auto flush_branch = [&](int idx) {
                if (!active[idx]) {
                    return;
                }
                if (current[idx].points.size() >= 6) {
                    branches.push_back(std::move(current[idx]));
                }
                current[idx] = BranchData{};
                active[idx] = false;
            };

            for (int i = 0; i <= v_samples; ++i) {
                double v = (static_cast<double>(i) / v_samples) * two_pi; // angle in radians
                double cosv = std::cos(v);
                double sinv = std::sin(v);

                double rr = R + r * cosv;
                double denom = rr * ab_norm;
                if (std::abs(denom) < eps) {
                    flush_branch(0);
                    flush_branch(1);
                    continue;
                }

                double rhs = -(c * r * sinv + dprime) / denom;
                if (rhs < -1.0 - rhs_tol || rhs > 1.0 + rhs_tol) {
                    flush_branch(0);
                    flush_branch(1);
                    continue;
                }

                rhs = std::clamp(rhs, -1.0, 1.0);
                double phi = std::atan2(b, a);
                double ang = std::acos(rhs);
                std::array<double, 2> u_values = {normalize_u(phi + ang), normalize_u(phi - ang)};

                // Tangency: avoid creating duplicate branches when both roots collapse.
                bool duplicate_roots = std::abs(wrap_angle_delta(u_values[0], u_values[1])) <= merge_u_tol;

                for (int idx = 0; idx < 2; ++idx) {
                    if (duplicate_roots && idx == 1) {
                        flush_branch(idx);
                        continue;
                    }

                    double u = u_values[idx];
                    if (active[idx]) {
                        double du = std::abs(wrap_angle_delta(prev_u[idx], u));
                        if (du > split_jump_tol) {
                            flush_branch(idx);
                        }
                    }

                    if (!active[idx]) {
                        active[idx] = true;
                        current[idx] = BranchData{};
                    }

                    push_solution(current[idx], u, normalize_u(v));
                    prev_u[idx] = u;
                }
            }

            flush_branch(0);
            flush_branch(1);
        }

        std::vector<SSIResult> results;

        // build BSpline curves from collected branches
        for (const auto &branch : branches) {
            if (branch.points.size() < 6)
                continue;

            // fit 3D curve
            int control_points_count = std::min(static_cast<size_t>(40), std::max(static_cast<size_t>(10), branch.points.size() / 2));
            auto &&curve = BSplineCurve3D::fit(branch.points, 5, control_points_count);
            curve.control_points_.front() = branch.points.front();
            curve.control_points_.back() = branch.points.back();
            BSplineCurve3D *curve_alloc = allocator->alloc_param_curve<BSplineCurve3D>(std::move(curve));

            // fit torus pcurve (u,v) params
            auto &&pcurve_t = BSplineCurve2D::fit(branch.params_torus, 5, control_points_count);
            BSplineCurve2D *pcurve_t_alloc = allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(pcurve_t));

            // fit plane pcurve (u,v) params
            auto &&pcurve_p = BSplineCurve2D::fit(branch.params_plane, 5, control_points_count);
            BSplineCurve2D *pcurve_p_alloc = allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(pcurve_p));

            SSIResult res;
            res.inter_curve = curve_alloc;
            res.pcurve1 = pcurve_p_alloc;   // plane pcurve
            res.pcurve2 = pcurve_t_alloc;   // torus pcurve
            results.push_back(res);
        }

        return results;
    }
};

} // namespace GraphicsLab::Geometry::BRep
