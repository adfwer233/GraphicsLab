#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/parametric_curves/ellipse.hpp"
#include "geometry/parametric/torus.hpp"
#include "ssi_results.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct TorusTorusIntersection {
    static bool is_concentric_coaxial(const Torus *torus1, const Torus *torus2,
                                      double tol = Tolerance::default_tolerance) {
        if (torus1 == nullptr || torus2 == nullptr) {
            return false;
        }

        if (glm::length(torus1->base_normal) <= tol || glm::length(torus2->base_normal) <= tol) {
            return false;
        }

        const auto n1 = glm::normalize(torus1->base_normal);
        const auto n2 = glm::normalize(torus2->base_normal);
        if (std::abs(std::abs(glm::dot(n1, n2)) - 1.0) > tol) {
            return false;
        }

        // Coaxial: center delta has no component orthogonal to the shared axis.
        const BRepVector3 center_delta = torus2->center - torus1->center;
        const BRepVector3 radial_delta = center_delta - glm::dot(center_delta, n1) * n1;
        return glm::length(radial_delta) <= tol;
    }

    static std::vector<SSIResult> solve_concentric_coaxial(const Torus *torus1, const Torus *torus2) {
        if (torus1 == nullptr || torus2 == nullptr) {
            throw cpptrace::runtime_error("[TorusTorusIntersection::solve_concentric_coaxial] torus1 or torus2 is null");
        }

        constexpr double eps = Tolerance::default_tolerance;

        const double R1 = torus1->major_radius;
        const double r1 = torus1->minor_radius;
        const double R2 = torus2->major_radius;
        const double r2 = torus2->minor_radius;

        if (R1 <= eps || r1 <= eps || R2 <= eps || r2 <= eps) {
            return {};
        }

        if (!is_concentric_coaxial(torus1, torus2, eps)) {
            return {};
        }

        const BRepVector3 axis_normal = glm::normalize(torus1->base_normal);
        const double dz = glm::dot(torus2->center - torus1->center, axis_normal);

        // Intersect meridional circles in (rho, z):
        // C1 center=(R1, 0), radius=r1; C2 center=(R2, dz), radius=r2.
        const BRepPoint2 c1{R1, 0.0};
        const BRepPoint2 c2{R2, dz};
        const BRepVector2 delta = c2 - c1;
        const double d = glm::length(delta);

        if (d <= eps && std::abs(r1 - r2) <= eps) {
            // Coincident meridional circles -> infinitely many intersection circles.
            return {};
        }
        if (d > r1 + r2 + eps || d < std::abs(r1 - r2) - eps || d <= eps) {
            return {};
        }

        const BRepVector2 ex = delta / d;
        const BRepVector2 ey{-ex.y, ex.x};

        const double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
        const double h2 = r1 * r1 - a * a;
        if (h2 < -eps) {
            return {};
        }

        const double h = std::sqrt(std::max(0.0, h2));
        const BRepPoint2 p = c1 + a * ex;

        std::vector<BRepPoint2> meridional_points;
        meridional_points.reserve(2);
        meridional_points.push_back(p + h * ey);
        if (h > eps) {
            meridional_points.push_back(p - h * ey);
        }

        auto allocator = BRepAllocator::instance();

        auto build_torus_pcurve = [allocator](const Torus *torus, ParamCurve3D *curve) -> ParamCurve2D * {
            std::vector<BRepPoint2> uv_samples;
            uv_samples.reserve(101);

            double prev_u = 0.0;
            bool has_prev = false;

            for (int i = 0; i <= 100; ++i) {
                const double t = static_cast<double>(i) / 100.0;
                auto uv = torus->project(curve->evaluate(t)).second;
                uv.x = uv.x - std::floor(uv.x);

                if (!has_prev) {
                    prev_u = uv.x;
                    has_prev = true;
                } else {
                    const double du = uv.x - prev_u;
                    if (du > 0.5) {
                        uv.x -= 1.0;
                    } else if (du < -0.5) {
                        uv.x += 1.0;
                    }
                    prev_u = uv.x;
                }

                uv_samples.push_back(uv);
            }

            auto fitted = BSplineCurve2D::fit(uv_samples, 3, 20);
            return allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(fitted));
        };

        auto build_branch = [&](double rho, double z) -> std::optional<SSIResult> {
            if (rho < -eps) {
                return std::nullopt;
            }
            rho = std::max(0.0, rho);

            if (std::abs(rho) <= eps) {
                const BRepPoint3 p = torus1->center + axis_normal * z;
                auto uv1 = torus1->project(p).second;
                auto uv2 = torus2->project(p).second;
                uv1.x = uv1.x - std::floor(uv1.x);
                uv2.x = uv2.x - std::floor(uv2.x);

                ParamCurve3D *inter_curve = allocator->alloc_param_curve<DegeneratedCurve3D>(p);
                ParamCurve2D *pcurve1 = allocator->alloc_param_pcurve<DegeneratedCurve2D>(uv1);
                ParamCurve2D *pcurve2 = allocator->alloc_param_pcurve<DegeneratedCurve2D>(uv2);
                return SSIResult{inter_curve, pcurve1, pcurve2};
            }

            const BRepPoint3 c = torus1->center + axis_normal * z;
            const BRepVector3 axis1 = glm::normalize(torus1->direction1) * rho;
            const BRepVector3 axis2 = glm::normalize(torus1->direction2) * rho;

            ParamCurve3D *inter_curve = allocator->alloc_param_curve<Ellipse3D>(c, axis1, axis2);
            ParamCurve2D *pcurve1 = build_torus_pcurve(torus1, inter_curve);
            ParamCurve2D *pcurve2 = build_torus_pcurve(torus2, inter_curve);
            return SSIResult{inter_curve, pcurve1, pcurve2};
        };

        std::vector<SSIResult> results;
        for (const auto &mp : meridional_points) {
            if (auto branch = build_branch(mp.x, mp.y); branch.has_value()) {
                results.push_back(branch.value());
            }
        }

        return results;
    }
};

} // namespace GraphicsLab::Geometry::BRep




