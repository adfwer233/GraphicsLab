#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/parametric_curves/ellipse.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/torus.hpp"
#include "ssi_results.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct SphereTorusIntersection {
    static bool is_concentric_coaxial(const Sphere *sphere, const Torus *torus,
                                      double tol = Tolerance::default_tolerance) {
        if (sphere == nullptr || torus == nullptr) {
            return false;
        }

        if (glm::length(torus->base_normal) <= tol) {
            return false;
        }

        const auto axis = glm::normalize(torus->base_normal);
        const BRepVector3 center_delta = sphere->center() - torus->center;
        const BRepVector3 radial_delta = center_delta - glm::dot(center_delta, axis) * axis;
        return glm::length(radial_delta) <= tol;
    }

    static std::vector<SSIResult> solve_concentric_coaxial(const Sphere *sphere, const Torus *torus) {
        if (sphere == nullptr || torus == nullptr) {
            throw cpptrace::runtime_error("[SphereTorusIntersection::solve_concentric_coaxial] sphere or torus is null");
        }

        constexpr double eps = Tolerance::default_tolerance;
        constexpr double pi = 3.14159265358979323846;
        constexpr double two_pi = 2.0 * pi;

        const double rs = sphere->radius();
        const double R = torus->major_radius;
        const double r = torus->minor_radius;

        if (r <= eps || R < eps || rs < eps) {
            return {};
        }

        if (!is_concentric_coaxial(sphere, torus, eps)) {
            return {};
        }

        const BRepVector3 axis_normal = glm::normalize(torus->base_normal);
        const double dz = glm::dot(sphere->center() - torus->center, axis_normal);

        // Intersect meridional circles in (rho, z):
        // torus circle center=(R, 0), radius=r; sphere circle center=(0, dz), radius=rs.
        const BRepPoint2 c1{R, 0.0};
        const BRepPoint2 c2{0.0, dz};
        const BRepVector2 delta = c2 - c1;
        const double d = glm::length(delta);

        if (d <= eps && std::abs(r - rs) <= eps) {
            // Coincident meridional circles -> infinitely many intersection circles.
            return {};
        }
        if (d > r + rs + eps || d < std::abs(r - rs) - eps || d <= eps) {
            return {};
        }

        const BRepVector2 ex = delta / d;
        const BRepVector2 ey{-ex.y, ex.x};

        const double a = (r * r - rs * rs + d * d) / (2.0 * d);
        const double h2 = r * r - a * a;
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
        std::vector<SSIResult> results;
        results.reserve(meridional_points.size());

        auto build_sphere_pcurve = [&](ParamCurve3D *curve) -> ParamCurve2D * {
            std::vector<BRepPoint2> uv_samples;
            uv_samples.reserve(101);

            double prev_u = 0.0;
            bool has_prev = false;
            for (int i = 0; i <= 100; ++i) {
                const double t = static_cast<double>(i) / 100.0;
                auto uv = sphere->project(curve->evaluate(t)).second;
                uv.x = uv.x - std::floor(uv.x);

                if (!has_prev) {
                    prev_u = uv.x;
                    has_prev = true;
                } else {
                    double du = uv.x - prev_u;
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

        for (const auto &mp : meridional_points) {
            double circle_r = mp.x;
            const double z = mp.y;

            if (circle_r < -eps) {
                continue;
            }
            circle_r = std::max(0.0, circle_r);

            double v = std::atan2(z, circle_r - R);
            if (v < 0.0) {
                v += two_pi;
            }

            if (std::abs(circle_r) < eps) {
                const BRepPoint3 p = torus->center + axis_normal * z;
                auto torus_uv = BRepPoint2{0.0, v / two_pi};
                auto sphere_uv = sphere->project(p).second;
                sphere_uv.x = sphere_uv.x - std::floor(sphere_uv.x);

                ParamCurve3D *inter_curve = allocator->alloc_param_curve<DegeneratedCurve3D>(p);
                ParamCurve2D *torus_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(torus_uv);
                ParamCurve2D *sphere_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(sphere_uv);
                results.push_back(SSIResult{inter_curve, sphere_pcurve, torus_pcurve});
                continue;
            }

            const BRepPoint3 c = torus->center + axis_normal * z;
            const BRepVector3 axis1 = glm::normalize(torus->direction1) * circle_r;
            const BRepVector3 axis2 = glm::normalize(torus->direction2) * circle_r;
            ParamCurve3D *inter_curve = allocator->alloc_param_curve<Ellipse3D>(c, axis1, axis2);

            ParamCurve2D *sphere_pcurve = build_sphere_pcurve(inter_curve);
            ParamCurve2D *torus_pcurve = allocator->alloc_param_pcurve<StraightLine2D>(
                BRepPoint2{0.0, v / two_pi}, BRepPoint2{1.0, v / two_pi});

            results.push_back(SSIResult{inter_curve, sphere_pcurve, torus_pcurve});
        }

        return results;
    }
};

} // namespace GraphicsLab::Geometry::BRep




