#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/parametric_curves/ellipse.hpp"
#include "geometry/parametric/sphere.hpp"
#include "ssi_results.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct SphereSphereIntersection {
    static std::vector<SSIResult> solve(const Sphere *sphere1, const Sphere *sphere2) {
        if (sphere1 == nullptr || sphere2 == nullptr) {
            throw cpptrace::runtime_error("[SphereSphereIntersection::solve] sphere1 or sphere2 is null");
        }

        constexpr double eps = Tolerance::default_tolerance;

        const BRepPoint3 c1 = sphere1->center();
        const BRepPoint3 c2 = sphere2->center();
        const double r1 = sphere1->radius();
        const double r2 = sphere2->radius();

        const BRepVector3 c12 = c2 - c1;
        const double d = glm::length(c12);

        // Coincident spheres have infinitely many intersections; keep behavior finite.
        if (d < eps && std::abs(r1 - r2) < eps) {
            return {};
        }

        if (d > (r1 + r2 + eps) || d < (std::abs(r1 - r2) - eps)) {
            return {};
        }

        auto allocator = BRepAllocator::instance();

        auto build_sphere_pcurve = [allocator](const Sphere *sphere, ParamCurve3D *curve) -> ParamCurve2D * {
            std::vector<BRepPoint2> uv_samples;
            uv_samples.reserve(101);

            double prev_u = 0.0;
            bool has_prev = false;

            for (int i = 0; i <= 100; ++i) {
                const double t = static_cast<double>(i) / 100.0;
                const auto p = curve->evaluate(t);

                auto uv = sphere->project(p).second;
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

        const bool tangent_external = std::abs(d - (r1 + r2)) < eps;
        const bool tangent_internal = std::abs(d - std::abs(r1 - r2)) < eps;
        if (tangent_external || tangent_internal) {
            const BRepVector3 dir = (d < eps) ? BRepVector3{1.0, 0.0, 0.0} : (c12 / d);
            const BRepPoint3 p = c1 + dir * r1;

            auto uv1 = sphere1->project(p).second;
            auto uv2 = sphere2->project(p).second;
            uv1.x = uv1.x - std::floor(uv1.x);
            uv2.x = uv2.x - std::floor(uv2.x);

            ParamCurve3D *inter_curve = allocator->alloc_param_curve<DegeneratedCurve3D>(p);
            ParamCurve2D *pcurve1 = allocator->alloc_param_pcurve<DegeneratedCurve2D>(uv1);
            ParamCurve2D *pcurve2 = allocator->alloc_param_pcurve<DegeneratedCurve2D>(uv2);
            return {SSIResult{inter_curve, pcurve1, pcurve2}};
        }

        if (d < eps) {
            return {};
        }

        const BRepVector3 ex = c12 / d;
        const double a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
        const double h2 = r1 * r1 - a * a;
        if (h2 < -eps) {
            return {};
        }

        const double h = std::sqrt(std::max(0.0, h2));
        const BRepPoint3 center = c1 + a * ex;

        BRepVector3 axis_hint = glm::cross(ex, BRepVector3{1.0, 0.0, 0.0});
        if (glm::length(axis_hint) < eps) {
            axis_hint = glm::cross(ex, BRepVector3{0.0, 1.0, 0.0});
        }
        const BRepVector3 axis1 = glm::normalize(axis_hint) * h;
        const BRepVector3 axis2 = glm::normalize(glm::cross(ex, glm::normalize(axis_hint))) * h;

        ParamCurve3D *inter_curve = allocator->alloc_param_curve<Ellipse3D>(center, axis1, axis2);
        ParamCurve2D *pcurve1 = build_sphere_pcurve(sphere1, inter_curve);
        ParamCurve2D *pcurve2 = build_sphere_pcurve(sphere2, inter_curve);

        return {SSIResult{inter_curve, pcurve1, pcurve2}};
    }
};

} // namespace GraphicsLab::Geometry::BRep



