#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/bspline_curve_2d.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/parametric_curves/ellipse.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "ssi_results.hpp"
#include <cmath>

namespace GraphicsLab::Geometry::BRep {

// compute the intersection between plane and sphere, together with pcurves
struct PlaneSphereIntersection {
    static std::vector<SSIResult> solve(const Plane *plane, const Sphere *sphere) {
        if (plane == nullptr || sphere == nullptr) {
            throw cpptrace::runtime_error("[PlaneSphereIntersection::solve] plane or sphere is null");
        }

        BRepPoint3 center = sphere->center();
        double radius = sphere->radius();
        auto [proj, proj_param] = plane->project(center);
        auto [proj_sphere, proj_sphere_param] = sphere->project(proj); // reused in tangential branch
        proj_sphere_param.x = proj_sphere_param.x - std::floor(proj_sphere_param.x);

        auto allocator = BRepAllocator::instance();

        double distance = glm::distance(center, proj);
        if (distance < radius - Tolerance::default_tolerance) {
            // Proper intersection: a circle lying on the plane.
            double circle_radius = std::sqrt(radius * radius - distance * distance);
            BRepVector3 n = glm::normalize(plane->normal_);

            BRepVector3 axis1 = plane->u_direction_ - glm::dot(plane->u_direction_, n) * n;
            if (glm::length(axis1) < Tolerance::default_tolerance) {
                axis1 = glm::cross(n, BRepVector3{1.0, 0.0, 0.0});
                if (glm::length(axis1) < Tolerance::default_tolerance) {
                    axis1 = glm::cross(n, BRepVector3{0.0, 1.0, 0.0});
                }
            }
            axis1 = glm::normalize(axis1);
            BRepVector3 axis2 = glm::normalize(glm::cross(n, axis1));

            axis1 *= circle_radius;
            axis2 *= circle_radius;

            ParamCurve3D *int_curve = allocator->alloc_param_curve<Ellipse3D>(proj, axis1, axis2);

            BRepPoint2 center_uv = plane->project(proj).second;
            BRepPoint2 major_uv = plane->project(proj + axis1).second - center_uv;
            BRepPoint2 minor_uv = plane->project(proj + axis2).second - center_uv;
            ParamCurve2D *plane_pcurve = allocator->alloc_param_pcurve<Ellipse2D>(center_uv, major_uv, minor_uv);

            std::vector<BRepPoint2> sphere_uv_samples;
            sphere_uv_samples.reserve(101);
            for (int i = 0; i <= 100; ++i) {
                double t = static_cast<double>(i) / 100.0;
                BRepPoint3 p = int_curve->evaluate(t);
                auto uv = sphere->project(p).second;
                uv.x = uv.x - std::floor(uv.x); // keep pcurve in [0, 1) texture domain for rendering
                sphere_uv_samples.push_back(uv);
            }

            auto fitted = BSplineCurve2D::fit(sphere_uv_samples, 3, 20);
            ParamCurve2D *sphere_pcurve = allocator->alloc_param_pcurve<BSplineCurve2D>(std::move(fitted));

            return {SSIResult{int_curve, plane_pcurve, sphere_pcurve}};
        } else if (distance < radius + Tolerance::default_tolerance) {
            // tangential: use a degenerated curve
            ParamCurve3D *int_curve = allocator->alloc_param_curve<DegeneratedCurve3D>(proj);
            ParamCurve2D *plane_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(proj_param);
            ParamCurve2D *sphere_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(proj_sphere_param);

            return {SSIResult{int_curve, plane_pcurve, sphere_pcurve}};
        } else {
            return {};
        }
    }
};

}
