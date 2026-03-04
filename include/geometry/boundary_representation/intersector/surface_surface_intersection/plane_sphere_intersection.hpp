#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/parametric/parametric_curves/degenerated_curve.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "ssi_results.hpp"

namespace GraphicsLab::Geometry::BRep {

// compute the intersection between plane and sphere, together with pcurves
struct PlaneSphereIntersection {
    static std::vector<SSIResult> solve(const Plane *plane, const Sphere *sphere) {
        if (plane == nullptr || sphere == nullptr) {
            throw cpptrace::runtime_error("[PlaneSphereIntersection::solve] plane or sphere is null");
        }

        // project the sphere center to the plane.

        BRepPoint3 center = sphere->center();
        double radius = sphere->radius();
        auto [proj, proj_param] = plane->project(center);
        auto [proj_sphere, proj_sphere_param] = sphere->project(proj);

        auto allocator = BRepAllocator::instance();

        double distance = glm::distance(center, proj);
        if (distance < radius - Tolerance::default_tolerance) {
            // intersected



        } else if (distance < radius + Tolerance::default_tolerance) {
            // tangential: use a degenerated curve
            ParamCurve3D* int_curve = allocator->alloc_param_curve<DegeneratedCurve3D>(proj);
            ParamCurve2D* plane_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(proj_param);
            ParamCurve2D* sphere_pcurve = allocator->alloc_param_pcurve<DegeneratedCurve2D>(proj_sphere_param);

            SSIResult result{int_curve, plane_pcurve, sphere_pcurve};
        } else {
            return {};
        }

        return {};
    }
};

}