#pragma once
#include "geometry/boundary_representation/geometry_defintition.hpp"
#include "geometry/parametric/parametric_surface.hpp"
#include "surface_surface_intersection/general_surface_surface_intersection.hpp"
#include "surface_surface_intersection/ssi_results.hpp"

#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct GeometricalIntersector {

    /**
     * Find the intersection curves between two Surface
     * @param surf1
     * @param surf2
     * @return
     */
    static std::vector<SSIResult> surface_surface_intersector(const Surface *surf1, const Surface *surf2) {

        auto param_surface1 = surf1->param_geometry();
        auto param_surface2 = surf2->param_geometry();

        // @todo: add special cases and dispatcher

        auto result = GeneralSurfaceSurfaceIntersection::solve(param_surface1, param_surface2);

        // @todo: cut intersection in surface param range.

        return result;
    }
};

}