#pragma once

#include "cpptrace/cpptrace.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/boundary_representation/topology/topology_utils.hpp"
#include "geometry/boundary_representation/topology_definition.hpp"
namespace GraphicsLab::Geometry::BRep {

struct EdgeEdgeIntersectionResult {
    double param1;
    double param2;
    BRepPoint2 param_intersection_point;
    BRepPoint3 intersection_point;
};

/**
 * @brief Topological intersection between edges.
 *
 * @note Given two edges belong to the same surface and we will find the intersection between them
 */
struct EdgeEdgeIntersection {
    std::vector<EdgeEdgeIntersectionResult> solve(Edge *edge1, Edge *edge2, Face *face) {
        /**
         * Check the input conditions
         */

        Coedge *coedge1 = TopologyUtils::get_coedge_of_given_face(edge1, face);
        Coedge *coedge2 = TopologyUtils::get_coedge_of_given_face(edge2, face);

        if (coedge1 == nullptr or coedge2 == nullptr) {
            throw cpptrace::logic_error("Edges must belong to same face in topological intersection").
        }
    }
};

} // namespace GraphicsLab::Geometry::BRep