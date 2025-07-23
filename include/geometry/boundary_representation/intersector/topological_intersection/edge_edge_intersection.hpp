#pragma once

#include "cpptrace/cpptrace.hpp"
#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/base/vec_def.hpp"
#include "geometry/boundary_representation/intersector/pcurve_pcurve_intersection/general_pcurve_pcurve_intersection.hpp"
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
 * @note Given two edges belong to the same surface, and we will find the intersection between them
 */
struct EdgeEdgeIntersection {
    std::vector<EdgeEdgeIntersectionResult> solve(Edge *edge1, Edge *edge2, Face *face) {
        /**
         * Check the input conditions
         */

        Coedge *coedge1 = TopologyUtils::get_coedge_of_given_face(edge1, face);
        Coedge *coedge2 = TopologyUtils::get_coedge_of_given_face(edge2, face);

        if (coedge1 == nullptr or coedge2 == nullptr) {
            throw cpptrace::logic_error("Edges must belong to same face in topological intersection");
        }

        if (face->geometry() == nullptr) {
            throw cpptrace::logic_error("Face has no geometry");
        }

        ParamSurface *param_surface = face->geometry()->param_geometry();

        PCurve *pcurve1 = coedge1->geometry();
        PCurve *pcurve2 = coedge2->geometry();

        ParamCurve3D *curve1 = edge1->geometry()->param_geometry();
        ParamCurve3D *curve2 = edge2->geometry()->param_geometry();

        ParamCurve2D *param_pcurve1 = pcurve1->param_geometry();
        ParamCurve2D *param_pcurve2 = pcurve2->param_geometry();

        std::vector<EdgeEdgeIntersectionResult> results;

        auto param_pcurve_inter_result = GeneralPCurvePCurveIntersection::solve(param_pcurve1, param_pcurve2);

        for (const auto &inter : param_pcurve_inter_result) {
            EdgeEdgeIntersectionResult intersection_result;
            auto position = param_surface->evaluate(inter.inter_position);

            // @todo: jointly optimize param1 and param2

            // use the parameter of the pcurve as a guess
            auto [pos1, param1] = curve1->projection(position, inter.param1);
            auto [pos2, param2] = curve2->projection(position, inter.param2);

            if (glm::distance(pos1, pos2) > Tolerance::default_tolerance) {
                throw cpptrace::logic_error("Curve projection failed. Maybe the intersection point is wrong");
            }

            intersection_result.param1 = param1;
            intersection_result.param2 = param2;

            if (not edge1->param_range().contains(param1) or not edge2->param_range().contains(param2)) {
                continue;
            }
            results.push_back(intersection_result);
        }

        return results;
    }
};

} // namespace GraphicsLab::Geometry::BRep