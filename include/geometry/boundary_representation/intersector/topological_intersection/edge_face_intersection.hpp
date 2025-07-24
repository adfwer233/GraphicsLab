#pragma once
#include "geometry/boundary_representation/base/vec_def.hpp"

#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/boundary_representation/intersector/curve_surface_intersection/general_curve_surface_intersection.hpp"
#include "geometry/boundary_representation/topology/trimming_utils.hpp"

namespace GraphicsLab::Geometry::BRep {

struct EdgeFaceIntersectionResult {
    BRepPoint3 position;
    double edge_param;
    BRepPoint2 face_param;
};

/**
 * @brief Intersection between given edge and face.
 *
 * @note
 *
 * 1. Intersect the curve with surface
 * 2. determine whether the point is in the edge param range.
 * 3. determine whether the point is contained in face
 */
struct EdgeFaceIntersection {
    static std::vector<EdgeFaceIntersectionResult> solve(const Edge *edge, const Face *face) {
        return intersect(edge, face);
    }

  private:
    static std::vector<EdgeFaceIntersectionResult> intersect(const Edge *edge, const Face *face) {
        std::vector<EdgeFaceIntersectionResult> result;

        ParamCurve3D *curve = edge->geometry()->param_geometry();
        ParamSurface *surface = face->geometry()->param_geometry();

        auto csi_results = GeneralCurveSurfaceIntersection::solve(curve, surface);

        for (auto csi : csi_results) {
            if (not edge->param_range().contains(csi.curve_parameter))
                continue;

            if (ContainmentQuery::contained(face, csi.surface_parameter) ==
                ContainmentQuery::ContainmentResult::Outside) {
                continue;
            }

            EdgeFaceIntersectionResult efi_result;
            efi_result.position = csi.inter_position;
            efi_result.edge_param = csi.curve_parameter;
            efi_result.face_param = csi.surface_parameter;

            result.emplace_back(efi_result);
        }

        return result;
    }
};

} // namespace GraphicsLab::Geometry::BRep