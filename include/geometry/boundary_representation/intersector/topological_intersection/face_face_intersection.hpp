#pragma once
#include "geometry/boundary_representation/base/param_range.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/boundary_representation/intersector/pcurve_pcurve_intersection/general_pcurve_pcurve_intersection.hpp"
#include "geometry/boundary_representation/intersector/surface_surface_intersection/general_surface_surface_intersection.hpp"
#include "geometry/boundary_representation/topology/topology_utils.hpp"
#include "geometry/boundary_representation/topology/trimming_utils.hpp"
#include "geometry/parametric/parametric_curves/parametric_curve.hpp"

namespace GraphicsLab::Geometry::BRep {

struct FFIResult {
    ParamCurve3D *curve = nullptr;
    ParamCurve2D *pcurve1 = nullptr;
    ParamCurve2D *pcurve2 = nullptr;

    ParamRange pcurve_range1;
    ParamRange pcurve_range2;

    ParamRange curve_range;

    [[nodiscard]] std::pair<Coedge*, Coedge*> create_face_coedges() const {
        Curve* curve1 = TopologyUtils::create_curve_from_param_curve(curve);
        Edge* edge = TopologyUtils::create_edge_from_curve(curve1);
        edge->set_param_range(curve_range);

        Coedge* coedge = TopologyUtils::create_coedge_from_edge(edge);
        edge->set_coedge(coedge);

        Coedge* coedge2 = TopologyUtils::create_coedge_from_edge(edge);
        coedge2->set_param_range(curve_range);
        coedge2->set_forward(false);

        coedge->set_partner(coedge2);
        coedge2->set_partner(coedge);

        PCurve* pcurve1_ = TopologyUtils::create_pcurve_from_param_pcurve(pcurve1);
        coedge->set_param_range(pcurve_range1);

        PCurve* pcurve2_ = TopologyUtils::create_pcurve_from_param_pcurve(pcurve2);
        coedge2->set_param_range(pcurve_range2);

        coedge->set_geometry(pcurve1_);
        coedge2->set_geometry(pcurve2_);

        return {coedge, coedge2};
    }
};

/**
 * @brief Intersection between faces.
 *
 * @note
 * 1. intersect between surfaces.
 * 2. break intersection curves by pcurves.
 * 3. clean curves not in the face.
 */
struct FaceFaceIntersection {

    static std::vector<FFIResult> solve(const Face *face1, const Face *face2) {
        return intersect(face1, face2);
    }

  private:
    static std::vector<FFIResult> intersect(const Face *face1, const Face *face2) {
        std::vector<FFIResult> ffi_results;

        ParamSurface *surface1 = face1->geometry()->param_geometry();
        ParamSurface *surface2 = face2->geometry()->param_geometry();

        auto ssi_results = GeneralSurfaceSurfaceIntersection::solve(surface1, surface2);

        for (auto &ssi_result : ssi_results) {

            // (curve param, pcurve1 param, pcurve2 param)
            std::vector<std::tuple<double, double, double>> inter_info;

            // break with pcurves in surface1
            for (auto edge : TopologyUtils::get_all_edges(face1)) {
                Coedge* coedge = TopologyUtils::get_coedge_of_given_face(edge, face1);
                PCurve *pcurve = coedge->geometry();
                auto ppi_results = GeneralPCurvePCurveIntersection::solve(ssi_result.pcurve1, pcurve->param_geometry());

                for (auto &ppi_result : ppi_results) {
                    if (not coedge->param_range().contains(ppi_result.param2))
                        continue;

                    auto pos_3d = surface1->evaluate(ppi_result.inter_position);
                    auto curve_param = ssi_result.inter_curve->projection(pos_3d, ppi_result.param1).second;
                    auto pcurve1_param = ppi_result.param1;

                    BRepPoint2 surface2_param = surface2->project(pos_3d).second;

                    // guess with pcurve1 param
                    double pcurve2_param = ssi_result.pcurve2->projection(surface2_param, pcurve1_param).second;

                    inter_info.push_back(std::make_tuple(curve_param, pcurve1_param, pcurve2_param));
                }
            }

            // break with pcurves in surface2
            for (auto edge : TopologyUtils::get_all_edges(face2)) {
                Coedge* coedge = TopologyUtils::get_coedge_of_given_face(edge, face2);
                PCurve *pcurve = coedge->geometry();
                auto ppi_results = GeneralPCurvePCurveIntersection::solve(ssi_result.pcurve2, pcurve->param_geometry());

                for (auto &ppi_result : ppi_results) {
                    if (not coedge->param_range().contains(ppi_result.param2))
                        continue;

                    auto pos_3d = surface2->evaluate(ppi_result.inter_position);
                    auto curve_param = ssi_result.inter_curve->projection(pos_3d, ppi_result.param2).second;
                    auto pcurve2_param = ppi_result.param1;

                    BRepPoint2 surface1_param = surface1->project(pos_3d).second;

                    // guess with pcurve1 param
                    double pcurve1_param = ssi_result.pcurve1->projection(surface1_param, pcurve2_param).second;

                    inter_info.push_back(std::make_tuple(curve_param, pcurve1_param, pcurve2_param));
                }
            }

            // add end points
            inter_info.push_back({0.0, 0.0, 0.0});
            inter_info.push_back({1.0, 1.0, 1.0});

            std::ranges::sort(inter_info);

            for (int i = 1; i < inter_info.size(); i++) {
                auto [curve_start, pc1_start, pc2_start] = inter_info[i - 1];
                auto [curve_end, pc1_end, pc2_end] = inter_info[i];
                auto pcurve1_segment_mid_param = (pc1_start + pc1_end) / 2;
                auto pcurve2_segment_mid_param = (pc2_start + pc2_end) / 2;

                BRepPoint2 pc1_mid_pos = ssi_result.pcurve1->evaluate(pcurve1_segment_mid_param);
                BRepPoint2 pc2_mid_pos = ssi_result.pcurve2->evaluate(pcurve2_segment_mid_param);

                if (ContainmentQuery::contained(face1, pc1_mid_pos) == ContainmentQuery::ContainmentResult::Outside) {
                    continue;
                }

                if (ContainmentQuery::contained(face2, pc2_mid_pos) == ContainmentQuery::ContainmentResult::Outside) {
                    continue;
                }

                FFIResult ffi{.curve = ssi_result.inter_curve,
                                           .pcurve1 = ssi_result.pcurve1,
                                           .pcurve2 = ssi_result.pcurve2,
                                           .pcurve_range1 = ParamRange{pc1_start, pc1_end},
                                           .pcurve_range2 = ParamRange{pc2_start, pc2_end},
                                           .curve_range = ParamRange{curve_start, curve_end}};

                ffi_results.push_back(ffi);
            }
        }

        return ffi_results;
    }
};

} // namespace GraphicsLab::Geometry::BRep