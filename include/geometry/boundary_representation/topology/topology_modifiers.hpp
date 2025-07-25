#pragma once

#include "cpptrace/cpptrace.hpp"

#include "geometry/boundary_representation/base/tor_def.hpp"
#include "geometry/boundary_representation/topology_definition.hpp"
#include "topology_utils.hpp"

namespace GraphicsLab::Geometry::BRep {

struct TopologyModifiers {

    /**
     * @brief stitch two surfaces along two edges. Edges are assumed same.
     * @param face1
     * @param face2
     * @param edge1
     * @param edge2
     */
    static void stitch_faces_along_edge(Face *face1, Face *face2, Edge *edge1, Edge *edge2) {
        Coedge *ce1 = TopologyUtils::get_coedge_of_given_face(edge1, face1);
        Coedge *ce2 = TopologyUtils::get_coedge_of_given_face(edge2, face2);

        ce1->set_partner(ce2);
        ce2->set_partner(ce1);

        ce2->set_edge(ce1->edge());
        ce2->set_forward(not ce1->is_forward());
    }

    static void stitch_faces(Face *face1, Face *face2) {
        auto face1_edges = TopologyUtils::get_all_edges(face1);
        auto face2_edges = TopologyUtils::get_all_edges(face2);

        std::vector<std::pair<Edge *, Edge *>> edge_pairs;
        for (auto e1 : face1_edges) {
            for (auto e2 : face2_edges) {
                //@todo enhance "same edge" check

                // sample midpoint from e1

                auto c1 = e1->geometry();
                auto c1_mid_param = (e1->param_range().start() + e1->param_range().end()) / 2;
                auto c1_mid_pos = c1->param_geometry()->evaluate(c1_mid_param);

                auto c2 = e2->geometry();
                auto c2_mid_param = (e2->param_range().start() + e2->param_range().end()) / 2;
                auto c2_mid_pos = c2->param_geometry()->evaluate(c2_mid_param);

                if (glm::distance(c1_mid_pos, c2_mid_pos) < Tolerance::default_tolerance) {
                    spdlog::debug("[Stitch Faces]: Stitch {} and {} with edge {}, mid position {} {} {}", (void *)face1,
                                  (void *)face2, (void *)e1, c1_mid_pos.x, c1_mid_pos.y, c2_mid_pos.z);
                    edge_pairs.emplace_back(e1, e2);
                }
            }
        }

        for (auto [e1, e2] : edge_pairs) {
            stitch_faces_along_edge(face1, face2, e1, e2);
        }

        if (edge_pairs.empty()) {
            throw cpptrace::runtime_error("No matching edge between faces");
        }
    }
};

} // namespace GraphicsLab::Geometry::BRep