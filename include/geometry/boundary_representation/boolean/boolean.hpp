#pragma once
#include "geometry/boundary_representation/intersector/topological_intersection/face_face_intersection.hpp"
#include "geometry/boundary_representation/topology_definition.hpp"
#include "geometry/spatial_datastructure/rtree.hpp"
#include "utils/graph.hpp"

#include <set>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Boolean operation between solids
 *
 * @note
 *
 * 1. intersect between solids, get the intersection graph of each face.
 * 2. split faces
 * 3. inside/outside classification
 * 4. rebuild topology
 *
 */
struct Boolean {
    enum class Operation {
        Union,
        Intersection,
        Difference
    };

    static std::vector<Face *> break_face_by_intersection(const Face* face, const Body* body) {
        std::vector<Face*> faces;

        struct NodeAttachment {
            BRepPoint2 par_pos;
        };
        struct EdgeAttachment {
            Coedge* coedge;
        };
        DirectedGraph<NodeAttachment, EdgeAttachment> face_intersection_graph(100);

        std::set<Coedge *> original_coedges;
        for (auto edge: TopologyUtils::get_all_edges(face)) {
            Coedge* coedge = TopologyUtils::get_coedge_of_given_face(edge, face);
            original_coedges.insert(coedge);
        }

        std::set<Coedge*> intersection_coedges;

        /**
         * Enumerate all faces in body and intersect with the given face.
         */
        for (auto body_face: TopologyUtils::get_all_faces(body)) {
            std::vector<FFIResult> ffi_results = FaceFaceIntersection::solve(face, body_face);
            for (auto ffi_result: ffi_results) {
                auto [coedge1, coedge2] = ffi_result.create_face_coedges();
                Coedge* coedge1_reverse = TopologyUtils::create_reverse_coedge(coedge1);

                intersection_coedges.insert(coedge1);
                intersection_coedges.insert(coedge1_reverse);
            }
        }

        // break original coedges with intersection curves.
        std::set<Coedge*> broken_coedges;
        for (auto original_coedge: original_coedges) {
            std::vector<double> inter_params;

            inter_params.push_back(original_coedge->param_range().start());
            inter_params.push_back(original_coedge->param_range().end());

            for (auto intersection_coedge: intersection_coedges) {
                std::vector<PPIResult> ppi_results = GeneralPCurvePCurveIntersection::solve(original_coedge->geometry()->param_geometry(), intersection_coedge->geometry()->param_geometry());

                for (const auto& ppi: ppi_results) {
                    inter_params.push_back(ppi.param1);
                }
            }

            remove_duplicates(inter_params, 1e-6);

            std::vector<Coedge *> coedges_broken =
                TopologyUtils::break_coedge_with_pcurve_params(original_coedge, face, inter_params);

            std::ranges::copy(coedges_broken, std::inserter(broken_coedges, broken_coedges.end()));
        }


        // build the intersection graph
        struct RTreeNode {
            BRepPoint3 pos;
            size_t index;
        };
        RTree<3, RTreeNode> rtree;

        std::vector<BRepPoint3> par_pos_of_vertices;

        auto add_to_rtree = [&](BRepPoint3 pos) -> size_t {
            RTreeNode find_node;
            auto found = rtree.findPointInRange(pos, find_node, 1e-6);
            if (not found) {
                rtree.insert(pos, {pos, par_pos_of_vertices.size()});
                par_pos_of_vertices.push_back(pos);
                return par_pos_of_vertices.size() - 1;
            } else {
                return find_node.index;
            }
        };

        // put all endpoints to Rtree and record
        std::map<Coedge*, std::pair<size_t, size_t>> coedge_vertices;
        for (Coedge* coedge: broken_coedges) {
            BRepPoint3 start_pos = coedge->edge()->geometry()->param_geometry()->evaluate(coedge->edge()->param_range().start());
            BRepPoint3 end_pos = coedge->edge()->geometry()->param_geometry()->evaluate(coedge->edge()->param_range().end());
            if (not coedge->is_forward()) std::swap(start_pos, end_pos);
            auto start_idx = add_to_rtree(start_pos);
            auto end_idx = add_to_rtree(end_pos);
            coedge_vertices[coedge] = {start_idx, end_idx};
        }

        for (Coedge* coedge: intersection_coedges) {
            BRepPoint3 start_pos = coedge->edge()->geometry()->param_geometry()->evaluate(coedge->edge()->param_range().start());
            BRepPoint3 end_pos = coedge->edge()->geometry()->param_geometry()->evaluate(coedge->edge()->param_range().end());
            if (not coedge->is_forward()) std::swap(start_pos, end_pos);
            auto start_idx = add_to_rtree(start_pos);
            auto end_idx = add_to_rtree(end_pos);
            coedge_vertices[coedge] = {start_idx, end_idx};
        }

        // create intersection graph vertices
        for (auto vert: par_pos_of_vertices) {
            face_intersection_graph.add_node({vert});
        }

        // add original coedges (with uni-direction)
        for (Coedge* coedge: broken_coedges) {
            auto [u, v] = coedge_vertices[coedge];
            face_intersection_graph.add_directed_edge(u, v, {coedge});
        }
        // add inter coedges (with bi-direction)
        for (Coedge* coedge: intersection_coedges) {
            auto [u, v] = coedge_vertices[coedge];
            face_intersection_graph.add_bidirectional_edge(u, v, {coedge});
        }

        auto circuits = face_intersection_graph.find_all_simple_circuits();

        // build topology from circuits.
        std::vector<Loop*> loops;

        for (const auto& graph_circuits: circuits) {
            Coedge* iter = graph_circuits.front().data.coedge;

            for (int i = 1; i < graph_circuits.size(); ++i) {
                iter->set_next(graph_circuits[i].data.coedge);
                iter = graph_circuits[i].data.coedge;
            }
            iter->set_next(graph_circuits.front().data.coedge);

            Loop* loop = TopologyUtils::create_loop_from_coedge(iter);

            // @todo: handle domains that not simply connected.
            loops.push_back(loop);
        }

        for (int i = 0; i < loops.size(); ++i) {
            Face* f = TopologyUtils::create_face_from_loop(loops[i]);
            f->set_geometry(face->geometry());
            faces.push_back(f);
        }

        return faces;
    }

private:

    static void remove_duplicates(std::vector<double>& vec, double eps) {
        // Sort the vector first
        std::sort(vec.begin(), vec.end());

        // Use std::unique with a custom predicate
        auto last = std::unique(vec.begin(), vec.end(), [eps](double a, double b) {
            return std::fabs(a - b) < eps;
        });

        // Erase the duplicates
        vec.erase(last, vec.end());
    }

    struct IntersectInfo {
        Face* face1 = nullptr;
        Face* face2 = nullptr;

        std::vector<FFIResult> inter_results;
    };
};

} // namespace GraphicsLab::Geometry::BRep