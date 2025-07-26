#pragma once
#include "geometry/boundary_representation/intersector/curve_surface_intersection/general_curve_surface_intersection.hpp"
#include "geometry/boundary_representation/intersector/topological_intersection/face_face_intersection.hpp"
#include "geometry/boundary_representation/topology_definition.hpp"
#include "geometry/spatial_datastructure/rtree.hpp"
#include "utils/graph.hpp"
#include "utils/sampler.hpp"

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

    static std::vector<Face *> break_face_by_intersection(const Face *face, const Body *body) {
        std::vector<Face *> faces;

        struct NodeAttachment {
            BRepPoint2 par_pos;
        };
        struct EdgeAttachment {
            Coedge *coedge;
        };
        DirectedGraph<NodeAttachment, EdgeAttachment> face_intersection_graph(100);

        std::set<Coedge *> original_coedges;
        for (auto edge : TopologyUtils::get_all_edges(face)) {
            Coedge *coedge = TopologyUtils::get_coedge_of_given_face(edge, face);
            original_coedges.insert(coedge);
        }

        std::set<Coedge *> intersection_coedges;
        std::set<Coedge *> intersection_coedges_one_dir;
        /**
         * Enumerate all faces in body and intersect with the given face.
         */
        for (auto body_face : TopologyUtils::get_all_faces(body)) {
            std::vector<FFIResult> ffi_results = FaceFaceIntersection::solve(face, body_face);
            for (auto ffi_result : ffi_results) {
                auto [coedge1, coedge2] = ffi_result.create_face_coedges();
                Coedge *coedge1_reverse = TopologyUtils::create_reverse_coedge(coedge1);

                intersection_coedges.insert(coedge1);
                intersection_coedges.insert(coedge1_reverse);

                intersection_coedges_one_dir.insert(coedge1);
            }
        }

        // break original coedges with intersection curves.
        std::set<Coedge *> broken_coedges;
        for (auto original_coedge : original_coedges) {
            std::vector<double> inter_params;

            inter_params.push_back(original_coedge->param_range().start());
            inter_params.push_back(original_coedge->param_range().end());

            for (auto intersection_coedge : intersection_coedges_one_dir) {
                std::vector<PPIResult> ppi_results = GeneralPCurvePCurveIntersection::solve(
                    original_coedge->geometry()->param_geometry(), intersection_coedge->geometry()->param_geometry());

                for (const auto &ppi : ppi_results) {
                    inter_params.push_back(ppi.param1);
                }
            }

            remove_duplicates(inter_params, 1e-3);
            std::vector<Coedge *> coedges_broken =
                TopologyUtils::break_coedge_with_pcurve_params(original_coedge, face, inter_params);

            std::ranges::copy(coedges_broken, std::inserter(broken_coedges, broken_coedges.end()));
        }

        // build the intersection graph
        struct RTreeNode {
            BRepPoint2 pos;
            size_t index;
        };
        RTree<2, RTreeNode> rtree;

        std::vector<BRepPoint2> par_pos_of_vertices;

        auto add_to_rtree = [&](const BRepPoint2 &pos) -> size_t {
            RTreeNode find_node{};
            auto found = rtree.findPointInRange(pos, find_node, 0.01);
            if (not found) {
                rtree.insert(pos, {pos, par_pos_of_vertices.size()}, 0.01);
                par_pos_of_vertices.push_back(pos);

                return par_pos_of_vertices.size() - 1;
            } else {
                return find_node.index;
            }
        };

        // put all endpoints to Rtree and record
        std::map<Coedge *, std::pair<size_t, size_t>> coedge_vertices;
        for (Coedge *coedge : broken_coedges) {
            BRepPoint2 start_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().start());
            BRepPoint2 end_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().end());
            if (not coedge->is_forward())
                std::swap(start_pos, end_pos);
            auto start_idx = add_to_rtree(start_pos);
            auto end_idx = add_to_rtree(end_pos);
            coedge_vertices[coedge] = {start_idx, end_idx};
        }

        for (Coedge *coedge : intersection_coedges) {
            BRepPoint2 start_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().start());
            BRepPoint2 end_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().end());
            if (not coedge->is_forward())
                std::swap(start_pos, end_pos);
            auto start_idx = add_to_rtree(start_pos);
            auto end_idx = add_to_rtree(end_pos);
            coedge_vertices[coedge] = {start_idx, end_idx};
        }

        // create intersection graph vertices
        for (auto vert : par_pos_of_vertices) {
            face_intersection_graph.add_node({vert});
        }

        // add original coedges (with uni-direction)
        for (Coedge *coedge : broken_coedges) {
            auto [u, v] = coedge_vertices[coedge];
            face_intersection_graph.add_directed_edge(u, v, {coedge});
        }
        // add inter coedges (with bi-direction)
        for (Coedge *coedge : intersection_coedges) {
            auto [u, v] = coedge_vertices[coedge];
            face_intersection_graph.add_directed_edge(u, v, {coedge});
        }

        auto simple_circuits = face_intersection_graph.find_all_simple_circuits();

        decltype(simple_circuits) circuits;

        // (edge) -> index in simple_circuits
        std::map<std::pair<int, int>, size_t> index_map;
        std::set<size_t> visited_index;
        for (int i = 0; i < simple_circuits.size(); ++i) {
            const auto &circuit = simple_circuits[i];
            size_t n = circuit.size();
            for (const auto &edge : circuit) {
                std::pair<int, int> e{edge.from, edge.to};
                if (not index_map.contains(e) or simple_circuits[index_map[e]].size() < n) {
                    index_map[e] = i;
                }
            }
        }

        for (auto [e, i] : index_map) {
            visited_index.insert(i);
        }
        for (auto i : visited_index) {
            circuits.push_back(simple_circuits[i]);
        }

        using G = decltype(face_intersection_graph);
        auto planar_face_extraction = [](G &graph) -> std::vector<std::vector<G::Edge>> {
            int n = graph.nodes.size();
            for (int i = 0; i < n; i++) {
                std::ranges::sort(graph.G[i], [graph](G::Edge a, G::Edge b) -> bool {
                    auto dira = graph.nodes[a.to].data.par_pos - graph.nodes[a.from].data.par_pos;
                    auto dirb = graph.nodes[b.to].data.par_pos - graph.nodes[b.from].data.par_pos;
                    double cross = dira.x * dirb.y - dira.y * dirb.x;
                    return cross < 0;
                });
            }

            std::set<std::pair<int, int>> visited;

            std::vector<std::vector<G::Edge>> result;
            std::vector<G::Edge> cur;
            auto dfs = [&](auto &&dfs_func, int v, int prev, int start) -> void {
                for (int i = 0; i < graph.G[v].size(); ++i) {
                    decltype(visited)::value_type p = std::make_pair(graph.G[v][i].from, graph.G[v][i].to);

                    if (p.second == prev)
                        continue;
                    if (not visited.contains(p)) {
                        visited.insert(p);
                        cur.push_back(graph.G[v][i]);
                        if (graph.G[v][i].to != start)
                            dfs_func(dfs_func, graph.G[v][i].to, v, start);
                        break;
                    }
                }
            };

            for (int i = 0; i < n; i++) {
                cur.clear();
                dfs(dfs, i, -1, i);
                if (not cur.empty())
                    result.push_back(cur);
            }

            return result;
        };

        auto circuits2 = planar_face_extraction(face_intersection_graph);

        // build topology from circuits.
        std::vector<Loop *> loops;

        for (const auto &graph_circuits : circuits2) {
            Coedge *iter = graph_circuits.front().data.coedge;

            for (int i = 1; i < graph_circuits.size(); ++i) {
                iter->set_next(graph_circuits[i].data.coedge);
                iter = graph_circuits[i].data.coedge;
            }
            iter->set_next(graph_circuits.front().data.coedge);

            Loop *loop = TopologyUtils::create_loop_from_coedge(iter);

            // @todo: handle domains that not simply connected.
            loops.push_back(loop);
        }

        // @todo: non-simply connected case.

        std::set<Face *> faces_set;
        std::vector<Loop *> hole_loops;
        for (auto &loop : loops) {

            bool is_hole = ContainmentQuery::is_hole(loop);

            if (not is_hole) {
                Face *f = TopologyUtils::create_face_from_loop(loop);
                f->set_geometry(face->geometry());
                faces_set.insert(f);
            } else {
                hole_loops.push_back(loop);
            }
        }

        for (Loop *lp : hole_loops) {
            Coedge *first_coedge = lp->coedge();
            double sample_param = first_coedge->param_range().get_mid();
            BRepVector2 derivative = first_coedge->geometry()->param_geometry()->derivative(sample_param);

            if (not first_coedge->is_forward())
                derivative = -derivative;
            BRepVector2 in_normal = glm::normalize(BRepVector2{-derivative.y, derivative.x});
            BRepPoint2 sample = first_coedge->geometry()->param_geometry()->evaluate(sample_param) + in_normal * 1e-3;

            bool flag = false;
            for (Face *f : faces_set) {
                //@todo handle nested loops
                if (ContainmentQuery::contained(f, sample) == ContainmentQuery::ContainmentResult::Inside) {
                    // add loop
                    lp->set_next(f->loop()->next());
                    f->loop()->set_next(lp);
                    lp->set_face(f);
                    flag = true;
                    break;
                }
            }

            if (not flag)
                throw cpptrace::runtime_error("Hole loop can not find corresponding loop");
        }

        for (auto f : faces_set) {
            faces.push_back(f);
        }

        return faces;
    }

    /**
     * @brief Boolean operation between bodies.
     * @param body1
     * @param body2
     * @param op
     * @return
     */
    static Body *boolean_operation(const Body *body1, const Body *body2, Operation op) {
        std::vector<Face *> body1_faces, body2_faces;
        std::vector<bool> body1_faces_inside_flag, body2_faces_inside_flag;
        // stage1: break faces by intersection

        auto break_faces_in_body = [](const Body *b1, const Body *b2, std::vector<Face *> &face_vec) {
            for (auto f : TopologyUtils::get_all_faces(b1)) {
                auto faces = break_face_by_intersection(f, b2);
                std::ranges::copy(faces, std::back_inserter(face_vec));
            }
        };

        break_faces_in_body(body1, body2, body1_faces);
        break_faces_in_body(body2, body1, body2_faces);

        // stage2: inside/outside classification

        auto inside_outside_classification = [](std::vector<Face *> &faces, std::vector<bool> &faces_inside_flag,
                                                const Body *another_body) {
            // we use a simple ray casting and determine inside/outside by odd-even rule
            faces_inside_flag.clear();
            faces_inside_flag.resize(faces.size());
            for (int i = 0; i < faces.size(); ++i) {
                auto [sample_point, sample_par_pos] = get_point_in_face(faces[i]);
                BRepPoint3 direction = Sampler::sampleUniformVec3();
                StraightLine3D test_line{sample_point + direction * 1e-3, sample_point + direction * 100.0};

                int inter_num = 0;
                for (auto f : TopologyUtils::get_all_faces(another_body)) {
                    auto inter = GeneralCurveSurfaceIntersection::solve(&test_line, f->geometry()->param_geometry());

                    for (auto csi : inter) {
                        spdlog::info("csi info: curve param {}, surf param {} {}, pos {} {} {}", csi.curve_parameter,
                                     csi.surface_parameter.x, csi.surface_parameter.y, csi.inter_position.x,
                                     csi.inter_position.y, csi.inter_position.z);
                        if (ContainmentQuery::contained(f, csi.surface_parameter) ==
                            ContainmentQuery::ContainmentResult::Inside) {
                            inter_num++;
                        }
                    }
                }

                faces_inside_flag[i] = inter_num % 2 == 1;
            }
        };

        inside_outside_classification(body1_faces, body1_faces_inside_flag, body2);
        inside_outside_classification(body2_faces, body2_faces_inside_flag, body1);

        // stage3: rebuild topology

        auto rebuild_topology_preprocess = [](Face *face, bool inside_flag, bool is_blank, Operation op) -> bool {
            if (op == Operation::Union) {
                return not inside_flag;
            } else if (op == Operation::Intersection) {
                return inside_flag;
            } else {
                if (is_blank) {
                    return not inside_flag;
                } else {
                    face->set_forward(not face->is_forward());
                    return inside_flag;
                }
            }
        };

        std::vector<Face *> faces_set;
        for (int i = 0; i < body1_faces.size(); ++i) {
            if (auto reserve = rebuild_topology_preprocess(body1_faces[i], body1_faces_inside_flag[i], true, op)) {
                faces_set.push_back(body1_faces[i]);
            }
        }
        for (int i = 0; i < body2_faces.size(); ++i) {
            if (auto reserve = rebuild_topology_preprocess(body2_faces[i], body2_faces_inside_flag[i], false, op)) {
                faces_set.push_back(body2_faces[i]);
            }
        }

        for (int i = 1; i < faces_set.size(); ++i) {
            faces_set[i - 1]->set_next(faces_set[i]);
        }

        // @todo stitch adjoint faces.

        Shell *shell = TopologyUtils::create_shell_from_faces(faces_set);
        Body *result = TopologyUtils::create_body_from_shell(shell);

        return result;
    }

  private:
    /**
     * @brief Sample one point from the face.
     * @param face
     * @return
     */
    static std::pair<BRepPoint3, BRepPoint2> get_point_in_face(const Face *face) {
        Loop *lp = face->loop();
        Coedge *ce = lp->coedge();
        PCurve *pc = ce->geometry();
        ParamCurve2D *param_pc = pc->param_geometry();

        double param = ce->param_range().get_mid();
        BRepPoint2 par_pos = param_pc->evaluate(param);

        BRepVector2 t = param_pc->derivative(param);

        if (not pc->is_forward())
            t = -t;

        BRepVector2 n{-t.y, t.x};

        BRepPoint2 sample = par_pos + n * 1e-2;

        if (ContainmentQuery::contained(face, sample) != ContainmentQuery::ContainmentResult::Inside) {
            throw cpptrace::runtime_error("get point from face failed");
        }

        return {face->geometry()->param_geometry()->evaluate(sample), sample};
    }

    static void remove_duplicates(std::vector<double> &vec, double eps) {
        // Sort the vector first
        std::sort(vec.begin(), vec.end());

        // Use std::unique with a custom predicate
        auto last = std::unique(vec.begin(), vec.end(), [eps](double a, double b) { return std::fabs(a - b) < eps; });

        // Erase the duplicates
        vec.erase(last, vec.end());
    }

    struct IntersectInfo {
        Face *face1 = nullptr;
        Face *face2 = nullptr;

        std::vector<FFIResult> inter_results;
    };
};

} // namespace GraphicsLab::Geometry::BRep