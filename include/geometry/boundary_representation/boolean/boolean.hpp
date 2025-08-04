#pragma once
#include "geometry/boundary_representation/intersector/curve_surface_intersection/general_curve_surface_intersection.hpp"
#include "geometry/boundary_representation/intersector/topological_intersection/face_face_intersection.hpp"
#include "geometry/boundary_representation/topology_definition.hpp"
#include "geometry/parametric/sphere.hpp"
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

        std::vector<Coedge *> intersection_coedges;
        std::vector<Coedge *> intersection_coedges_one_dir;
        /**
         * Enumerate all faces in body and intersect with the given face.
         */
        for (auto body_face : TopologyUtils::get_all_faces(body)) {
            std::vector<FFIResult> ffi_results = FaceFaceIntersection::solve(face, body_face);
            for (auto ffi_result : ffi_results) {
                auto [coedge1, coedge2] = ffi_result.create_face_coedges();
                Coedge *coedge1_reverse = TopologyUtils::create_reverse_coedge(coedge1);
                coedge1->set_partner(coedge1_reverse);
                coedge1_reverse->set_partner(coedge1);
                intersection_coedges.push_back(coedge1);
                intersection_coedges.push_back(coedge1_reverse);

                intersection_coedges_one_dir.push_back(coedge1);
            }
        }

        // break intersection curves

        std::vector<Coedge *> intersection_coedges_broken_one_dir;
        for (int i = 0; i < intersection_coedges_one_dir.size(); i++) {
            std::vector<double> inter_params;
            inter_params.push_back(intersection_coedges_one_dir[i]->param_range().start());
            inter_params.push_back(intersection_coedges_one_dir[i]->param_range().end());
            for (int j = 0; j < intersection_coedges_one_dir.size(); j++) {
                if (i == j)
                    continue;

                std::vector<PPIResult> ppi_results = GeneralPCurvePCurveIntersection::solve(
                    intersection_coedges_one_dir[i]->geometry()->param_geometry(),
                    intersection_coedges_one_dir[j]->geometry()->param_geometry());

                if (face->geometry()->param_geometry()->u_periodic) {
                    std::vector<PPIResult> u_offset_1 = GeneralPCurvePCurveIntersection::solve(
                        intersection_coedges_one_dir[i]->geometry()->param_geometry(),
                        intersection_coedges_one_dir[j]->geometry()->param_geometry(), {1.0, 0.0});

                    std::vector<PPIResult> u_offset_2 = GeneralPCurvePCurveIntersection::solve(
                        intersection_coedges_one_dir[i]->geometry()->param_geometry(),
                        intersection_coedges_one_dir[j]->geometry()->param_geometry(), {-1.0, 0.0});

                    for (const auto &ppi : u_offset_1) {
                        inter_params.push_back(ppi.param1);
                    }

                    for (const auto &ppi : u_offset_2) {
                        inter_params.push_back(ppi.param1);
                    }
                }

                if (face->geometry()->param_geometry()->v_periodic) {
                    std::vector<PPIResult> v_offset_1 = GeneralPCurvePCurveIntersection::solve(
                        intersection_coedges_one_dir[i]->geometry()->param_geometry(),
                        intersection_coedges_one_dir[j]->geometry()->param_geometry(), {0.0, 1.0});

                    std::vector<PPIResult> v_offset_2 = GeneralPCurvePCurveIntersection::solve(
                        intersection_coedges_one_dir[i]->geometry()->param_geometry(),
                        intersection_coedges_one_dir[j]->geometry()->param_geometry(), {0.0, -1.0});

                    for (const auto &ppi : v_offset_1) {
                        inter_params.push_back(ppi.param1);
                    }

                    for (const auto &ppi : v_offset_2) {
                        inter_params.push_back(ppi.param1);
                    }
                }

                for (const auto &ppi : ppi_results) {
                    inter_params.push_back(ppi.param1);
                }
            }

            remove_duplicates(inter_params, 1e-3);
            std::vector<Coedge *> coedges_broken =
                TopologyUtils::break_coedge_with_pcurve_params(intersection_coedges_one_dir[i], face, inter_params);

            std::ranges::copy(coedges_broken, std::back_inserter(intersection_coedges_broken_one_dir));
        }

        if (intersection_coedges_broken_one_dir.size() > intersection_coedges_one_dir.size()) {
            intersection_coedges_one_dir = intersection_coedges_broken_one_dir;
            intersection_coedges.clear();

            for (Coedge *coedge : intersection_coedges_one_dir) {
                Coedge *coedge1_reverse = TopologyUtils::create_reverse_coedge(coedge);
                coedge->set_partner(coedge1_reverse);
                coedge1_reverse->set_partner(coedge);
                intersection_coedges.push_back(coedge);
                intersection_coedges.push_back(coedge1_reverse);
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
        // struct RTreeNode {
        //     BRepPoint2 pos;
        //     size_t index;
        // };
        // RTree<2, RTreeNode> rtree;

        std::vector<BRepPoint2> par_pos_of_vertices;
        std::vector<BRepPoint2> par_pos_offset;

        auto is_par_pos_match = [face](const BRepPoint2 &pos1, const BRepPoint2 &pos2) -> bool {
            constexpr double inter_graph_tol = 0.05;

            bool found = false;
            if (glm::distance(pos1, pos2) < inter_graph_tol) {
                return true;
            }

            if (face->geometry()->param_geometry()->u_periodic and
                not face->geometry()->param_geometry()->is_singular(pos1)) {
                BRepPoint2 dx{1.0, 0.0};
                if (glm::distance(pos1 + dx, pos2) < inter_graph_tol or
                    glm::distance(pos1 - dx, pos2) < inter_graph_tol) {
                    return true;
                }
            }

            if (face->geometry()->param_geometry()->v_periodic and
                not face->geometry()->param_geometry()->is_singular(pos1)) {
                BRepPoint2 dy{0.0, 1.0};
                if (glm::distance(pos1 + dy, pos2) < inter_graph_tol or
                    glm::distance(pos1 - dy, pos2) < inter_graph_tol) {
                    return true;
                }
            }

            return false;
        };

        auto add_to_rtree = [&](const BRepPoint2 &pos) -> size_t {
            // RTreeNode find_node{};
            // auto found = rtree.findPointInRange(pos, find_node, 0.05);
            // if (not found) {
            //     rtree.insert(pos, {pos, par_pos_of_vertices.size()}, 0.05);
            //     par_pos_of_vertices.push_back(pos);
            //
            //     return par_pos_of_vertices.size() - 1;
            // } else {
            //     return find_node.index;
            // }
            for (size_t i = 0; i < par_pos_of_vertices.size(); i++) {
                BRepPoint2 p = par_pos_of_vertices[i];
                if (is_par_pos_match(pos, p)) {
                    return i;
                }
            }

            par_pos_of_vertices.push_back(pos);
            return par_pos_of_vertices.size() - 1;
        };

        // put all endpoints to Rtree and record
        std::map<Coedge *, std::pair<size_t, size_t>> coedge_vertices;
        for (Coedge *coedge : broken_coedges) {
            BRepPoint2 start_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().start());
            BRepPoint2 end_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().end());
            if (not coedge->geometry()->is_forward())
                std::swap(start_pos, end_pos);
            auto start_idx = add_to_rtree(start_pos);
            auto end_idx = add_to_rtree(end_pos);
            coedge_vertices[coedge] = {start_idx, end_idx};
        }

        for (Coedge *coedge : intersection_coedges) {
            BRepPoint2 start_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().start());
            BRepPoint2 end_pos = coedge->geometry()->param_geometry()->evaluate(coedge->param_range().end());
            if (not coedge->geometry()->is_forward())
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
        auto planar_face_extraction = [face](G &graph) -> std::vector<std::vector<G::Edge>> {
            int n = graph.nodes.size();

            auto sort_comp = [face, graph](BRepVector3 last_tangent, G::Edge a, G::Edge b) -> bool {
                Coedge *ce_a = a.data.coedge;
                Edge *e_a = ce_a->edge();
                double start_param_a = ce_a->is_forward() ? e_a->param_range().start() : e_a->param_range().end();
                BRepVector3 dir1 = e_a->geometry()->param_geometry()->derivative(start_param_a);
                if (not ce_a->is_forward())
                    dir1 *= -1;

                Coedge *ce_b = b.data.coedge;
                Edge *e_b = ce_b->edge();
                double start_param_b = ce_b->is_forward() ? e_b->param_range().start() : e_b->param_range().end();
                BRepVector3 dir2 = e_b->geometry()->param_geometry()->derivative(start_param_b);
                if (not ce_b->is_forward())
                    dir2 *= -1;

                auto par_pos = graph.nodes[a.from].data.par_pos;
                BRepVector3 face_normal = face->geometry()->param_geometry()->normal(par_pos);
                if (not face->is_forward())
                    face_normal *= -1;

                // the frame
                BRepVector3 x = glm::normalize(last_tangent);
                BRepVector3 z = glm::normalize(face_normal);
                BRepVector3 y = glm::cross(z, x);

                double d1_x = glm::dot(dir1, x);
                double d1_y = glm::dot(dir1, y);
                double d2_x = glm::dot(dir2, x);
                double d2_y = glm::dot(dir2, y);

                return std::atan2(d1_y, d1_x) > std::atan2(d2_y, d2_x);
            };

            std::vector<G::Edge> edges;
            for (int i = 0; i < n; i++) {
                for (auto e : graph.G[i]) {
                    edges.push_back(e);
                }
            }

            std::set<Coedge *> visited;
            std::vector<std::vector<G::Edge>> result;
            std::vector<G::Edge> cur;
            auto dfs = [&](auto &&dfs_func, int v, int start, Edge *prev_edge, BRepVector3 last_end_tangent) -> void {
                if (v != start) {
                    auto comp = std::bind(sort_comp, last_end_tangent, std::placeholders::_1, std::placeholders::_2);
                    std::ranges::sort(graph.G[v], comp);
                }

                for (auto e : graph.G[v]) {
                    if (prev_edge == e.data.coedge->edge())
                        continue;
                    if (not visited.contains(e.data.coedge)) {
                        visited.insert(e.data.coedge);
                        cur.push_back(e);
                        if (e.to == start)
                            break;

                        // compute tangent at the end;
                        Coedge *ce = e.data.coedge;
                        Edge *edge = ce->edge();
                        double end_param = ce->is_forward() ? edge->param_range().end() : edge->param_range().start();
                        BRepVector3 end_tangent = edge->geometry()->param_geometry()->derivative(end_param);
                        if (not ce->is_forward())
                            end_tangent *= -1;

                        dfs_func(dfs_func, e.to, start, edge, end_tangent);

                        break;
                    }
                }
            };

            // for (int i = 0; i < n; i++) {
            //     cur.clear();
            //     dfs(dfs, i, i, -1, BRepVector3(0));
            //     if (not cur.empty())
            //         result.push_back(cur);
            // }

            for (auto e : edges) {
                cur.clear();
                dfs(dfs, e.from, e.from, nullptr, BRepVector3(0));

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

            for (int i = 0; i < graph_circuits.size(); ++i) {
                graph_circuits[i].data.coedge->set_loop(loop);
            }

            // @todo: handle domains that not simply connected.
            loops.push_back(loop);
        }

        // @todo: non-simply connected case.
        bool is_simply_connected =
            not face->geometry()->param_geometry()->u_periodic and not face->geometry()->param_geometry()->v_periodic;

        std::set<Face *> faces_set;
        std::vector<Loop *> hole_loops;
        std::map<std::pair<int, int>, std::vector<Loop *>> non_contractible;

        for (auto &loop : loops) {
            if (not is_simply_connected) {
                auto [p, q] = TopologyUtils::get_loop_homology(loop);
                if (p != 0 or q != 0) {
                    non_contractible[std::make_pair(p, q)].push_back(loop);
                    continue;
                }
            }

            bool is_hole = ContainmentQuery::is_hole(loop);

            if (not is_hole) {
                Face *f = TopologyUtils::create_face_from_loop(loop);
                f->set_geometry(face->geometry());
                faces_set.insert(f);
            } else {
                hole_loops.push_back(loop);
            }
        }

        if (dynamic_cast<Sphere *>(face->geometry()->param_geometry())) {
            int x = 0;
        }

        if (not is_simply_connected) {
            if (non_contractible.size() % 2 != 0) {
                throw cpptrace::runtime_error("can not pair non contractible loops.");
            }

            auto [coeff1, loop_vec1] = *non_contractible.begin();
            auto [coeff2, loop_vec2] = *std::prev(non_contractible.end());

            if (loop_vec1.size() != loop_vec2.size()) {
                throw cpptrace::runtime_error("loop mismatch.");
            }

            for (int i = 0; i < loop_vec1.size(); i++) {
                Face *new_face = BRepAllocator::instance()->alloc_face();
                new_face->set_geometry(face->geometry());
                loop_vec1[i]->set_next(loop_vec2[i]);
                loop_vec2[i]->set_next(loop_vec1[i]);
                loop_vec1[i]->set_face(new_face);
                loop_vec2[i]->set_face(new_face);
                new_face->set_loop(loop_vec1[i]);
                faces_set.insert(new_face);
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
                    if (f->loop() != nullptr) {
                        lp->set_next(f->loop()->next());
                        f->loop()->set_next(lp);
                    } else {
                        f->set_loop(lp);
                    }
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
        spdlog::info("boolean slice1 done");

        break_faces_in_body(body2, body1, body2_faces);
        spdlog::info("boolean slice2 done");

        // stage2: inside/outside classification

        auto inside_outside_classification = [](std::vector<Face *> &faces, std::vector<bool> &faces_inside_flag,
                                                const Body *another_body) {
            // we use a simple ray casting and determine inside/outside by odd-even rule
            faces_inside_flag.clear();
            faces_inside_flag.resize(faces.size());
            for (int i = 0; i < faces.size(); ++i) {
                auto [sample_point, sample_par_pos] = get_point_in_face(faces[i]);
                BRepPoint3 direction = Sampler::sampleUniformVec3();
                StraightLine3D test_line{sample_point + direction * 1e-3, sample_point + direction * 5.0};

                int inter_num = 0;
                int j = 0;
                for (auto f : TopologyUtils::get_all_faces(another_body)) {
                    // spdlog::info("{} {}", i, j);
                    j += 1;
                    auto inter = GeneralCurveSurfaceIntersection::solve(&test_line, f->geometry()->param_geometry());

                    for (auto csi : inter) {
                        spdlog::info("csi info: curve param {}, surf param {} {}, pos {} {} {}", csi.curve_parameter,
                                     csi.surface_parameter.x, csi.surface_parameter.y, csi.inter_position.x,
                                     csi.inter_position.y, csi.inter_position.z);
                        if (ContainmentQuery::contained(f, csi.surface_parameter) ==
                                ContainmentQuery::ContainmentResult::Inside and
                            csi.curve_parameter > 0) {
                            inter_num++;
                        }
                    }
                }

                faces_inside_flag[i] = inter_num % 2 == 1;
            }
        };

        inside_outside_classification(body1_faces, body1_faces_inside_flag, body2);
        inside_outside_classification(body2_faces, body2_faces_inside_flag, body1);

        spdlog::info("boolean classification done");

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

        if (lp == nullptr) {
            return {face->geometry()->param_geometry()->evaluate({0.5, 0.5}), {0.5, 0.5}};
        }

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
            // throw cpptrace::runtime_error("get point from face failed");
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