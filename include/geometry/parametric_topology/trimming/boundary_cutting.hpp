#pragma once

#include "geometry/parametric_topology/brep_face.hpp"
#include "geometry/spatial_datastructure/kd_tree.hpp"
#include "geometry/spatial_datastructure/rtree.hpp"
#include "utils/graph.hpp"

namespace GraphicsLab::Geometry {

struct BoundaryCutting {

    BRepFace *face;

    explicit BoundaryCutting(BRepFace *_face) : face(_face), graph(1000) {
    }

    struct EdgeAttachment {
        ParamCurve2D *curve;
        double start_param, end_param;

        REFLECT(Property{"start_param", &EdgeAttachment::start_param},
                Property{"end_param", &EdgeAttachment::end_param});
    };

    struct GraphNodeAttachment {
        glm::dvec2 position;

        REFLECT(Property{"position", &GraphNodeAttachment::position});
    };
    struct VertexAttachment {
        size_t node_id;
    };

    DirectedGraph<GraphNodeAttachment, EdgeAttachment> graph;

    std::vector<std::vector<decltype(graph)::Edge>> simple_loops;

    StraightLine2D left_line{{0.0, 1.0}, {0.0, 0.0}};
    StraightLine2D right_line{{1.0, 0.0}, {1.0, 1.0}};
    StraightLine2D top_line{{1.0, 1.0}, {0.0, 1.0}};
    StraightLine2D bottom_line{{0.0, 0.0}, {1.0, 0.0}};

    void cut_boundary() {
        std::vector<double> left_params, right_params, top_params, bottom_params;
        std::vector<size_t> left_nodes, right_nodes, top_nodes, bottom_nodes;
        RTree<2, VertexAttachment> rtree;

        auto get_node_ids = [&](glm::dvec2 pos) {
            VertexAttachment attachment;
            auto std_pos = pos;
            if (not rtree.findPointInRange(std_pos, attachment)) {
                auto node_id = graph.add_node(GraphNodeAttachment{.position = std_pos});
                rtree.insert(std_pos, VertexAttachment{.node_id = node_id});
                return node_id;
            }
            return attachment.node_id;
        };

        auto update_graph_with_info = [&](BSplineCurve2D *pcurve, CurveCurveIntersectionResult2D &inter, bool reverse) {
            std::vector<std::pair<double, glm::dvec2>> split_points;
            split_points.emplace_back(0.0, pcurve->evaluate(0.0));
            for (size_t j = 0; j < inter.inter_points.size(); j++)
                split_points.emplace_back(inter.curve2_param[j], inter.inter_points[j]);
            split_points.emplace_back(1.0, pcurve->evaluate(1.0));

            for (size_t j = 1; j < split_points.size(); j++) {
                EdgeAttachment attachment;
                auto [start_param, start_pos] = split_points[j - 1];
                auto [end_param, end_pos] = split_points[j];

                auto mid_param = (start_param + end_param) / 2;
                auto mid_pos = pcurve->evaluate(mid_param);

                auto std_mid_pos = move_to_original_param(mid_pos);
                auto offset = std_mid_pos - mid_pos;

                auto start_node_id = get_node_ids(start_pos + offset);
                auto end_node_id = get_node_ids(end_pos + offset);

                attachment.start_param = start_param;
                attachment.end_param = end_param;
                attachment.curve = pcurve;
                auto start = start_pos + offset;
                auto end = end_pos + offset;

                if (not reverse) {
                    graph.add_directed_edge(start_node_id, end_node_id, attachment);
                } else {
                    graph.add_directed_edge(end_node_id, start_node_id, attachment);
                }
            }
        };

        for (auto loop : face->boundary) {
            for (auto coedge : loop->coedges) {
                auto box = coedge->geometry->get_box();

                int x_floor = std::floor(box.min.x);
                int y_floor = std::floor(box.min.y);
                int x_ceil = std::ceil(box.max.x);
                int y_ceil = std::ceil(box.max.y);

                if (auto pcurve = dynamic_cast<BSplineCurve2D *>(coedge->geometry)) {
                    pcurve->insert_all_knots_to_bezier_form();

                    CurveCurveIntersectionResult2D inter_all;
                    for (int i = x_floor; i <= x_ceil; i++) {
                        double x_param = i;
                        StraightLine2D line = {{x_param, static_cast<double>(y_floor)},
                                               {x_param, static_cast<double>(y_ceil)}};

                        auto inter = LineBSplineParamIntersector2D::intersect(line, *pcurve);

                        for (int j = 0; j < inter.inter_points.size(); j++) {
                            inter_all.curve1_param.push_back(inter.curve1_param[j]);
                            inter_all.curve2_param.push_back(inter.curve2_param[j]);
                            inter_all.inter_points.push_back(inter.inter_points[j]);
                        }
                    }

                    for (int i = y_floor; i <= y_ceil; i++) {
                        StraightLine2D line = {{x_floor, i}, {x_ceil, i}};

                        auto inter = LineBSplineParamIntersector2D::intersect(line, *pcurve);

                        for (int j = 0; j < inter.inter_points.size(); j++) {
                            inter_all.curve1_param.push_back(inter.curve1_param[j]);
                            inter_all.curve2_param.push_back(inter.curve2_param[j]);
                            inter_all.inter_points.push_back(inter.inter_points[j]);
                        }
                    }

                    update_graph_with_info(pcurve, inter_all, coedge->orientation == BRepCoedge::Orientation::BACKWARD);
                }
            }
        }

        for (int i = 0; i < graph.nodes.size(); i++) {
            auto pos = graph.nodes[i].data.position;
            if (pos.x < 1e-4) {
                left_params.push_back(pos.y);
            }
            if (pos.y < 1e-4)
                bottom_params.push_back(pos.x);
            if (pos.x > 1 - 1e-4)
                right_params.push_back(pos.y);
            if (pos.y > 1 - 1e-4)
                top_params.push_back(pos.x);
        }

        std::ranges::sort(left_params);
        std::ranges::sort(right_params);
        std::ranges::sort(top_params);
        std::ranges::sort(bottom_params);

        auto handle_edges = [](std::vector<double> &vec) {
            if (not vec.empty()) {
                if (vec.front() > 1e-4)
                    vec.insert(vec.begin(), 0.0);
                if (vec.front() < 1 - 1e-4)
                    vec.push_back(1.0);
            } else {
                vec.push_back(0.0);
                vec.push_back(1.0);
            }
        };

        handle_edges(left_params);
        handle_edges(right_params);
        handle_edges(top_params);
        handle_edges(bottom_params);

        std::ranges::reverse(left_params);
        std::ranges::reverse(top_params);

        for (int i = 0; i < left_params.size(); i++) {
            glm::dvec2 pos{0.0, left_params[i]};
            left_nodes.push_back(get_node_ids(pos));
        }

        for (int i = 0; i < right_params.size(); i++) {
            glm::dvec2 pos{1.0, right_params[i]};
            right_nodes.push_back(get_node_ids(pos));
        }

        for (int i = 0; i < top_params.size(); i++) {
            glm::dvec2 pos{top_params[i], 1.0};
            top_nodes.push_back(get_node_ids(pos));
        }

        for (int i = 0; i < bottom_params.size(); i++) {
            glm::dvec2 pos{bottom_params[i], 0.0};
            bottom_nodes.push_back(get_node_ids(pos));
        }

        auto insert_boundary_to_graph = [&](const std::vector<double> &param, const std::vector<size_t> &node_ids,
                                            ParamCurve2D *pcurve) {
            for (int i = 1; i < param.size(); i++) {
                auto last_node = node_ids[i - 1];
                auto node = node_ids[i];
                graph.add_directed_edge(
                    last_node, node,
                    EdgeAttachment{.curve = pcurve, .start_param = param[i - 1], .end_param = param[i]});
            }
        };

        insert_boundary_to_graph(left_params, left_nodes, &left_line);
        insert_boundary_to_graph(right_params, right_nodes, &right_line);
        insert_boundary_to_graph(top_params, top_nodes, &top_line);
        insert_boundary_to_graph(bottom_params, bottom_nodes, &bottom_line);

        graph.print_graph_info();
        simple_loops = graph.find_all_simple_circuits();
    }

  private:
    glm::dvec2 move_to_original_param(glm::dvec2 param) {
        return {param.x - std::floor(param.x), param.y - std::floor(param.y)};
    }
};

} // namespace GraphicsLab::Geometry