#pragma once

#include "cpptrace/cpptrace.hpp"
#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/parametric/bspline_curve_3d.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "spdlog/spdlog.h"

namespace GraphicsLab::Geometry::BRep {

struct TopologyUtils {

    static Coedge *get_coedge_of_given_face(const Edge *edge, const Face *face) {
        Coedge *start_coedge = edge->coedge();
        Coedge *coedge_iter = start_coedge;

        while (coedge_iter != nullptr) {
            if (coedge_iter->loop() == nullptr) {
                throw cpptrace::runtime_error("coedge loop is null");
            }
            if (coedge_iter->loop()->face() == face) {
                return coedge_iter;
            }
            coedge_iter = coedge_iter->partner();

            if (coedge_iter == start_coedge) {
                break;
            }
        }

        throw cpptrace::logic_error("The edge belongs no coedge referring to the face.");
    }

    static std::vector<Shell *> get_all_shells(const Body *body) {
        Shell *shell_start = body->shell();
        Shell *shell_iter = shell_start;
        std::vector<Shell *> shells;

        while (shell_iter != nullptr) {
            shells.push_back(shell_iter);
            shell_iter = shell_iter->next();
        }

        return shells;
    }

    static std::vector<Face *> get_all_faces(const Shell *shell) {
        Face *face_start = shell->face();
        Face *face_iter = face_start;
        std::vector<Face *> faces;

        while (face_iter != nullptr) {
            faces.push_back(face_iter);
            face_iter = face_iter->next();
        }

        return faces;
    }

    static std::vector<Face *> get_all_faces(const Body *body) {
        std::vector<Face *> faces;
        for (auto shell : get_all_shells(body)) {
            std::vector<Face *> shell_faces = get_all_faces(shell);
            std::ranges::copy(shell_faces, std::back_inserter(faces));
        }
        return faces;
    }

    static std::vector<Coedge *> get_all_coedges(const Loop *loop) {
        Coedge *coedge_start = loop->coedge();
        Coedge *coedge_iter = coedge_start;
        std::vector<Coedge *> coedges;
        while (coedge_iter != nullptr) {
            coedges.push_back(coedge_iter);
            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start) {
                break;
            }
        }
        return coedges;
    }

    static std::vector<Loop *> get_all_loops(const Face *face) {
        Loop *loop_start = face->loop();
        Loop *loop_iter = face->loop();
        std::vector<Loop *> loops;
        while (loop_iter != nullptr) {
            loops.push_back(loop_iter);
            loop_iter = loop_iter->next();
            if (loop_iter == loop_start) {
                break;
            }
        }
        return loops;
    }

    static std::vector<Edge *> get_all_edges(const Loop *loop) {
        std::vector<Edge *> edges;
        Coedge *coedge_start = loop->coedge();
        Coedge *coedge_iter = coedge_start;

        while (coedge_iter != nullptr) {
            edges.push_back(coedge_iter->edge());

            coedge_iter = coedge_iter->next();
            if (coedge_iter == coedge_start or coedge_iter == nullptr)
                break;
        }

        return edges;
    }

    static std::vector<Edge *> get_all_edges(const Face *face) {
        std::vector<Edge *> edges;
        for (auto loop : get_all_loops(face)) {
            auto loop_edges = get_all_edges(loop);
            std::ranges::copy(loop_edges, std::back_inserter(edges));
        }
        return edges;
    }

    static Loop *create_loop_from_coedge(Coedge *coedge) {
        auto allocator = BRepAllocator::instance();
        Loop *loop = allocator->alloc_loop();
        loop->set_coedge(coedge);
        return loop;
    }

    static Coedge *create_coedge_from_edge(Edge *edge) {
        auto allocator = BRepAllocator::instance();
        Coedge *coedge = allocator->alloc_coedge();
        coedge->set_edge(edge);
        return coedge;
    }

    static Edge *create_edge_from_curve(Curve *curve, double param_start = 0.0, double param_end = 1.0) {
        auto allocator = BRepAllocator::instance();
        auto edge = allocator->alloc_edge();
        edge->set_geometry(curve);
        edge->set_start(create_vertex_from_position(curve->param_geometry()->evaluate(param_start)));
        edge->set_end(create_vertex_from_position(curve->param_geometry()->evaluate(param_end)));
        edge->set_param_range(ParamRange(param_start, param_end));
        edge->start()->set_edge(edge);
        edge->end()->set_edge(edge);

        return edge;
    }

    static Curve *create_curve_from_param_curve(ParamCurve3D *param_curve) {
        auto allocator = BRepAllocator::instance();
        auto curve = allocator->alloc_curve();
        curve->set_param_geometry(param_curve);
        return curve;
    }

    static PCurve *create_pcurve_from_param_pcurve(ParamCurve2D *param_pcurve) {
        auto allocator = BRepAllocator::instance();
        auto pcurve = allocator->alloc_pcurve();
        pcurve->set_param_geometry(param_pcurve);
        return pcurve;
    }

    static PCurve *create_straight_line_pcurve(const glm::dvec2 &start, const glm::dvec2 &end) {
        auto allocator = BRepAllocator::instance();
        auto param_pcurve = allocator->alloc_param_pcurve<StraightLine2D>(start, end);
        auto pcurve = allocator->alloc_pcurve();
        pcurve->set_param_geometry(param_pcurve);

        return pcurve;
    }

    static Vertex *create_vertex_from_position(const BRepPoint3 &position) {
        auto allocator = BRepAllocator::instance();
        Point *point = allocator->alloc_point();
        point->set_position(position);

        Vertex *vertex = allocator->alloc_vertex();
        vertex->set_geometry(point);

        return vertex;
    }

    static Coedge *create_reverse_coedge(const Coedge *coedge) {
        if (coedge->edge() == nullptr) {
            throw cpptrace::logic_error("edge of given coedge is null");
        }
        Coedge *coedge_reverse = create_coedge_from_edge(coedge->edge());
        coedge_reverse->set_forward(not coedge->is_forward());
        coedge_reverse->set_edge(coedge->edge());
        coedge_reverse->set_geometry(coedge->geometry());
        coedge_reverse->set_param_range(coedge->param_range());

        PCurve *pcurve_reverse = create_pcurve_from_param_pcurve(coedge->geometry()->param_geometry());
        pcurve_reverse->set_forward(not coedge->geometry()->is_forward());
        coedge_reverse->set_geometry(pcurve_reverse);

        return coedge_reverse;
    }

    static std::vector<Coedge *> break_coedge_with_pcurve_params(const Coedge *coedge, const Face *face,
                                                                 const std::vector<double> &pcurve_params) {
        if (pcurve_params.empty() or pcurve_params.size() <= 1) {
            throw cpptrace::logic_error("At least two params should be provided");
        }
        std::vector<Coedge *> coedges;

        Curve *curve = coedge->edge()->geometry();
        PCurve *pcurve = coedge->geometry();
        Surface *surface = face->geometry();

        std::vector<double> curve_params;
        for (double pcurve_param : pcurve_params) {
            auto pos = surface->param_geometry()->evaluate(pcurve->param_geometry()->evaluate(pcurve_param));
            curve_params.push_back(curve->param_geometry()->projection(pos, pcurve_param).second);
        }

        for (size_t i = 1; i < pcurve_params.size(); ++i) {
            Edge *edge = create_edge_from_curve(curve, curve_params[i - 1], curve_params[i]);
            Coedge *coedge_new = create_coedge_from_edge(edge);
            coedge_new->set_param_range(ParamRange(pcurve_params[i - 1], pcurve_params[i]));
            coedge_new->set_forward(coedge->is_forward());
            edge->set_coedge(coedge_new);
            coedge_new->set_geometry(pcurve);

            coedges.push_back(coedge_new);
        }

        if (not coedge->is_forward()) {
            std::ranges::reverse(coedges);
        }

        for (size_t i = 1; i < coedges.size(); ++i) {
            coedges[i - 1]->set_next(coedges[i]);
        }

        return coedges;
    }

    static Face *create_face_from_loop(Loop *loop) {
        auto allocator = BRepAllocator::instance();
        auto face = allocator->alloc_face();
        face->set_loop(loop);
        loop->set_face(face);
        return face;
    }

    static Shell *create_shell_from_faces(const std::vector<Face *> &faces) {
        auto allocator = BRepAllocator::instance();
        auto shell = allocator->alloc_shell();
        shell->set_face(faces.front());
        faces.front()->set_shell(shell);
        for (size_t i = 1; i < faces.size(); ++i) {
            faces[i - 1]->set_next(faces[i]);
            faces[i]->set_shell(shell);
        }
        return shell;
    }

    static Body *create_body_from_shell(Shell *shell) {
        auto allocator = BRepAllocator::instance();
        auto body = allocator->alloc_body();
        body->set_shell(shell);
        shell->set_body(body);
        return body;
    }

    /**
     * @brief Create a curve from then pcurve. Sample some points and fit by BSplineCurve3D
     * @param surface
     * @param pcurve
     * @param n
     * @return
     */
    static ParamCurve3D *create_param_curve_from_pcurve(const ParamSurface *surface, const ParamCurve2D *pcurve,
                                                        int n = 50) {
        auto allocator = BRepAllocator::instance();

        std::vector<BRepPoint3> points;
        std::vector<BRepPoint2> param_points;

        for (int i = 0; i <= n; i++) {
            double param = 1.0 * i / (double)n;
            auto param_point = pcurve->evaluate(param);
            param_points.push_back(param_point);
            points.emplace_back(surface->evaluate(param_point));
        }

        auto &&curve_temp = BSplineCurve3D::fit(points, 3, 10);
        auto curve = allocator->alloc_param_curve<BSplineCurve3D>(std::move(curve_temp));
        return curve;
    }

    static std::pair<BRepPoint2, BRepPoint2> get_end_points_of_pcurve(const Coedge *coedge) {
        double start_param = coedge->param_range().start();
        double end_param = coedge->param_range().end();
        if (not coedge->geometry()->is_forward()) {
            std::swap(start_param, end_param);
        }
        ParamCurve2D *param_pcurve = coedge->geometry()->param_geometry();
        return {param_pcurve->evaluate(start_param), param_pcurve->evaluate(end_param)};
    }

    static auto get_loop_homology(Loop *loop) -> std::pair<int, int> {
        Coedge *start_coedge = loop->coedge();
        Coedge *coedge_iter = start_coedge;
        std::optional<std::pair<BRepPoint2, BRepPoint2>> end_points_prev;

        BRepVector2 offset(0);

        while (true) {
            auto [s, t] = get_end_points_of_pcurve(coedge_iter);

            if (end_points_prev.has_value()) {
                offset += s - end_points_prev.value().second;
            }
            end_points_prev = {s, t};

            if (coedge_iter->next() == nullptr)
                break;
            // throw cpptrace::runtime_error("coedges in Loop refer to null next.");
            if (coedge_iter->next() == start_coedge) {
                break;
            }

            coedge_iter = coedge_iter->next();
        }

        Coedge *end_coedge = coedge_iter;
        PCurve *start_pc = start_coedge->geometry(), *end_pc = end_coedge->geometry();

        BRepPoint2 start_pos = start_pc->is_forward()
                                   ? start_pc->param_geometry()->evaluate(start_coedge->param_range().start())
                                   : start_pc->param_geometry()->evaluate(start_coedge->param_range().end());

        BRepPoint2 end_pos = end_pc->is_forward()
                                 ? end_pc->param_geometry()->evaluate(end_coedge->param_range().end())
                                 : end_pc->param_geometry()->evaluate(end_coedge->param_range().start());

        return {std::round(end_pos.x - start_pos.x - offset.x), std::round(end_pos.y - start_pos.y - offset.y)};
    }
};

} // namespace GraphicsLab::Geometry::BRep