#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/boundary_representation/topology/topology_modifiers.hpp"
#include "geometry/parametric/bspline_curve_3d.hpp"
#include "geometry/parametric/cone.hpp"
#include "geometry/parametric/explicit_surface.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/plane.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/torus.hpp"

namespace GraphicsLab::Geometry::BRep {

struct FaceConstructors {

    static Face *explicit_surface(const ExplicitSurface::ExpressionType &f) {
        auto allocator = BRepAllocator::instance();
        auto param_geo = allocator->alloc_param_surface<ExplicitSurface>(f);
        auto surface = allocator->alloc_surface();
        surface->set_param_geometry(param_geo);

        auto face = allocator->alloc_face();
        face->set_geometry(surface);

        create_basic_topology(param_geo, face);
        return face;
    }

    static Face *plane(glm::vec3 base_pos, glm::vec3 direction1, glm::vec3 direction2) {
        auto allocator = BRepAllocator::instance();
        auto param_geo = allocator->alloc_param_surface<Plane>(base_pos, direction1, direction2);
        auto surface = allocator->alloc_surface();
        surface->set_param_geometry(param_geo);

        auto face = allocator->alloc_face();
        face->set_geometry(surface);

        create_basic_topology(param_geo, face);
        return face;
    }

    static Face *torus(const BRepPoint3 &center, const double major_radius, const double minor_radius,
                       const BRepPoint3 &base_normal, const BRepPoint3 &direction1) {
        auto allocator = BRepAllocator::instance();
        auto param_geometry =
            allocator->alloc_param_surface<Torus>(center, major_radius, minor_radius, base_normal, direction1);
        auto surface = allocator->alloc_surface();
        surface->set_param_geometry(param_geometry);

        auto face = allocator->alloc_face();
        face->set_geometry(surface);

        return face;
    }

    static Face *sphere(const BRepPoint3 &center, const double radius) {
        auto allocator = BRepAllocator::instance();

        auto param_geometry = allocator->alloc_param_surface<Sphere>(center, radius);
        auto surface = allocator->alloc_surface();
        surface->set_param_geometry(param_geometry);

        auto face = allocator->alloc_face();
        face->set_geometry(surface);

        return face;
    }

  private:
    static void create_basic_topology(Plane *plane, Face *face) {
        auto allocator = BRepAllocator::instance();
        auto v1_pos = plane->evaluate({0.0, 0.0});
        auto v2_pos = plane->evaluate({1.0, 0.0});
        auto v3_pos = plane->evaluate({1.0, 1.0});
        auto v4_pos = plane->evaluate({0.0, 1.0});

        auto geo_c1 = allocator->alloc_param_curve<StraightLine3D>(v1_pos, v2_pos);
        auto geo_c2 = allocator->alloc_param_curve<StraightLine3D>(v2_pos, v3_pos);
        auto geo_c3 = allocator->alloc_param_curve<StraightLine3D>(v3_pos, v4_pos);
        auto geo_c4 = allocator->alloc_param_curve<StraightLine3D>(v4_pos, v1_pos);

        auto c1 = create_curve_from_param_curve(geo_c1);
        auto c2 = create_curve_from_param_curve(geo_c2);
        auto c3 = create_curve_from_param_curve(geo_c3);
        auto c4 = create_curve_from_param_curve(geo_c4);

        auto pc1 = create_straight_line_pcurve({0.0, 0.0}, {1.0, 0.0});
        auto pc2 = create_straight_line_pcurve({1.0, 0.0}, {1.0, 1.0});
        auto pc3 = create_straight_line_pcurve({1.0, 1.0}, {0.0, 1.0});
        auto pc4 = create_straight_line_pcurve({0.0, 1.0}, {0.0, 0.0});

        auto e1 = create_edge_from_curve(c1);
        auto e2 = create_edge_from_curve(c2);
        auto e3 = create_edge_from_curve(c3);
        auto e4 = create_edge_from_curve(c4);

        auto ce1 = create_coedge_from_edge(e1);
        auto ce2 = create_coedge_from_edge(e2);
        auto ce3 = create_coedge_from_edge(e3);
        auto ce4 = create_coedge_from_edge(e4);

        ce1->set_geometry(pc1);
        ce2->set_geometry(pc2);
        ce3->set_geometry(pc3);
        ce4->set_geometry(pc4);

        e1->set_coedge(ce1);
        e2->set_coedge(ce2);
        e3->set_coedge(ce3);
        e4->set_coedge(ce4);

        ce1->set_next(ce2);
        ce2->set_next(ce3);
        ce3->set_next(ce4);
        ce4->set_next(ce1);

        Loop *lp = create_loop_from_coedge(ce1);
        lp->set_face(face);

        ce1->set_loop(lp);
        ce2->set_loop(lp);
        ce3->set_loop(lp);
        ce4->set_loop(lp);

        face->set_loop(lp);
    }

    static void create_basic_topology(ParamSurface* surface, Face *face) {
        auto allocator = BRepAllocator::instance();

        bool is_u_periodic = surface->u_periodic_check();
        bool is_v_periodic = surface->v_periodic_check();

        BRepPoint2 v1_param{0.0, 0.0}, v2_param{1.0, 0.0}, v3_param{1.0, 1.0}, v4_param{0.0, 1.0};

        auto v1_pos = surface->evaluate({0.0, 0.0});
        auto v2_pos = surface->evaluate({1.0, 0.0});
        auto v3_pos = surface->evaluate({1.0, 1.0});
        auto v4_pos = surface->evaluate({0.0, 1.0});

        ParamCurve2D* param_pcurve_left = nullptr, *param_pcurve_right = nullptr, *param_pcurve_top = nullptr, *param_pcurve_bottom = nullptr;
        ParamCurve3D* param_curve_left = nullptr, *param_curve_right = nullptr, *param_curve_top = nullptr, *param_curve_bottom = nullptr;
        Edge *edge_left = nullptr, *edge_right = nullptr, *edge_top = nullptr, *edge_bottom = nullptr;
        Coedge *coedge_left = nullptr, *coedge_right = nullptr, *coedge_top = nullptr, *coedge_bottom = nullptr;
        PCurve *pcurve_left = nullptr, *pcurve_right = nullptr, *pcurve_top = nullptr, *pcurve_bottom = nullptr;
        Curve *curve_left = nullptr, *curve_right = nullptr, *curve_top = nullptr, *curve_bottom = nullptr;

        // if the surface is not u_periodic, add boundary curves (left and right)
        if (not is_u_periodic) {
            param_pcurve_left = allocator->alloc_param_pcurve<StraightLine2D>(v4_param, v1_param);
            param_pcurve_right = allocator->alloc_param_pcurve<StraightLine2D>(v2_param, v3_param);

            param_curve_left = create_curve_from_pcurve(surface, param_pcurve_left);
            param_curve_right = create_curve_from_pcurve(surface, param_pcurve_right);

            pcurve_left = create_pcurve_from_param_pcurve(param_pcurve_left);
            pcurve_right = create_pcurve_from_param_pcurve(param_pcurve_right);

            curve_left = create_curve_from_param_curve(param_curve_left);
            curve_right = create_curve_from_param_curve(param_curve_right);

            edge_left = create_edge_from_curve(curve_left);
            edge_right = create_edge_from_curve(curve_right);

            coedge_left = create_coedge_from_edge(edge_left);
            coedge_right = create_coedge_from_edge(edge_right);

            pcurve_left->set_param_geometry(param_pcurve_left);
            pcurve_right->set_param_geometry(param_pcurve_right);

            edge_left->set_coedge(coedge_left);
            edge_right->set_coedge(coedge_right);
        }

        // if the surface is not v_periodic, add boundary curves (top and bottom)
        if (not is_v_periodic) {
            param_pcurve_top = allocator->alloc_param_pcurve<StraightLine2D>(v3_param, v4_param);
            param_pcurve_bottom = allocator->alloc_param_pcurve<StraightLine2D>(v1_param, v2_param);

            param_curve_top = create_curve_from_pcurve(surface, param_pcurve_top);
            param_curve_bottom = create_curve_from_pcurve(surface, param_pcurve_bottom);

            pcurve_top = create_pcurve_from_param_pcurve(param_pcurve_top);
            pcurve_bottom = create_pcurve_from_param_pcurve(param_pcurve_bottom);

            curve_top = create_curve_from_param_curve(param_curve_top);
            curve_bottom = create_curve_from_param_curve(param_curve_bottom);

            edge_top = create_edge_from_curve(curve_top);
            edge_bottom = create_edge_from_curve(curve_bottom);

            coedge_top = create_coedge_from_edge(edge_top);
            coedge_bottom = create_coedge_from_edge(edge_bottom);

            pcurve_top->set_param_geometry(param_pcurve_top);
            pcurve_bottom->set_param_geometry(param_pcurve_bottom);

            edge_top->set_coedge(coedge_top);
            edge_bottom->set_coedge(coedge_bottom);
        }

        // Now build the topology

        if (not is_u_periodic and not is_v_periodic) {
            coedge_bottom->set_next(coedge_right);
            coedge_right->set_next(coedge_top);
            coedge_top->set_next(coedge_left);
            coedge_left->set_next(coedge_bottom);

            Loop *lp = create_loop_from_coedge(coedge_bottom);
            lp->set_face(face);
            face->set_loop(lp);
        } else if (is_u_periodic and not is_v_periodic) {
            Loop* loop_top = create_loop_from_coedge(coedge_top);
            Loop* loop_bottom = create_loop_from_coedge(coedge_bottom);

            loop_top->set_face(face);
            loop_bottom->set_face(face);

            loop_bottom->set_next(loop_top);
            face->set_loop(loop_bottom);
        } else if (not is_u_periodic and is_v_periodic) {
            Loop* loop_left = create_loop_from_coedge(coedge_left);
            Loop* loop_right = create_loop_from_coedge(coedge_right);

            loop_left->set_face(face);
            loop_right->set_face(face);

            loop_left->set_next(loop_left);
            face->set_loop(loop_left);
        } else {
            // nothing to do.
        }

    }

    static void create_basic_topology(Torus *torus, Face *face) {
        auto allocator = BRepAllocator::instance();
        // no boundary curves needed from complete torus
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

    static Edge *create_edge_from_curve(Curve *curve) {
        auto allocator = BRepAllocator::instance();
        auto edge = allocator->alloc_edge();
        edge->set_geometry(curve);
        edge->set_start(create_vertex_from_position(curve->param_geometry()->evaluate(curve->param_range().start())));
        edge->set_end(create_vertex_from_position(curve->param_geometry()->evaluate(curve->param_range().end())));

        edge->start()->set_edge(edge);
        edge->end()->set_edge(edge);

        return edge;
    }

    static Curve *create_curve_from_param_curve(ParamCurve3D *param_curve) {
        auto allocator = BRepAllocator::instance();
        auto curve = allocator->alloc_curve();
        curve->set_param_range(ParamRange{0.0, 1.0});
        curve->set_param_geometry(param_curve);
        return curve;
    }

    static PCurve* create_pcurve_from_param_pcurve(ParamCurve2D* param_pcurve) {
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

    /**
     * @brief Create curve from pcruve. Sample some points and fit by BSplineCurve3D
     * @param surface
     * @param pcurve
     * @param n
     * @return
     */
    static ParamCurve3D* create_curve_from_pcurve(const ParamSurface* surface, const ParamCurve2D* pcurve, int n = 50) {
        auto allocator = BRepAllocator::instance();

        std::vector<BRepPoint3> points;
        std::vector<BRepPoint2> param_points;

        for (int i = 0; i <= n; i++) {
            double param = 1.0 * i / (double) n;
            auto param_point = pcurve->evaluate(param);
            param_points.push_back(param_point);
            points.emplace_back(surface->evaluate(param_point));
        }

        auto&& curve_temp = BSplineCurve3D::fit(points, 3, 10);
        auto curve = allocator->alloc_param_curve<BSplineCurve3D>(std::move(curve_temp));
        return curve;
    }
};

struct BodyConstructors {

    static Body *cube(BRepPoint3 min_pos, BRepPoint3 max_pos) {
        auto allocator = BRepAllocator::instance();

        auto dx = (max_pos - min_pos) * BRepPoint3{1.0, 0.0, 0.0};
        auto dy = (max_pos - min_pos) * BRepPoint3{0.0, 1.0, 0.0};
        auto dz = (max_pos - min_pos) * BRepPoint3{0.0, 0.0, 1.0};

        auto v1 = min_pos;
        auto v2 = v1 + dx;
        auto v3 = v1 + dx + dy;
        auto v4 = v1 + dy;

        auto v5 = v1 + dz;
        auto v6 = v2 + dz;
        auto v7 = v3 + dz;
        auto v8 = v4 + dz;

        Face *front = FaceConstructors::plane(v2, dy, dz);
        Face *back = FaceConstructors::plane(v1, dz, dy);

        Face *left = FaceConstructors::plane(v1, dx, dz);
        Face *right = FaceConstructors::plane(v4, dz, dx);

        Face *top = FaceConstructors::plane(v5, dx, dy);
        Face *bottom = FaceConstructors::plane(v1, dy, dx);

        spdlog::debug("[Cube constructor]: Top {}, Bottom {}, Front {}, Back {}, Right {}, Left {} ", (void*)top, (void*)bottom, (void*)front, (void*)back, (void*)right, (void*)left);

        TopologyModifiers::stitch_faces(front, right);
        TopologyModifiers::stitch_faces(right, back);
        TopologyModifiers::stitch_faces(back, left);
        TopologyModifiers::stitch_faces(left, top);

        TopologyModifiers::stitch_faces(top, front);
        TopologyModifiers::stitch_faces(top, right);
        TopologyModifiers::stitch_faces(top, back);
        TopologyModifiers::stitch_faces(top, left);

        TopologyModifiers::stitch_faces(bottom, front);
        TopologyModifiers::stitch_faces(bottom, right);
        TopologyModifiers::stitch_faces(bottom, back);
        TopologyModifiers::stitch_faces(bottom, left);

        return create_body_from_list_of_faces({front, right, back, left, top, bottom});
    }

  private:
    static Body *create_body_from_list_of_faces(const std::vector<Face *> &faces) {
        auto allocator = BRepAllocator::instance();

        for (int i = 1; i < faces.size(); i++) {
            faces[i - 1]->set_next(faces[i]);
        }

        Shell *shell = allocator->alloc_shell();
        shell->set_face(faces.front());

        Body *body = allocator->alloc_body();
        shell->set_body(body);

        body->set_shell(shell);
        return body;
    }
};

} // namespace GraphicsLab::Geometry::BRep