#pragma once

#include "geometry/boundary_representation/allocator/brep_allocator.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/parametric/parametric_curves/straight_line.hpp"
#include "geometry/parametric/plane.hpp"

namespace GraphicsLab::Geometry::BRep {

struct FaceConstructors {

    static Face* plane(glm::vec3 base_pos, glm::vec3 direction1, glm::vec3 direction2) {
        auto allocator = BRepAllocator::instance();
        auto param_geo = allocator->alloc_param_surface<Plane>(base_pos, direction1, direction2);
        auto surface = allocator->alloc_surface();
        surface->set_param_geometry(param_geo);

        auto face = allocator->alloc_face();
        face->set_geometry(surface);

        create_basic_topology(param_geo, face);
        return face;
    }

private:
    static void create_basic_topology(Plane* plane, Face* face) {
        auto allocator = BRepAllocator::instance();
        auto v1_pos = plane->evaluate({0.0, 0.0});
        auto v2_pos = plane->evaluate({1.0, 0.0});
        auto v3_pos = plane->evaluate({1.0, 1.0});
        auto v4_pos = plane->evaluate({0.0, 0.0});

        auto geo_c1 = allocator->alloc_param_curve<StraightLine3D>(v1_pos, v2_pos);
        auto geo_c2 = allocator->alloc_param_curve<StraightLine3D>(v2_pos, v3_pos);
        auto geo_c3 = allocator->alloc_param_curve<StraightLine3D>(v3_pos, v4_pos);
        auto geo_c4 = allocator->alloc_param_curve<StraightLine3D>(v4_pos, v1_pos);

        auto c1 = create_curve_from_param_curve(geo_c1);
        auto c2 = create_curve_from_param_curve(geo_c2);
        auto c3 = create_curve_from_param_curve(geo_c3);
        auto c4 = create_curve_from_param_curve(geo_c4);

        auto pc1 = create_straight_line_pcurve({0.0, 0.0}, {1.0 ,0.0});
        auto pc2 = create_straight_line_pcurve({1.0, 0.0}, {1.0 ,1.0});
        auto pc3 = create_straight_line_pcurve({1.0, 1.0}, {0.0 ,1.0});
        auto pc4 = create_straight_line_pcurve({0.0, 1.0}, {0.0 ,0.0});

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

        Loop* lp = create_loop_from_coedge(ce1);
        lp->set_face(face);

        face->set_loop(lp);
    }

    static Loop* create_loop_from_coedge(Coedge* coedge) {
        auto allocator = BRepAllocator::instance();
        Loop* loop = allocator->alloc_loop();
        loop->set_coedge(coedge);
        return loop;
    }

    static Coedge* create_coedge_from_edge(Edge* edge) {
        auto allocator = BRepAllocator::instance();
        Coedge* coedge = allocator->alloc_coedge();
        coedge->set_edge(edge);
        return coedge;
    }

    static Edge* create_edge_from_curve(Curve* curve) {
        auto allocator = BRepAllocator::instance();
        auto edge = allocator->alloc_edge();
        edge->set_geometry(curve);
        edge->set_start(create_vertex_from_position(curve->param_geometry()->evaluate(curve->param_range().start())));
        edge->set_end(create_vertex_from_position(curve->param_geometry()->evaluate(curve->param_range().end())));

        edge->start()->set_edge(edge);
        edge->end()->set_edge(edge);

        return edge;
    }

    static Curve* create_curve_from_param_curve(ParamCurve3D* param_curve) {
        auto allocator = BRepAllocator::instance();
        auto curve = allocator->alloc_curve();
        curve->set_param_range(ParamRange{0.0, 1.0});
        curve->set_param_geometry(param_curve);
        return curve;
    }

    static PCurve* create_straight_line_pcurve(const glm::dvec2& start, const glm::dvec2& end) {
        auto allocator = BRepAllocator::instance();
        auto param_pcurve = allocator->alloc_param_pcurve<StraightLine2D>(start, end);
        auto pcurve = allocator->alloc_pcurve();
        pcurve->set_param_geometry(param_pcurve);

        return pcurve;
    }

    static Vertex* create_vertex_from_position(const BRepPoint3& position) {
        auto allocator = BRepAllocator::instance();
        Point* point = allocator->alloc_point();
        point->set_position(position);

        Vertex* vertex = allocator->alloc_vertex();
        vertex->set_geometry(point);

        return vertex;
    }
};

struct BodyConstructors {

    static Body *cube(glm::vec3 min_pos, glm::vec3 max_pos) {
        auto allocator = BRepAllocator::instance();

        // Plane* plane1 = allocator->alloc_surface<Plane>()

        return nullptr;
    }
};

}