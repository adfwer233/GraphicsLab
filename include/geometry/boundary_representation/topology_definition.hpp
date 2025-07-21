#pragma once

#include "geometry_defintition.hpp"

namespace GraphicsLab::Geometry::BRep {

struct Vertex;
struct Edge;
struct Coedge;
struct Loop;
struct Face;
struct Shell;

struct Vertex {
    Point* geometry() const {return geometry_; }

    void set_geometry(Point* geometry) { geometry_ = geometry; }

    Edge* edge() const { return edge_; }
    void set_edge(Edge* edge) { edge_ = edge; }
private:
    Point* geometry_ = nullptr;
    Edge* edge_ = nullptr;
};

struct Edge {
    Curve* geometry() { return geometry_; }

    bool is_forward() const { return forward; }
    void set_forward(bool is_forward) { forward = is_forward; }

    void set_geometry(Curve* geometry) { geometry_ = geometry; }
    void set_coedge(Coedge* coedge) {coedge_ = coedge;}
private:
    bool forward = true;
    Curve* geometry_ = nullptr;
    Coedge* coedge_ = nullptr;
};

struct Coedge {
    Edge* edge() const { return edge_; }
    void set_edge(Edge* edge) { edge_ = edge; }

    PCurve* geometry() const { return geometry_; }
    void set_geometry(PCurve* geometry) { geometry_ = geometry; }

    bool is_forward() const { return forward; }
    void set_forward(bool is_forward) { forward = is_forward; }

    Coedge* next() const { return next_; }
    void set_next(Coedge* next) { next_ = next; }

    Loop* loop() const { return loop_; }
    void set_loop(Loop* loop) { loop_ = loop; }
private:
    bool forward = true;
    Edge* edge_ = nullptr;
    PCurve* geometry_ = nullptr;
    Coedge* next_ = nullptr;
    Loop* loop_ = nullptr;
};

struct Loop {
    Coedge* coedge() const { return coedge_; }
    void set_coedge(Coedge* coedge) { coedge_ = coedge; }

    Face* face() const { return face_; }
    void set_face(Face* face) { face_ = face; }

    Loop* next() const { return next_; }
    void set_next(Loop* next) { next_ = next; }

private:
    Coedge* coedge_ = nullptr;
    Face* face_ = nullptr;
    Loop* next_ = nullptr;
};

struct Face {
    Loop* loop() const { return loop_; }
    void set_loop(Loop* loop) { loop_ = loop; }

    Shell* shell() const { return shell_; }
    void set_shell(Shell* shell) { shell_ = shell; }

    Surface* geometry() const { return geometry_; }
    void set_geometry(Surface* geometry) { geometry_ = geometry; }

    Face* next() const { return next_; }
    void set_next(Face* next) { next_ = next; }

    bool is_forward() const { return forward; }
    void set_forward(bool is_forward) { forward = is_forward; }
private:
    bool forward = true;

    Loop* loop_ = nullptr;
    Shell* shell_ = nullptr;
    Face* next_ = nullptr;
    Surface* geometry_ = nullptr;
};

struct Shell {};

struct Body{};

}