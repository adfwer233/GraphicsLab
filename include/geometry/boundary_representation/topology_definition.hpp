#pragma once

#include "base/vec_def.hpp"
#include "geometry_defintition.hpp"

namespace GraphicsLab::Geometry::BRep {

struct Vertex;
struct Edge;
struct Coedge;
struct Loop;
struct Face;
struct Shell;
struct Body;

struct Vertex {
    [[nodiscard]] Point *geometry() const {
        return geometry_;
    }

    void set_geometry(Point *geometry) {
        geometry_ = geometry;
    }

    [[nodiscard]] Edge *edge() const {
        return edge_;
    }
    void set_edge(Edge *edge) {
        edge_ = edge;
    }

  private:
    Point *geometry_ = nullptr;
    Edge *edge_ = nullptr;
};

struct Edge {
    Curve *geometry() {
        return geometry_;
    }

    [[nodiscard]] bool is_forward() const {
        return forward;
    }
    void set_forward(bool is_forward) {
        forward = is_forward;
    }

    void set_geometry(Curve *geometry) {
        geometry_ = geometry;
    }
    void set_coedge(Coedge *coedge) {
        coedge_ = coedge;
    }

    Coedge *coedge() const {
        return coedge_;
    }

    [[nodiscard]] Vertex *start() const {
        return start_;
    }
    [[nodiscard]] Vertex *end() const {
        return end_;
    }

    void set_start(Vertex *start) {
        start_ = start;
    }
    void set_end(Vertex *end) {
        end_ = end;
    }

    [[nodiscard]] BRepPoint3 start_position() const {
        return start_->geometry()->position();
    }
    [[nodiscard]] BRepPoint3 end_position() const {
        return end_->geometry()->position();
    }

  private:
    bool forward = true;
    Curve *geometry_ = nullptr;
    Coedge *coedge_ = nullptr;
    Vertex *start_ = nullptr;
    Vertex *end_ = nullptr;
};

struct Coedge {
    [[nodiscard]] Edge *edge() const {
        return edge_;
    }
    void set_edge(Edge *edge) {
        edge_ = edge;
    }

    [[nodiscard]] PCurve *geometry() const {
        return geometry_;
    }
    void set_geometry(PCurve *geometry) {
        geometry_ = geometry;
    }

    [[nodiscard]] bool is_forward() const {
        return forward;
    }
    void set_forward(bool is_forward) {
        forward = is_forward;
    }

    [[nodiscard]] Coedge *next() const {
        return next_;
    }
    void set_next(Coedge *next) {
        next_ = next;
    }

    [[nodiscard]] Loop *loop() const {
        return loop_;
    }
    void set_loop(Loop *loop) {
        loop_ = loop;
    }

    [[nodiscard]] Coedge *partner() const {
        return partner_;
    }
    void set_partner(Coedge *partner) {
        partner_ = partner;
    }

  private:
    bool forward = true;
    Edge *edge_ = nullptr;
    PCurve *geometry_ = nullptr;
    Coedge *next_ = nullptr;
    Coedge *partner_ = nullptr;
    Loop *loop_ = nullptr;
};

struct Loop {
    [[nodiscard]] Coedge *coedge() const {
        return coedge_;
    }
    void set_coedge(Coedge *coedge) {
        coedge_ = coedge;
    }

    [[nodiscard]] Face *face() const {
        return face_;
    }
    void set_face(Face *face) {
        face_ = face;
    }

    [[nodiscard]] Loop *next() const {
        return next_;
    }
    void set_next(Loop *next) {
        next_ = next;
    }

  private:
    Coedge *coedge_ = nullptr;
    Face *face_ = nullptr;
    Loop *next_ = nullptr;
};

struct Face {
    [[nodiscard]] Loop *loop() const {
        return loop_;
    }
    void set_loop(Loop *loop) {
        loop_ = loop;
    }

    [[nodiscard]] Shell *shell() const;
    void set_shell(Shell *shell) {
        shell_ = shell;
    }

    [[nodiscard]] Surface *geometry() const {
        return geometry_;
    }
    void set_geometry(Surface *geometry) {
        geometry_ = geometry;
    }

    [[nodiscard]] Face *next() const {
        return next_;
    }
    void set_next(Face *next) {
        next_ = next;
    }

    [[nodiscard]] bool is_forward() const {
        return forward;
    }
    void set_forward(bool is_forward) {
        forward = is_forward;
    }

  private:
    bool forward = true;

    Loop *loop_ = nullptr;
    Shell *shell_ = nullptr;
    Face *next_ = nullptr;
    Surface *geometry_ = nullptr;
};

struct Shell {
    [[nodiscard]] Face *face() const {
        return face_;
    }
    void set_face(Face *face) {
        face_ = face;
    }

    [[nodiscard]] Body *body() const {
        return body_;
    }
    void set_body(Body *body) {
        body_ = body;
    }

    [[nodiscard]] Shell *next() const {
        return next_;
    }
    void set_next(Shell *next) {
        next_ = next;
    }

  private:
    Body *body_ = nullptr;
    Face *face_ = nullptr;
    ;
    Shell *next_ = nullptr;
};

struct Body {
    [[nodiscard]] Shell *shell() const {
        return shell_;
    }
    void set_shell(Shell *shell) {
        shell_ = shell;
    }

  private:
    Shell *shell_ = nullptr;
};

} // namespace GraphicsLab::Geometry::BRep