#pragma once

#include "utils/graph.hpp"
#include "geometry/parametric_topology/brep_face.hpp"

namespace GraphicsLab::Geometry {

struct BoundaryCutting {

    BRepFace* face;

    struct EdgeAttachment {
        ParamCurve2D* curve;
        double start_pos, end_pos;
    };

    struct VertexAttachment {};

    DirectedGraph<VertexAttachment, EdgeAttachment> graph;

    void cut_boundary() {

    }
};

}