#pragma once

#include <vector>

#include "geometry/parametric/parametric_curve.hpp"
#include "geometry/parametric/parametric_surface.hpp"

namespace GraphicsLab::Geometry::BoundaryRepresentation {
    struct ParamRange {
        double l, r;
    };

    struct Face;
    struct Edge;

    struct Vertex {
        glm::dvec3 position;
    };

    struct Solid {
        std::vector<Face> faces;
    };

    struct Coedge {
        Face* face;
        ParamCurve2D* pcurve;
    };

    struct Loop {
        std::vector<Edge> edges;
    };

    struct Face {
        ParamSurface* surface;
        std::vector<Loop> loops;
    };

    struct Edge {
        Coedge* coedge;
        ParamCurve3D* curve;
        Vertex* start_position;
        Vertex* end_position;
    };
}