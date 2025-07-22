#pragma once
#include "cpptrace/cpptrace.hpp"
#include "geometry/boundary_representation/brep_definition.hpp"
#include "geometry/mesh/mesh.hpp"
#include "geometry/parametric/tessellator.hpp"

namespace GraphicsLab::Geometry::BRep {

struct NaiveFaceter {
    static Mesh3D naive_facet(Face *face, int n = 10, int m = 10) {
        if (face->geometry() == nullptr) {
            throw cpptrace::logic_error("Face has no geometry");
        }

        if (face->geometry()->param_geometry() == nullptr) {
            throw cpptrace::logic_error("Surface has no parametric geometry");
        }

        Tessellator::tessellate(*face->geometry()->param_geometry());

        return *face->geometry()->param_geometry()->mesh.get();
    }

    static CurveMesh3D naive_edge_facet(Edge *edge, int n) {
        if (edge->geometry() == nullptr) {
            throw cpptrace::logic_error("Edge has no geometry");
        }

        if (edge->geometry()->param_geometry() == nullptr) {
            throw cpptrace::logic_error("Curve has no parametric geometry");
        }

        Tessellator::tessellate(*edge->geometry()->param_geometry());

        return *edge->geometry()->param_geometry()->mesh.get();
    }
};

} // namespace GraphicsLab::Geometry::BRep