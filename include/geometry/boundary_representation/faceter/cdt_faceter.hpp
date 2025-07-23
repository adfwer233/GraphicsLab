#pragma once
#include "geometry/boundary_representation/topology_definition.hpp"
#include "geometry/mesh/mesh.hpp"

namespace GraphicsLab::Geometry::BRep {

struct CDTFaceter {
    static Mesh3D naive_facet(Face *face, int n = 10, int m = 10);
};

}