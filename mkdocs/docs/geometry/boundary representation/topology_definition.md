# Topology Definition

This page defines the core B-Rep topology entities used in GraphicsLab.

## Topology Entities

- `Face`: bounded parametric surface region
- `Loop`: ordered boundary around a face
- `Edge`: topological edge shared by one or more faces
- `Coedge`: directed use of an edge in a loop

## Data Model Goals

- Explicit adjacency traversal (face-loop-edge relationships)
- Stable ownership semantics for construction and editing
- Clear separation between topology and geometry

## Related Headers

- `include/geometry/parametric_topology/brep_face.hpp`
- `include/geometry/parametric_topology/brep_loop.hpp`
- `include/geometry/parametric_topology/brep_edge.hpp`
- `include/geometry/parametric_topology/brep_coedge.hpp`
