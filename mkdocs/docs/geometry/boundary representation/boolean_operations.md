**Boolean Operations**

This page summarizes the boolean implementation in `include/geometry/boundary_representation/boolean/boolean.hpp`.

**Overview:**

- **Purpose:** Compute boolean operations (Union, Intersection, Difference) between `Body` objects represented as B-Rep.
- **Main struct:** `Boolean` with `enum class Operation { Union, Intersection, Difference }`.

**Public API:**

- `static std::vector<Face *> break_face_by_intersection(const Face *face, const Body *body)` —
  breaks a single `face` by intersecting it against all faces of `body` and returns the resulting faces after splitting and topology rebuild.
- `static Body *boolean_operation(const Body *body1, const Body *body2, Operation op)` —
  high-level boolean between two bodies. Stages: slice/break faces, inside/outside classification, rebuild topology and assemble result `Body`.

**Key helper functions (internal):**

- `get_point_in_face(const Face *face)` — sample a point inside a face for classification.
- `remove_duplicates(std::vector<double>&, double eps)` — numeric dedup for parameter lists.

**Algorithm (high level):**

1. Intersection & slicing
    - For each face pair, compute face-face intersections via `FaceFaceIntersection::solve` and construct intersection `Coedge`s.
    - Break intersection curves (PCurves) where they cross using pcurve–pcurve intersections; handle periodic parameter domains (u/v offsets).
    - Break original face coedges by intersection parameters and build an intersection graph in parametric space.
    - Extract planar circuits from the intersection graph, build loops and create new `Face` objects for resulting face patches.
    - Handle non-simply-connected surfaces by pairing non-contractible loops when possible.

2. Inside/Outside classification
    - For each resulting face patch, sample an interior point and cast a ray (random unit-sphere direction) to count intersections with the other body’s faces.
    - Use odd-even rule to decide whether the patch is inside or outside the other body.

3. Rebuild topology and assemble result
    - Decide which face patches to keep or flip orientation using a small `rebuild_topology_preprocess` rule depending on `Operation`.
    - Chain selected faces, create a `Shell` and then a `Body` for the final result. (Stitching of adjacent faces is noted as TODO.)

**Usage / Tests:**

- Tests call `Boolean::boolean_operation(blank, tool, Boolean::Operation::Union|Difference|Intersection)` (see `include/geometry/boundary_representation/test/boolean_tests.hpp`).
- `break_face_by_intersection` is exercised in break-face tests.

**Notes & Limitations:**

- Classification uses simple ray casting; robustness depends on sampling and intersection primitives.
- Non-simply connected parameter domains and loop pairing are partly handled but throw if pairing fails.
- Topology stitching between adjacent faces is marked TODO — result may need additional stitching/cleanup.
- Performance notes: many stages (pairwise face intersections, PCurve intersections, graph extraction) can be expensive for complex bodies.

**References:**

- Source: `include/geometry/boundary_representation/boolean/boolean.hpp`
- Tests: `include/geometry/boundary_representation/test/boolean_tests.hpp`
