# Surface-Surface Intersection (SSI)

GraphicsLab supports SSI workflows across both analytic and spline-like surfaces.

## Typical Surface Pairs

- Plane vs. analytic surfaces (sphere, cone, torus)
- Analytic vs. analytic
- Spline/parametric variants through generic pathways

## Problem Statement

Given two surfaces

$$
S_1(u, v), \quad S_2(s, t),
$$

find parameter tuples such that

$$
S_1(u, v) = S_2(s, t).
$$

## Practical Strategy

1. Build candidate seeds (analytic hints or coarse sampling).
2. Refine with iterative nonlinear solving.
3. Trace connected solution branches.
4. Validate and classify intersection curves/points.

## Related APIs

- `include/geometry/parametric_intersector/surface_surface_intersector.hpp`
- `include/geometry/parametric_intersector/bezier_root_finder.hpp`
