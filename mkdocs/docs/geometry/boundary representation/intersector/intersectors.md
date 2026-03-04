# Intersectors

Intersectors compute intersections between geometric entities, especially curve-curve and surface-surface pairs.

## Main Types

- Curve-curve intersectors (2D parametric curves)
- Surface-surface intersectors
- Specialized analytic pair intersectors (for robust fast paths)

## Header Map

- `include/geometry/parametric_intersector/curve_curve_intersector_2d/*`
- `include/geometry/parametric_intersector/surface_surface_intersector.hpp`
- `include/geometry/parametric_intersector/torus_torus_intersector.hpp`

## Output Expectations

Intersectors generally return parameter values and/or geometric points, often with metadata about multiplicity or classification.
