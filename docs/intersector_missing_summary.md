# Missing Intersectors Summary (CSI / PPI / SSI)

This note summarizes gaps in `include/geometry/boundary_representation/intersector` based on the current headers and dispatch logic.

## Scope reviewed

- `include/geometry/boundary_representation/intersector/geometrical_intersector.hpp`
- `include/geometry/boundary_representation/intersector/curve_surface_intersection/*`
- `include/geometry/boundary_representation/intersector/pcurve_pcurve_intersection/*`
- `include/geometry/boundary_representation/intersector/surface_surface_intersection/*`
- `include/geometry/boundary_representation/intersector/topological_intersection/*`

## 1) CSI (Curve-Surface Intersection)

### Implemented

`GeneralCurveSurfaceIntersection::solve(...)` has explicit special cases only for:

- line-plane (`LinePlaneIntersection`)
- line-sphere (`LineSphereIntersection`)
- line-torus (`LineTorusIntersection`)

Everything else falls back to the generic sampling + Newton refinement path in `GeneralCurveSurfaceIntersection::intersect(...)`.

### Missing intersectors

No dedicated CSI intersector headers/dispatch branches exist for:

- curve-plane where curve is not `StraightLine3D` (Bezier/BSpline/NURBS/etc.)
- curve-sphere where curve is not `StraightLine3D`
- curve-torus where curve is not `StraightLine3D`
- any CSI involving other surface types (for example cylinder/cone/NURBS/tensor product surfaces)

### Notes

- The generic path exists, so these pairs are not "unsupported", but they are missing specialized intersectors.
- Boundary handling in the geometric wrapper is still marked TODO (`geometrical_intersector.hpp`: cut in param range).

## 2) PPI (PCurve-PCurve Intersection)

### Implemented

`GeneralPCurvePCurveIntersection` currently works by converting each pcurve to Bezier segments, then calling `BezierBezierIntersector2D`.

Accepted input pcurve classes in conversion:

- `BezierCurve2D`
- `StraightLine2D` (converted to degree-1 Bezier)
- `BSplineCurve2D` (converted to Bezier segments)

### Missing intersectors

- No direct intersector for NURBS pcurves (or any 2D curve type not convertible in current lambda); current code throws on unknown type.
- No dedicated analytic intersectors (for example line-line, line-conic, conic-conic) as separate robust kernels.
- No explicit dispatcher layer by pcurve type pair; everything is funneled through Bezier clipping.

### Notes

- Topological face/face and edge/edge intersection depend on this PPI path, so missing pcurve-type coverage propagates upward.

## 3) SSI (Surface-Surface Intersection)

### Implemented

`GeneralSurfaceSurfaceIntersection::solve(...)` has explicit special cases only for:

- plane-plane (`PlanePlaneIntersection`)
- plane-sphere (`PlaneSphereIntersection`)
- plane-torus (`PlaneTorusIntersection`)

Plus mirrored handling for:

- sphere-plane (by calling plane-sphere and swapping pcurves)
- torus-plane (by calling plane-torus and swapping pcurves)

All other pairs go to the generic marching path `intersect_all(...)`.

### Missing intersectors

No dedicated SSI intersector headers/dispatch branches exist for common analytic pairs such as:

- sphere-sphere
- sphere-torus
- torus-torus
- cylinder-* / cone-* / NURBS-* / tensor-product-* pairings

### Notes

- As with CSI, these are mostly missing specialized kernels rather than missing any computational path.
- `GeometricalIntersector` currently only exposes surface-surface at wrapper level; no parallel wrapper entry points for CSI/PPI are present there.

## TODO Matrix (Geometry x Geometry)

Legend:

- `D`: dedicated intersector exists
- `T`: dedicated intersector is still missing (TODO)
- `-`: not applicable for this intersector family

### CSI TODO matrix (curve geometry x surface geometry)

Rows use 3D curve geometry list. Columns use surface geometry list.

| Curve \\ Surface | Plane | Sphere | Torus | TensorProductBezier | ExplicitSurface | NURBSSurface |
|---|---:|---:|---:|---:|---:|---:|
| StraightLine3D | D | D | D | T | T | T |
| BezierCurve3D | T | T | T | T | T | T |
| BSplineCurve3D | T | T | T | T | T | T |
| NURBSCurve3D | T | T | T | T | T | T |

### PPI TODO matrix (pcurve geometry x pcurve geometry)

Rows/columns use 2D pcurve geometry list.

| Pcurve1 \\ Pcurve2 | StraightLine2D | BezierCurve2D | BSplineCurve2D | NURBSCurve2D |
|---|---:|---:|---:|---:|
| StraightLine2D | T | T | T | T |
| BezierCurve2D | T | D | T | T |
| BSplineCurve2D | T | T | T | T |
| NURBSCurve2D | T | T | T | T |

Note: current `GeneralPCurvePCurveIntersection` can execute many non-`D` cells by converting to Bezier and using `BezierBezierIntersector2D`; the matrix tracks missing **dedicated** pair intersectors.

### SSI TODO matrix (surface geometry x surface geometry)

Rows/columns use surface geometry list.

| Surface1 \\ Surface2 | Plane | Sphere | Torus | TensorProductBezier | ExplicitSurface | NURBSSurface |
|---|---:|---:|---:|---:|---:|---:|
| Plane | D | D | D | T | T | T |
| Sphere | D | T | T | T | T | T |
| Torus | D | T | T | T | T | T |
| TensorProductBezier | T | T | T | T | T | T |
| ExplicitSurface | T | T | T | T | T | T |
| NURBSSurface | T | T | T | T | T | T |

## Practical impact

- **Correctness/robustness risk**: hard cases rely on generic marching/clipping and may be less robust near tangencies/singularities.
- **Performance risk**: type-specific closed-form solutions are absent for most pairs.
- **Extensibility gap**: adding new parametric types currently requires extending conversion/dispatch logic manually.

## Priority candidates to add next

1. SSI: `sphere-sphere` and `sphere-torus` (high-value analytic pairs)
2. CSI: non-line curve with analytic surfaces (at least Bezier/BSpline vs plane/sphere)
3. PPI: NURBS2D conversion/support path, then type-pair-specific fast paths if needed


