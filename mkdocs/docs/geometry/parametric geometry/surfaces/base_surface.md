# Base Surface

Parametric surfaces map a 2D parameter domain to 3D space.

A surface is generally represented as:

$$
S(u, v) \in \mathbb{R}^3, \quad (u, v) \in D
$$

## Core Responsibilities

- Evaluate surface points `S(u, v)`
- Evaluate first-order derivatives (`S_u`, `S_v`)
- Evaluate higher-order derivatives when required by curvature/intersection logic

## Common Operations

- Point evaluation
- Surface normal computation via cross product of tangents
- Trimming and domain clipping

## Related Headers

- `include/geometry/parametric/parametric_surface.hpp`
- `include/geometry/parametric/traits.hpp`
