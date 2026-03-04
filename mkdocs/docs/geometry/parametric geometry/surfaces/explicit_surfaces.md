# Explicit Surfaces

An explicit surface is typically represented in form:

$$
z = f(x, y)
$$

or as a sampled field converted to renderable geometry.

## In GraphicsLab

Explicit surfaces are used in demo applications and algorithm experiments where function-driven geometry is convenient.

## Practical Notes

- Great for prototyping and visualization
- Often converted to mesh patches for rendering
- Intersections generally reduce to solving nonlinear equations

## Related Files

- `include/geometry/parametric/explicit_surface.hpp`
- `src/applications/project_demo/explicit_surface_examples.*`
