# Geometry

The geometry module provides data structures and algorithms for geometric modeling.

## Prerequisites

Before diving into geometry internals:

- Confirm your environment is set up via [Get Started](../graphicslab/get_started.md)
- Run the basic sphere example from [Create an Application](../graphicslab/create_application.md)

## Coverage

- Parametric curves and surfaces
- Boundary representation (B-Rep) topology
- Spatial data structures and intersectors
- Geometry processing utilities (normals, curvature, geodesics)

## Important Include Roots

- `include/geometry/parametric/`
- `include/geometry/parametric_topology/`
- `include/geometry/boundary_representation/`
- `include/geometry/spatial_datastructure/`
- `include/geometry_processing/`

## Expected Output

After reading this section and linked pages, you should be able to:

- Choose the right geometry subsystem for a task (curve/surface/B-Rep)
- Find the main headers for implementation work
- Navigate from concepts in docs to concrete source locations

## Verify

- You can identify where a sphere/torus type is defined (`include/geometry/parametric/`)
- You can identify where boolean/intersection code lives (`include/geometry/boundary_representation/`)
- You can identify where geometry processing utilities live (`include/geometry_processing/`)
