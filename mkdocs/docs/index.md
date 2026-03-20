# GraphicsLab Documentation

GraphicsLab is a C++20 playground for graphics rendering and geometric modeling.

## Prerequisites

Before starting module pages, complete:

- [Get Started](graphicslab/get_started.md)
- [Create an Application](graphicslab/create_application.md)

This documentation focuses on how the project is organized and where to find core systems:

- Rendering backend (`vkl`) based on Vulkan
- Geometry kernel for curves, surfaces, and B-Rep
- Numeric and language utilities used by geometry and rendering modules

Use the left navigation to start with setup, then explore modules by domain.

## Suggested Path

1. Build and run the main app from [Get Started](graphicslab/get_started.md).
2. Follow [Create an Application](graphicslab/create_application.md) and run the sphere project.
3. Read [Rendering Scene](rendering/scene.md) to understand scene/render flow.
4. Continue with Geometry and Numeric pages for algorithm details.

## Expected Output

After the first pass through the docs, you should be able to:

- Build GraphicsLab locally
- Run the main app
- Create a small custom app that visualizes a sphere
- Locate relevant code by module using the docs structure
