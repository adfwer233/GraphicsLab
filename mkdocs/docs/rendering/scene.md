# Rendering Scene

The rendering subsystem centers around a Vulkan-based backend (`vkl`) and scene tree abstractions.

## Prerequisites

Before this page, make sure:

- You can build and run GraphicsLab (see [Get Started](../graphicslab/get_started.md))
- You know how to create a custom app/project (see [Create an Application](../graphicslab/create_application.md))

## Core Pieces

- `include/vkl/core/*`: low-level Vulkan wrappers (device, swap chain, pipeline, buffer)
- `include/vkl/scene_tree/*`: scene graph, meshes, materials, cameras
- `include/graphics_lab/render_graph/*`: render graph orchestration

## Rendering Flow (High Level)

1. Create device/swapchain resources.
2. Build scene objects (mesh/material/camera).
3. Compile and execute render passes.
4. Present final image to swapchain.

## Example: Visualizing a 3D Sphere

Use a parametric sphere, tessellate it into a renderable mesh, then add it to the scene tree.

```cpp
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/tessellator.hpp"

// Place this in GraphicsLabApplication::initialize()
using GraphicsLab::Geometry::Sphere;
using GraphicsLab::Geometry::Tessellator;

Sphere sphere({0.0, 0.0, 0.0}, 1.0);
Tessellator::tessellate(sphere, 64, 32);

auto *sphereNode = appContext.sceneTree->addGeometryNode<Sphere>(std::move(sphere), "Sphere");
sphereNode->updateColor({0.2f, 0.7f, 1.0f});

appContext.sceneTree->addCameraNode("Sphere Camera", Camera({0, 0, 5}, {0, 1, 0}, {0, 0, 0}));
```

This follows the same rendering path as other scene geometry: once the node is in the scene tree, it is consumed by the render passes and shown in the 3D viewport.

## Expected Output

When the example is wired into your app/project, you should see:

- A cyan sphere centered near the world origin
- A camera view that frames the sphere directly
- The sphere present as a scene node named `Sphere`

## Verify

- Build succeeds with no missing include errors for sphere/tessellator headers
- App launches and 3D viewport is visible
- Sphere appears immediately after startup

## Troubleshooting

- Blank viewport: verify the camera position/target and that the sphere node is added
- Geometry not visible: ensure `Tessellator::tessellate(...)` is called before adding the node
- No docs preview: use `python -m mkdocs serve` from the `mkdocs/` directory

## Build Your Own App

Application creation and library integration are documented in:

- [Create an Application](../graphicslab/create_application.md)

## Related Source Areas

- `src/vkl/core/`
- `src/main/render/internal_render_pass/`
- `include/graphics_lab/render_passes/`

## Shaders

Project shaders are stored in `shader/` and app-specific shader directories.
