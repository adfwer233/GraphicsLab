# Rendering Scene

The rendering subsystem centers around a Vulkan-based backend (`vkl`) and scene tree abstractions.

## Core Pieces

- `include/vkl/core/*`: low-level Vulkan wrappers (device, swap chain, pipeline, buffer)
- `include/vkl/scene_tree/*`: scene graph, meshes, materials, cameras
- `include/graphics_lab/render_graph/*`: render graph orchestration

## Rendering Flow (High Level)

1. Create device/swapchain resources.
2. Build scene objects (mesh/material/camera).
3. Compile and execute render passes.
4. Present final image to swapchain.

## Related Source Areas

- `src/vkl/core/`
- `src/main/render/internal_render_pass/`
- `include/graphics_lab/render_passes/`

## Shaders

Project shaders are stored in `shader/` and app-specific shader directories.
