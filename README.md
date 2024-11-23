## GraphicsLab

GraphicsLab is a developing toy engine designed for computer graphics experimentation. 
Its goal is offering a range of features and tools to explore and create with graphics technologies efficiently.

### Build and Install

GraphicsLab use cmake as build system and the preferred toolchain is `clang`+`ninja`.
You can build GraphicsLab and install it with following commands

```bash
mkdir build
cd build
cmake .. -DGRAPHICS_LAB_INSTALL:BOOL=ON -DCMAKE_INSTALL_PREFIX:PATH=your_path_to_install
cmake --build .
cmake install .
```

The target executable `LAB_main` is the user interface of Graphics Lab, you can load an external project in plugin manager.

### Build Project and Run with GraphicsLab

Firstly, build your external project with following option
```bash
-DGRAPHICS_LAB_INSTALL_PATH:STRING=your_install_path
```
Then load your project to GraphicsLab

### Features

- Language Features
  - Meta-programming tools
  - Basic reflection
- Rendering Backend
  - A Vulkan-based rendering backend
  - Simple Render Graph System and Scene Tree System
  - Supports both rasterization and path tracing using Vulkan compute shaders
- Geometry Kernel
  - A lightweight kernel for handling meshes, parametric geometry, and point clouds
  - Includes basic geometry processing tasks
- Project Integration
  - Create and manage independent graphics projects, run them in GraphicsLab
  - Hot-load projects within GraphicsLab for seamless experimentation
