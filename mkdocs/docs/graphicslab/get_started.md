# Get Started

## Requirements

- CMake `>= 3.26`
- A C++20 compiler (Clang is the preferred setup in this project)
- Ninja (recommended)
- Vulkan SDK (`find_package(Vulkan REQUIRED)` is used in CMake)
- Ubuntu/X11 dependencies used by GLFW:

```bash
sudo apt install libwayland-dev libxkbcommon-dev xorg-dev
```

## Build

From the repository root:

```bash
mkdir -p build
cd build
cmake .. -G Ninja
cmake --build .
```

## Run Main Application

The main target is `GraphicsLab_main` (sometimes built as `LAB_main` depending on local target naming in wrappers).

Typical run pattern:

```bash
./bin/GraphicsLab_main
```

If your build tree uses a different output layout, locate the executable with:

```bash
find . -type f -name '*GraphicsLab*main*'
```

## Enable Install

If you want to install headers and CMake package files:

```bash
cmake .. -G Ninja \
  -DGRAPHICS_LAB_INSTALL=ON \
  -DCMAKE_INSTALL_PREFIX=/your/install/prefix
cmake --build .
cmake --install .
```

## Build Tests

Tests are enabled by default through `GRAPHICS_LAB_ENABLE_TEST=ON`.

```bash
cmake .. -G Ninja -DGRAPHICS_LAB_ENABLE_TEST=ON
cmake --build .
ctest --output-on-failure
```

## Common CMake Options

- `GRAPHICS_LAB_ENABLE_TEST`: build test targets
- `GRAPHICS_LAB_INSTALL`: enable install rules
- `CUDA_ENABLE`: enable CUDA language support
- `RENDERDOC_ENABLE`: toggle RenderDoc integration hooks

## Project Layout (Quick Map)

- `include/`: public headers and module APIs
- `src/`: core implementations and apps
- `test/`: unit tests
- `shader/`: shader sources
- `mkdocs/`: this documentation site
