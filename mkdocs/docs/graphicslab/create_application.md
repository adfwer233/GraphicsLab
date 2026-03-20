# Create an Application

This guide shows how to create your own application and use GraphicsLab as a library.

## In-repo application

The demo applications in `src/applications/` use the in-repo library target `GraphicsLab_Lib`.

Create a folder under `src/applications/your_app/` with a `CMakeLists.txt` and `main.cpp`.

```cmake
project(GraphicsLab_YourApp VERSION 1.0 DESCRIPTION "Your GraphicsLab App" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${GRAPHICS_LAB_SOURCE_DIR}/data")
add_definitions(-DSHADER_DIR="${GRAPHICS_LAB_SOURCE_DIR}/src/vkl/shader")

target_include_directories(${PROJECT_NAME} PUBLIC ${GRAPHICS_LAB_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} PUBLIC GraphicsLab_Lib)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${GRAPHICS_LAB_SOURCE_DIR}/configs" $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/font
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${GRAPHICS_LAB_SOURCE_DIR}/assets/font" $<TARGET_FILE_DIR:${PROJECT_NAME}>/font
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${GRAPHICS_LAB_SOURCE_DIR}/shader" $<TARGET_FILE_DIR:${PROJECT_NAME}>/shader
)
```

Then register it in `src/applications/CMakeLists.txt`:

```cmake
add_subdirectory(your_app)
```

Example custom project: create `visualization.hpp` and visualize a sphere.

```cpp
#pragma once

#include "graphics_lab/project.hpp"
#include "geometry/parametric/sphere.hpp"
#include "geometry/parametric/tessellator.hpp"

struct SphereVisualizationProject : IGraphicsLabProject {
    void tick() override {
    }

    std::string name() override {
        return "Sphere Visualization";
    }

    void visualize() {
        using GraphicsLab::Geometry::Sphere;
        using GraphicsLab::Geometry::Tessellator;

        Sphere sphere({0.0, 0.0, 0.0}, 1.0);
        Tessellator::tessellate(sphere, 64, 32);

        auto *node = context.sceneTree->addGeometryNode<Sphere>(std::move(sphere), "Sphere");
        node->updateColor({0.2f, 0.7f, 1.0f});
    }

    void afterLoad() override {
        visualize();
    }

    ReflectDataType reflect() override {
        auto result = IGraphicsLabProject::reflect();
        result.emplace("visualize", TypeErasedValue(&SphereVisualizationProject::visualize, this));
        return result;
    }
};
```

The `reflect()` function above registers a `visualize` button/action in the main UI, so you can run the sphere creation again interactively.

Then use it from your entry point:

```cpp
#include "graphics_lab/application.hpp"
#include "argparse/argparse.hpp"
#include "visualization.hpp"
#include <iostream>

int main(int argc, char *argv[]) {
    argparse::ArgumentParser args;
    GraphicsLabApplication::set_args(args);

    try {
        args.parse_args(argc, argv);
    } catch (const std::exception &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << args;
        return EXIT_FAILURE;
    }

    GraphicsLabApplication app(args);

    app.projectFactory = []() { return new SphereVisualizationProject(); };

    if (args.is_used("--input")) {
        app.appOption.load_obj_path = args.get<std::string>("--input");
    }

    try {
        app.run();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}
```

## External application (use installed GraphicsLab package)

After installing GraphicsLab (`GRAPHICS_LAB_INSTALL=ON`), your external `CMakeLists.txt` can use:

```cmake
find_package(GraphicsLab CONFIG REQUIRED)

add_executable(MyApp main.cpp)
target_link_libraries(MyApp PRIVATE GraphicsLab::GraphicsLabLib)
```

Configure your external project with:

```bash
cmake -S . -B build -DGRAPHICS_LAB_INSTALL_PATH=your_install_path
```

## Runtime resources

For both approaches, keep runtime resources (`configs/`, `assets/font/`, `shader/`) next to the executable, matching the demo apps.

Target naming differs by context:
- In-repo app (under `src/applications/*`): link `GraphicsLab_Lib`
- External app via `find_package(GraphicsLab)`: link `GraphicsLab::GraphicsLabLib`
