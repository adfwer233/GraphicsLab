project(GraphicsLab_FluidDemo VERSION 1.0 DESCRIPTION "Fluid Simulatino Demo" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp
        application.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/vkl/shader")

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw spdlog argparse spirv-cross-core spirv-cross-cpp nlohmann_json LAB_vkl LAB_geometry)