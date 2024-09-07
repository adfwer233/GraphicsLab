project(LAB_main VERSION 1.0 DESCRIPTION "Graphics Lab Main Application" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp
        application.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/vkl/shader")

if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_definitions(-DIMGUI_ENABLE_WIN32_DEFAULT_IME_FUNCTIONS)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw spirv-cross-core spirv-cross-cpp nlohmann_json LAB_vkl LAB_ray_tracer LAB_geometry LAB_simulation LAB_geometry_processing igl_copyleft::core)
