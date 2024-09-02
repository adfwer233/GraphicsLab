project(LAB_geometry_processing VERSION 1.0 DESCRIPTION "Geometry Processing Module" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE gauss_curvature.cpp
        single_source_exact_geodesic_distance.cpp
        normal_vector.cpp
        laplacian_boundary_z_axis_valued.cpp)

add_library(${PROJECT_NAME} ${SRC_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw LAB_vkl igl_copyleft::core eigen)