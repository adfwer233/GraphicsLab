project(LAB_vklbezier_curve VERSION 1.0 DESCRIPTION "2D Bezier Curve Rendering" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp
        application.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DPARTICLE_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shader")

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw LAB_vklvkl)
