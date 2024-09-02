project(LAB_ray_tracer VERSION 1.0 DESCRIPTION "Ray Tracer Module" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE ray.cpp ray_picker.cpp)

add_library(${PROJECT_NAME} ${SRC_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw LAB_vkl)