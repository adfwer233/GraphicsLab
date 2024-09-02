project(LAB_simulation VERSION 1.0 DESCRIPTION "Simulation Module" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE
        walk_on_sphere/walk_on_sphere.cpp
        walk_on_sphere/walk_on_sphere_anisotropic.cpp
)

set(CUDA_SOURCE_FILE
        walk_on_sphere/gpu_kernel/cuda/wos_anisotropic.cu
)

if(CUDA_ENABLE)
    add_library(${PROJECT_NAME} ${SRC_FILE} ${CUDA_SOURCE_FILE})
else ()
    add_library(${PROJECT_NAME} ${SRC_FILE})
endif ()

find_package(OpenMP)
target_link_libraries(${PROJECT_NAME} glm autodiff LAB_geometry OpenMP::OpenMP_CXX Boost::math)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME}
        PUBLIC ${Boost_INCLUDE_DIRS}
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})