cmake_minimum_required(VERSION 3.26)

project(GraphicsLab)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CUDA_ENABLE OFF CACHE BOOL "Whether enable cuda kernel")

message(STATUS "cmake version ${CMAKE_VERSION} ${CMAKE_GENERATOR}")

if (CUDA_ENABLE)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES 50)
    endif()

    enable_language(CUDA)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# install eigen for autodiff
set(EIGEN_BUILD_CMAKE_PACKAGE ON)

# autodiff settings
set(AUTODIFF_BUILD_DOCS OFF)
set(AUTODIFF_BUILD_EXAMPLES OFF)
set(AUTODIFF_BUILD_PYTHON OFF)
set(AUTODIFF_BUILD_TESTS OFF)

set(SPIRV_CROSS_SHARED ON)

option(GRAPHICASLAB_INSTALL ON)

include(fetch-glfw)
include(fetch-glm)
include(fetch-assimp)
include(fetch-boost)
include(fetch-eigen)
include(fetch-stb)
include(fetch-imgui)
include(fetch-jsoncpp)
include(fetch-libigl)
include(fetch-googletest)
include(fetch-tinyobjloader)
include(fetch-autodiff)
include(fetch-random)
include(fetch-rootfinder)
include(fetch-svgpp)
include(fetch-spirv_cross)
include(fetch-spdlog)
include(fetch-boost_di)

find_package( OpenCV )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

igl_include(copyleft core)

INCLUDE_DIRECTORIES(${autodiff_SOURCE_DIR})
INCLUDE_DIRECTORIES(${stb_SOURCE_DIR})
INCLUDE_DIRECTORIES(${svgpp_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${svgpp_SOURCE_DIR}/third_party)
INCLUDE_DIRECTORIES(${root_finder_SOURCE_DIR}/root_finder/include)
INCLUDE_DIRECTORIES(${tinyobjloader_SOURCE_DIR})
INCLUDE_DIRECTORIES(${random_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${imgui_fetch_SOURCE_DIR})
INCLUDE_DIRECTORIES(${imgui_fetch_SOURCE_DIR}/backends)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${boost_di_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${spirv_cross_SOURCE_DIRS})

message(STATUS ${gtest_SOURCE_DIR})
message(STATUS ${Boost_INCLUDE_DIRS})
message(STATUS ${boost_di_SOURCE_DIR}/include)

# set vulkan SDK
if(NOT DEFINED ENV{VULKAN_SDK})
    message(FATAL_ERROR "VK_SDK_PATH not defined!")
endif()

set(VK_SDK_PATH $ENV{VULKAN_SDK})
set(VK_SDK_INCLUDE ${VK_SDK_PATH}/Include)
set(VK_SDK_LIB ${VK_SDK_PATH}/Lib/vulkan-1.lib)

if (UNIX)
    message(STATUS "Set Linux Vulkan SDK Path")
    set(VK_SDK_LIB ${VK_SDK_PATH}/lib/libvulkan.so.1)
endif ()

add_library(imgui
        ${imgui_fetch_SOURCE_DIR}/imgui.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_fetch_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_fetch_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

target_include_directories(imgui
        PUBLIC
        $<BUILD_INTERFACE:${GLFW_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(imgui
        PUBLIC
        $<BUILD_INTERFACE:${VK_SDK_INCLUDE}>
        $<INSTALL_INTERFACE:include>
)


install(TARGETS imgui
        EXPORT GraphicsLabTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

set(SPIRV_CROSS_SHARED ON)

add_subdirectory(src/vkl)
add_subdirectory(src/ray_tracer)
add_subdirectory(src/demo)
#add_subdirectory(src/particle)
add_subdirectory(src/geometry_processing)
add_subdirectory(src/bezier_curve)
add_subdirectory(src/geometry)
add_subdirectory(src/simulation)
#add_subdirectory(src/rendergraph_demo)
add_subdirectory(src/main)
add_subdirectory(test)

find_package(OpenMP)

if(OpenCV_FOUND)
    message(STATUS "found opencv")
    target_link_libraries(LAB_vkl glfw glm spdlog ${VK_SDK_LIB} assimp imgui spirv-cross-core spirv-cross-cpp OpenMP::OpenMP_CXX ${OpenCV_LIBS})
else()
    target_link_libraries(LAB_vkl glfw glm spdlog ${VK_SDK_LIB} assimp imgui spirv-cross-core spirv-cross-cpp OpenMP::OpenMP_CXX)
endif()

if(MSVC)
    add_compile_options("/source-charset:utf-8")
endif()

message(STATUS "Build documentation open")
message(STATUS ${Boost_LIBRARIES})
include(build-doxygen)
build_doxygen()

include(clang-format)

add_custom_target(BUILD_ALL)
add_dependencies(BUILD_ALL LAB_vkl LAB_demo LAB_geometry LAB_test LAB_main)

add_library(GraphicsLabLib INTERFACE)
target_link_libraries(GraphicsLabLib INTERFACE LAB_vkl LAB_geometry)

set(GLM_BUILD_INSTALL ON)
set(SPDLOG_INSTALL ON)

target_include_directories(GraphicsLabLib INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Install the library, include files, and CMake package configuration
install(TARGETS GraphicsLabLib
        EXPORT GraphicsLabTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
#
#install(EXPORT GraphicsLabTargets
#        FILE GraphicsLabTargets.cmake
#        NAMESPACE GraphicsLab::
#        DESTINATION lib/cmake/GraphicsLab
#)
#
#install(DIRECTORY include/ DESTINATION include)
#
##export(TARGETS GraphicsLabLib FILE "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabTargets.cmake")
#
## Generate a Config file for find_package()
#include(CMakePackageConfigHelpers)
#write_basic_package_version_file(
#        "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfigVersion.cmake"
#        VERSION 1.0.0
#        COMPATIBILITY AnyNewerVersion
#)
#
#configure_package_config_file(
#        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GraphicsLabConfig.cmake.in"
#        "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfig.cmake"
#        INSTALL_DESTINATION lib/cmake/GraphicsLab
#)
#
#install(FILES
#        "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfig.cmake"
#        "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfigVersion.cmake"
#        DESTINATION lib/cmake/GraphicsLab
#)
