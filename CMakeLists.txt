cmake_minimum_required(VERSION 3.26)

project(GraphicsLab)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CUDA_ENABLE OFF CACHE BOOL "Whether enable cuda kernel")
set(RENDERDOC_ENABLE OFF CACHE BOOL "Whether enable render doc")
set(GRAPHICS_LAB_INSTALL OFF CACHE BOOL "Whether enable graphics lab install")
set(RENDERDOC_INSTALL_PATH "" CACHE STRING "RenderDoc Install Path")

message(STATUS "cmake version ${CMAKE_VERSION} ${CMAKE_GENERATOR}")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

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
set(GLM_BUILD_INSTALL ON)
set(GLFW_INSTALL ON)
set(SPDLOG_INSTALL ON)
set(JSON_Install ON)
set(LIBIGL_INSTALL ON)
set(PYBIND11_INSTALL ON)
set(GLM_BUILD_INSTALL ON)
set(Random_BuildTests OFF)

set(SPDLOG_BUILD_SHARED ON)
set(SPIRV_CROSS_FORCE_PIC ON)
set(ASSIMP_BUILD_ZLIB ON)

## assimp importer, we only use obj format
#set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF)
#set(ASSIMP_BUILD_OBJ_IMPORTER ON)

set(XTL_CMAKECONFIG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
set(XTENSOR_CMAKECONFIG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX})

include(fetch-glfw)
include(fetch-glm)
include(fetch-assimp)
include(fetch-boost)
include(fetch-eigen)
include(fetch-stb)
include(fetch-imgui)
include(fetch-imgui_node_editor)
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
include(fetch-pybind11)
include(fetch-xtensor)

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
INCLUDE_DIRECTORIES(${imgui_node_editor_SOURCE_DIR})
INCLUDE_DIRECTORIES(${imgui_fetch_SOURCE_DIR}/backends)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${boost_di_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${spirv_cross_SOURCE_DIRS})
INCLUDE_DIRECTORIES(${RENDERDOC_INSTALL_PATH})

message(STATUS ${gtest_SOURCE_DIR})
message(STATUS ${Boost_INCLUDE_DIRS})
message(STATUS ${boost_di_SOURCE_DIR}/include)

find_package(Vulkan REQUIRED)
find_package(OpenMP REQUIRED)

set(SPIRV_CROSS_SHARED ON)

add_subdirectory(src/vkl)
#add_subdirectory(src/ray_tracer)
#add_subdirectory(src/demo)
#add_subdirectory(src/particle)
add_subdirectory(src/geometry_processing)
#add_subdirectory(src/bezier_curve)
add_subdirectory(src/geometry)
add_subdirectory(src/simulation)
#add_subdirectory(src/rendergraph_demo)

add_subdirectory(src/main)
add_subdirectory(test)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    add_compile_options("/source-charset:utf-8")
endif()

message(STATUS "Build documentation open")
message(STATUS ${Boost_LIBRARIES})
include(build-doxygen)
build_doxygen()

include(clang-format)

add_custom_target(BUILD_ALL)
add_dependencies(BUILD_ALL LAB_vkl LAB_geometry LAB_test LAB_main)

add_library(GraphicsLabLib INTERFACE)
target_link_libraries(GraphicsLabLib INTERFACE LAB_vkl LAB_geometry)

if (GRAPHICS_LAB_INSTALL)
    set(GLM_BUILD_INSTALL ON)

    target_include_directories(GraphicsLabLib INTERFACE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    # Install the library, include files, and CMake package configuration
    install(TARGETS GraphicsLabLib LAB_vkl LAB_geometry
            EXPORT GraphicsLabTargets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
    )

    install(DIRECTORY include/ DESTINATION include)
    install(FILES
            ${imgui_fetch_SOURCE_DIR}/imgui.h
            ${imgui_fetch_SOURCE_DIR}/imconfig.h
            ${imgui_fetch_SOURCE_DIR}/backends/imgui_impl_vulkan.h
            DESTINATION include)
    install(DIRECTORY ${Boost_INCLUDE_DIRS} DESTINATION include/boost)
    install(FILES ${stb_SOURCE_DIR}/stb_image.h DESTINATION include)

    # Generate a Config file for find_package()
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
            "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfigVersion.cmake"
            VERSION 1.0.0
            COMPATIBILITY AnyNewerVersion
    )

    configure_package_config_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/GraphicsLabConfig.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfig.cmake"
            INSTALL_DESTINATION lib/cmake/GraphicsLab
    )

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfig.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/GraphicsLabConfigVersion.cmake"
            DESTINATION lib/cmake/GraphicsLab
    )

    install(EXPORT GraphicsLabTargets
            FILE GraphicsLabTargets.cmake
            NAMESPACE GraphicsLab::
            DESTINATION lib/cmake/GraphicsLab
    )

endif()