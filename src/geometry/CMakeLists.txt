project(LAB_geometry VERSION 1.0 DESCRIPTION "Geometry Kernel" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE
        autodiff/autodiff.cpp
        surface/tensor_product_bezier.cpp
        function/bernstein.cpp
        curve/bezier_curve_2d.cpp
        io/svg_importer.cpp
        loop/path_2d.cpp
        curve/bezier_root_finder.cpp
        curve/bezier_clipping.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILE})

target_link_libraries(${PROJECT_NAME} glm autodiff eigen LAB_vkl Boost::math)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
#target_include_directories(${PROJECT_NAME}
#        PUBLIC
#        $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>
#        $<INSTALL_INTERFACE:include>
#)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})

install(TARGETS LAB_geometry
        EXPORT GraphicsLabTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS boost_math boost_assert boost_concept_check boost_config boost_core boost_integer boost_lexical_cast boost_predef boost_random boost_static_assert boost_throw_exception
        EXPORT GraphicsLabTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)