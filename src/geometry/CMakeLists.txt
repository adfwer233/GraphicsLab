project(LAB_geometry VERSION 1.0 DESCRIPTION "Geometry Kernel" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE
        parametric/tessellator.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILE})
# set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} PUBLIC glm spdlog eigen nlohmann_json autodiff CDT)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${GRAPHICS_LAB_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
