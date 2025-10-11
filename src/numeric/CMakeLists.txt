project(GraphicsLab_Numeric VERSION 1.0 DESCRIPTION "Graphics Lab numeric" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE polynomial_solver.cpp)

add_library(${PROJECT_NAME} ${SRC_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ${GRAPHICS_LAB_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog_header_only)