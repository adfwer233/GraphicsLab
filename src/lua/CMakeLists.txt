project(GraphicsLab_LuaBinding VERSION 1.0 DESCRIPTION "Geometry Kernel" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE
        main.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})
# set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} PUBLIC GraphicsLab_Geometry lua_static sol2)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${GRAPHICS_LAB_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)