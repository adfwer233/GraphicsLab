project(LAB_demo VERSION 1.0 DESCRIPTION "Demo Application" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp
        application.cpp
        utils/controller.cpp
        ui/components/scene_ui.cpp
        ui/components/picking_ui.cpp
        ui/ui_manager.cpp
        ui/components/render_mode_ui.cpp
        ui/components/material_ui.cpp
        ui/components/scene_render_ui.cpp
        ui/components/scene_manager.cpp
        ui/components/geometry_processing_ui.cpp
        ui/components/uv_render_ui.cpp
        ui/components/scene_info.cpp
        ui/components/bezier_editor/bezier_editor_panel_ui.cpp
        ui/components/bezier_editor/bezier_editor_ui.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/vkl/shader")

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})

target_link_libraries(${PROJECT_NAME} glm glfw spirv-cross-core spirv-cross-cpp nlohmann_json LAB_vkl LAB_ray_tracer LAB_geometry LAB_simulation LAB_geometry_processing igl_copyleft::core)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/configs" $<TARGET_FILE_DIR:${PROJECT_NAME}>
)