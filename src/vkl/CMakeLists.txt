project(LAB_vkl VERSION 1.0 DESCRIPTION "Vulkan Learning Project Engine" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE core/vkl_window.cpp
        core/vkl_device.cpp
        core/vkl_swap_chain.cpp
        core/vkl_renderer.cpp
        core/vkl_buffer.cpp
        core/vkl_descriptor.cpp
        core/vkl_texture.cpp
        core/vkl_compute_pipeline.cpp
        core/vkl_image.cpp
        core/vkl_offscreen_renderer.cpp
        core/vkl_graphic_pipeline.cpp
        core/vkl_vma_impl.cpp
        utils/vkl_camera.cpp
        utils/stb_src.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_fetch_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_fetch_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_fetch_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
#        ${imgui_node_editor_SOURCE_DIR}/crude_json.cpp
#        ${imgui_node_editor_SOURCE_DIR}/imgui_canvas.cpp
#        ${imgui_node_editor_SOURCE_DIR}/imgui_node_editor.cpp
#        ${imgui_node_editor_SOURCE_DIR}/imgui_node_editor_api.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILE})

# set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_definitions(-DDATA_DIR="${GRAPHICS_LAB_SOURCE_DIR}/data")

if (OpenCV_FOUND)
    add_definitions(-DOPENCV_FOUND)
endif ()

target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${GRAPHICS_LAB_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${VK_SDK_INCLUDE}>
            $<INSTALL_INTERFACE:include>
)

if(OpenCV_FOUND)
    message(STATUS "found opencv")
    target_link_libraries(LAB_vkl PUBLIC glfw glm spdlog assimp spirv-cross-core spirv-cross-cpp OpenMP::OpenMP_CXX LAB_geometry ${OpenCV_LIBS})
else()
    target_link_libraries(LAB_vkl PUBLIC glfw glm spdlog assimp spirv-cross-core spirv-cross-cpp OpenMP::OpenMP_CXX LAB_geometry)
endif()
target_link_libraries(LAB_vkl PUBLIC Vulkan::Vulkan GPUOpen::VulkanMemoryAllocator)

############## Build SHADERS #######################

# Find all vertex and fragment sources within shaders directory
# taken from VBlancos vulkan tutorial
# https://github.com/vblanco20-1/vulkan-guide/blob/all-chapters/CMakeLists.txt
find_program(GLSL_VALIDATOR glslangValidator HINTS
        ${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}
        /usr/bin
        /usr/local/bin
        ${VULKAN_SDK_PATH}/Bin
        ${VULKAN_SDK_PATH}/Bin32
        $ENV{VULKAN_SDK}/Bin/
        $ENV{VULKAN_SDK}/Bin32/
)

# get all .vert and .frag files in shaders directory
file(GLOB_RECURSE GLSL_SOURCE_FILES
        "${GRAPHICS_LAB_SOURCE_DIR}/shader/*.frag"
        "${GRAPHICS_LAB_SOURCE_DIR}/shader/*.vert"
        "${GRAPHICS_LAB_SOURCE_DIR}/shader/*.comp"
        "${GRAPHICS_LAB_SOURCE_DIR}/shader/*.geom"
)

foreach(GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    set(SPIRV "${GRAPHICS_LAB_SOURCE_DIR}/shader/${FILE_NAME}.spv")
    add_custom_command(
            OUTPUT ${SPIRV}
            COMMAND ${GLSL_VALIDATOR} -V ${GLSL} -o ${SPIRV}
            DEPENDS ${GLSL})
    list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach(GLSL)

add_custom_target(
        ${PROJECT_NAME}_shader
        DEPENDS ${SPIRV_BINARY_FILES}
)

add_custom_command(
        TARGET ${PROJECT_NAME}_shader POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${GRAPHICS_LAB_SOURCE_DIR}/shader" $<TARGET_FILE_DIR:${PROJECT_NAME}>/shader
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${GRAPHICS_LAB_SOURCE_DIR}/shader" $<TARGET_FILE_DIR:${PROJECT_NAME}>/shader
)

if (GLSL_VALIDATOR)
    add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_shader)
endif ()