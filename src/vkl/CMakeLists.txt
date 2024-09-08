project(LAB_vkl VERSION 1.0 DESCRIPTION "Vulkan Learning Project Engine" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE core/vkl_window.cpp
        core/vkl_device.cpp
        core/vkl_swap_chain.cpp
        core/vkl_renderer.cpp
        core/vkl_buffer.cpp
        scene/vkl_model.cpp
        utils/vkl_camera.cpp
        core/vkl_descriptor.cpp
        scene/vkl_object.cpp
        io/assimp_loader.cpp
        core/vkl_texture.cpp
        core/vkl_compute_pipeline.cpp
        bvh/vkl_bvh.cpp
        core/vkl_image.cpp
        core/vkl_offscreen_renderer.cpp
        io/tiny_obj_loader.cpp
        scene/vkl_scene.cpp
        scene/vkl_collection.cpp
        io/vkl_image_export.cpp
        core/vkl_graphic_pipeline.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DSHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shader")
add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")

if (OpenCV_FOUND)
    add_definitions(-DOPENCV_FOUND)
endif ()

target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${VK_SDK_INCLUDE}>
            $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} glm glfw LAB_geometry)

install(TARGETS ${PROJECT_NAME}
        EXPORT GraphicsLabTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

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
        "${PROJECT_SOURCE_DIR}/shader/*.frag"
        "${PROJECT_SOURCE_DIR}/shader/*.vert"
        "${PROJECT_SOURCE_DIR}/shader/*.comp"
        "${PROJECT_SOURCE_DIR}/shader/*.geom"
)

foreach(GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    set(SPIRV "${PROJECT_SOURCE_DIR}/shader/${FILE_NAME}.spv")
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
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/shader" $<TARGET_FILE_DIR:${PROJECT_NAME}>/shader
)

if (GLSL_VALIDATOR)
    add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_shader)
endif ()