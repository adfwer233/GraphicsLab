project(LAB_main VERSION 1.0 DESCRIPTION "Graphics Lab Main Application" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE main.cpp
        application.cpp
        project/project_manager.cpp
        project/file_system.cpp
)

add_executable(${PROJECT_NAME} ${SRC_FILE})

add_definitions(-DDATA_DIR="${CMAKE_SOURCE_DIR}/data")
add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/vkl/shader")

if (${RENDERDOC_ENABLE})
    add_definitions(-DRENDERDOC_DIR="${RENDERDOC_INSTALL_PATH}/")
endif ()

if (NOT CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_definitions(-DIMGUI_ENABLE_WIN32_DEFAULT_IME_FUNCTIONS)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${VK_SDK_INCLUDE})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC LAB_vkl LAB_geometry)
target_link_libraries(${PROJECT_NAME} PUBLIC argparse assimp glm glfw spdlog Vulkan::Vulkan spirv-cross-core spirv-cross-cpp nlohmann_json igl_copyleft::core pybind11::embed xtensor)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/configs" $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${PROJECT_NAME}>/font
)

add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/assets/font" $<TARGET_FILE_DIR:${PROJECT_NAME}>/font
)