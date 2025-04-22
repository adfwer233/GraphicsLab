project(LAB_test VERSION 1.0 DESCRIPTION "Test Project" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SRC_FILE
        templates/vkl_type_list_test.cpp
        templates/meta_programming_test.cpp
        coroutine/generator.cpp
        reflection/static_reflector_test.cpp
        reflection/dynamic_reflector_test.cpp
)

enable_testing()

add_executable(${PROJECT_NAME} ${SRC_FILE})

target_link_libraries(${PROJECT_NAME} PRIVATE glm glfw LAB_vkl LAB_geometry GTest::gtest GTest::gtest_main nlohmann_json)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})