include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        reflectcpp
        GIT_REPOSITORY https://github.com/getml/reflect-cpp.git
        GIT_TAG v0.14.1
)

FetchContent_MakeAvailable(reflectcpp)