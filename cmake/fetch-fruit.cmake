include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        fruit
        GIT_REPOSITORY https://github.com/google/fruit.git
        GIT_TAG v3.7.1
)

# Set CMake build type to Release
set(CMAKE_BUILD_TYPE Release)

FetchContent_MakeAvailable(fruit)