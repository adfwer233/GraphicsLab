include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
)

FetchContent_MakeAvailable(glm)