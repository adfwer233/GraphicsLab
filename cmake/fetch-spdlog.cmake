include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.14.1
)

FetchContent_MakeAvailable(spdlog)