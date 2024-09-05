include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        boost_di
        GIT_REPOSITORY https://github.com/boost-experimental/di.git
        GIT_TAG v1.3.0
)

FetchContent_MakeAvailable(boost_di)