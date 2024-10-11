include(FetchContent)

FetchContent_Declare(
        xtl
        GIT_REPOSITORY https://github.com/xtensor-stack/xtl.git
)

FetchContent_MakeAvailable(xtl)

FetchContent_Declare(
        xtensor
        GIT_REPOSITORY https://github.com/xtensor-stack/xtensor.git
        GIT_TAG 0.24.7
)

FetchContent_MakeAvailable(xtensor)