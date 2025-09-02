include(FetchContent)
FetchContent_Declare(
        sol2
        GIT_REPOSITORY https://github.com/ThePhD/sol2.git
        GIT_TAG v3.3.0   # or latest
)
FetchContent_MakeAvailable(sol2)
