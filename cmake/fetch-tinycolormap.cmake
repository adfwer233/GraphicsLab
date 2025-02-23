include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        tinycolormap
        GIT_REPOSITORY https://github.com/yuki-koyama/tinycolormap.git
)

FetchContent_MakeAvailable(tinycolormap)