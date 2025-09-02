include(FetchContent)

# Lua (with CMake support)
FetchContent_Declare(
        lua
        GIT_REPOSITORY https://github.com/walterschell/Lua.git
)
FetchContent_MakeAvailable(lua)