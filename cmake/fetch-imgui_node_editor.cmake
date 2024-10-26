include(FetchContent)

set(CMAKE_CXX_STANDARD 20)

FetchContent_Declare(
        imgui_node_editor
        GIT_REPOSITORY https://github.com/thedmd/imgui-node-editor.git
        GIT_TAG origin/master
)

FetchContent_MakeAvailable(imgui_node_editor)