#include "project/file_system.hpp"

#include <cstdlib>
#include <string>

#include "spdlog/spdlog.h"

std::string FileSystem::chooseDirectory() {
    char path[1024];
    FILE *pipe = popen("zenity --file-selection --directory", "r");
    if (!pipe)
        return std::string();

    fgets(path, 1024, pipe);
    pclose(pipe);

    // Remove newline character from the path
    std::string dir = path;
    std::erase(dir, '\n');

    return dir;
}

std::string FileSystem::chooseFile() {
    char path[1024];
    FILE *pipe = popen("zenity --file-selection", "r");
    if (!pipe)
        return std::string();

    fgets(path, 1024, pipe);
    pclose(pipe);

    // Remove newline character from the path
    std::string dir = path;
    std::erase(dir, '\n');

    return dir;
}
