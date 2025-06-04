#include "platform/file_system.hpp"

#include <cstdlib>
#include <string>
#include <unistd.h>
#include <limits.h>

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

std::filesystem::path getExecutablePath() {
    char result[1024];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    return std::filesystem::path(std::string(result, count)).parent_path();
}