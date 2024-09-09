#include "file_system.hpp"

#include <cstdlib>
#include <string>

std::string FileSystem::chooseDirectory() {
    char path[1024];
    FILE *pipe = popen("zenity --file-selection --directory", "r");
    if (!pipe)
        return std::string();

    fgets(path, 1024, pipe);
    pclose(pipe);

    // Remove newline character from the path
    std::string dir = path;
    dir.erase(std::remove(dir.begin(), dir.end(), '\n'), dir.end());

    return dir;
}