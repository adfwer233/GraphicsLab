#pragma once

#include <filesystem>
#include <string>

class FileSystem {
  public:
    static std::filesystem::path getExecutablePath();

    static std::string chooseDirectory();
    static std::string chooseFile();
};