#pragma once

#include <string>

class FileSystem {
  public:
    static std::string chooseDirectory();
    static std::string chooseFile();
};