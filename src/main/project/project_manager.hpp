#pragma once

#include "graphics_lab/project.hpp"

#include <string>

class ProjectManager {
  public:
    ProjectManager() = default;
    ~ProjectManager();

    bool loadProject(const std::string &projectPath);
    void unloadProject();
    IGraphicsLabProject *getProject();

  private:
    void *projectHandle = nullptr;
    IGraphicsLabProject *(*createProject)() = nullptr;
};
