#include "project_manager.hpp"
#include <dlfcn.h>
#include <iostream>

bool ProjectManager::loadProject(const std::string &pluginPath) {
    projectHandle = dlopen(pluginPath.c_str(), RTLD_LAZY);
    if (!projectHandle) {
        std::cerr << "Failed to load plugin: " << GetLastError() << std::endl;
        return false;
    }

    // Load the symbol (factory function)
    createRenderable = reinterpret_cast<IGraphicsLabProject *(*)()>(dlsym(pluginHandle, "createProject"));
    if (!createProject) {
        std::cerr << "Failed to load createProject: " << GetLastError() << std::endl;
        dlclose(projectHandle);
        return false;
    }

    return true;
}

void ProjectManager::unloadProject() {
    if (projectHandle) {
        dlclose(projectHandle);
        projectHandle = nullptr;
        createProject = nullptr;
    }
}

IGraphicsLabProject *ProjectManager::getProject() {
    if (createProject) {
        return createProject();
    }
    return nullptr;
}

ProjectManager::~ProjectManager() {
    unloadProject();
}
