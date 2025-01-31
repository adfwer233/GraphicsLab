#include "project/project_manager.hpp"
#include <iostream>
#include <windows.h>

bool ProjectManager::loadProject(const std::string &pluginPath) {
    projectHandle = LoadLibraryEx(pluginPath.c_str(), NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!projectHandle) {
        std::cerr << "Failed to load plugin: " << GetLastError() << std::endl;
        return false;
    }

    // Load the symbol (factory function)
    createProject =
        reinterpret_cast<IGraphicsLabProject *(*)()>(GetProcAddress((HMODULE)projectHandle, "createProject"));
    if (!createProject) {
        std::cerr << "Failed to load createProject: " << GetLastError() << std::endl;
        FreeLibrary((HMODULE)projectHandle);
        return false;
    }

    return true;
}

void ProjectManager::unloadProject() {
    if (projectHandle) {
        FreeLibrary((HMODULE)projectHandle);
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
