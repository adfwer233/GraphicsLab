#include "platform/file_system.hpp"

#include "spdlog/spdlog.h"

#include <ShlObj.h>
#include <commdlg.h>
#include <windows.h>

std::string FileSystem::chooseDirectory() {
    BROWSEINFO bi = {0};
    bi.lpszTitle = TEXT("Select a folder");
    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    char path[MAX_PATH];

    if (pidl != nullptr) {
        // Get the name of the folder
        if (SHGetPathFromIDListA(pidl, path)) {
            return {path};
        }
        CoTaskMemFree(pidl); // Free memory allocated by the dialog
    }
    return {};
}

std::string FileSystem::chooseFile() {
    BROWSEINFO bi = {0};
    bi.lpszTitle = TEXT("Select a file");
    bi.ulFlags = BIF_BROWSEINCLUDEFILES;
    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    char path[MAX_PATH];

    if (pidl != nullptr) {
        // Get the name of the folder
        if (SHGetPathFromIDListA(pidl, path)) {
            return {path};
        }
        CoTaskMemFree(pidl); // Free memory allocated by the dialog
    }
    return {};
}

std::filesystem::path FileSystem::getExecutablePath() {
    char path[MAX_PATH];
    GetModuleFileNameA(NULL, path, MAX_PATH);
    return std::filesystem::path(path).parent_path();
}