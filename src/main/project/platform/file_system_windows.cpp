#include "file_system.hpp"

#include <windows.h>
#include <ShlObj.h>

std::string FileSystem::chooseDirectory() {
    BROWSEINFO bi = { 0 };
    bi.lpszTitle = "Select a folder";
    LPITEMIDLIST pidl = SHBrowseForFolder(&bi);
    char path[MAX_PATH];

    if (pidl != nullptr) {
        // Get the name of the folder
        if (SHGetPathFromIDList(pidl, path)) {
            return std::string(path);
        }
        CoTaskMemFree(pidl);  // Free memory allocated by the dialog
    }
    return std::string();
}