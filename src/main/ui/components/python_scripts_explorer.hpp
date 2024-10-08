#pragma once

#include "component.hpp"
#include "controller/controller.hpp"
#include <filesystem>

class PythonScriptsExplorer : public UIComponent {
    UIState &uiState_;

    // Function to get the file list from a directory
    std::vector<std::string> getFileList(const std::string &directoryPath) {
        std::vector<std::string> fileList;

        // Iterate over directory and collect filenames
        for (const auto &entry : std::filesystem::directory_iterator(directoryPath)) {
            if (entry.is_regular_file()) {                            // Ensure it's a file, not a directory
                fileList.push_back(entry.path().filename().string()); // Get only the file name
            }
        }
        return fileList;
    }

    std::vector<std::string> filenames;
    std::filesystem::path script_dir_path;

  public:
    PythonScriptsExplorer(SceneTree::VklSceneTree &sceneTree, UIState &uiState)
        : UIComponent(sceneTree), uiState_(uiState) {
    }

    void render() final {

        namespace py = pybind11;

        ImGui::Begin("Project Scripts List");

        // Custom styling to make the widget look more attractive
        ImGui::Text("Available Files");
        ImGui::Separator();

        if (uiState_.project != nullptr) {
            if (ImGui::Button("Fresh Script Lists")) {
                auto project_config_path = uiState_.projectStatus.projectPath;
                auto project_dir = std::filesystem::path(project_config_path).parent_path();
                script_dir_path = project_dir / "scripts";

                filenames = getFileList(script_dir_path.string());
            }
        }

        if (ImGui::BeginListBox("##fileList", ImVec2(-FLT_MIN, 10 * ImGui::GetTextLineHeightWithSpacing()))) {
            for (const auto &file : filenames) {
                auto file_path = script_dir_path / file;

                ImGui::PushID(file.c_str()); // Ensure unique IDs for each item
                // Create a horizontally aligned section
                ImGui::BeginGroup();

                // Display file icon (using a Unicode character here as an example)
                ImGui::Text("📄");
                ImGui::SameLine(); // Align the file name next to the icon

                // Display file name as selectable
                if (ImGui::Selectable(file.c_str())) {
                    std::ifstream ifs(file_path.c_str());
                    std::stringstream ss;
                    ss << ifs.rdbuf();

                    try {
                        py::exec(ss.str());
                        auto res = py::eval("sys.stdout.get_output()").cast<std::string>();
                        py::exec("sys.stdout.clear_output()");
                        spdlog::info(res);
                    } catch (std::exception &e) {
                        spdlog::error(e.what());
                    }
                }

                ImGui::EndGroup();
                ImGui::PopID(); // Pop the ID after each file
            }
            ImGui::EndListBox();
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, PythonScriptsExplorer)