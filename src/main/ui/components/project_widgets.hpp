#pragma once

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "project/file_system.hpp"
#include "project/project_manager.hpp"

#include "ui/render_resources.hpp"

class ProjectWidgetComponent : public UIComponent {
    UIState &uiState_;

  public:
    ProjectWidgetComponent(SceneTree::VklSceneTree &sceneTree, UIState &uiState)
        : UIComponent(sceneTree), uiState_(uiState) {
    }

    void render() final {
        ImGui::Begin("Project Manager");
        {

            if (ImGui::Button("Choose Path")) {
                std::filesystem::path path =
                    std::filesystem::path(FileSystem::chooseDirectory()) / "GraphicsLabProject.json";
                spdlog::info(path.string());
                std::ifstream f(path.string());
                json data = json::parse(f);

                auto project_name = data["Project"].get<std::string>();
                uiState_.projectStatus.name = project_name;
                uiState_.projectStatus.projectPath = path.string();

                spdlog::info(project_name);
                for (const auto &build_info : data["Built"]) {
                    spdlog::info(build_info["build_type"].get<std::string>());
                    spdlog::info(build_info["dll_path"].get<std::string>());

                    uiState_.projectStatus.buildConfigs.emplace_back(build_info["build_type"].get<std::string>(),
                                                                     build_info["dll_path"].get<std::string>());
                }
            }

            ImGui::Text("Current Project: %s", uiState_.projectStatus.name.c_str());
            for (auto info : uiState_.projectStatus.buildConfigs) {
                if (ImGui::TreeNode(std::format("Config {}", info.buildType).c_str())) {
                    if (ImGui::Button(std::format("Run {}", info.buildType).c_str())) {
                        ProjectManager projectManager;

                        if (projectManager.loadProject(info.dllPath)) {
                            IGraphicsLabProject *project = projectManager.getProject();
                            if (project) {
                                project->tick();
                                delete project;
                            } else {
                                spdlog::error("load project failed");
                            }
                        }
                    }
                    ImGui::TreePop();
                }
            }

            if (ImGui::Button("serialize")) {
                spdlog::info(uiState_.projectStatus.serialization());
            }

            if (ImGui::Button("deserialize")) {
                json j = json::parse("{\"buildConfigs\":\"[{buildType: Debug, dllPath: Debug}]\",\"name\":\"GraphicsLabProjectTempla\n"
                                     "te\",\"projectPath\":\"C:\\\\Users\\\\Anchang\\\\Desktop\\\\GraphicsLabProjectTemplate\\\\GraphicsLabProject.json\"}");
                uiState_.projectStatus.deserialization(j);
            }
        }
        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, ProjectWidgetComponent)