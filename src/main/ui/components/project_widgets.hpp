#pragma once

#include <future>

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "project/file_system.hpp"
#include "project/project_manager.hpp"

#include "ui/render_resources.hpp"

struct mystruct : public Reflectable {
    glm::vec3 pos;

    ReflectDataType reflect() override {
        return {{"pos", TypeErasedValue(&pos)}};
    }
};

class ProjectWidgetComponent : public UIComponent {
    UIState &uiState_;

    std::future<void> projectFunctionResult;

    bool showFunctionCallDialog = false;
    GraphicsLabReflection::GraphicsLabFunction functionWithParamPack;
    std::vector<std::any> functionCallParameters;
    std::set<std::string> bindClassNames;

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

            if (uiState_.project != nullptr) {
                ImGui::SameLine();
                if (ImGui::Button("Unload Project")) {
                    sceneTree_.cleanSceneTree();

                    uiState_.project = nullptr;
                    uiState_.projectManager.unloadProject();

                    projectFunctionResult = std::future<void>();
                }
            }

            ImGui::Text("Current Project: %s", uiState_.projectStatus.name.c_str());
            for (auto info : uiState_.projectStatus.buildConfigs) {
                if (ImGui::TreeNode(std::format("Config {}", info.buildType).c_str())) {
                    if (ImGui::Button(std::format("Load {}", info.buildType).c_str())) {

                        if (uiState_.projectManager.loadProject(info.dllPath)) {
                            uiState_.project = uiState_.projectManager.getProject();
                            if (uiState_.project) {

                                auto loadProject = [&]() {
                                    uiState_.project->updateContext(GraphicsLabContext(&sceneTree_.device_, &sceneTree_,
                                                                                       &LogManager::getInstance()));
                                    uiState_.project->afterLoad();
                                };

                                if (!projectFunctionResult.valid()) {
                                    projectFunctionResult = std::async(std::launch::async, loadProject);
                                }

                            } else {
                                spdlog::error("load project failed");
                            }
                        }
                    }
                    ImGui::TreePop();
                }
            }

            if (ImGui::Button("save configuration")) {
                auto result = uiState_.projectStatus.serialization();
                std::ofstream ofstream("status.json");
                ofstream << result.dump(4);
            }

            if (ImGui::Button("load last configuration")) {
                std::ifstream file("status.json");
                std::stringstream buffer;
                buffer << file.rdbuf();
                json j;
                buffer >> j;
                spdlog::critical(j.dump());
                uiState_.projectStatus.deserialization(j);
            }

            ImGui::Text("Project Functions");

            // spdlog::info("stat {}", int(projectFunctionResult.wait_for(std::chrono::milliseconds(500))));

            if (projectFunctionResult.valid() and
                projectFunctionResult.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {

                // projectFunctionResult.get();
                // projectFunctionResult = std::future<void>();

                if (uiState_.project != nullptr) {

                    if (ImGui::Button("export to python")) {
                        pybind11::module m;
                        if (not bindClassNames.contains(uiState_.project->name())) {
                            uiState_.project->bindPython(m);
                            bindClassNames.insert(uiState_.project->name());
                        }

                        pybind11::globals()["proj"] = uiState_.project;
                    }

                    for (auto [name, erased] : uiState_.project->reflect()) {
                        if (not erased.get() and erased.call_func != nullptr) {
                            if (ImGui::Button(name.c_str())) {
                                // erased.call();
                                projectFunctionResult = std::async(std::launch::async, [erased]() { erased.call(); });
                            }
                        }

                        if (erased.function_with_pack.has_value()) {
                            if (ImGui::Button(name.c_str())) {
                                functionCallParameters.clear();
                                for (auto &arg : erased.function_with_pack.value().meta.arguments) {
                                    functionCallParameters.push_back(arg.default_value);
                                    spdlog::info(arg.name);
                                }

                                for (int i = 0; i < functionCallParameters.size(); i++) {
                                    if (functionCallParameters[i].type() == typeid(int)) {
                                        spdlog::info("has type");
                                    }
                                }

                                showFunctionCallDialog = true;

                                functionWithParamPack = erased.function_with_pack.value();
                                // erased.call();
                                // projectFunctionResult = std::async(std::launch::async, [erased]() { erased.call();
                                // });
                            }
                        }
                    }
                }

                if (showFunctionCallDialog) {
                    functionCallDialogRender();
                }
            }
        }
        ImGui::End();
    }

    void functionCallDialogRender() {
        // Center the window
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        ImGui::Begin("Input Dialog", &showFunctionCallDialog,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("Enter your arguments:");

        GraphicsLabReflection::GraphicsLabFunctionParameterPack parameterPack;
        for (int i = 0; auto &arg : functionWithParamPack.meta.arguments) {
            GraphicsLabReflection::Parameter parameter;

            if (arg.type_info_func() == typeid(float)) {
                float &val = std::any_cast<float &>(functionCallParameters[i]);
                ImGui::InputFloat(std::format("Param: {} (Float)", arg.name).c_str(), &val);
                parameter.param = val;
            }

            if (arg.type_info_func() == typeid(int)) {
                int &val = std::any_cast<int &>(functionCallParameters[i]);
                ImGui::InputInt(std::format("Param: {} (Int)", arg.name).c_str(), &val);
                parameter.param = val;
            }

            parameterPack.parameters.push_back(parameter);
            i++;
        }

        if (ImGui::Button("OK")) {
            showFunctionCallDialog = false;

            projectFunctionResult =
                std::async(std::launch::async, [&]() { functionWithParamPack.function_with_parameter(parameterPack); });

            spdlog::info("called");
        }

        ImGui::SameLine();

        if (ImGui::Button("Cancel")) {
            // Close the dialog
            showFunctionCallDialog = false;
        }

        ImGui::End();
    }
};

META_REGISTER_TYPE(MainComponentRegisterTag, ProjectWidgetComponent)