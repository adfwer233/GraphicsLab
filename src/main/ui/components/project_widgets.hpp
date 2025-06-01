#pragma once

#include <future>

#include "component.hpp"
#include "controller/ui_states.hpp"

#include "project/file_system.hpp"
#include "project/project_manager.hpp"

#include "ui/render_resources.hpp"

#include "graphics_lab/render_passes/simple_pass.hpp"
#include "vkl/imgui/imgui_reflection.hpp"

struct mystruct : public Reflectable {
    glm::vec3 pos;

    ReflectDataType reflect() override {
        return {{"pos", TypeErasedValue(&pos)}};
    }
};

class ProjectWidgetComponent : public UIComponent {
    UIState &uiState_;
    std::future<void> projectFunctionResult;

    std::set<std::string> bindClassNames;

  public:
    ProjectWidgetComponent(GraphicsLab::GraphicsLabInternalContext &context, UIState &uiState)
        : UIComponent(context), uiState_(uiState) {
    }

    void render() final {
        ImGui::Begin("Project Manager");
        {
            if (ImGui::Button("add render pass")) {
                auto simple_pass = new GraphicsLab::RenderGraph::SimpleRenderPass(context_.device_);
                simple_pass->set_extent(2048, 2048);
                context_.renderGraph->add_pass(simple_pass, "New simple pass");

                context_.compileRenderGraph();
            }

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
                    context_.sceneTree->cleanSceneTree();

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
                                    spdlog::info(context_.sceneTree->root->name);
                                    uiState_.project->updateContext(
                                        GraphicsLabContext(&context_.sceneTree->device_, context_.sceneTree.get(),
                                                           &LogManager::getInstance(), &uiState_, &context_));
                                    uiState_.project->afterLoad();
                                    ControllerCallbackHandler::project_controller = uiState_.project->getController();
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
                auto result = StaticReflect::serialization(uiState_.projectStatus);
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
                StaticReflect::deserialization(uiState_.projectStatus, j);
            }

            ImGui::Text("Project Functions");

            // spdlog::info("stat {}", int(projectFunctionResult.wait_for(std::chrono::milliseconds(500))));

            if ((projectFunctionResult.valid() and
                 projectFunctionResult.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) or
                uiState_.project_load_by_factory) {

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
                }

                render_reflection_functions.render_functions(uiState_.project);
            }
        }
        ImGui::End();
    }

  private:
    vkl::ImGuiReflection render_reflection_functions;
};

META_REGISTER_TYPE(MainComponentRegisterTag, ProjectWidgetComponent)