#pragma once

#include <cstdlib>
#include <utility>

#include "graphics_lab/graphics_lab_controller.hpp"
#include "../project/project_manager.hpp"
#include "language/reflection/reflectors.hpp"
#include "pybind11/embed.h"

struct UIState : Reflectable {
    enum class RenderMode {
        raw,
        wireframe,
        material,
        path_tracing
    };

    enum class LightingMode {
        solid,
        simple
    };

    RenderMode renderMode = RenderMode::raw;
    LightingMode lightingMode = LightingMode::simple;

    bool isMouseLeftPressing{false};
    bool isMouseMidPressing{false};
    bool mouseFlag = true; // true means that lase mouse pos should be updated
    bool isMouseInRegion{false};
    bool isPressingShift{false};

    // for scaling
    bool isPressingS{false};

    // for rotation
    bool isPressingR{false};

    // for translation
    bool isPressingG{false};

    float mouseXPos{}, lastMouseXPos{};
    float mouseYPos{}, lastMouseYPos{};

    glm::vec2 scope_min;
    glm::vec2 scope_max;

    struct ProjectStatus : Reflectable {
        std::string name;
        std::string projectPath;

        struct BuildConfig : Reflectable {
            std::string buildType;
            std::string dllPath;

            BuildConfig() = default;
            BuildConfig(std::string t_buildType, std::string t_dllPath)
                : buildType(std::move(t_buildType)), dllPath(std::move(t_dllPath)) {
            }

            ReflectDataType reflect() override {
                return {{"buildType", TypeErasedValue(&buildType)}, {"dllPath", TypeErasedValue(&dllPath)}};
            }
        };

        std::vector<BuildConfig> buildConfigs;

        ReflectDataType reflect() override {
            return {{"name", TypeErasedValue(&name)},
                    {"projectPath", TypeErasedValue(&projectPath)},
                    {"buildConfigs", TypeErasedValue(&buildConfigs)}};
        }
    };

    ProjectStatus projectStatus;
    ProjectManager projectManager;
    IGraphicsLabProject *project = nullptr;

    /**
     * picked bounding box data
     */

    AABB3D box;
    bool boxMeshRecreated = false;

    /**
     * Python scripts supporting
     */

    // Initialize the Python interpreter
    pybind11::scoped_interpreter python;

    std::string python_interpreter_path;

    ReflectDataType reflect() override {
        return {{"python_interpreter_path", TypeErasedValue(&python_interpreter_path)}};
    }

    UIState() {
        spdlog::critical("ui state constructed");

        std::ifstream file("graphics_lab_state_config.json");
        if (file.is_open()) {
            std::stringstream buffer;
            buffer << file.rdbuf();

            json j;
            buffer >> j;
            spdlog::critical(j.dump());

            deserialization(j);

            // spdlog::info(Py_EncodeLocale(Py_GetPath(), nullptr));
            if (not python_interpreter_path.empty()) {
                // wchar_t *program = Py_DecodeLocale(python_interpreter_path.c_str(), nullptr);
                //
                // Py_SetProgramName(program);  // Optional: sets the program name (default to the binary)
                // Py_SetPath(program);         // Set Python interpreter path
            }
        } else {
            auto result = serialization();
            std::ofstream ofstream("graphics_lab_state_config.json");
            ofstream << result.dump(4);
        }

        // Py_Initialize();
    }

    ~UIState() {
        auto result = serialization();
        std::ofstream ofstream("graphics_lab_state_config.json");
        ofstream << result.dump(4);
        spdlog::critical("ui state destructed");

        // Py_Finalize();
    }
};