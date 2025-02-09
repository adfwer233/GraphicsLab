#pragma once

#include <cstdlib>
#include <utility>

#include "../project/project_manager.hpp"
#include "graphics_lab/graphics_lab_controller.hpp"
#include "language/reflection/reflectors.hpp"
#include "pybind11/embed.h"

struct UIState : Reflectable {
    enum class RenderMode {
        raw,
        color,
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
    bool showNormal = false;
    bool showAxis = false;

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

    std::map<std::string, glm::vec2> scope_min;
    std::map<std::string, glm::vec2> scope_max;

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
        return {{"python_interpreter_path", TypeErasedValue(&python_interpreter_path)},
                {"scope_min", TypeErasedValue(&scope_min)},
                {"scope_max", TypeErasedValue(&scope_max)}};
    }

    bool reset_camera = false;

    UIState() = default;

    ~UIState() = default;
};