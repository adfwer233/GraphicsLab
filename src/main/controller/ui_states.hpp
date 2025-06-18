#pragma once

#include <cstdlib>
#include <utility>

#include "../project/project_manager.hpp"
#include "graphics_lab/graphics_lab_controller.hpp"
#include "language/crtp/auto_serialize_singleton.hpp"
#include "language/reflection/reflectors.hpp"
#include "storage/recent_models.hpp"

#ifdef ENABLE_PYTHON
#include "pybind11/embed.h"
#endif

struct UIState : Reflectable, AutoSerializeSingleton<UIState, "UIState"> {
    enum class RenderMode {
        raw,
        color,
        wireframe,
        material,
        path_tracing,
        soft_rasterizer
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

    struct ProjectStatus {
        std::string name;
        std::string projectPath;

        struct BuildConfig {
            std::string buildType;
            std::string dllPath;

            BuildConfig() = default;
            BuildConfig(std::string t_buildType, std::string t_dllPath)
                : buildType(std::move(t_buildType)), dllPath(std::move(t_dllPath)) {
            }

            REFLECT(Property{"buildType", &BuildConfig::buildType}, Property{"dllPath", &BuildConfig::dllPath});
        };

        std::vector<BuildConfig> buildConfigs;

        REFLECT(Property{"name", &ProjectStatus::name}, Property{"projectPath", &ProjectStatus::projectPath},
                Property{"buildConfigs", &ProjectStatus::projectPath});
    };

    ProjectStatus projectStatus;
    ProjectManager projectManager;
    IGraphicsLabProject *project = nullptr;
    bool project_load_by_factory = false;

    /**
     * picked bounding box data
     */

    AABB3D box;
    bool boxMeshRecreated = false;
    bool showBox = false;

    /**
     * Python scripts supporting
     */

    std::string python_interpreter_path;

    ReflectDataType reflect() override {
        return {{"python_interpreter_path", TypeErasedValue(&python_interpreter_path)},
                {"scope_min", TypeErasedValue(&scope_min)},
                {"scope_max", TypeErasedValue(&scope_max)}};
    }

    bool reset_camera = false;
    bool reset_gpu_bvh = false;

    // persistent data

    GraphicsLab::RecentModels recent_models;

    REFLECT(Property{"recent_models", &UIState::recent_models});

    explicit UIState() {
        initialize();
    };

    ~UIState() {
        finalize();
    };
};