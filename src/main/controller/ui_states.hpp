#pragma once

#include "reflection/reflectors.hpp"

struct UIState {
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

    RenderMode renderMode;
    LightingMode lightingMode;

    bool isMouseLeftPressing{false};
    bool isMouseMidPressing{false};
    bool mouseFlag = true; // true means that lase mouse pos should be updated
    bool isMouseInRegion{false};
    bool isPressingShift{false};

    float mouseXPos{}, lastMouseXPos{};
    float mouseYPos{}, lastMouseYPos{};

    glm::vec2 scope_min;
    glm::vec2 scope_max;

    struct ProjectStatus {
        std::string name;
        std::string projectPath;

        struct BuildConfig {
            std::string buildType;
            std::string dllPath;
        };

        std::vector<BuildConfig> buildConfigs;
    };

    ProjectStatus projectStatus;
};