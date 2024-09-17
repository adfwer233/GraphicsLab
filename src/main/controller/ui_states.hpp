#pragma once

#include <utility>

#include "language/reflection/reflectors.hpp"
#include "project/project_manager.hpp"

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

    RenderMode renderMode = RenderMode::raw;
    LightingMode lightingMode = LightingMode::simple;

    bool isMouseLeftPressing{false};
    bool isMouseMidPressing{false};
    bool mouseFlag = true; // true means that lase mouse pos should be updated
    bool isMouseInRegion{false};
    bool isPressingShift{false};

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

            // struct ReflectImpl {
            //     static std::string serialize(BuildConfig &config) {
            //         auto map = std::map<std::string, std::string> {
            //                 {"buildType", config.buildType},
            //                 {"dllPath", config.buildType},
            //         };
            //
            //         return Reflection::serialize(map);
            //     }
            //
            //     static BuildConfig deserialize(const std::string &str) {
            //         auto map = Reflection::deserialize<std::map<std::string, std::string>>(str);
            //         return BuildConfig (map["buildType"], map["dllPath"]);
            //     }
            // };
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

    UIState() {
        spdlog::critical("ui state constructed");
    }

    ~UIState() {
        spdlog::critical("ui state destructed");
    }
};