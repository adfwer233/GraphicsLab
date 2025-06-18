#pragma once

#include "language/reflection/reflectors.hpp"
#include "language/reflection/static_reflector.hpp"
#include "platform/file_system.hpp"

#include <filesystem>
#include <future>
#include <string>
#include <vector>

namespace GraphicsLab {

struct RecentModels : Reflectable {

    struct ModelData {
        std::string name;
        std::string path;

        REFLECT(Property{"name", &ModelData::name}, Property{"path", &ModelData::path})
    };
    std::vector<ModelData> models;

    void add_model() {
        std::string path = FileSystem::chooseFile();
        if (path != "") {
            ModelData model;
            std::filesystem::path filePath(path);
            model.name = filePath.filename().string();
            model.path = path;

            models.push_back(model);
        }
    }

    ReflectDataType reflect() override {
        return {
            {"add_model", TypeErasedValue(&RecentModels::add_model, this)},
        };
    }

    REFLECT(Property{"models", &RecentModels::models})
};

} // namespace GraphicsLab