#pragma once
#include "language/reflection/static_reflector.hpp"
#include <fstream>
#include <iostream>
#include <string>

#include "language/template/string_literal.hpp"
#include "platform/file_system.hpp"
#include "spdlog/spdlog.h"

template <typename Derived, StringLiteral Name> class AutoSerializeSingleton {
  public:
    AutoSerializeSingleton(const AutoSerializeSingleton &) = delete;
    AutoSerializeSingleton &operator=(const AutoSerializeSingleton &) = delete;

    static Derived &instance() {
        static Derived instance;
        return instance;
    }

  protected:
    explicit AutoSerializeSingleton() = default;

    ~AutoSerializeSingleton() {
    }

    void finalize() {
        auto serialized_json = StaticReflect::serialization(*static_cast<Derived *>(this));
        std::ofstream ofs(get_path());
        ofs << serialized_json.dump(4);

        spdlog::info("serialize {}, {}", Name.value, serialized_json.dump(4));
    }

    void initialize() {
        std::filesystem::path file_path(get_path());

        if (std::filesystem::exists(file_path)) {
            std::ifstream file(file_path);
            std::stringstream buffer;
            buffer << file.rdbuf();
            nlohmann::json j;
            buffer >> j;

            spdlog::info("here");
            spdlog::info("{}", j.dump(4));

            StaticReflect::deserialization(*static_cast<Derived *>(this), j);

            spdlog::info("here 2");
        }
    }

    static std::string get_path() {
        std::string name = Name.value;
        std::filesystem::path exe_path(FileSystem::getExecutablePath());
        std::filesystem::path target_path = exe_path / std::format("{}.json", name);

        return target_path.string();
    }
};