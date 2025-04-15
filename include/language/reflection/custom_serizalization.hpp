#pragma once

#include <format>

#include "glm/glm.hpp"
#include "nlohmann/json.hpp"

/**
 * Custom serialization
 */
namespace custom {
// Default fallback
template <typename T> nlohmann::json custom_serialize(const T &) {
    return "<non-serializable>";
}

template <typename T> T custom_deserialize(const nlohmann::json &json, std::type_identity<T>) {
    // spdlog::info("Custom deserialize failed");
    throw std::runtime_error(std::format("Custom deserialize not implemented T: {}", typeid(T).name()));
    return {};
}

template <typename T> inline nlohmann::json custom_serialize(const glm::vec<3, T> &v) {
    return {{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

template <typename T> inline nlohmann::json custom_serialize(const glm::vec<2, T> &v) {
    return {{"x", v.x}, {"y", v.y}};
}

template <typename T>
inline glm::vec<3, T> custom_deserialize(const nlohmann::json &j, std::type_identity<glm::vec<3, T>>) {
    return glm::vec<3, T>{j.at("x").get<T>(), j.at("y").get<T>(), j.at("z").get<T>()};
}

template <typename T>
inline glm::vec<2, T> custom_deserialize(const nlohmann::json &j, std::type_identity<glm::vec<2, T>>) {
    return glm::vec<2, T>{j.at("x").get<T>(), j.at("y").get<T>()};
}

} // namespace custom