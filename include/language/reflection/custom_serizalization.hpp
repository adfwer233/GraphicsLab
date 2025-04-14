#pragma once

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

template <typename T> inline nlohmann::json custom_serialize(const glm::vec<3, T> &v) {
    return {{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

template <typename T> inline nlohmann::json custom_serialize(const glm::vec<2, T> &v) {
    return {{"x", v.x}, {"y", v.y}};
}
} // namespace custom