#pragma once

#include "glm/glm.hpp"

#include "custom_reflector_template.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

/**
 * Example of writing custom reflector
 */

template <> struct CustomReflector<glm::vec3> {

    static json serialization(const glm::vec3 &vec) {
        json j = json::array();
        j.push_back(vec.x);
        j.push_back(vec.y);
        j.push_back(vec.z);

        return j;
    }

    static glm::vec3 deserialization(const json &j) {
        auto tmp = j.get<std::array<float, 3>>();
        return {tmp[0], tmp[1], tmp[2]};
    }
};