#pragma once

#include <random>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

namespace GraphicsLab {

struct Sampler {
    static std::mt19937 &rng() {
        thread_local std::mt19937 gen(std::random_device{}());
        return gen;
    }

    static float sampleUniform(float min = 0.0, float max = 1.0) {
        thread_local std::uniform_real_distribution<float> dist(min, max);
        return dist(rng());
    }

    template <size_t dim> static glm::vec<dim, float> sampleUnitSphere() {
        static_assert(dim == 2 || dim == 3, "Only 2D or 3D sampling is supported.");

        if constexpr (dim == 2) {
            float theta = sampleUniform() * glm::two_pi<float>();
            return glm::vec<2, float>(std::cos(theta), std::sin(theta));
        } else if constexpr (dim == 3) {
            float z = 2.0f * sampleUniform() - 1.0f;
            float phi = sampleUniform() * glm::two_pi<float>();
            float r = std::sqrt(1.0f - z * z);
            return glm::vec<3, float>(r * std::cos(phi), r * std::sin(phi), z);
        }
    }
};
} // namespace GraphicsLab