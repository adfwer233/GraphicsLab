#pragma once

#include <iostream>
#include <glm/vec3.hpp> // For glm::vec3
#include <glm/glm.hpp>  // For glm::clamp

#define TINYCOLORMAP_WITH_GLM
#include "tinycolormap.hpp"

// Base class for interpolation
class ColorInterpolator {
protected:
    double minInput, maxInput;

    // Normalize the input to the [0, 1] range
    double normalizeInput(double input) const {
        return (input - minInput) / (maxInput - minInput);
    }

    // Linear interpolation between two colors (glm::vec3)
    glm::vec3 lerp(const glm::vec3& c1, const glm::vec3& c2, float t) const {
        return c1 + t * (c2 - c1);
    }

public:
    ColorInterpolator(double minInput, double maxInput)
            : minInput(minInput), maxInput(maxInput) {}

    // Input is a real number between [minInput, maxInput] and output is the interpolated color
    virtual glm::vec3 interpolate(double input) const = 0;

    virtual ~ColorInterpolator() = default;
};

class ViridisInterpolator : public ColorInterpolator {
public:
    ViridisInterpolator(double minInput, double maxInput)
            : ColorInterpolator(minInput, maxInput) {}

    glm::vec3 interpolate(double input) const override {
        input = std::clamp(input, minInput, maxInput);
        double normalizedInput = normalizeInput(input);
        auto res = tinycolormap::GetColor(normalizedInput, tinycolormap::ColormapType::Viridis).ConvertToGLM();

        float gamma = 1;
        res = {std::pow(res.x, 1 / gamma), std::pow(res.y, 1 / gamma), std::pow(res.z, 1 / gamma)};

        return res;
    }
};

// Derived class for two-point interpolation
class TwoPointInterpolator : public ColorInterpolator {
private:
    glm::vec3 color1, color2;

public:
    TwoPointInterpolator(const glm::vec3& c1, const glm::vec3& c2, double minInput, double maxInput)
            : ColorInterpolator(minInput, maxInput), color1(c1), color2(c2) {}

    glm::vec3 interpolate(double input) const override {
        // Normalize the input and clamp it between 0 and 1
        double normalizedInput = glm::clamp(normalizeInput(input), 0.0, 1.0);
        // Perform linear interpolation between color1 and color2
        return lerp(color1, color2, normalizedInput);
    }
};

// Derived class for three-point interpolation
class ThreePointInterpolator : public ColorInterpolator {
private:
    glm::vec3 color1, color2, color3;

public:
    ThreePointInterpolator(const glm::vec3& c1, const glm::vec3& c2, const glm::vec3& c3,
                           double minInput, double maxInput)
            : ColorInterpolator(minInput, maxInput), color1(c1), color2(c2), color3(c3) {}

    glm::vec3 interpolate(double input) const override {
        // Normalize the input and clamp it between 0 and 1
        double normalizedInput = glm::clamp(normalizeInput(input), 0.0, 1.0);

        if (normalizedInput < 0.5) {
            // First half: interpolate between color1 and color2
            return lerp(color1, color2, normalizedInput * 2);  // Scale [0, 0.5] to [0, 1]
        } else {
            // Second half: interpolate between color2 and color3
            return lerp(color2, color3, (normalizedInput - 0.5) * 2);  // Scale [0.5, 1] to [0, 1]
        }
    }
};