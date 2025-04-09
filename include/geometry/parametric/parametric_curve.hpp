#pragma once

#include <glm/glm.hpp>

namespace GraphicsLab::Geometry {
struct ParamCurve3D {
    virtual ~ParamCurve3D() = default;

    virtual glm::dvec3 evaluate(double t) const = 0;
};

struct ParamCurve2D {
    virtual ~ParamCurve2D() = default;

    virtual glm::dvec2 evaluate(double t) const = 0;
};
} // namespace GraphicsLab::Geometry