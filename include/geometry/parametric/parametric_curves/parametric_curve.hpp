#pragma once

#include <glm/glm.hpp>

namespace GraphicsLab::Geometry {
struct ParamCurve3D {
    virtual ~ParamCurve3D() = default;

    virtual glm::dvec3 evaluate(double t) const = 0;

    std::unique_ptr<CurveMesh3D> mesh = nullptr;
};

struct ParamCurve2D {
    virtual ~ParamCurve2D() = default;

    virtual glm::dvec2 evaluate(double t) const = 0;

    std::unique_ptr<CurveMesh2D> mesh = nullptr;
};

template<size_t dim>
using ParamCurveBase = std::conditional_t<dim == 3, ParamCurve3D, ParamCurve2D>;
} // namespace GraphicsLab::Geometry