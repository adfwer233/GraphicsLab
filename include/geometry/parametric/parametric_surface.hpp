#pragma once

#include "geometry/mesh/mesh.hpp"
#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

struct ParamSurface {
    virtual ~ParamSurface() = default;
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    using T = double;
    using ParamType = glm::vec<2, double>;
    using PointType = glm::vec<3, double>;
    using VectorType = glm::vec<3, double>;

    virtual PointType evaluate(const ParamType param) = 0;
    virtual PointType normal(const ParamType param) = 0;
    virtual std::pair<PointType, ParamType> project(const PointType point) = 0;
    virtual bool test_point(const PointType point) = 0;

    std::unique_ptr<Mesh3D> mesh = nullptr;
};

} // namespace GraphicsLab::Geometry