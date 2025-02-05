#pragma once

#include "geometry/mesh/mesh.hpp"
#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

struct ParamSurface {
    virtual ~ParamSurface() = default;
    struct SurfaceTrait {};
    struct SceneTreeGeometryTypeTrait {};

    using ParamType = glm::vec<2, double>;
    using PointType = glm::vec<3, double>;

    virtual PointType evaluate(const ParamType param) = 0;

    std::unique_ptr<Mesh3D> mesh = nullptr;
};

} // namespace GraphicsLab::Geometry