#pragma once

#include "glm/glm.hpp"
#include "geometry/mesh/mesh.hpp"

namespace GraphicsLab::Geometry {

struct ParamSurface {
    virtual ~ParamSurface() = default;
    struct SurfaceTrait{};
    struct SceneTreeGeometryTypeTrait {};

    using ParamType = glm::vec<2, double>;
    using PointType = glm::vec<3, double>;

    virtual PointType evaluate(const ParamType param) = 0;

    std::unique_ptr<Mesh3D> mesh = nullptr;
};

}