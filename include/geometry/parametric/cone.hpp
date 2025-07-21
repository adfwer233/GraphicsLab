#pragma once

#include "glm/glm.hpp"
#include "parametric_surface.hpp"

namespace GraphicsLab::Geometry {
struct Cone : public ParamSurface {
private:
    PointType base_point_;
    VectorType base_normal_;
    VectorType direction1_;
};
} // namespace GraphicsLab::Geometry