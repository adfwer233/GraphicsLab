#pragma once

#include "geometry/parametric/explicit_surface.hpp"

struct ExplicitSurfaceExample {

    static GraphicsLab::Geometry::ExplicitSurface createDeformedTorus(glm::vec3 offset = {0.0, 0.0, 0.0});

    static GraphicsLab::Geometry::ExplicitSurface createDeformedTorus2(glm::vec3 offset = {0.0, 0.0, 0.0},
                                                                       glm::vec2 param_offset = {0.0, 0.0});
};