#pragma once

#include "glm/glm.hpp"

#include "geometry/parametric/torus.hpp"

namespace GraphicsLab::Geometry {

struct TorusTorusIntersector {
    using PointType = glm::dvec3;
    using ParamType = glm::dvec2;
    static std::pair<ParamType, ParamType> find_inital_guess(const Torus &torus1, const Torus &torus2) {
        // Implement the logic to find the initial guess for the intersection

        // auto torus1_sample = torus1.sample(10);
    }

    static std::vector<PointType> intersect(const Torus &torus1, const Torus &torus2) {
        std::vector<PointType> intersections;

        // Implement the intersection logic here
        // This is a placeholder for the actual intersection algorithm

        return intersections;
    }

};

}