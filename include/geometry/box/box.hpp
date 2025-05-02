#pragma once

#include "glm/glm.hpp"

namespace GraphicsLab::Geometry {

template<size_t dim, typename T>
struct Box {
    using point_type = glm::vec<dim, T>;
    point_type min, max;

    Box() {
        for (int i = 0; i < dim; i++) {
            min[i] = std::numeric_limits<T>::max();
            max[i] = -std::numeric_limits<T>::max();
        }
    }
};

using Box2D = Box<2, float>;
using Box3D = Box<3, float>;

using ParamBox2D = Box<2, double>;
using ParamBox3D = Box<3, double>;

}