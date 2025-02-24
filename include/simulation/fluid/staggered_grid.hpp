#pragma once

#include "grid.hpp"

namespace GraphicsLab::Simulation {

template <typename T>
    requires std::is_floating_point_v<T>
struct StaggeredGrid2D {
    int width, height;

    Grid2D<float> velocity_x;
    Grid2D<float> velocity_y;
    Grid2D<float> density;
    Grid2D<float> pressure;
    Grid2D<float> divergence;

    StaggeredGrid2D(int width, int height)
        : velocity_x(width + 1, height), velocity_y(width, height + 1), pressure(width, height),
          divergence(width, height), density(width, height) {
    }
};

} // namespace GraphicsLab::Simulation