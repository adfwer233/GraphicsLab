#pragma once

#include <algorithm>

#include "grid.hpp"

namespace GraphicsLab::Simulation {

/**
 * Base class for fluid simulation, using basic Euler view
 */
struct BaseFluidSimulator {
    Grid2D<float> velocity_x;
    Grid2D<float> velocity_y;
    Grid2D<float> pressure;
    Grid2D<float> divergence;
    Grid2D<float> density;
    Grid2D<float> curl;
    Grid2D<float> speed;

    const float dt = 0.1f;
    const float diffusion = 0.0f;
    const float viscosity = 1.0f;

    BaseFluidSimulator(int width, int height)
        : velocity_x(width, height), velocity_y(width, height), pressure(width, height), divergence(width, height),
          density(width, height), curl(width, height), speed(width, height) {
    }

    void add_source(Grid2D<float> &grid, const Grid2D<float> &source) {
#pragma omp parallel for
        for (int y = 0; y < grid.height; ++y) {
            for (int x = 0; x < grid.width; ++x) {
                grid(x, y) += dt * source(x, y);
            }
        }
    }

    void diffuse(Grid2D<float> &grid, const Grid2D<float> &prev, float diff) {
        float a = dt * diff * grid.width * grid.height;
        for (int k = 0; k < 20; ++k) {
#pragma omp parallel for
            for (int y = 1; y < grid.height - 1; ++y) {
                for (int x = 1; x < grid.width - 1; ++x) {
                    grid(x, y) =
                        (prev(x, y) + a * (grid(x - 1, y) + grid(x + 1, y) + grid(x, y - 1) + grid(x, y + 1))) /
                        (1 + 4 * a);
                }
            }
        }
    }

    void advect(Grid2D<float> &grid, const Grid2D<float> &prev, const Grid2D<float> &vel_x,
                const Grid2D<float> &vel_y) {
#pragma omp parallel for
        for (int y = 1; y < grid.height - 1; ++y) {
            for (int x = 1; x < grid.width - 1; ++x) {
                float x0 = x - dt * vel_x(x, y);
                float y0 = y - dt * vel_y(x, y);

                x0 = std::clamp(x0, 0.5f, float(grid.width) - 1.5f);
                y0 = std::clamp(y0, 0.5f, float(grid.height) - 1.5f);

                int i0 = int(x0), i1 = i0 + 1;
                int j0 = int(y0), j1 = j0 + 1;

                float s1 = x0 - i0, s0 = 1 - s1;
                float t1 = y0 - j0, t0 = 1 - t1;

                grid(x, y) =
                    s0 * (t0 * prev(i0, j0) + t1 * prev(i0, j1)) + s1 * (t0 * prev(i1, j0) + t1 * prev(i1, j1));
            }
        }
    }

    void project(Grid2D<float> &vel_x, Grid2D<float> &vel_y, Grid2D<float> &pressure, Grid2D<float> &divergence) {
#pragma omp parallel for
        for (int y = 1; y < vel_x.height - 1; ++y) {
            for (int x = 1; x < vel_x.width - 1; ++x) {
                divergence(x, y) = -0.5f * (vel_x(x + 1, y) - vel_x(x - 1, y) + vel_y(x, y + 1) - vel_y(x, y - 1));
                pressure(x, y) = 0;
            }
        }

        for (int k = 0; k < 20; ++k) {
#pragma omp parallel for
            for (int y = 1; y < pressure.height - 1; ++y) {
                for (int x = 1; x < pressure.width - 1; ++x) {
                    pressure(x, y) = (divergence(x, y) + pressure(x - 1, y) + pressure(x + 1, y) + pressure(x, y - 1) +
                                      pressure(x, y + 1)) /
                                     4.0f;
                }
            }
        }

#pragma omp parallel for
        for (int y = 1; y < vel_x.height - 1; ++y) {
            for (int x = 1; x < vel_x.width - 1; ++x) {
                vel_x(x, y) -= 0.5f * (pressure(x + 1, y) - pressure(x - 1, y));
                vel_y(x, y) -= 0.5f * (pressure(x, y + 1) - pressure(x, y - 1));
            }
        }
    }

    void step() {
        Grid2D<float> prev_density = density;
        Grid2D<float> prev_vel_x = velocity_x;
        Grid2D<float> prev_vel_y = velocity_y;

        diffuse(velocity_x, prev_vel_x, viscosity);
        diffuse(velocity_y, prev_vel_y, viscosity);
        project(velocity_x, velocity_y, pressure, divergence);

        advect(density, prev_density, velocity_x, velocity_y);
        advect(velocity_x, prev_vel_x, velocity_x, velocity_y);
        advect(velocity_y, prev_vel_y, velocity_x, velocity_y);

        project(velocity_x, velocity_y, pressure, divergence);

        compute_curl(velocity_x, velocity_y, curl);
        compute_speed(velocity_x, velocity_y, speed);
    }

    void compute_curl(const Grid2D<float> &vel_x, const Grid2D<float> &vel_y, Grid2D<float> &curl) {
        #pragma omp parallel for
        for (int y = 1; y < curl.height - 1; ++y) {
            for (int x = 1; x < curl.width - 1; ++x) {
                float vx_dy = (vel_x(x, y) - vel_x(x, y - 1));
                float vy_dx = (vel_y(x, y) - vel_y(x - 1, y));
                curl(x, y) = vx_dy - vy_dx;
            }
        }
    }

    void compute_speed(const Grid2D<float> &vel_x, const Grid2D<float> &vel_y, Grid2D<float> &speed) {
        #pragma omp parallel for
        for (int y = 1; y < speed.height - 1; ++y) {
            for (int x = 1; x < speed.width - 1; ++x) {
                speed(x, y) = std::sqrt(std::pow(vel_x(x, y), 2) + std::pow(vel_y(x, y), 2));
            }
        }
    }
};
}; // namespace GraphicsLab::Simulation