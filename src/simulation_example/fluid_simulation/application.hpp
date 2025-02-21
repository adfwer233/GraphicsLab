#pragma once

#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/render_graph/render_context.hpp"

struct FluidSimulationApplication {
private:
    static constexpr int WIDTH = 1980;
    static constexpr int HEIGHT = 1080;

    GraphicsLab::GraphicsLabInternalContext appContext;

public:
    FluidSimulationApplication() : appContext(WIDTH, HEIGHT) {};
    ~FluidSimulationApplication();

    FluidSimulationApplication(const FluidSimulationApplication &) = delete;
    FluidSimulationApplication &operator=(const FluidSimulationApplication &) = delete;

    void run();
};