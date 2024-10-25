#pragma once

#include "graphics_lab/render_graph/render_context.hpp"

struct ApplicationExperimental {
private:
    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1280;

    VklWindow window_{WIDTH, HEIGHT};
    VklDevice device_;
    GraphicsLab::RenderGraph::RenderContext context;

public:
    ApplicationExperimental() : device_(window_), context(device_, window_, {WIDTH, HEIGHT}) {};
    ~ApplicationExperimental();

    ApplicationExperimental(const ApplicationExperimental &) = delete;
    ApplicationExperimental &operator=(const ApplicationExperimental &) = delete;

    void run();
};