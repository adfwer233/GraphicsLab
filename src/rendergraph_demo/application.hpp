#pragma once

#include "graphics_lab/render_graph/render_context.hpp"

#ifndef DATA_DIR
#define DATA_DIR "./shader/"
#endif

class Application {
  private:
    static constexpr int WIDTH = 1024;
    static constexpr int HEIGHT = 1024;

    VklWindow window_{WIDTH, HEIGHT};
    VklDevice device_;
    GraphicsLab::RenderGraph::RenderContext context;

  public:
    Application() : device_(window_), context(device_, window_, {WIDTH, HEIGHT}) {};
    ~Application();

    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;

    void run();
};