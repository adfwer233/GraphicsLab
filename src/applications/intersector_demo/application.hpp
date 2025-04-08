#pragma once

#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/render_graph/render_context.hpp"

struct IntersectorDemoApplication {
  private:
    static constexpr int WIDTH = 1980;
    static constexpr int HEIGHT = 1080;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    IntersectorDemoApplication() : appContext(WIDTH, HEIGHT) {};
    ~IntersectorDemoApplication();

    IntersectorDemoApplication(const IntersectorDemoApplication &) = delete;
    IntersectorDemoApplication &operator=(const IntersectorDemoApplication &) = delete;

    void run();
};