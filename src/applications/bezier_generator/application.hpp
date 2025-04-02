#pragma once

#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/render_graph/render_context.hpp"

struct BezierGeneratorApplication {
  private:
    static constexpr int WIDTH = 1980;
    static constexpr int HEIGHT = 1080;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    BezierGeneratorApplication() : appContext(WIDTH, HEIGHT) {};
    ~BezierGeneratorApplication();

    BezierGeneratorApplication(const BezierGeneratorApplication &) = delete;
    BezierGeneratorApplication &operator=(const BezierGeneratorApplication &) = delete;

    void run();
};