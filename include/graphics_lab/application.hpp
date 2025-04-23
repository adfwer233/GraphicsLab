#pragma once

#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/render_graph/render_context.hpp"
#include "project.hpp"

struct ApplicationOption {
    std::optional<std::string> load_obj_path = std::nullopt;
};

struct GraphicsLabApplication {
  private:
    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1280;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    std::function<IGraphicsLabProject *()> projectFactory = nullptr;

    GraphicsLabApplication() : appContext(WIDTH, HEIGHT) {};
    ~GraphicsLabApplication();

    GraphicsLabApplication(const GraphicsLabApplication &) = delete;
    GraphicsLabApplication &operator=(const GraphicsLabApplication &) = delete;

    ApplicationOption appOption;
    void run();
};