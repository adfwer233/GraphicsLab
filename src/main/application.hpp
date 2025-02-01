#pragma once

#include "graphics_lab/graphics_lab_context.hpp"
#include "graphics_lab/render_graph/render_context.hpp"

struct ApplicationOption {
    std::optional<std::string> load_obj_path = std::nullopt;
};

struct ApplicationExperimental {
  private:
    static constexpr int WIDTH = 1024 + 768;
    static constexpr int HEIGHT = 1280;

    GraphicsLab::GraphicsLabInternalContext appContext;

  public:
    ApplicationExperimental() : appContext(WIDTH, HEIGHT) {};
    ~ApplicationExperimental();

    ApplicationExperimental(const ApplicationExperimental &) = delete;
    ApplicationExperimental &operator=(const ApplicationExperimental &) = delete;

    ApplicationOption appOption;
    void run();
};